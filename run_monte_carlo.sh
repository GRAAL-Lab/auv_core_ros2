#!/usr/bin/env bash
set -Eeuo pipefail
shopt -s nullglob

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
DEFAULT_CONFIG="$SCRIPT_DIR/monte_carlo/config/campaign.example.json"
CONFIG="$DEFAULT_CONFIG"
CAMPAIGN_DIR=""
ASSUME_YES=0
COMMAND=""
COMMAND_ARGS=()

usage() {
    cat <<'EOF'
Usage: ./run_monte_carlo.sh [--config FILE] [--campaign-dir DIR] [--yes] COMMAND [ARG...]

Commands:
  preflight              Validate software, configs, plugins, hardware, and disk.
  plan PHASE             Deterministically generate calibration, pilot, or main paths.
  dry-run PHASE          Generate and validate all per-run configs without launching ROS/Isaac.
  estimate PHASE         Estimate calibration, pilot, main, or full campaign duration.
  run-one MANIFEST       Run or resume one accepted scenario.
  run-phase PHASE        Run or resume every scenario in a planned phase.
  calibrate              Plan and run one scenario/family, then save measured rates.
  pilot                  Plan and run the fixed pilot count per family.
  freeze                 Freeze powered main N from the complete pilot.
  run                    Plan and run exactly the frozen main N.
  evaluate               Run calibrate, pilot, freeze, main, and report in order.
  resume PHASE           Alias for run-phase.
  report                 Produce paired main confidence intervals and win rates.
  status                 Show the latest compact status and resource snapshot.
EOF
}

while (($#)); do
    case "$1" in
        --config)
            CONFIG=${2:?--config requires a file}
            shift 2
            ;;
        --campaign-dir)
            CAMPAIGN_DIR=${2:?--campaign-dir requires a directory}
            shift 2
            ;;
        --yes)
            ASSUME_YES=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            COMMAND=$1
            shift
            COMMAND_ARGS=("$@")
            break
            ;;
    esac
done

[[ -n "$COMMAND" ]] || { usage >&2; exit 2; }
CONFIG=$(realpath "$CONFIG")
[[ -f "$CONFIG" ]] || { echo "Config does not exist: $CONFIG" >&2; exit 2; }

export PYTHONPATH="$SCRIPT_DIR/monte_carlo/python${PYTHONPATH:+:$PYTHONPATH}"

json_value() {
    jq -er "$1" "$CONFIG"
}

if [[ -z "$CAMPAIGN_DIR" ]]; then
    CAMPAIGN_DIR=$(python3 - "$CONFIG" <<'PY'
from mc_common import campaign_directory, load_json
import sys
print(campaign_directory(load_json(sys.argv[1])))
PY
)
fi
CAMPAIGN_DIR=$(realpath -m "$CAMPAIGN_DIR")
WORKSPACE_ROOT=$(json_value '.paths.workspace_root')
WORKSPACE_SETUP="$WORKSPACE_ROOT/install/setup.bash"
ISAACSIM_ROOT=$(json_value '.paths.isaacsim_root')
ISAAC_UW_ROOT=$(json_value '.paths.simulator_root')
MVM_ROOT=$(json_value '.paths.mvm_root')
OCEANSIM_ROOT="$ISAACSIM_ROOT/extsUser/OceanSim"
MVM_PY_PATH="$MVM_ROOT/underwater_vehicle_model/build_isaac_py311"
STATUS_PATH="$CAMPAIGN_DIR/status.json"
RESOURCE_LOG="$CAMPAIGN_DIR/resources.jsonl"
RUN_LOCK_FD=""

declare -a CHILD_PIDS=()
declare -a CHILD_NAMES=()
LAUNCHED_PID=0
INTERRUPTED=0
INTERNAL_REPLAY_WORKER=${MONTE_CARLO_INTERNAL_REPLAY_WORKER:-0}
SIDECAR_WORKER=${MONTE_CARLO_SIDECAR_WORKER:-0}
DOMAIN_BASE_OVERRIDE=${MONTE_CARLO_DOMAIN_BASE_OVERRIDE:-}
SIDECAR_CLAIM_DIR=""
PHASE_TOTAL=1
PHASE_COMPLETED=0
PHASE_STARTED_EPOCH=$(date +%s)

append_event() {
    local event=$1
    local run_id=${2:-}
    local detail=${3:-}
    python3 - "$CAMPAIGN_DIR/events.jsonl" "$event" "$run_id" "$detail" <<'PY'
from datetime import datetime, timezone
from mc_common import append_jsonl
import sys
append_jsonl(sys.argv[1], {
    "timestamp": datetime.now(timezone.utc).isoformat(),
    "event": sys.argv[2], "run_id": sys.argv[3], "detail": sys.argv[4]
})
PY
}

update_status() {
    [[ "$SIDECAR_WORKER" != 1 ]] || return 0
    local stage=$1
    local current=${2:-}
    local workers=${3:-0}
    python3 "$SCRIPT_DIR/monte_carlo/python/stage_artifacts.py" status \
        --campaign-dir "$CAMPAIGN_DIR" --output "$STATUS_PATH" \
        --stage "$stage" --current-experiment "$current" \
        --completed "$PHASE_COMPLETED" --total "$PHASE_TOTAL" \
        --active-workers "$workers" --phase-started-epoch "$PHASE_STARTED_EPOCH"
}

launch_group() {
    local name=$1
    local log=$2
    shift 2
    mkdir -p "$(dirname "$log")"
    setsid env --default-signal=INT --default-signal=TERM --default-signal=HUP \
        "$@" >>"$log" 2>&1 &
    LAUNCHED_PID=$!
    CHILD_PIDS+=("$LAUNCHED_PID")
    CHILD_NAMES+=("$name")
    append_event process_started "${CURRENT_RUN_ID:-}" "$name pid=$LAUNCHED_PID"
}

unregister_pid() {
    local target=$1
    local index
    for index in "${!CHILD_PIDS[@]}"; do
        if [[ ${CHILD_PIDS[$index]} == "$target" ]]; then
            CHILD_PIDS[$index]=0
            CHILD_NAMES[$index]=""
        fi
    done
}

signal_group() {
    local pid=$1
    local signal=$2
    # Every child is started through setsid, so its PID is also its process-group
    # ID.  A launcher (notably viz) may exit after spawning RViz; in that case
    # the group is still alive even though the original PID no longer exists.
    if kill -0 -- "-$pid" 2>/dev/null; then
        kill -"$signal" -- "-$pid" 2>/dev/null || true
        return 0
    fi
    kill -0 "$pid" 2>/dev/null || return 0
    local pgid
    pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')
    if [[ "$pgid" == "$pid" ]]; then
        kill -"$signal" -- "-$pid" 2>/dev/null || true
    else
        echo "Refusing broad group signal for pid=$pid pgid=${pgid:-unknown}; signaling pid only" >&2
        kill -"$signal" "$pid" 2>/dev/null || true
    fi
}

stop_group() {
    local pid=$1
    local grace_value=${2:-20}
    local grace
    grace=$(python3 -c "import math; print(math.ceil(float('$grace_value')))")
    [[ "$pid" != 0 ]] || return 0
    if kill -0 "$pid" 2>/dev/null || kill -0 -- "-$pid" 2>/dev/null; then
        signal_group "$pid" INT
        local deadline=$((SECONDS + grace))
        while { kill -0 "$pid" 2>/dev/null || kill -0 -- "-$pid" 2>/dev/null; } \
            && ((SECONDS < deadline)); do sleep 0.25; done
        if kill -0 "$pid" 2>/dev/null || kill -0 -- "-$pid" 2>/dev/null; then
            signal_group "$pid" TERM
            deadline=$((SECONDS + 5))
            while { kill -0 "$pid" 2>/dev/null || kill -0 -- "-$pid" 2>/dev/null; } \
                && ((SECONDS < deadline)); do sleep 0.25; done
        fi
        if kill -0 "$pid" 2>/dev/null || kill -0 -- "-$pid" 2>/dev/null; then
            signal_group "$pid" KILL
        fi
    fi
    wait "$pid" 2>/dev/null || true
    unregister_pid "$pid"
}

stop_recorder_group() {
    local pid=$1
    local grace_value=$2
    local grace
    grace=$(python3 -c "import math; print(math.ceil(float('$grace_value')))")
    [[ "$pid" != 0 ]] || return 0
    if ! kill -0 "$pid" 2>/dev/null && ! kill -0 -- "-$pid" 2>/dev/null; then
        wait "$pid" 2>/dev/null || true
        unregister_pid "$pid"
        return 0
    fi

    signal_group "$pid" INT
    local deadline=$((SECONDS + grace))
    while { kill -0 "$pid" 2>/dev/null || kill -0 -- "-$pid" 2>/dev/null; } \
        && ((SECONDS < deadline)); do
        sleep 0.5
    done
    if kill -0 "$pid" 2>/dev/null || kill -0 -- "-$pid" 2>/dev/null; then
        append_event recorder_flush_timeout "${CURRENT_RUN_ID:-}" "pid=$pid grace_s=$grace"
        stop_group "$pid" 0
        return 124
    fi
    wait "$pid" 2>/dev/null || true
    unregister_pid "$pid"
}

stop_all_children() {
    local index recorder_grace
    local -a recorder_pids=()
    for index in "${!CHILD_PIDS[@]}"; do
        [[ ${CHILD_PIDS[$index]} != 0 ]] || continue
        if [[ ${CHILD_NAMES[$index]} == recorder ]]; then
            recorder_pids+=("${CHILD_PIDS[$index]}")
            continue
        fi
        stop_group "${CHILD_PIDS[$index]}" 10
    done
    recorder_grace=$(jq -er '.recording.shutdown_grace_s // 300' "$CONFIG" 2>/dev/null || echo 300)
    local pid
    for pid in "${recorder_pids[@]}"; do
        stop_recorder_group "$pid" "$recorder_grace" || true
    done
}

on_interrupt() {
    if [[ "$COMMAND" == status ]]; then
        INTERRUPTED=1
        exit 0
    fi
    INTERRUPTED=1
    append_event interrupted "${CURRENT_RUN_ID:-}" "signal received"
    update_status interrupted "${CURRENT_RUN_ID:-}" 0 || true
    stop_all_children
    exit 130
}

on_exit() {
    local code=$?
    stop_all_children
    if [[ -n "$SIDECAR_CLAIM_DIR" ]]; then
        rmdir -- "$SIDECAR_CLAIM_DIR" 2>/dev/null || true
        SIDECAR_CLAIM_DIR=""
    fi
    if ((code != 0 && INTERRUPTED == 0)); then
        append_event runner_exit "${CURRENT_RUN_ID:-}" "exit_code=$code" || true
    fi
}

trap on_interrupt INT TERM HUP
trap on_exit EXIT

wait_group() {
    local pid=$1
    local timeout=$2
    local deadline=$((SECONDS + timeout))
    while kill -0 "$pid" 2>/dev/null; do
        if ((SECONDS >= deadline)); then
            stop_group "$pid" 10
            return 124
        fi
        sleep 0.5
    done
    local code=0
    if wait "$pid"; then code=0; else code=$?; fi
    unregister_pid "$pid"
    return "$code"
}

ros_foreground() {
    bash -lc 'source "$1"; shift; exec "$@"' bash "$WORKSPACE_SETUP" "$@"
}

ros_domain_foreground() {
    local domain=$1
    shift
    bash -lc '
        source "$1"
        export ROS_DOMAIN_ID="$2"
        export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
        export ROS2CLI_NO_DAEMON=1
        shift 2
        exec "$@"
    ' bash "$WORKSPACE_SETUP" "$domain" "$@"
}

launch_ros_group() {
    local name=$1
    local log=$2
    local domain=$3
    shift 3
    launch_group "$name" "$log" bash -lc '
        source "$1"
        export ROS_DOMAIN_ID="$2"
        export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
        export LD_LIBRARY_PATH="$3:${LD_LIBRARY_PATH:-}"
        shift 3
        exec "$@"
    ' bash "$WORKSPACE_SETUP" "$domain" "$MVM_PY_PATH" "$@"
}

launch_isaac_group() {
    local log=$1
    local domain=$2
    local simulator_config=$3
    launch_group isaac "$log" bash -lc '
        isaac_root=$1
        isaac_uw_root=$2
        oceansim_root=$3
        mvm_root=$4
        mvm_python=$5
        domain=$6
        simulator_config=$7
        export ISAACSIM_ROOT="$isaac_root"
        export ISAAC_UW_ROOT="$isaac_uw_root"
        export OCEANSIM_ROOT="$oceansim_root"
        export MVM_ROOT="$mvm_root"
        export MVM_PY_PATH="$mvm_python"
        export __NV_PRIME_RENDER_OFFLOAD=1
        export __GLX_VENDOR_LIBRARY_NAME=nvidia
        export __VK_LAYER_NV_optimus=NVIDIA_only
        unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH
        unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION RMW_IMPLEMENTATION
        source "$ISAACSIM_ROOT/setup_ros_env.sh"
        export ROS_DOMAIN_ID="$domain"
        export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
        export LD_LIBRARY_PATH="$MVM_PY_PATH:${LD_LIBRARY_PATH:-}"
        export PYTHONPATH="$OCEANSIM_ROOT:${PYTHONPATH:-}"
        exec "$ISAACSIM_ROOT/python.sh" "$ISAAC_UW_ROOT/run_underwater_sim.py" \
            --config "$simulator_config"
    ' bash "$ISAACSIM_ROOT" "$ISAAC_UW_ROOT" "$OCEANSIM_ROOT" "$MVM_ROOT" \
        "$MVM_PY_PATH" "$domain" "$simulator_config"
}

acquire_campaign_lock() {
    mkdir -p "$CAMPAIGN_DIR"
    exec {RUN_LOCK_FD}>"$CAMPAIGN_DIR/.campaign.lock"
    flock -n "$RUN_LOCK_FD" || {
        echo "Another campaign runner holds $CAMPAIGN_DIR/.campaign.lock" >&2
        exit 1
    }
}

resource_gate() {
    local poll
    poll=$(json_value '.resources.poll_interval_s')
    while true; do
        local output code=0
        output=$(python3 "$SCRIPT_DIR/monte_carlo/python/resource_monitor.py" gate \
            --config "$CONFIG" --disk-path "$CAMPAIGN_DIR" 2>&1) || code=$?
        append_event resource_gate "${CURRENT_RUN_ID:-}" "$output"
        if ((code == 0)); then return 0; fi
        if ((code != 2)); then echo "$output" >&2; return "$code"; fi
        echo "Resource gate closed; current work may finish, new work waits: $output" >&2
        sleep "$poll"
    done
}

preflight() {
    local failed=0
    local command
    for command in bash jq flock setsid realpath python3 nvidia-smi git; do
        command -v "$command" >/dev/null || { echo "Missing command: $command" >&2; failed=1; }
    done
    local required_files=(
        "$WORKSPACE_SETUP"
        "$ISAACSIM_ROOT/python.sh"
        "$ISAACSIM_ROOT/setup_ros_env.sh"
        "$ISAAC_UW_ROOT/run_underwater_sim.py"
        "$(json_value '.paths.simulator_config')"
        "$(json_value '.paths.slam_config')"
        "$WORKSPACE_ROOT/install/monte_carlo_tools/lib/monte_carlo_tools/path_manifest_tool"
        "$WORKSPACE_ROOT/install/monte_carlo_tools/lib/monte_carlo_tools/scenario_driver_node"
        "$WORKSPACE_ROOT/install/monte_carlo_tools/lib/monte_carlo_tools/trajectory_capture_node"
    )
    local path
    for path in "${required_files[@]}"; do
        [[ -e "$path" ]] || { echo "Missing required path: $path" >&2; failed=1; }
    done
    local source_name
    for source_name in path_manifest_tool scenario_driver_node trajectory_capture_node; do
        if [[ "$SCRIPT_DIR/monte_carlo_tools/src/$source_name.cpp" \
            -nt "$WORKSPACE_ROOT/install/monte_carlo_tools/lib/monte_carlo_tools/$source_name" ]]; then
            echo "Installed $source_name is older than its source; rebuild from $WORKSPACE_ROOT" >&2
            failed=1
        fi
    done
    python3 - <<'PY' || failed=1
import numpy, scipy, yaml, psutil
print("Python numerical/config/resource dependencies: OK")
PY
    ros_foreground python3 - <<'PY' || failed=1
import rosbag2_py, rclpy
print("ROS Python and rosbag2_py: OK")
PY
    ros_foreground ros2 pkg executables kcl | grep -q 'kinematic_control_layer_node' || failed=1
    ros_foreground ros2 pkg executables dcl | grep -q 'dynamic_control_layer_node' || failed=1
    ros_foreground ros2 pkg executables fast_lio | grep -q 'fastlio_mapping' || failed=1
    ros_foreground ros2 pkg executables viz | grep -q 'visualizer_node' || failed=1
    ros_foreground ros2 pkg executables rviz2 | grep -q 'rviz2' || failed=1
    ros_foreground ros2 bag record --help | grep -q 'zstd_fast' || failed=1
    ros_foreground ros2 bag play --help | grep -q -- '--start-paused' || failed=1
    ros_foreground ros2 interface show rosbag2_interfaces/srv/Resume >/dev/null || failed=1
    python3 "$SCRIPT_DIR/monte_carlo/python/resource_monitor.py" gate \
        --config "$CONFIG" --disk-path "$(dirname "$CAMPAIGN_DIR")" || failed=1
    echo "Workspace: $WORKSPACE_ROOT"
    echo "Campaign:  $CAMPAIGN_DIR"
    echo "Config SHA256: $(sha256sum "$CONFIG" | cut -d' ' -f1)"
    ((failed == 0)) || { echo "Preflight failed" >&2; return 1; }
    echo "Preflight passed"
}

plan_phase() {
    local phase=${1:?phase is required}
    case "$phase" in
        calibration)
            python3 "$SCRIPT_DIR/monte_carlo/python/plan_campaign.py" --config "$CONFIG" \
                --campaign-dir "$CAMPAIGN_DIR" --phase calibration --count-per-family 1
            ;;
        pilot)
            local count
            count=$(json_value '.statistics.pilot_per_family')
            python3 "$SCRIPT_DIR/monte_carlo/python/plan_campaign.py" --config "$CONFIG" \
                --campaign-dir "$CAMPAIGN_DIR" --phase pilot --count-per-family "$count"
            ;;
        main)
            [[ -f "$CAMPAIGN_DIR/frozen_design.json" ]] || {
                echo "Freeze the design before planning main scenarios" >&2
                return 1
            }
            local total
            total=$(jq -er '.main_n' "$CAMPAIGN_DIR/frozen_design.json")
            python3 "$SCRIPT_DIR/monte_carlo/python/plan_campaign.py" --config "$CONFIG" \
                --campaign-dir "$CAMPAIGN_DIR" --phase main --total "$total"
            ;;
        *) echo "Unknown phase: $phase" >&2; return 2 ;;
    esac
}

dry_run_phase() {
    local phase=${1:?phase is required}
    local index="$CAMPAIGN_DIR/${phase}_index.json"
    [[ -f "$index" ]] || { echo "Plan phase first: $phase" >&2; return 1; }
    python3 "$SCRIPT_DIR/monte_carlo/python/provenance.py" --config "$CONFIG" \
        --source-root "$SCRIPT_DIR" --output "$CAMPAIGN_DIR/provenance.json"
    local manifest
    while IFS= read -r manifest; do
        local run_dir
        run_dir=$(dirname "$manifest")
        python3 "$SCRIPT_DIR/monte_carlo/python/generate_configs.py" \
            --config "$CONFIG" --manifest "$manifest" --run-dir "$run_dir"
    done < <(jq -r '.manifests[]' "$index")
    echo "Dry run validated $(jq '.manifests | length' "$index") scenarios; no ROS/Isaac process launched."
}

domain_for() {
    local key=$1
    local offset=${2:-0}
    local base slots hash
    if [[ -n "$DOMAIN_BASE_OVERRIDE" ]]; then
        [[ "$DOMAIN_BASE_OVERRIDE" =~ ^[0-9]+$ ]] || {
            echo "Invalid MONTE_CARLO_DOMAIN_BASE_OVERRIDE: $DOMAIN_BASE_OVERRIDE" >&2
            return 2
        }
        base=$DOMAIN_BASE_OVERRIDE
    else
        base=$(json_value '.ros.domain_base')
    fi
    slots=$(json_value '.ros.domain_slots')
    hash=$(printf '%s' "$key" | cksum | awk '{print $1}')
    echo $((base + (hash + offset) % slots))
}

mission_timeout() {
    local manifest=$1
    python3 - "$CONFIG" "$manifest" "$CAMPAIGN_DIR/calibration.json" <<'PY'
import json, math, pathlib, sys
config=json.load(open(sys.argv[1])); manifest=json.load(open(sys.argv[2]))
cal_path=pathlib.Path(sys.argv[3])
if cal_path.is_file():
    cal=json.load(open(cal_path)); speed=max(0.05, float(cal["effective_path_speed_mps"]))
    expected=float(manifest["geometry"]["nominal_mission_length_m"])/speed
else:
    g=manifest["geometry"]
    expected=(float(g["straight_length_m"])+float(g["nominal_return_distance_m"]))/2.0
    expected+=float(g["curved_length_m"])/0.5+float(g["vertical_length_m"])/0.2
scale=float(config["simulation"]["mission_timeout_scale"])
floor=float(config["simulation"]["mission_timeout_floor_wall_s"])
print(f"{math.ceil(max(floor, expected*scale)):.1f}")
PY
}

remove_transient_bag() {
    local run_dir=$1
    local bag=$2
    [[ "$bag" == "$run_dir/transient_bag" ]] || {
        echo "Refusing unexpected bag deletion target: $bag" >&2
        return 1
    }
    [[ ! -L "$bag" ]] || { echo "Refusing symlink bag target: $bag" >&2; return 1; }
    if [[ -d "$bag" ]]; then
        rm -rf -- "$bag"
        append_event bag_deleted "${CURRENT_RUN_ID:-}" "$bag"
    fi
}

reset_incomplete_triplet() {
    local run_dir=$1
    [[ "$run_dir" == "$CAMPAIGN_DIR/scenarios/"* ]] || {
        echo "Refusing result reset outside campaign: $run_dir" >&2
        return 1
    }
    rm -rf -- "$run_dir/estimators" "$run_dir/metrics"
    rm -f -- "$run_dir/result.json" "$run_dir/.complete.json"
    append_event triplet_reset "$CURRENT_RUN_ID" "shared bag unavailable; removed partial estimator results"
}

validate_bag() {
    local run_dir=$1
    local bag=$2
    ros_foreground python3 "$SCRIPT_DIR/monte_carlo/python/bag_extract.py" \
        --config "$CONFIG" --bag "$bag" \
        --ground-truth "$run_dir/live/ground_truth.csv" \
        --output "$run_dir/live/bag_manifest.json"
}

driver_result_evaluable() {
    local result=$1
    jq -e '
        (.status == "completed" or .status == "mission_timeout" or
         .status == "boundary_violation" or .status == "attitude_divergence") and
        (.mission_end_sim_time_s > .mission_start_sim_time_s)
    ' "$result" >/dev/null 2>&1
}

run_live_once() {
    local manifest=$1
    local run_dir=$2
    local bag="$run_dir/transient_bag"
    local driver_result="$run_dir/live/driver_result.json"
    local bag_manifest="$run_dir/live/bag_manifest.json"

    if [[ -f "$driver_result" && -d "$bag" ]]; then
        if driver_result_evaluable "$driver_result"; then
            if validate_bag "$run_dir" "$bag"; then
                append_event bag_recovered "$CURRENT_RUN_ID" "validated flushed bag after interruption"
                return 0
            fi
        fi
    fi
    remove_transient_bag "$run_dir" "$bag"
    rm -f -- "$driver_result" "$bag_manifest" "$run_dir/live/ground_truth.csv"
    resource_gate
    update_status simulating "$CURRENT_RUN_ID" 1

    local domain logs startup_timeout warmup timeout grace
    domain=$(domain_for "$CURRENT_RUN_ID" 0)
    logs="$run_dir/logs"
    startup_timeout=$(json_value '.simulation.startup_timeout_wall_s')
    warmup=$(json_value '.simulation.stationary_warmup_sim_s')
    timeout=$(mission_timeout "$manifest")
    grace=$(json_value '.simulation.shutdown_grace_s')
    local storage preset cache max_bag_size recorder_grace
    storage=$(json_value '.recording.storage')
    preset=$(json_value '.recording.storage_preset_profile')
    cache=$(json_value '.recording.max_cache_size_bytes')
    max_bag_size=$(json_value '.recording.max_bag_size_bytes')
    recorder_grace=$(json_value '.recording.shutdown_grace_s')
    local -a topics=()
    mapfile -t topics < <(jq -r '.recording.topics[]' "$CONFIG")

    launch_ros_group recorder "$logs/recorder.log" "$domain" ros2 bag record \
        --output "$bag" --storage "$storage" --storage-preset-profile "$preset" \
        --max-cache-size "$cache" --max-bag-size "$max_bag_size" \
        --disable-keyboard-controls --use-sim-time \
        --qos-profile-overrides-path "$SCRIPT_DIR/monte_carlo/config/record_qos.yaml" \
        --custom-data "run_id=$CURRENT_RUN_ID" --topics "${topics[@]}"
    local recorder_pid=$LAUNCHED_PID
    sleep 1
    launch_ros_group kcl "$logs/kcl.log" "$domain" ros2 run kcl kinematic_control_layer_node \
        --ros-args -p config_name:=BlueROV
    local kcl_pid=$LAUNCHED_PID
    launch_ros_group dcl "$logs/dcl.log" "$domain" ros2 run dcl dynamic_control_layer_node \
        --ros-args -p config_name:=BlueROV
    local dcl_pid=$LAUNCHED_PID
    local visualizer_pid=0
    if [[ $(json_value '.simulation.visualizer') == true ]]; then
        launch_ros_group visualizer "$logs/visualizer.log" "$domain" \
            ros2 run viz visualizer_node --ros-args -p config_name:=BlueROV
        visualizer_pid=$LAUNCHED_PID
    fi
    launch_ros_group driver "$logs/driver.log" "$domain" ros2 run monte_carlo_tools scenario_driver_node \
        --ros-args -p "manifest_path:=$manifest" -p "result_path:=$driver_result" \
        -p "startup_timeout_s:=$startup_timeout" -p "mission_timeout_s:=$timeout" \
        -p "warmup_sim_s:=$warmup" \
        -p "boundary_tolerance_m:=$(json_value '.simulation.actual_boundary_tolerance_m')" \
        -p "max_abs_roll_pitch_deg:=$(json_value '.simulation.max_abs_roll_pitch_deg')"
    local driver_pid=$LAUNCHED_PID
    launch_isaac_group "$logs/isaac.log" "$domain" "$run_dir/configs/simulator.json"
    local isaac_pid=$LAUNCHED_PID

    local driver_limit
    driver_limit=$(python3 -c "print(int(float('$startup_timeout') + float('$timeout') + 60))")
    local driver_code=0
    wait_group "$driver_pid" "$driver_limit" || driver_code=$?
    stop_group "$isaac_pid" "$grace"
    stop_group "$visualizer_pid" 10
    stop_group "$kcl_pid" 10
    stop_group "$dcl_pid" 10
    local recorder_code=0
    stop_recorder_group "$recorder_pid" "$recorder_grace" || recorder_code=$?

    if ((driver_code != 0 || recorder_code != 0)) || [[ ! -f "$driver_result" ]]; then
        append_event simulation_failed "$CURRENT_RUN_ID" \
            "driver_exit=$driver_code recorder_exit=$recorder_code"
        remove_transient_bag "$run_dir" "$bag"
        return 1
    fi
    local driver_status
    driver_status=$(jq -r '.status' "$driver_result")
    if ! driver_result_evaluable "$driver_result"; then
        append_event simulation_failed "$CURRENT_RUN_ID" "$driver_status"
        remove_transient_bag "$run_dir" "$bag"
        return 1
    fi
    if ! validate_bag "$run_dir" "$bag"; then
        append_event simulation_failed "$CURRENT_RUN_ID" "bag_validation_failed"
        remove_transient_bag "$run_dir" "$bag"
        return 1
    fi
    append_event simulation_completed "$CURRENT_RUN_ID" "domain=$domain controller_status=$driver_status"
}

wait_file_stable() {
    local path=$1
    local maximum_value=$2
    local stable_required=$3
    local maximum
    maximum=$(python3 -c "import math; print(math.ceil(float('$maximum_value')))")
    local deadline=$((SECONDS + maximum))
    local previous=-1 stable=0 size
    while ((SECONDS < deadline)); do
        size=$(stat -c %s "$path" 2>/dev/null || echo 0)
        if [[ "$size" == "$previous" && "$size" -gt 0 ]]; then
            stable=$((stable + 1))
            ((stable >= stable_required)) && return 0
        else
            stable=0
        fi
        previous=$size
        sleep 1
    done
    return 0
}

wait_replay_nodes() {
    local domain=$1
    local slam_pid=$2
    local capture_pid=$3
    local timeout
    timeout=$(json_value '.replay.node_startup_timeout_s')
    local deadline
    deadline=$(python3 -c "import time; print(time.monotonic()+float('$timeout'))")
    while true; do
        kill -0 "$slam_pid" 2>/dev/null || return 1
        kill -0 "$capture_pid" 2>/dev/null || return 1
        local nodes
        nodes=$(ros_domain_foreground "$domain" ros2 node list 2>/dev/null || true)
        if grep -Fxq '/laser_mapping' <<<"$nodes" \
            && grep -Fxq '/monte_carlo_trajectory_capture' <<<"$nodes"; then
            return 0
        fi
        if python3 -c "import time; raise SystemExit(0 if time.monotonic() >= float('$deadline') else 1)"; then
            return 1
        fi
        sleep 0.5
    done
}

topic_has_subscription() {
    local domain=$1
    local topic=$2
    local count
    count=$(ros_domain_foreground "$domain" ros2 topic info "$topic" 2>/dev/null \
        | awk '/^Subscription count:/ {print $3}')
    [[ ${count:-0} =~ ^[0-9]+$ ]] && ((count > 0))
}

wait_playback_ready() {
    local domain=$1
    local player_pid=$2
    local algorithm=$3
    local timeout
    timeout=$(json_value '.replay.node_startup_timeout_s')
    local deadline
    deadline=$(python3 -c "import time; print(time.monotonic()+float('$timeout'))")
    local -a topics=(/auv/imu/data_raw /Odometry)
    if [[ "$algorithm" != ins ]]; then
        topics+=(/sonar_point_cloud)
    else
        topics+=(/auv/dvl /auv/pressure/scaled2 /auv/imu/magnetic_field)
    fi
    if [[ "$algorithm" == uwfl2 ]]; then
        topics+=(/auv/dvl /auv/pressure/scaled2 /auv/imu/magnetic_field)
    fi
    while true; do
        kill -0 "$player_pid" 2>/dev/null || return 1
        local services ready=1 topic
        services=$(ros_domain_foreground "$domain" ros2 service list 2>/dev/null || true)
        grep -Fxq '/rosbag2_player/resume' <<<"$services" || ready=0
        for topic in "${topics[@]}"; do
            topic_has_subscription "$domain" "$topic" || ready=0
        done
        if ((ready)); then
            sleep "$(json_value '.replay.publisher_discovery_delay_s')"
            ros_domain_foreground "$domain" timeout 10s ros2 service call \
                /rosbag2_player/resume rosbag2_interfaces/srv/Resume '{}' >/dev/null
            return $?
        fi
        if python3 -c "import time; raise SystemExit(0 if time.monotonic() >= float('$deadline') else 1)"; then
            return 1
        fi
        sleep 0.5
    done
}

write_player_timing() {
    local output=$1
    local wall=$2
    local code=$3
    python3 - "$output" "$wall" "$code" <<'PY'
from mc_common import atomic_json
import sys
atomic_json(sys.argv[1], {"player_wall_s": float(sys.argv[2]), "player_exit_code": int(sys.argv[3])})
PY
}

run_estimator() {
    local algorithm=$1
    local manifest=$2
    local run_dir=$3
    local metric="$run_dir/metrics/$algorithm.json"
    [[ ! -f "$metric" ]] || { append_event estimator_skipped "$CURRENT_RUN_ID" "$algorithm complete"; return 0; }
    if [[ "$INTERNAL_REPLAY_WORKER" != 1 ]]; then
        resource_gate
        update_status "replay-$algorithm" "$CURRENT_RUN_ID" 1
    fi
    local estimator_dir="$run_dir/estimators/$algorithm"
    mkdir -p "$estimator_dir" "$run_dir/metrics"
    local domain
    case "$algorithm" in uwfl2) domain=$(domain_for "$CURRENT_RUN_ID" 3);; ins) domain=$(domain_for "$CURRENT_RUN_ID" 7);; fl2) domain=$(domain_for "$CURRENT_RUN_ID" 11);; esac
    local log_dir="$run_dir/logs"
    launch_ros_group "slam-$algorithm" "$log_dir/slam-$algorithm.log" "$domain" ros2 launch fast_lio mapping.launch.py \
        "config_file:=$run_dir/configs/$algorithm.yaml" use_sim_time:=true \
        "rviz:=$(json_value '.simulation.visualizer')"
    local slam_pid=$LAUNCHED_PID
    launch_ros_group "capture-$algorithm" "$log_dir/capture-$algorithm.log" "$domain" \
        ros2 run monte_carlo_tools trajectory_capture_node --ros-args \
        -p "algorithm:=$algorithm" -p topic:=/Odometry \
        -p "output_path:=$estimator_dir/trajectory.csv" \
        -p "summary_path:=$estimator_dir/capture_summary.json"
    local capture_pid=$LAUNCHED_PID
    sleep "$(json_value '.replay.startup_delay_s')"

    local startup_failed=0
    wait_replay_nodes "$domain" "$slam_pid" "$capture_pid" || startup_failed=1

    local rate bag_duration timeout scale minimum
    rate=$(json_value '.replay.target_rate')
    bag_duration=$(jq -r '.duration_s' "$run_dir/live/bag_manifest.json")
    scale=$(json_value '.replay.timeout_scale')
    minimum=$(json_value '.replay.min_timeout_wall_s')
    timeout=$(python3 -c "import math; print(math.ceil(max(float('$minimum'), float('$bag_duration')/float('$rate')*float('$scale'))))")
    local player_start player_end player_code=0
    player_start=$(date +%s.%N)
    if ((startup_failed)); then
        player_code=125
    else
        launch_ros_group "player-$algorithm" "$log_dir/player-$algorithm.log" "$domain" ros2 bag play \
            --storage "$(json_value '.recording.storage')" --rate "$rate" --start-paused \
            --wait-for-all-acked "$(json_value '.replay.wait_for_all_acked_ms')" \
            --disable-keyboard-controls --qos-profile-overrides-path \
            "$SCRIPT_DIR/monte_carlo/config/playback_qos.yaml" "$run_dir/transient_bag"
        local player_pid=$LAUNCHED_PID
        if wait_playback_ready "$domain" "$player_pid" "$algorithm"; then
            wait_group "$player_pid" "$timeout" || player_code=$?
        else
            player_code=125
            stop_group "$player_pid" 10
        fi
    fi
    player_end=$(date +%s.%N)
    local player_wall
    player_wall=$(python3 -c "print(float('$player_end')-float('$player_start'))")
    write_player_timing "$estimator_dir/timing.json" "$player_wall" "$player_code" || return 1
    wait_file_stable "$estimator_dir/trajectory.csv.tmp" \
        "$(json_value '.replay.drain_timeout_s')" "$(json_value '.replay.drain_stable_s')"

    local forced_reason=""
    if ((player_code == 125)); then forced_reason=slam_startup_failed
    elif ((player_code == 124)); then forced_reason=replay_timeout
    elif ((player_code != 0)); then forced_reason=replay_failed
    elif ! kill -0 "$slam_pid" 2>/dev/null; then forced_reason=slam_process_exit
    fi
    stop_group "$slam_pid" 20
    sleep 1
    stop_group "$capture_pid" 20
    if ! python3 "$SCRIPT_DIR/monte_carlo/python/validate_replay.py" \
        --algorithm "$algorithm" --bag-manifest "$run_dir/live/bag_manifest.json" \
        --diagnostics "$estimator_dir/diagnostics/front_end_summary.json" \
        --output "$estimator_dir/replay_validation.json"; then
        if [[ -z "$forced_reason" ]]; then forced_reason=input_delivery_failed; fi
    fi
    if [[ ! -f "$estimator_dir/capture_summary.json" || -n "$forced_reason" ]]; then
        python3 "$SCRIPT_DIR/monte_carlo/python/stage_artifacts.py" capture-failure \
            --algorithm "$algorithm" --estimator-dir "$estimator_dir" \
            --reason "${forced_reason:-capture_process_exit}" || return 1
    fi
    python3 "$SCRIPT_DIR/monte_carlo/python/compute_metrics.py" --config "$CONFIG" \
        --manifest "$manifest" --driver-result "$run_dir/live/driver_result.json" \
        --ground-truth "$run_dir/live/ground_truth.csv" \
        --estimate "$estimator_dir/trajectory.csv" \
        --capture-summary "$estimator_dir/capture_summary.json" \
        --algorithm "$algorithm" --output "$metric" || return 1
    append_event estimator_completed "$CURRENT_RUN_ID" \
        "$algorithm status=$(jq -r '.status' "$metric") rmse=$(jq -r '.analysis_rmse_m' "$metric")"
}

run_estimators_parallel() {
    local manifest=$1
    local run_dir=$2
    local maximum_workers
    maximum_workers=$(json_value '.resources.max_replay_workers')
    ((maximum_workers > 0)) || { echo "max_replay_workers must be positive" >&2; return 2; }
    ((maximum_workers > 3)) && maximum_workers=3

    local -a pending=()
    local algorithm
    for algorithm in uwfl2 ins fl2; do
        if [[ -f "$run_dir/metrics/$algorithm.json" ]]; then
            append_event estimator_skipped "$CURRENT_RUN_ID" "$algorithm complete"
        else
            pending+=("$algorithm")
        fi
    done
    ((${#pending[@]} > 0)) || return 0

    local bag_duration rate scale minimum worker_timeout
    bag_duration=$(jq -r '.duration_s' "$run_dir/live/bag_manifest.json")
    rate=$(json_value '.replay.target_rate')
    scale=$(json_value '.replay.timeout_scale')
    minimum=$(json_value '.replay.min_timeout_wall_s')
    worker_timeout=$(python3 -c "import math; print(math.ceil(max(float('$minimum'), float('$bag_duration')/float('$rate')*float('$scale')) + 180.0))")

    local offset=0
    while ((offset < ${#pending[@]})); do
        local -a batch=("${pending[@]:offset:maximum_workers}")
        resource_gate
        update_status "replay-parallel:${batch[*]}" "$CURRENT_RUN_ID" "${#batch[@]}"
        local -a worker_pids=()
        for algorithm in "${batch[@]}"; do
            launch_group "replay-worker-$algorithm" "$run_dir/logs/worker-$algorithm.log" \
                env MONTE_CARLO_INTERNAL_REPLAY_WORKER=1 "$SCRIPT_DIR/run_monte_carlo.sh" \
                --config "$CONFIG" --campaign-dir "$CAMPAIGN_DIR" \
                _replay-worker "$manifest" "$algorithm"
            worker_pids+=("$LAUNCHED_PID")
        done
        local failed=0 pid
        for pid in "${worker_pids[@]}"; do
            if ! wait_group "$pid" "$worker_timeout"; then failed=1; fi
        done
        ((failed == 0)) || return 1
        offset=$((offset + ${#batch[@]}))
    done
}

finalize_scenario() {
    local run_dir=$1
    python3 "$SCRIPT_DIR/monte_carlo/python/stage_artifacts.py" aggregate --run-dir "$run_dir" \
        || return 1
    local bag_hash
    bag_hash=$(jq -r '.sha256' "$run_dir/live/bag_manifest.json")
    remove_transient_bag "$run_dir" "$run_dir/transient_bag" || return 1
    python3 - "$run_dir/live/bag_deleted.json" "$bag_hash" <<'PY'
from datetime import datetime, timezone
from mc_common import atomic_json
import sys
atomic_json(sys.argv[1], {"deleted": True, "bag_sha256": sys.argv[2],
    "timestamp": datetime.now(timezone.utc).isoformat(),
    "reason": "all three metric artifacts were atomically committed"})
PY
    append_event scenario_completed "$CURRENT_RUN_ID" "all estimator metrics committed"
}

run_one() {
    local manifest
    manifest=$(realpath "${1:?manifest is required}")
    [[ -f "$manifest" ]] || { echo "Manifest not found: $manifest" >&2; return 2; }
    local run_dir
    run_dir=$(dirname "$manifest")
    CURRENT_RUN_ID=$(jq -er '.run_id' "$manifest")
    [[ "$run_dir" == "$CAMPAIGN_DIR/scenarios/"* ]] || {
        echo "Manifest is outside campaign directory: $manifest" >&2
        return 2
    }
    if [[ -f "$run_dir/.complete.json" ]]; then
        remove_transient_bag "$run_dir" "$run_dir/transient_bag"
        append_event scenario_skipped "$CURRENT_RUN_ID" "already complete"
        return 0
    fi
    mkdir -p "$run_dir/live" "$run_dir/logs" "$run_dir/metrics" "$run_dir/estimators"
    python3 "$SCRIPT_DIR/monte_carlo/python/provenance.py" --config "$CONFIG" \
        --source-root "$SCRIPT_DIR" --output "$CAMPAIGN_DIR/provenance.json" || return 1
    python3 "$SCRIPT_DIR/monte_carlo/python/generate_configs.py" \
        --config "$CONFIG" --manifest "$manifest" --run-dir "$run_dir" || return 1

    if [[ ! -d "$run_dir/transient_bag" ]]; then
        local existing_metrics=("$run_dir"/metrics/*.json)
        if ((${#existing_metrics[@]} > 0)); then reset_incomplete_triplet "$run_dir"; fi
    fi
    if [[ ! -f "$run_dir/live/bag_manifest.json" || ! -d "$run_dir/transient_bag" ]] \
        || [[ $(jq -r '.valid // false' "$run_dir/live/bag_manifest.json" 2>/dev/null || echo false) != true ]] \
        || ! driver_result_evaluable "$run_dir/live/driver_result.json"; then
        run_live_once "$manifest" "$run_dir" || return 1
    fi
    local sequential
    sequential=$(jq -r '.replay.sequential_algorithms' "$CONFIG")
    if [[ "$sequential" == true ]] || (( $(json_value '.resources.max_replay_workers') <= 1 )); then
        local algorithm
        for algorithm in uwfl2 ins fl2; do
            run_estimator "$algorithm" "$manifest" "$run_dir" || return 1
        done
    else
        run_estimators_parallel "$manifest" "$run_dir" || return 1
    fi
    finalize_scenario "$run_dir" || return 1
}

estimate_phase() {
    python3 "$SCRIPT_DIR/monte_carlo/python/estimate_runtime.py" --config "$CONFIG" \
        --campaign-dir "$CAMPAIGN_DIR" --phase "${1:?phase is required}"
}

confirm_phase() {
    local phase=$1
    estimate_phase "$phase"
    if ((ASSUME_YES)); then return 0; fi
    if [[ ! -t 0 ]]; then
        echo "Non-interactive campaign start requires --yes" >&2
        return 1
    fi
    local response
    read -r -p 'Proceed? [y/N] ' response
    [[ "$response" =~ ^[Yy]$ ]]
}

run_phase() {
    local phase=${1:?phase is required}
    local index="$CAMPAIGN_DIR/${phase}_index.json"
    [[ -f "$index" ]] || { echo "Plan phase first: $phase" >&2; return 1; }
    PHASE_TOTAL=$(jq '.manifests | length' "$index")
    PHASE_COMPLETED=0
    PHASE_STARTED_EPOCH=$(date +%s)
    local manifest
    while IFS= read -r manifest; do
        [[ -f "$(dirname "$manifest")/.complete.json" ]] && PHASE_COMPLETED=$((PHASE_COMPLETED + 1))
    done < <(jq -r '.manifests[]' "$index")
    update_status starting "$phase" 0
    launch_group monitor "$CAMPAIGN_DIR/resource-monitor.log" python3 \
        "$SCRIPT_DIR/monte_carlo/python/resource_monitor.py" watch \
        --status "$STATUS_PATH" --log "$RESOURCE_LOG" --disk-path "$CAMPAIGN_DIR" \
        --campaign-dir "$CAMPAIGN_DIR" \
        --interval "$(json_value '.resources.poll_interval_s')"
    local monitor_pid=$LAUNCHED_PID
    local failures=0
    while IFS= read -r manifest; do
        local run_id
        run_id=$(jq -r '.run_id' "$manifest")
        if [[ -f "$(dirname "$manifest")/.complete.json" ]]; then continue; fi
        update_status queued "$run_id" 0
        if run_one "$manifest"; then
            PHASE_COMPLETED=$((PHASE_COMPLETED + 1))
        else
            failures=$((failures + 1))
            append_event scenario_incomplete "$run_id" "resume will retry infrastructure stage"
        fi
        update_status running "$phase" 0
    done < <(jq -r '.manifests[]' "$index")
    stop_group "$monitor_pid" 5
    if ((failures > 0)); then
        update_status incomplete "$phase" 0
        echo "$failures scenario(s) remain incomplete; rerun resume $phase" >&2
        return 1
    fi
    update_status completed "$phase" 0
}

write_sidecar_status() {
    local state=$1
    local run_id=${2:-}
    local detail=${3:-}
    python3 - "$CAMPAIGN_DIR/sidecar_status.json" "$state" "$run_id" "$detail" \
        "$DOMAIN_BASE_OVERRIDE" <<'PY'
from datetime import datetime, timezone
from mc_common import atomic_json
import os, sys
atomic_json(sys.argv[1], {
    "state": sys.argv[2],
    "run_id": sys.argv[3],
    "detail": sys.argv[4],
    "domain_base": int(sys.argv[5]),
    "pid": os.getppid(),
    "timestamp": datetime.now(timezone.utc).isoformat(),
})
PY
}

run_sidecar_phase() {
    local phase=${1:?phase is required}
    local index="$CAMPAIGN_DIR/${phase}_index.json"
    [[ -f "$index" ]] || { echo "Plan phase first: $phase" >&2; return 1; }
    [[ "$SIDECAR_WORKER" == 1 ]] || {
        echo "Sidecar mode requires MONTE_CARLO_SIDECAR_WORKER=1" >&2
        return 2
    }
    [[ -n "$DOMAIN_BASE_OVERRIDE" ]] || {
        echo "Sidecar mode requires an isolated MONTE_CARLO_DOMAIN_BASE_OVERRIDE" >&2
        return 2
    }
    (( $(json_value '.resources.max_sim_workers') >= 2 )) || {
        echo "Second simulator worker is disabled by configuration" >&2
        return 0
    }

    local sidecar_lock_fd
    exec {sidecar_lock_fd}>"$CAMPAIGN_DIR/.sidecar.lock"
    flock -n "$sidecar_lock_fd" || {
        echo "Another sidecar worker is already running" >&2
        return 1
    }
    append_event sidecar_started "" "domain_base=$DOMAIN_BASE_OVERRIDE"
    write_sidecar_status starting "" "waiting for a safely separated pending scenario"

    local -a manifests=()
    mapfile -t manifests < <(jq -r '.manifests[]' "$index")
    while (( $(json_value '.resources.max_sim_workers') >= 2 )); do
        local current active_index=-1 candidate="" candidate_index=-1 i manifest run_dir
        current=$(jq -r '.current_experiment // empty' "$STATUS_PATH" 2>/dev/null || true)
        for i in "${!manifests[@]}"; do
            if [[ $(basename "$(dirname "${manifests[$i]}")") == "$current" ]]; then
                active_index=$i
                break
            fi
        done
        for ((i=${#manifests[@]}-1; i>active_index+2; i--)); do
            manifest=${manifests[$i]}
            run_dir=$(dirname "$manifest")
            [[ ! -f "$run_dir/.complete.json" ]] || continue
            if mkdir "$run_dir/.sidecar-claim" 2>/dev/null; then
                SIDECAR_CLAIM_DIR="$run_dir/.sidecar-claim"
                if [[ -f "$run_dir/.complete.json" ]]; then
                    rmdir -- "$SIDECAR_CLAIM_DIR" 2>/dev/null || true
                    SIDECAR_CLAIM_DIR=""
                    continue
                fi
                candidate=$manifest
                candidate_index=$i
                break
            fi
        done
        if [[ -z "$candidate" ]]; then
            write_sidecar_status stopped "" "no safely separated pending scenario"
            append_event sidecar_stopped "" "no safely separated pending scenario"
            return 0
        fi

        CURRENT_RUN_ID=$(jq -r '.run_id' "$candidate")
        write_sidecar_status running "$CURRENT_RUN_ID" \
            "index=$((candidate_index + 1))/${#manifests[@]}"
        append_event sidecar_scenario_started "$CURRENT_RUN_ID" \
            "domain_base=$DOMAIN_BASE_OVERRIDE index=$((candidate_index + 1))"
        if run_one "$candidate"; then
            append_event sidecar_scenario_completed "$CURRENT_RUN_ID" "complete"
        else
            append_event sidecar_scenario_incomplete "$CURRENT_RUN_ID" \
                "primary runner may retry this scenario"
            rmdir -- "$SIDECAR_CLAIM_DIR" 2>/dev/null || true
            SIDECAR_CLAIM_DIR=""
            write_sidecar_status failed "$CURRENT_RUN_ID" "scenario incomplete"
            return 1
        fi
        rmdir -- "$SIDECAR_CLAIM_DIR" 2>/dev/null || true
        SIDECAR_CLAIM_DIR=""
    done
    write_sidecar_status stopped "" "max_sim_workers reduced below two"
    append_event sidecar_stopped "" "max_sim_workers reduced below two"
}

show_status() {
    local -a status_args=(
        dashboard
        --config "$CONFIG"
        --campaign-dir "$CAMPAIGN_DIR"
        --status "$STATUS_PATH"
        --resource-log "$RESOURCE_LOG"
        --disk-path "$(dirname "$CAMPAIGN_DIR")"
        --interval "$(json_value '.resources.poll_interval_s')"
    )
    if [[ -t 1 ]]; then status_args+=(--watch); fi
    python3 "$SCRIPT_DIR/monte_carlo/python/resource_monitor.py" "${status_args[@]}"
}

case "$COMMAND" in
    preflight)
        preflight
        ;;
    plan)
        acquire_campaign_lock
        preflight
        plan_phase "${COMMAND_ARGS[0]:?plan requires a phase}"
        ;;
    dry-run)
        acquire_campaign_lock
        preflight
        dry_run_phase "${COMMAND_ARGS[0]:?dry-run requires a phase}"
        ;;
    estimate)
        estimate_phase "${COMMAND_ARGS[0]:?estimate requires a phase}"
        ;;
    run-one)
        acquire_campaign_lock
        preflight
        PHASE_STARTED_EPOCH=$(date +%s)
        CURRENT_RUN_ID=$(jq -er '.run_id' "${COMMAND_ARGS[0]:?run-one requires a manifest}")
        update_status starting "$CURRENT_RUN_ID" 0
        launch_group monitor "$CAMPAIGN_DIR/resource-monitor.log" python3 \
            "$SCRIPT_DIR/monte_carlo/python/resource_monitor.py" watch \
            --status "$STATUS_PATH" --log "$RESOURCE_LOG" --disk-path "$CAMPAIGN_DIR" \
            --campaign-dir "$CAMPAIGN_DIR" \
            --interval "$(json_value '.resources.poll_interval_s')"
        monitor_pid=$LAUNCHED_PID
        if run_one "${COMMAND_ARGS[0]}"; then
            PHASE_COMPLETED=1
            update_status completed "$CURRENT_RUN_ID" 0
            stop_group "$monitor_pid" 5
        else
            update_status incomplete "$CURRENT_RUN_ID" 0
            stop_group "$monitor_pid" 5
            exit 1
        fi
        ;;
    run-phase)
        acquire_campaign_lock
        preflight
        confirm_phase "${COMMAND_ARGS[0]:?run-phase requires a phase}"
        run_phase "${COMMAND_ARGS[0]}"
        ;;
    resume)
        acquire_campaign_lock
        preflight
        run_phase "${COMMAND_ARGS[0]:?resume requires a phase}"
        ;;
    calibrate)
        acquire_campaign_lock
        preflight
        plan_phase calibration
        dry_run_phase calibration
        confirm_phase calibration
        run_phase calibration
        python3 "$SCRIPT_DIR/monte_carlo/python/stage_artifacts.py" calibration \
            --campaign-dir "$CAMPAIGN_DIR"
        ;;
    pilot)
        acquire_campaign_lock
        preflight
        plan_phase pilot
        dry_run_phase pilot
        confirm_phase pilot
        run_phase pilot
        ;;
    freeze)
        acquire_campaign_lock
        python3 "$SCRIPT_DIR/monte_carlo/python/analyze_campaign.py" freeze \
            --config "$CONFIG" --campaign-dir "$CAMPAIGN_DIR"
        ;;
    run)
        acquire_campaign_lock
        preflight
        plan_phase main
        dry_run_phase main
        confirm_phase main
        run_phase main
        ;;
    evaluate)
        acquire_campaign_lock
        preflight
        plan_phase calibration
        plan_phase pilot
        dry_run_phase calibration
        dry_run_phase pilot
        estimate_phase campaign
        confirm_phase calibration
        run_phase calibration
        python3 "$SCRIPT_DIR/monte_carlo/python/stage_artifacts.py" calibration \
            --campaign-dir "$CAMPAIGN_DIR"
        confirm_phase pilot
        run_phase pilot
        python3 "$SCRIPT_DIR/monte_carlo/python/analyze_campaign.py" freeze \
            --config "$CONFIG" --campaign-dir "$CAMPAIGN_DIR"
        plan_phase main
        dry_run_phase main
        confirm_phase main
        run_phase main
        python3 "$SCRIPT_DIR/monte_carlo/python/analyze_campaign.py" report \
            --config "$CONFIG" --campaign-dir "$CAMPAIGN_DIR"
        ;;
    report)
        python3 "$SCRIPT_DIR/monte_carlo/python/analyze_campaign.py" report \
            --config "$CONFIG" --campaign-dir "$CAMPAIGN_DIR"
        ;;
    status)
        show_status
        ;;
    _sidecar-phase)
        preflight
        run_sidecar_phase "${COMMAND_ARGS[0]:?_sidecar-phase requires a phase}"
        ;;
    _replay-worker)
        [[ "$INTERNAL_REPLAY_WORKER" == 1 ]] || {
            echo "Internal replay worker cannot be invoked directly" >&2
            exit 2
        }
        worker_manifest=$(realpath "${COMMAND_ARGS[0]:?_replay-worker requires a manifest}")
        worker_algorithm=${COMMAND_ARGS[1]:?_replay-worker requires an algorithm}
        [[ "$worker_algorithm" =~ ^(uwfl2|ins|fl2)$ ]] || {
            echo "Unknown replay algorithm: $worker_algorithm" >&2
            exit 2
        }
        CURRENT_RUN_ID=$(jq -er '.run_id' "$worker_manifest")
        run_estimator "$worker_algorithm" "$worker_manifest" "$(dirname "$worker_manifest")"
        ;;
    *)
        echo "Unknown command: $COMMAND" >&2
        usage >&2
        exit 2
        ;;
esac
