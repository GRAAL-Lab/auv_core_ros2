#!/usr/bin/env python3
"""Validate a rosbag, extract exact simulator pose ground truth, and hash it."""

from __future__ import annotations

import argparse
import csv
import hashlib
import os
import struct
import tempfile
from collections import Counter
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from mc_common import atomic_json, load_json


MCAP_MAGIC = b"\x89MCAP0\r\n"


def validate_mcap_file(path: Path) -> None:
    """Require a clean MCAP footer and a non-empty chunk-index summary."""
    footer_size = 1 + 8 + 20 + len(MCAP_MAGIC)
    if path.stat().st_size < footer_size:
        raise RuntimeError(f"MCAP file is too short to contain a footer: {path}")
    with path.open("rb") as stream:
        stream.seek(-footer_size, os.SEEK_END)
        footer = stream.read(footer_size)
        if footer[-len(MCAP_MAGIC) :] != MCAP_MAGIC:
            raise RuntimeError(f"MCAP file was not cleanly closed (missing trailing magic): {path}")
        opcode = footer[0]
        record_length = struct.unpack_from("<Q", footer, 1)[0]
        summary_start, summary_offset_start = struct.unpack_from("<QQ", footer, 9)
        if opcode != 0x02 or record_length != 20:
            raise RuntimeError(f"MCAP file has an invalid footer record: {path}")
        if not (0 < summary_start < summary_offset_start < path.stat().st_size - footer_size):
            raise RuntimeError(f"MCAP file has no complete summary index: {path}")

        stream.seek(summary_start)
        position = summary_start
        chunk_indexes = 0
        while position < summary_offset_start:
            header = stream.read(9)
            if len(header) != 9:
                raise RuntimeError(f"MCAP summary is truncated: {path}")
            length = struct.unpack_from("<Q", header, 1)[0]
            position += 9 + length
            if position > summary_offset_start:
                raise RuntimeError(f"MCAP summary record exceeds its index boundary: {path}")
            if header[0] == 0x08:
                chunk_indexes += 1
                chunk_index = stream.read(length)
                if len(chunk_index) != length or length < 48:
                    raise RuntimeError(f"MCAP chunk index is truncated: {path}")
                message_index_map_size = struct.unpack_from("<I", chunk_index, 32)[0]
                message_index_length_offset = 36 + message_index_map_size
                if (
                    message_index_map_size == 0
                    or message_index_map_size % 10 != 0
                    or message_index_length_offset + 8 > len(chunk_index)
                    or struct.unpack_from("<Q", chunk_index, message_index_length_offset)[0] == 0
                ):
                    raise RuntimeError(f"MCAP chunk has no message index: {path}")
            else:
                stream.seek(length, os.SEEK_CUR)
        if position != summary_offset_start or chunk_indexes == 0:
            raise RuntimeError(f"MCAP file has no message chunk index: {path}")


def hash_bag_files(bag_path: Path) -> tuple[str, list[dict[str, object]]]:
    aggregate = hashlib.sha256()
    files: list[dict[str, object]] = []
    for path in sorted(item for item in bag_path.rglob("*") if item.is_file()):
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            while chunk := stream.read(1024 * 1024):
                digest.update(chunk)
        relative = str(path.relative_to(bag_path))
        file_hash = digest.hexdigest()
        aggregate.update(relative.encode())
        aggregate.update(file_hash.encode())
        files.append({"path": relative, "size_bytes": path.stat().st_size, "sha256": file_hash})
    return aggregate.hexdigest(), files


def extract(args: argparse.Namespace) -> None:
    config = load_json(args.config)
    bag_path = Path(args.bag).expanduser().resolve()
    if not bag_path.is_dir():
        raise FileNotFoundError(f"bag directory does not exist: {bag_path}")
    storage = config["recording"]["storage"]
    metadata = rosbag2_py.Info().read_metadata(str(bag_path), storage)
    if storage == "mcap":
        if not metadata.relative_file_paths:
            raise RuntimeError("MCAP metadata contains no bag files")
        for relative_path in metadata.relative_file_paths:
            validate_mcap_file(bag_path / relative_path)
    ground_truth_path = Path(args.ground_truth).expanduser().resolve()
    ground_truth_path.parent.mkdir(parents=True, exist_ok=True)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=storage),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    configured_topics = list(config["recording"]["topics"])
    optional_topics = set(config["recording"].get("optional_topics", []))
    required = [topic for topic in configured_topics if topic not in optional_topics]
    missing_definitions = sorted(set(required) - set(topic_types))
    pose_type = topic_types.get("/auv/pose_actual")
    if pose_type is None:
        raise RuntimeError("bag has no /auv/pose_actual topic")
    pose_message_type = get_message(pose_type)

    fd, temporary_name = tempfile.mkstemp(
        prefix=f".{ground_truth_path.name}.", suffix=".tmp", dir=ground_truth_path.parent
    )
    counts: Counter[str] = Counter()
    first_receive_ns: int | None = None
    last_receive_ns: int | None = None
    first_pose_stamp_ns: int | None = None
    last_pose_stamp_ns: int | None = None
    pose_regressions = 0
    pose_nonfinite = 0
    try:
        with os.fdopen(fd, "w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(("timestamp_s", "x", "y", "z", "qx", "qy", "qz", "qw"))
            while reader.has_next():
                topic, serialized, receive_ns = reader.read_next()
                counts[topic] += 1
                first_receive_ns = receive_ns if first_receive_ns is None else min(first_receive_ns, receive_ns)
                last_receive_ns = receive_ns if last_receive_ns is None else max(last_receive_ns, receive_ns)
                if topic != "/auv/pose_actual":
                    continue
                message = deserialize_message(serialized, pose_message_type)
                stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
                if last_pose_stamp_ns is not None and stamp_ns < last_pose_stamp_ns:
                    pose_regressions += 1
                first_pose_stamp_ns = stamp_ns if first_pose_stamp_ns is None else first_pose_stamp_ns
                last_pose_stamp_ns = stamp_ns
                values = (
                    stamp_ns * 1e-9,
                    message.pose.position.x,
                    message.pose.position.y,
                    message.pose.position.z,
                    message.pose.orientation.x,
                    message.pose.orientation.y,
                    message.pose.orientation.z,
                    message.pose.orientation.w,
                )
                if not all(__import__("math").isfinite(float(value)) for value in values):
                    pose_nonfinite += 1
                writer.writerow(values)
            messages_read = sum(counts.values())
            if messages_read != metadata.message_count:
                raise RuntimeError(
                    f"bag reader returned {messages_read} of {metadata.message_count} indexed messages"
                )
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, ground_truth_path)
    except Exception:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise

    missing_messages = sorted(topic for topic in required if counts[topic] == 0)
    missing_optional_definitions = sorted(optional_topics - set(topic_types))
    missing_optional_messages = sorted(topic for topic in optional_topics if counts[topic] == 0)
    bag_hash, files = hash_bag_files(bag_path)
    manifest = {
        "valid": not missing_definitions and not missing_messages and pose_regressions == 0 and pose_nonfinite == 0,
        "bag_path": str(bag_path),
        "storage": storage,
        "sha256": bag_hash,
        "files": files,
        "topic_types": topic_types,
        "message_counts": dict(sorted(counts.items())),
        "missing_topic_definitions": missing_definitions,
        "missing_topic_messages": missing_messages,
        "missing_optional_topic_definitions": missing_optional_definitions,
        "missing_optional_topic_messages": missing_optional_messages,
        "receive_start_s": None if first_receive_ns is None else first_receive_ns * 1e-9,
        "receive_end_s": None if last_receive_ns is None else last_receive_ns * 1e-9,
        "duration_s": None
        if first_receive_ns is None or last_receive_ns is None
        else (last_receive_ns - first_receive_ns) * 1e-9,
        "pose_start_s": None if first_pose_stamp_ns is None else first_pose_stamp_ns * 1e-9,
        "pose_end_s": None if last_pose_stamp_ns is None else last_pose_stamp_ns * 1e-9,
        "pose_timestamp_regressions": pose_regressions,
        "pose_nonfinite_messages": pose_nonfinite,
        "ground_truth_path": str(ground_truth_path),
    }
    atomic_json(args.output, manifest)
    if not manifest["valid"]:
        raise RuntimeError("bag validation failed; inspect " + str(args.output))
    print(args.output)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--bag", required=True)
    parser.add_argument("--ground-truth", required=True)
    parser.add_argument("--output", required=True)
    extract(parser.parse_args())


if __name__ == "__main__":
    main()
