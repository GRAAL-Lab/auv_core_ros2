#include "auv_core_helper/helper_lib.hpp"

namespace fs = std::filesystem;

void LoadParamsFromConf(const std::string& config_name, double* m, Eigen::Vector3d* CG, Eigen::Matrix3d* I,
                        Eigen::Matrix<double, 6, 1>* M_a_diag, Eigen::Matrix<double, 6, 1>* D_diag,
                        double* B, Eigen::Vector3d* CB, Eigen::Vector3d* G, Eigen::MatrixXd* thruster_positions,
                        Eigen::MatrixXd* thruster_orientations, Eigen::VectorXd* thruster_upper_limits,
                        Eigen::VectorXd* thruster_lower_limits, Eigen::VectorXd* thruster_allocation_weights,
                        Eigen::VectorXd* gainsX, Eigen::VectorXd* gainsY, Eigen::VectorXd* gainsZ,
                        Eigen::VectorXd* gainsRoll, Eigen::VectorXd* gainsPitch, Eigen::VectorXd* gainsYaw,
                        Eigen::VectorXd* max_linear_angular_velocities, Eigen::VectorXd* min_linear_angular_velocities){

    libconfig::Config cfg;
    try {
        std::string package_path = ament_index_cpp::get_package_share_directory("auv_core_helper");
        std::string conf_file_path = package_path + "/param/" + config_name + ".conf";

        // Check if the configuration file exists
        if (!fs::exists(conf_file_path)) {
            std::cerr << "Configuration file not found: " << conf_file_path << std::endl;
            return;  // Exit the function gracefully
        }

        cfg.readFile(conf_file_path.c_str());

        // Load scalars and vectors
        if (m) {
            try {
                *m = cfg.lookup(config_name + ".mass");
            } catch (const libconfig::SettingNotFoundException &nfex) {
                std::cerr << "Failed to load 'mass': " << nfex.what() << std::endl;
            }
        }
        if (B) {
            try {
                *B = cfg.lookup(config_name + ".buoyancy");
            } catch (const libconfig::SettingNotFoundException &nfex) {
                std::cerr << "Failed to load 'buoyancy': " << nfex.what() << std::endl;
            }
        }

        if (CG) {
            try {
                for (int i = 0; i < 3; ++i) {
                    (*CG)[i] = cfg.lookup(config_name + ".center_of_gravity")[i];
                }
            } catch (const libconfig::SettingNotFoundException &nfex) {
                std::cerr << "Failed to load 'center_of_gravity': " << nfex.what() << std::endl;
            }
        }

        if (CB) {
            try {
                for (int i = 0; i < 3; ++i) {
                    (*CB)[i] = cfg.lookup(config_name + ".center_of_buoyancy")[i];
                }
            } catch (const libconfig::SettingNotFoundException &nfex) {
                std::cerr << "Failed to load 'center_of_buoyancy': " << nfex.what() << std::endl;
            }
        }

        if (G) {
            try {
                for (int i = 0; i < 3; ++i) {
                    (*G)[i] = cfg.lookup(config_name + ".gravity_vector")[i];
                }
            } catch (const libconfig::SettingNotFoundException &nfex) {
                std::cerr << "Failed to load 'gravity_vector': " << nfex.what() << std::endl;
            }
        }

        if (I) {
            try {
                const libconfig::Setting& inertia = cfg.lookup(config_name + ".inertia_tensor");
                (*I) << inertia[0], 0, 0,
                        0, inertia[1], 0,
                        0, 0, inertia[2];
            } catch (const libconfig::SettingNotFoundException &nfex) {
                std::cerr << "Failed to load 'inertia_tensor': " << nfex.what() << std::endl;
            }
        }

        // Load matrices and vectors
        auto loadMatrix = [](const libconfig::Setting& setting, Eigen::MatrixXd& matrix, int numCols) {
            try {
                int length = setting.getLength();
                int numRows = length / numCols;
                matrix.resize(numRows, numCols);
                for (int i = 0; i < numRows; ++i) {
                    for (int j = 0; j < numCols; ++j) {
                        matrix(i, j) = static_cast<double>(setting[i * numCols + j]);
                    }
                }
            } catch (const libconfig::SettingNotFoundException &nfex) {
                std::cerr << "Failed to load matrix: " << nfex.what() << std::endl;
            }
        };

        auto loadVector = [](const libconfig::Setting& setting, Eigen::VectorXd& vector) {
            try {
                int length = setting.getLength();
                vector.resize(length);
                for (int i = 0; i < length; ++i) {
                    vector[i] = static_cast<double>(setting[i]);
                }
            } catch (const libconfig::SettingNotFoundException &nfex) {
                std::cerr << "Failed to load vector: " << nfex.what() << std::endl;
            }
        };

        if (M_a_diag) {
            try {
                for (int i = 0; i < 6; ++i) {
                    (*M_a_diag)[i] = cfg.lookup(config_name + ".added_mass")[i];
                }
            } catch (const libconfig::SettingNotFoundException &nfex) {
                std::cerr << "Failed to load 'added_mass': " << nfex.what() << std::endl;
            }
        }

        if (D_diag) {
            try {
                for (int i = 0; i < 6; ++i) {
                    (*D_diag)[i] = cfg.lookup(config_name + ".damping_coefficients")[i];
                }
            } catch (const libconfig::SettingNotFoundException &nfex) {
                std::cerr << "Failed to load 'damping_coefficients': " << nfex.what() << std::endl;
            }
        }

        if (thruster_positions) {
            try {
                const libconfig::Setting& positions = cfg.lookup(config_name + ".thruster_positions");
                loadMatrix(positions, *thruster_positions, 3);
            } catch (const libconfig::SettingNotFoundException &nfex) {
                std::cerr << "Failed to load 'thruster_positions': " << nfex.what() << std::endl;
            }
        }

        if (thruster_orientations) {
            try {
                const libconfig::Setting& orientations = cfg.lookup(config_name + ".thruster_orientations_degrees");
                loadMatrix(orientations, *thruster_orientations, 3);
                *thruster_orientations = thruster_orientations->array() * M_PI / 180.0; // Convert degrees to radians
            } catch (const libconfig::SettingNotFoundException &nfex) {
                std::cerr << "Failed to load 'thruster_orientations_degrees': " << nfex.what() << std::endl;
            }
        }

        if (thruster_upper_limits) {
            try {
                const libconfig::Setting& upperLimits = cfg.lookup(config_name + ".thruster_upper_limits");
                loadVector(upperLimits, *thruster_upper_limits);
            } catch (const libconfig::SettingNotFoundException &nfex) {
                std::cerr << "Failed to load 'thruster_upper_limits': " << nfex.what() << std::endl;
            }
        }

        if (thruster_lower_limits) {
            try {
                const libconfig::Setting& lowerLimits = cfg.lookup(config_name + ".thruster_lower_limits");
                loadVector(lowerLimits, *thruster_lower_limits);
            } catch (const libconfig::SettingNotFoundException &nfex) {
                std::cerr << "Failed to load 'thruster_lower_limits': " << nfex.what() << std::endl;
            }
        }

        if (thruster_allocation_weights) {
            try {
                const libconfig::Setting& allocationWeights = cfg.lookup(config_name + ".thruster_allocation_weights");
                loadVector(allocationWeights, *thruster_allocation_weights);
            } catch (const libconfig::SettingNotFoundException &nfex) {
                std::cerr << "Failed to load 'thruster_allocation_weights': " << nfex.what() << std::endl;
            }
        }

        // Load gains
        if (gainsX) {
            try {
                for (int i = 0; i < 6; ++i) {
                    (*gainsX)[i] = cfg.lookup(config_name + ".gainsX")[i];
                }
            } catch (const libconfig::SettingNotFoundException &nfex) {
                std::cerr << "Failed to load 'gainsX': " << nfex.what() << std::endl;
            }
        }

        if (gainsY) {
            try {
                for (int i = 0; i < 6; ++i) {
                    (*gainsY)[i] = cfg.lookup(config_name + ".gainsY")[i];
                }
            } catch (const libconfig::SettingNotFoundException &nfex) {
                std::cerr << "Failed to load 'gainsY': " << nfex.what() << std::endl;
            }
        }

        if (gainsZ) {
            try {
                for (int i = 0; i < 6; ++i) {
                    (*gainsZ)[i] = cfg.lookup(config_name + ".gainsZ")[i];
                }
            } catch (const libconfig::SettingNotFoundException &nfex) {
                std::cerr << "Failed to load 'gainsZ': " << nfex.what() << std::endl;
            }
        }

        if (gainsRoll) {
            try {
                for (int i = 0; i < 6; ++i) {
                    (*gainsRoll)[i] = cfg.lookup(config_name + ".gainsRoll")[i];
                }
            } catch (const libconfig::SettingNotFoundException &nfex) {
                std::cerr << "Failed to load 'gainsRoll': " << nfex.what() << std::endl;
            }
        }

        if (gainsPitch) {
            try {
                for (int i = 0; i < 6; ++i) {
                    (*gainsPitch)[i] = cfg.lookup(config_name + ".gainsPitch")[i];
                }
            } catch (const libconfig::SettingNotFoundException &nfex) {
                std::cerr << "Failed to load 'gainsPitch': " << nfex.what() << std::endl;
            }
        }

        if (gainsYaw) {
            try {
                for (int i = 0; i < 6; ++i) {
                    (*gainsYaw)[i] = cfg.lookup(config_name + ".gainsYaw")[i];
                }
            } catch (const libconfig::SettingNotFoundException &nfex) {
                std::cerr << "Failed to load 'gainsYaw': " << nfex.what() << std::endl;
            }
        }

        if (max_linear_angular_velocities) {
            try {
                const libconfig::Setting& maxVelocities = cfg.lookup(config_name + ".max_linear_angular_velocities");
                loadVector(maxVelocities, *max_linear_angular_velocities);
            } catch (const libconfig::SettingNotFoundException &nfex) {
                std::cerr << "Failed to load 'max_linear_angular_velocities': " << nfex.what() << std::endl;
            }
        }

        if (min_linear_angular_velocities) {
            try {
                const libconfig::Setting& minVelocities = cfg.lookup(config_name + ".min_linear_angular_velocities");
                loadVector(minVelocities, *min_linear_angular_velocities);
            } catch (const libconfig::SettingNotFoundException &nfex) {
                std::cerr << "Failed to load 'min_linear_angular_velocities': " << nfex.what() << std::endl;
            }
        }

    } catch (const libconfig::FileIOException &fioex) {
        std::cerr << "I/O error while reading file: " << fioex.what() << std::endl;
    } catch (const libconfig::ParseException &pex) {
        std::cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine() << " - " << pex.getError() << std::endl;
    } catch (const libconfig::ConfigException &cex) {
        std::cerr << "Configuration Error: " << cex.what() << std::endl;
    }
}




void PublishEigenPose(const rclcpp::Publisher<auv_core_helper::msg::PoseStamped>::SharedPtr& publisher, const Eigen::Matrix<double, 6, 1>& pose, const rclcpp::Time& time) {
    auto message = std::make_unique<auv_core_helper::msg::PoseStamped>();
    message->header.stamp = time;
    message->x = pose(0);
    message->y = pose(1);
    message->z = pose(2);
    message->roll = pose(3);
    message->pitch = pose(4);
    message->yaw = pose(5);

    publisher->publish(std::move(message));
}
void PublishEigenVelocity(const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr& publisher, const Eigen::Matrix<double, 6, 1>& velocity) {
    auto message = std::make_unique<geometry_msgs::msg::Twist>();
    message->linear.x = velocity(0);
    message->linear.y = velocity(1);
    message->linear.z = velocity(2);
    message->angular.x = velocity(3);
    message->angular.y = velocity(4);
    message->angular.z = velocity(5);
    publisher->publish(std::move(message));
}
void PublishEigenAcceleration(const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr& publisher, const Eigen::Matrix<double, 6, 1>& acceleration) {   
    auto message = std::make_unique<geometry_msgs::msg::Twist>();
    message->linear.x = acceleration(0);
    message->linear.y = acceleration(1);
    message->linear.z = acceleration(2);
    message->angular.x = acceleration(3);
    message->angular.y = acceleration(4);
    message->angular.z = acceleration(5);
    publisher->publish(std::move(message));
}

//TO DO: MOVE TO KCL JOYSTICK STATE AND ADD CALIBRATION!
void MapJoystickToVelocity(const std::vector<float>& axes, geometry_msgs::msg::Twist* velocity_desired) {
    if (axes.size() < 6) return; // Ensure there are enough axes

    // Linear velocities based on joystick input
    double raw_linear_x = 1.0 * axes[0]; // Left/right
    double raw_linear_y = 1.0 * axes[1]; // Forward/backward
    double raw_linear_z = ((axes[3] + 1) / 2) * 1.0 - ((axes[4] + 1) / 2) * 1.0; // Up/down

    // Angular velocities based on joystick input
    double raw_angular_z = 0.5 * axes[2]; // Yaw (turn left/right)
    double raw_angular_y = 0.5 * axes[5]; // Pitch (tilt forward/backward)

    // Calculate magnitudes and scale
    double magnitude_linear = sqrt(raw_linear_x * raw_linear_x + raw_linear_y * raw_linear_y + raw_linear_z * raw_linear_z);
    double magnitude_angular = sqrt(raw_angular_z * raw_angular_z + raw_angular_y * raw_angular_y);

    // Normalize and scale linear velocities
    if (magnitude_linear > 0) {
        double max_linear_speed = 1.5; // Adjust as needed
        velocity_desired->linear.x = (raw_linear_x / magnitude_linear) * max_linear_speed;
        velocity_desired->linear.y = (raw_linear_y / magnitude_linear) * max_linear_speed;
        velocity_desired->linear.z = (raw_linear_z / magnitude_linear) * max_linear_speed;
    } else {
        velocity_desired->linear.x = 0;
        velocity_desired->linear.y = 0;
        velocity_desired->linear.z = 0;
    }

    // Normalize and scale angular velocities
    if (magnitude_angular > 0) {
        double max_angular_speed = 2.8; // Adjust as needed
        velocity_desired->angular.z = (raw_angular_z / magnitude_angular) * max_angular_speed;
        velocity_desired->angular.y = (raw_angular_y / magnitude_angular) * max_angular_speed;
    } else {
        velocity_desired->angular.z = 0;
        velocity_desired->angular.y = 0;
    }
}

// Function to convert body angular velocities to Euler angle rates
Eigen::Vector3d ConvertAngularVelocitiesToEulerRates(double rollActual, double pitchActual, const Eigen::Vector3d& bOmegaDesired) {
    // Check for singularity (cos(pitchActual) == 0)
    if (std::abs(std::cos(pitchActual)) < 1e-6) {
        std::cerr << "Singularity detected: cos(pitchActual) is too close to zero. Returning zero Euler rates." << std::endl;
        // Return zero Euler rates as a fallback
        return Eigen::Vector3d::Zero();
    }

    // Construct the inverse Jacobian matrix directly
    Eigen::Matrix3d Jinv;
    Jinv << 1, std::sin(rollActual) * std::tan(pitchActual), std::cos(rollActual) * std::tan(pitchActual),
            0, std::cos(rollActual),                       -std::sin(rollActual),
            0, std::sin(rollActual) / std::cos(pitchActual), std::cos(rollActual) / std::cos(pitchActual);

    // Compute the desired Euler angle rates
    Eigen::Vector3d eulerRatesDesired = Jinv * bOmegaDesired;

    return eulerRatesDesired;
}