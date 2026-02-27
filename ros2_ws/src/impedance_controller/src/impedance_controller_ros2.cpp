#include "impedance_controller_ros2.hpp"
#include "finger_kinematics.hpp"

#define RPM_TO_RAD_PER_SEC (2.0 * M_PI / 60.0)

ImpedanceControllerROS2::ImpedanceControllerROS2()
: Node("impedance_controller_node")
{
    RCLCPP_INFO(this->get_logger(), "Constructing... Impedance Controller Node");
    
    // Initialize impedance controller instances for each finger with parameters from TOML file
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("impedance_controller");
    std::string toml_path = package_share_directory + "/config.toml";
    impedance_controller_params::params_t params(toml_path);
    impedance_controller_1_ = ImpedanceControllerBase(
            params.virtual_inertia_matrix_,
            params.virtual_damping_matrix_,
            params.virtual_stiffness_matrix_,
            params.force_feedback_gain_,
            params.link_mass_vector_,
            params.link_length_vector_,
            params.dist_to_com_vector_,
            params.max_actuator_torque_,
            params.control_loop_duration_);
    impedance_controller_2_ = ImpedanceControllerBase(
            params.virtual_inertia_matrix_,
            params.virtual_damping_matrix_,
            params.virtual_stiffness_matrix_,
            params.force_feedback_gain_,
            params.link_mass_vector_,
            params.link_length_vector_,
            params.dist_to_com_vector_,
            params.max_actuator_torque_,
            params.control_loop_duration_);
    impedance_controller_3_ = ImpedanceControllerBase(
            params.virtual_inertia_matrix_,
            params.virtual_damping_matrix_,
            params.virtual_stiffness_matrix_,
            params.force_feedback_gain_,
            params.link_mass_vector_,
            params.link_length_vector_,
            params.dist_to_com_vector_,
            params.max_actuator_torque_,
            params.control_loop_duration_);
    impedance_controller_4_ = ImpedanceControllerBase(
            params.virtual_inertia_matrix_,
            params.virtual_damping_matrix_,
            params.virtual_stiffness_matrix_,
            params.force_feedback_gain_,
            params.link_mass_vector_,
            params.link_length_vector_,
            params.dist_to_com_vector_,
            params.max_actuator_torque_,
            params.control_loop_duration_);

    // Callback timer for control loop
    control_loop_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(params.control_loop_duration_),
        std::bind(&ImpedanceControllerROS2::controlLoopCallback, this)
    );
    // Publisher
    torque_command_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/command/set_torque_nm", 10);
    // Subscribers
    measured_position_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/position_rad", 10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            if (msg->data.size() >= 8) {
                // Parse message content
                Eigen::Vector2d joint_position_1(msg->data[0], msg->data[1]);
                Eigen::Vector2d joint_position_2(msg->data[2], msg->data[3]);
                Eigen::Vector2d joint_position_3(msg->data[4], msg->data[5]);
                Eigen::Vector2d joint_position_4(msg->data[6], msg->data[7]);
                // Set measured joint positions for all four fingers (2 joints per finger)
                impedance_controller_1_.setJointPositionMeasured(joint_position_1);
                impedance_controller_2_.setJointPositionMeasured(joint_position_2);
                impedance_controller_3_.setJointPositionMeasured(joint_position_3);
                impedance_controller_4_.setJointPositionMeasured(joint_position_4);
                // Set measured Cartesian positions for all four fingers using forward kinematics
                impedance_controller_1_.setCartesianPositionMeasured(finger_kinematics::forward(joint_position_1, impedance_controller_1_.getLinkLengths()));
                impedance_controller_2_.setCartesianPositionMeasured(finger_kinematics::forward(joint_position_2, impedance_controller_2_.getLinkLengths()));
                impedance_controller_3_.setCartesianPositionMeasured(finger_kinematics::forward(joint_position_3, impedance_controller_3_.getLinkLengths()));
                impedance_controller_4_.setCartesianPositionMeasured(finger_kinematics::forward(joint_position_4, impedance_controller_4_.getLinkLengths()));
            }
        }
    );
    measured_velocity_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/velocity_rpm", 10,
        [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            if (msg->data.size() >= 8) {
                // Parse message content
                Eigen::Vector2d joint_velocity_1(msg->data[0], msg->data[1]) * RPM_TO_RAD_PER_SEC;
                Eigen::Vector2d joint_velocity_2(msg->data[2], msg->data[3]) * RPM_TO_RAD_PER_SEC;
                Eigen::Vector2d joint_velocity_3(msg->data[4], msg->data[5]) * RPM_TO_RAD_PER_SEC;
                Eigen::Vector2d joint_velocity_4(msg->data[6], msg->data[7]) * RPM_TO_RAD_PER_SEC;
                // Set measured joint velocities for all four fingers (2 joints per finger)
                impedance_controller_1_.setJointVelocityMeasured(joint_velocity_1);
                impedance_controller_2_.setJointVelocityMeasured(joint_velocity_2);
                impedance_controller_3_.setJointVelocityMeasured(joint_velocity_3);
                impedance_controller_4_.setJointVelocityMeasured(joint_velocity_4);
                // Set measured Cartesian velocities for all four fingers using Jacobian
                impedance_controller_1_.setCartesianVelocityMeasured(finger_kinematics::jacobian(impedance_controller_1_.getJointPositionMeasured(), impedance_controller_1_.getLinkLengths()) * joint_velocity_1);
                impedance_controller_2_.setCartesianVelocityMeasured(finger_kinematics::jacobian(impedance_controller_2_.getJointPositionMeasured(), impedance_controller_2_.getLinkLengths()) * joint_velocity_2);
                impedance_controller_3_.setCartesianVelocityMeasured(finger_kinematics::jacobian(impedance_controller_3_.getJointPositionMeasured(), impedance_controller_3_.getLinkLengths()) * joint_velocity_3);
                impedance_controller_4_.setCartesianVelocityMeasured(finger_kinematics::jacobian(impedance_controller_4_.getJointPositionMeasured(), impedance_controller_4_.getLinkLengths()) * joint_velocity_4);
            }
        }
    );
    measured_torque_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/torque_nm", 10,
        [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            if (msg->data.size() >= 8) {
                impedance_controller_1_.setJointTorqueMeasured(Eigen::Vector2d(msg->data[0], msg->data[1]));
                impedance_controller_2_.setJointTorqueMeasured(Eigen::Vector2d(msg->data[2], msg->data[3]));
                impedance_controller_3_.setJointTorqueMeasured(Eigen::Vector2d(msg->data[4], msg->data[5]));
                impedance_controller_4_.setJointTorqueMeasured(Eigen::Vector2d(msg->data[6], msg->data[7]));
            }
        }
    );
    desired_joint_position_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/desired_joint_position_rad", 10, /* TODO: Check type */
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            if (msg->data.size() >= 8) {
                // Parse message content
                Eigen::Vector2d joint_position_1(msg->data[0], msg->data[1]); /* TODO: Check units */
                Eigen::Vector2d joint_position_2(msg->data[2], msg->data[3]); /* TODO: Check units */
                Eigen::Vector2d joint_position_3(msg->data[4], msg->data[5]); /* TODO: Check units */
                Eigen::Vector2d joint_position_4(msg->data[6], msg->data[7]); /* TODO: Check units */
                // Set desired joint positions for all four fingers (2 joints per finger)
                impedance_controller_1_.setJointPositionDesired(joint_position_1);
                impedance_controller_2_.setJointPositionDesired(joint_position_2);
                impedance_controller_3_.setJointPositionDesired(joint_position_3);
                impedance_controller_4_.setJointPositionDesired(joint_position_4);
                // Set desired Cartesian positions for all four fingers using forward kinematics
                impedance_controller_1_.setCartesianPositionDesired(finger_kinematics::forward(joint_position_1, impedance_controller_1_.getLinkLengths()));
                impedance_controller_2_.setCartesianPositionDesired(finger_kinematics::forward(joint_position_2, impedance_controller_2_.getLinkLengths()));
                impedance_controller_3_.setCartesianPositionDesired(finger_kinematics::forward(joint_position_3, impedance_controller_3_.getLinkLengths()));
                impedance_controller_4_.setCartesianPositionDesired(finger_kinematics::forward(joint_position_4, impedance_controller_4_.getLinkLengths()));
            }
        }
    );
    desired_joint_velocity_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/desired_joint_velocity_rpm", 10, /* TODO: Check type */
        [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            if (msg->data.size() >= 8) {
                Eigen::Vector2d desired_joint_velocity_1(msg->data[0], msg->data[1]); /* TODO: Check units */
                Eigen::Vector2d desired_joint_velocity_2(msg->data[2], msg->data[3]); /* TODO: Check units */
                Eigen::Vector2d desired_joint_velocity_3(msg->data[4], msg->data[5]); /* TODO: Check units */
                Eigen::Vector2d desired_joint_velocity_4(msg->data[6], msg->data[7]); /* TODO: Check units */
                impedance_controller_1_.setCartesianVelocityDesired(finger_kinematics::jacobian(impedance_controller_1_.getJointPositionDesired(), impedance_controller_1_.getLinkLengths()) * desired_joint_velocity_1);
                impedance_controller_2_.setCartesianVelocityDesired(finger_kinematics::jacobian(impedance_controller_2_.getJointPositionDesired(), impedance_controller_2_.getLinkLengths()) * desired_joint_velocity_2);
                impedance_controller_3_.setCartesianVelocityDesired(finger_kinematics::jacobian(impedance_controller_3_.getJointPositionDesired(), impedance_controller_3_.getLinkLengths()) * desired_joint_velocity_3);
                impedance_controller_4_.setCartesianVelocityDesired(finger_kinematics::jacobian(impedance_controller_4_.getJointPositionDesired(), impedance_controller_4_.getLinkLengths()) * desired_joint_velocity_4);
            }
        }
    );
}

ImpedanceControllerROS2::~ImpedanceControllerROS2()
{
    RCLCPP_INFO(this->get_logger(), "Destructing... Impedance Controller Node");
}

void ImpedanceControllerROS2::controlLoopCallback()
{
    // Compute impedance control torques for all four fingers
    Eigen::Vector2d torque_command_1 = impedance_controller_1_.computeImpedanceTorque();
    Eigen::Vector2d torque_command_2 = impedance_controller_2_.computeImpedanceTorque();
    Eigen::Vector2d torque_command_3 = impedance_controller_3_.computeImpedanceTorque();
    Eigen::Vector2d torque_command_4 = impedance_controller_4_.computeImpedanceTorque();

    // Publish torque commands as a single message
    std_msgs::msg::Float32MultiArray command_msg;
    command_msg.data = {torque_command_1.x(), torque_command_1.y(),
                        torque_command_2.x(), torque_command_2.y(),
                        torque_command_3.x(), torque_command_3.y(),
                        torque_command_4.x(), torque_command_4.y()};
    torque_command_publisher_->publish(command_msg);
}