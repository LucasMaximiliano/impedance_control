#include "combined_controller_ros2.hpp"
#include "impedance_controller_params.hpp"
#include "finger_kinematics.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

// Macros for topic names, service names, and queue size
#define QUEUE_SIZE 10

#define PUBLISH_TORQUE_CMD_TOPIC "/command/set_torque_nm"

#define SUBSCRIBE_MEASURED_POSITION_TOPIC "/position_rad"
#define SUBSCRIBE_MEASURED_VELOCITY_TOPIC "/velocity"
#define SUBSCRIBE_MEASURED_TORQUE_TOPIC "/torque_nm"
#define SUBSCRIBE_DESIRED_POSITION_TOPIC "/desired_joint_position_rad"
#define SUBSCRIBE_DESIRED_VELOCITY_TOPIC "/desired_joint_velocity"

#define ENABLE_IMPEDANCE_TORQUE_SERVICE "/impedance_control_enabled"
#define ENABLE_GRAVITY_COMPENSATION_SERVICE "/gravity_compensation_enabled"
#define SET_INERTIA_GAIN_SERVICE "/set_inertia_gain"
#define SET_DAMPING_GAIN_SERVICE "/set_damping_gain"
#define SET_STIFFNESS_GAIN_SERVICE "/set_stiffness_gain"
#define SET_TORQUE_GAIN_SERVICE "/set_torque_gain"

static CombinedControllerBase initializeController(const int controller_id)
{
  std::string package_share_directory = ament_index_cpp::get_package_share_directory(
    "impedance_controller");
  std::string toml_path = package_share_directory + "/config.toml";
  impedance_controller_params::params_t params(toml_path);

  std::cout << "\n================\n" << "[CombinedControllerBase] Initializing controller for finger " << controller_id << std::endl;

  return CombinedControllerBase(
    params.virtual_inertia_matrix_,
    params.virtual_damping_matrix_,
    params.virtual_stiffness_matrix_,
    params.force_feedback_gain_,
    params.link_mass_vector_,
    params.link_length_vector_,
    params.dist_to_com_vector_,
    params.max_actuator_torque_,
    params.control_loop_duration_);
}

CombinedControllerROS2::CombinedControllerROS2()
: Node("impedance_controller_node"),
  combined_controller_1_(initializeController(1)),
  combined_controller_2_(initializeController(2)),
  combined_controller_3_(initializeController(3)),
  combined_controller_4_(initializeController(4))
{
  RCLCPP_INFO(this->get_logger(), "Constructing... Combined Controller Node");

  // Read parameters from TOML file
  std::string package_share_directory = ament_index_cpp::get_package_share_directory(
    "impedance_controller");
  std::string toml_path = package_share_directory + "/config.toml";
  impedance_controller_params::params_t params(toml_path);
  // Callback timer for control loop
  control_loop_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(params.control_loop_duration_),
    std::bind(&CombinedControllerROS2::controlLoopCallback, this)
  );
  // Publisher
  torque_command_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
    PUBLISH_TORQUE_CMD_TOPIC, QUEUE_SIZE);
  // Subscribers
  measured_position_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    SUBSCRIBE_MEASURED_POSITION_TOPIC, QUEUE_SIZE,
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      if (msg->data.size() >= 8) {
        // Parse message content
        Eigen::Vector2d joint_position_1(msg->data[0], msg->data[1]);
        Eigen::Vector2d joint_position_2(msg->data[2], msg->data[3]);
        Eigen::Vector2d joint_position_3(msg->data[4], msg->data[5]);
        Eigen::Vector2d joint_position_4(msg->data[6], msg->data[7]);
        // Set measured joint positions for all four fingers (2 joints per finger)
        combined_controller_1_.setJointPositionMeasured(joint_position_1);
        combined_controller_2_.setJointPositionMeasured(joint_position_2);
        combined_controller_3_.setJointPositionMeasured(joint_position_3);
        combined_controller_4_.setJointPositionMeasured(joint_position_4);
        // Set measured Cartesian positions for all four fingers using forward kinematics
        combined_controller_1_.setCartesianPositionMeasured(
          finger_kinematics::forward(
            joint_position_1, combined_controller_1_.getLinkLengths()));
        combined_controller_2_.setCartesianPositionMeasured(
          finger_kinematics::forward(
            joint_position_2, combined_controller_2_.getLinkLengths()));
        combined_controller_3_.setCartesianPositionMeasured(
          finger_kinematics::forward(
            joint_position_3, combined_controller_3_.getLinkLengths()));
        combined_controller_4_.setCartesianPositionMeasured(
          finger_kinematics::forward(
            joint_position_4, combined_controller_4_.getLinkLengths()));
      }
    }
  );
  measured_velocity_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    SUBSCRIBE_MEASURED_VELOCITY_TOPIC, QUEUE_SIZE,
    [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
      if (msg->data.size() >= 8) {
        // Parse message content
        Eigen::Vector2d joint_velocity_1(msg->data[0], msg->data[1]);
        Eigen::Vector2d joint_velocity_2(msg->data[2], msg->data[3]);
        Eigen::Vector2d joint_velocity_3(msg->data[4], msg->data[5]);
        Eigen::Vector2d joint_velocity_4(msg->data[6], msg->data[7]);
        // Set measured joint velocities for all four fingers (2 joints per finger)
        combined_controller_1_.setJointVelocityMeasured(joint_velocity_1);
        combined_controller_2_.setJointVelocityMeasured(joint_velocity_2);
        combined_controller_3_.setJointVelocityMeasured(joint_velocity_3);
        combined_controller_4_.setJointVelocityMeasured(joint_velocity_4);
        // Set measured Cartesian velocities for all four fingers using Jacobian
        combined_controller_1_.setCartesianVelocityMeasured(
          finger_kinematics::jacobian(
            combined_controller_1_.getJointPositionMeasured(),
            combined_controller_1_.getLinkLengths()) * joint_velocity_1);
        combined_controller_2_.setCartesianVelocityMeasured(
          finger_kinematics::jacobian(
            combined_controller_2_.getJointPositionMeasured(),
            combined_controller_2_.getLinkLengths()) * joint_velocity_2);
        combined_controller_3_.setCartesianVelocityMeasured(
          finger_kinematics::jacobian(
            combined_controller_3_.getJointPositionMeasured(),
            combined_controller_3_.getLinkLengths()) * joint_velocity_3);
        combined_controller_4_.setCartesianVelocityMeasured(
          finger_kinematics::jacobian(
            combined_controller_4_.getJointPositionMeasured(),
            combined_controller_4_.getLinkLengths()) * joint_velocity_4);
      }
    }
  );
  measured_torque_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    SUBSCRIBE_MEASURED_TORQUE_TOPIC, QUEUE_SIZE,
    [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
      if (msg->data.size() >= 8) {
        combined_controller_1_.setJointTorqueMeasured(Eigen::Vector2d(msg->data[0], msg->data[1]));
        combined_controller_2_.setJointTorqueMeasured(Eigen::Vector2d(msg->data[2], msg->data[3]));
        combined_controller_3_.setJointTorqueMeasured(Eigen::Vector2d(msg->data[4], msg->data[5]));
        combined_controller_4_.setJointTorqueMeasured(Eigen::Vector2d(msg->data[6], msg->data[7]));
      }
    }
  );
  desired_position_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    SUBSCRIBE_DESIRED_POSITION_TOPIC, QUEUE_SIZE,
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      if (msg->data.size() >= 8) {
        // Parse message content
        Eigen::Vector2d joint_position_1(msg->data[0], msg->data[1]);
        Eigen::Vector2d joint_position_2(msg->data[2], msg->data[3]);
        Eigen::Vector2d joint_position_3(msg->data[4], msg->data[5]);
        Eigen::Vector2d joint_position_4(msg->data[6], msg->data[7]);
        // Set desired joint positions for all four fingers (2 joints per finger)
        combined_controller_1_.setJointPositionDesired(joint_position_1);
        combined_controller_2_.setJointPositionDesired(joint_position_2);
        combined_controller_3_.setJointPositionDesired(joint_position_3);
        combined_controller_4_.setJointPositionDesired(joint_position_4);
        // Set desired Cartesian positions for all four fingers using forward kinematics
        combined_controller_1_.setCartesianPositionDesired(
          finger_kinematics::forward(
            joint_position_1, combined_controller_1_.getLinkLengths()));
        combined_controller_2_.setCartesianPositionDesired(
          finger_kinematics::forward(
            joint_position_2, combined_controller_2_.getLinkLengths()));
        combined_controller_3_.setCartesianPositionDesired(
          finger_kinematics::forward(
            joint_position_3, combined_controller_3_.getLinkLengths()));
        combined_controller_4_.setCartesianPositionDesired(
          finger_kinematics::forward(
            joint_position_4, combined_controller_4_.getLinkLengths()));
      }
    }
  );
  desired_velocity_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    SUBSCRIBE_DESIRED_VELOCITY_TOPIC, QUEUE_SIZE,
    [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
      if (msg->data.size() >= 8) {
        Eigen::Vector2d desired_joint_velocity_1(msg->data[0], msg->data[1]);                                                                                   /* TODO: Check units */
        Eigen::Vector2d desired_joint_velocity_2(msg->data[2], msg->data[3]);                                                                                   /* TODO: Check units */
        Eigen::Vector2d desired_joint_velocity_3(msg->data[4], msg->data[5]);                                                                                   /* TODO: Check units */
        Eigen::Vector2d desired_joint_velocity_4(msg->data[6], msg->data[7]);                                                                                   /* TODO: Check units */
        combined_controller_1_.setCartesianVelocityDesired(
          finger_kinematics::jacobian(
            combined_controller_1_.getJointPositionDesired(),
            combined_controller_1_.getLinkLengths()) * desired_joint_velocity_1);
        combined_controller_2_.setCartesianVelocityDesired(
          finger_kinematics::jacobian(
            combined_controller_2_.getJointPositionDesired(),
            combined_controller_2_.getLinkLengths()) * desired_joint_velocity_2);
        combined_controller_3_.setCartesianVelocityDesired(
          finger_kinematics::jacobian(
            combined_controller_3_.getJointPositionDesired(),
            combined_controller_3_.getLinkLengths()) * desired_joint_velocity_3);
        combined_controller_4_.setCartesianVelocityDesired(
          finger_kinematics::jacobian(
            combined_controller_4_.getJointPositionDesired(),
            combined_controller_4_.getLinkLengths()) * desired_joint_velocity_4);
      }
    }
  );
  // Services
  impedance_control_enabled_service_ = this->create_service<std_srvs::srv::SetBool>(
    ENABLE_IMPEDANCE_TORQUE_SERVICE,
    [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
           std_srvs::srv::SetBool::Response::SharedPtr response) {
      combined_controller_1_.setImpedanceControlEnabled(request->data);
      combined_controller_2_.setImpedanceControlEnabled(request->data);
      combined_controller_3_.setImpedanceControlEnabled(request->data);
      combined_controller_4_.setImpedanceControlEnabled(request->data);
      response->success = true;
      response->message = request->data ? "Impedance control enabled" : "Impedance control disabled";
    }
  );
  gravity_compensation_enabled_service_ = this->create_service<std_srvs::srv::SetBool>(
    ENABLE_GRAVITY_COMPENSATION_SERVICE,
    [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
           std_srvs::srv::SetBool::Response::SharedPtr response) {
      combined_controller_1_.setGravityCompensationEnabled(request->data);
      combined_controller_2_.setGravityCompensationEnabled(request->data);
      combined_controller_3_.setGravityCompensationEnabled(request->data);
      combined_controller_4_.setGravityCompensationEnabled(request->data);
      response->success = true;
      response->message = request->data ? "Gravity compensation enabled" : "Gravity compensation disabled";
    }
  );
  inertia_gain_service_ = this->create_service<impedance_controller_interfaces::srv::SetGain>(
    SET_INERTIA_GAIN_SERVICE,
    [this](const impedance_controller_interfaces::srv::SetGain::Request::SharedPtr request,
           impedance_controller_interfaces::srv::SetGain::Response::SharedPtr response) {
      combined_controller_1_.setVirtualInertiaMatrix(request->gain);
      combined_controller_2_.setVirtualInertiaMatrix(request->gain);
      combined_controller_3_.setVirtualInertiaMatrix(request->gain);
      combined_controller_4_.setVirtualInertiaMatrix(request->gain);
      response->success = true;
      response->message = "Inertia gain updated to " + std::to_string(request->gain);
    }
  );
  damping_gain_service_ = this->create_service<impedance_controller_interfaces::srv::SetGain>(
    SET_DAMPING_GAIN_SERVICE,
    [this](const impedance_controller_interfaces::srv::SetGain::Request::SharedPtr request,
           impedance_controller_interfaces::srv::SetGain::Response::SharedPtr response) {
      combined_controller_1_.setVirtualDampingMatrix(request->gain);
      combined_controller_2_.setVirtualDampingMatrix(request->gain);
      combined_controller_3_.setVirtualDampingMatrix(request->gain);
      combined_controller_4_.setVirtualDampingMatrix(request->gain);
      response->success = true;
      response->message = "Damping gain updated to " + std::to_string(request->gain);
    }
  );
  stiffness_gain_service_ = this->create_service<impedance_controller_interfaces::srv::SetGain>(
    SET_STIFFNESS_GAIN_SERVICE,
    [this](const impedance_controller_interfaces::srv::SetGain::Request::SharedPtr request,
           impedance_controller_interfaces::srv::SetGain::Response::SharedPtr response) {
      combined_controller_1_.setVirtualStiffnessMatrix(request->gain);
      combined_controller_2_.setVirtualStiffnessMatrix(request->gain);
      combined_controller_3_.setVirtualStiffnessMatrix(request->gain);
      combined_controller_4_.setVirtualStiffnessMatrix(request->gain);
      response->success = true;
      response->message = "Stiffness gain updated to " + std::to_string(request->gain);
    }
  );
  torque_gain_service_ = this->create_service<impedance_controller_interfaces::srv::SetGain>(
    SET_TORQUE_GAIN_SERVICE,
    [this](const impedance_controller_interfaces::srv::SetGain::Request::SharedPtr request,
           impedance_controller_interfaces::srv::SetGain::Response::SharedPtr response) {
      combined_controller_1_.setForceFeedbackGain(request->gain);
      combined_controller_2_.setForceFeedbackGain(request->gain);
      combined_controller_3_.setForceFeedbackGain(request->gain);
      combined_controller_4_.setForceFeedbackGain(request->gain);
      response->success = true;
      response->message = "Torque gain updated to " + std::to_string(request->gain);
    }
  );
}

CombinedControllerROS2::~CombinedControllerROS2()
{
  RCLCPP_INFO(this->get_logger(), "Destructing... Combined Controller Node");
}

void CombinedControllerROS2::controlLoopCallback()
{
  // Compute control torques for all four fingers
  Eigen::Vector2d torque_command_1 = combined_controller_1_.computeCombinedTorque();
  Eigen::Vector2d torque_command_2 = combined_controller_2_.computeCombinedTorque();
  Eigen::Vector2d torque_command_3 = combined_controller_3_.computeCombinedTorque();
  Eigen::Vector2d torque_command_4 = combined_controller_4_.computeCombinedTorque();

  // Publish torque commands as a single message
  std_msgs::msg::Float32MultiArray command_msg;
  command_msg.data = {
    static_cast<float>(torque_command_1.x()), static_cast<float>(torque_command_1.y()),
    static_cast<float>(torque_command_2.x()), static_cast<float>(torque_command_2.y()),
    static_cast<float>(torque_command_3.x()), static_cast<float>(torque_command_3.y()),
    static_cast<float>(torque_command_4.x()), static_cast<float>(torque_command_4.y())};
  torque_command_publisher_->publish(command_msg);
}
