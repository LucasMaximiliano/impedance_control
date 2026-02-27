#include "impedance_controller_ros2.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImpedanceControllerROS2>());
  rclcpp::shutdown();
  return 0;
}