#include "extrinsic_calibration_by_hand_cpp/node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<extrinsic_calibration_by_hand_cpp::ExtrinsicCalibrationNode>());
  rclcpp::shutdown();
  return 0;
}
