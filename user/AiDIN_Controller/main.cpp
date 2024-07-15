/*!
 * @file main.cpp
 * @brief Main Function for the WBC Controller
 *
 * The main function parses command line arguments and starts the appropriate
 * driver.
 */

#include "rclcpp/rclcpp.hpp"
#include "AiDIN_Controller.hpp"
// #include "main_helper.h"

int main(int argc, char** argv) {
  // ROS2 초기화
  rclcpp::init(argc, argv);

  // 기존 main_helper 함수 호출
//   main_helper(argc, argv, new MIT_Controller()); // wbc_aidin/robot/src/main_helper.cpp

  // ROS2 종료
  rclcpp::shutdown();
  return 0;
}
