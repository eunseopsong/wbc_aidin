/*!
 * @file main.cpp
 * @brief Main Function for the WBC Controller
 *
 * The main function parses command line arguments and starts the appropriate
 * driver.
 */

#include "rclcpp/rclcpp.hpp"
#include "AiDIN_Controller.hpp"
#include "main_helper.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  main_helper(argc, argv, new AiDIN_Controller()); // wbc_aidin/robot/src/main_helper.cpp
  rclcpp::shutdown();
  return 0;
}
