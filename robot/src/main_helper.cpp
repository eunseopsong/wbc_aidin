/*!
 * @file main_helper.cpp
 * @brief Helper Function for the WBC Controller
 *
 * This file contains the main_helper function that sets up and runs the robot controller.
 */

#include <cassert>
#include <iostream>

#include "HardwareBridge.h"   // wbc_aidin/robot/include/HardwareBridge.h
#include "SimulationBridge.h" // wbc_aidin/robot/include/SimulationBridge.h
#include "main_helper.h"      // wbc_aidin/robot/include
#include "RobotController.h"  // wbc_aidin/roobt/include
#include "rclcpp/rclcpp.hpp"  //// ROS1 -> ROS2: rclcpp 헤더 파일 포함

MasterConfig gMasterConfig;   // wbc_aidin/common/include/Types.h

/*!
 * Print a message describing the command line flags for the robot program
 */
void printUsage() {
  printf(
      "Usage: robot [robot-id] [sim-or-robot] [parameters-from-file]\n"
      "\twhere robot-id:     3 for cheetah 3, m for mini-cheetah\n"
      "\t      sim-or-robot: s for sim, r for robot\n"
      "\t      param-file:   f for loading parameters from file, l (or nothing) for LCM\n"
      "                      this option can only be used in robot mode\n");
}

/*!
 * Setup and run the given robot controller
 */
int main_helper(int argc, char** argv, RobotController* ctrl) {
  if (argc != 3 && argc != 4) {
    printUsage();
    return EXIT_FAILURE;
  }

  if (argv[1][0] == '3') {
    gMasterConfig._robot = RobotType::CHEETAH_3;
  } else if (argv[1][0] == 'm') {
    gMasterConfig._robot = RobotType::MINI_CHEETAH;
  } else {
    printUsage();
    return EXIT_FAILURE;
  }

  if (argv[2][0] == 's') {
    gMasterConfig.simulated = true;
  } else if (argv[2][0] == 'r') {
    gMasterConfig.simulated = false;
  } else {
    printUsage();
    return EXIT_FAILURE;
  }

  if(argc == 4 && argv[3][0] == 'f') {
    gMasterConfig.load_from_file = true;
    printf("Load parameters from file\n");
  } else {
    gMasterConfig.load_from_file = false;
    printf("Load parameters from network\n");
  }

  printf("[Quadruped] Cheetah Software\n");
  printf("        Quadruped:  %s\n",
         gMasterConfig._robot == RobotType::MINI_CHEETAH ? "Mini Cheetah"
                                                         : "Cheetah 3");
  printf("        Driver: %s\n", gMasterConfig.simulated
                                     ? "Development Simulation Driver"
                                     : "Quadruped Driver");

  // dispatch the appropriate driver (적절한 드라이버를 선택하여 실행)
  if (gMasterConfig.simulated) {
    if(argc != 3) {
      printUsage();
      return EXIT_FAILURE;
    }
    if (gMasterConfig._robot == RobotType::MINI_CHEETAH) {
      SimulationBridge simulationBridge(gMasterConfig._robot, ctrl);
      simulationBridge.run();
      printf("[Quadruped] SimDriver run() has finished!\n");
    } else if (gMasterConfig._robot == RobotType::CHEETAH_3) {
      SimulationBridge simulationBridge(gMasterConfig._robot, ctrl);
      simulationBridge.run();
    } else {
      printf("[ERROR] unknown robot\n");
      assert(false);
    }
  } else {
#ifdef linux
    if (gMasterConfig._robot == RobotType::MINI_CHEETAH) {
      MiniCheetahHardwareBridge hw(ctrl, gMasterConfig.load_from_file);
      hw.run();
      printf("[Quadruped] SimDriver run() has finished!\n");
    } else if (gMasterConfig._robot == RobotType::CHEETAH_3) {
      Cheetah3HardwareBridge hw(ctrl);
      hw.run();
    } else {
      printf("[ERROR] unknown robot\n");
      assert(false);
    }
#endif
  }

  return 0;
}
