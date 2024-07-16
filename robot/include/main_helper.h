/*!
 * @file main_helper.h
 * @brief Function which should be called in main to start your robot control code
 */

#ifndef ROBOT_MAIN_H // ifndef checks if ROBOT_MAIN_H has not been defined yet.
#define ROBOT_MAIN_H // If it hasn't, the #define statement defines ROBOT_MAIN_H.

#include "Types.h"           // wbc_aidin/common/include
#include "RobotController.h" // wbc_aidin/robot/include

extern MasterConfig gMasterConfig; // at "Types.h"
// 'extern' keyword indicates that this variable is defined elsewhere, possibly in a different source file.
int main_helper(int argc, char** argv, RobotController* ctrl);

#endif  // ROBOT_MAIN_H
// This line ends the include guard. It matches the #ifndef ROBOT_MAIN_H line and ensures that the contents of this header file are only included once.
