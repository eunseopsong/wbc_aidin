/*!
 * @file RobotController.h
 * @brief Parent class of user robot controllers.
 * This is an interface between the control code and the common hardware code
 */

#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "Controllers/LegController.h"           // wbc_aidin/common/include/Controllers/LegController.h
#include "Dynamics/FloatingBaseModel.h"          // wbc_aidin/common/include/Dynamics/FloatingBaseModel.h
#include "Controllers/StateEstimatorContainer.h" // wbc_aidin/common/include/Controllers/StateEstimatorContainer.h
#include "Controllers/DesiredStateCommand.h"     // wbc_aidin/common/include/Controllers/DesiredStateCommand.h
#include "SimUtilities/VisualizationData.h"      // wbc_aidin/common/include/SimUtilities/VisualizationData.h
#include "SimUtilities/GamepadCommand.h"         // wbc_aidin/common/include/SimUtilities/GamepadCommand.h

/*!
 * Parent class of user robot controllers // This comment explains that RobotController is the parent class for user robot controllers.
 */
class RobotController{
  friend class RobotRunner;                // The 'friend' allows the RobotRunner class to access the private and protected members of RobotController.
public:
  RobotController(){}
  virtual ~RobotController(){}             // The virtual destructor ensures that the destructors of derived classes are called correctly.

  virtual void initializeController() = 0; // This is a pure virtual function for initializing the controller. It must be implemented by any derived class.
/**
 * Called one time every control loop
 */
  virtual void runController() = 0;
  virtual void updateVisualization() = 0;
  virtual ControlParameters* getUserControlParameters() = 0;
  virtual void Estop() {}

protected:
  Quadruped<float>* _quadruped = nullptr;
  FloatingBaseModel<float>* _model = nullptr;
  LegController<float>* _legController = nullptr;
  StateEstimatorContainer<float>* _stateEstimator = nullptr;
  StateEstimate<float>* _stateEstimate = nullptr;
  GamepadCommand* _driverCommand = nullptr;
  RobotControlParameters* _controlParameters = nullptr;
  DesiredStateCommand<float>* _desiredStateCommand = nullptr;

  VisualizationData* _visualizationData = nullptr;
  RobotType _robotType;
};

#endif
