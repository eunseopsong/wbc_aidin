#ifndef AiDIN_CONTROLLER
#define AiDIN_CONTROLLER

#include <RobotController.h>
#include "Controllers/GaitScheduler.h"
#include "Controllers/ContactEstimator.h"
#include "FSM_States/ControlFSM.h"
#include "AiDIN_UserParameters.h"
//#include <gui_main_control_settings_t.hpp>

class AiDIN_Controller: public RobotController{
public:
  AiDIN_Controller();
  virtual ~AiDIN_Controller(){}

  virtual void initializeController();
  virtual void runController();
  virtual void updateVisualization(){}
  virtual ControlParameters* getUserControlParameters() {
    return &userParameters;
  }
  virtual void Estop(){ _controlFSM->initialize(); }


protected:
  ControlFSM<float>* _controlFSM;
  // Gait Scheduler controls the nominal contact schedule for the feet
  GaitScheduler<float>* _gaitScheduler;
  AiDIN_UserParameters userParameters;

};


#endif
