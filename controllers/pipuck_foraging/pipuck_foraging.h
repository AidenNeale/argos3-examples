/*
 * AUTHOR: Carlo Pinciroli <cpinciro@ulb.ac.be>
 *
 * An example controller for obstacle avoidance with the e-puck.
 *
 * The controller uses the proximity sensor to detect obstacles and the
 * wheels to move the robot around.
 *
 * This controller is meant to be used with the XML files:
 *    experiments/pipuck_Foraging.argos
 */

#ifndef PIPUCK_FORAGING_H
#define PIPUCK_FORAGING_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_differential_drive_actuator.h>
/* Definition of the Pipuck Rangefinders sensor */
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_rangefinders_sensor.h>
/* Definition of the Pipuck Ground Colour sensor */
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_ground_colour_sensor.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CPiPuckForaging : public CCI_Controller {

public:

  /* Class constructor. */
  CPiPuckForaging();

  /* Class destructor. */
  virtual ~CPiPuckForaging() {}

  /*
  * This function initializes the controller.
  * The 't_node' variable points to the <parameters> section in the XML
  * file in the <controllers><pipuck_foraging_controller> section.
  */
  virtual void Init(TConfigurationNode& t_node);

  /*
  * This function is called once every time step.
  * The length of the time step is set in the XML file.
  */
  virtual void ControlStep();

  /*
  * This function resets the controller to its state right after the
  * Init().
  * It is called when you press the reset button in the GUI.
  * In this example controller there is no need for resetting anything,
  * so the function could have been omitted. It's here just for
  * completeness.
  */
  virtual void Reset() {}

  /*
  * Called to cleanup what done by Init() when the experiment finishes.
  * In this example controller there is no need for clean anything up,
  * so the function could have been omitted. It's here just for
  * completeness.
  */
  virtual void Destroy() {}

  void readGroundColorSensor();

  bool getOnFood() {
    return onFood;
  }
  void setOnFood(bool food) {
    onFood = food;
  }

  bool getOnWater() {
    return onWater;
  }
  void setOnWater(bool water) {
    onWater = water;
  }

  CColor getGroundColor() {
    return groundColor;
  }

  void setGroundColor(CColor color) {
    groundColor = color;
  }

private:

  /* Pointer to the differential steering actuator */
  CCI_PiPuckDifferentialDriveActuator* pcWheels;
  /* Pointer to the pi-puck proximity sensor */
  CCI_PiPuckRangefindersSensor* pcProximity;
  /* Pointer to the pi-puck ground colour sensor */
  CCI_PiPuckGroundColourSensor* pcGround;

  bool onFood;
  bool onWater;

  CColor groundColor;
};

#endif
