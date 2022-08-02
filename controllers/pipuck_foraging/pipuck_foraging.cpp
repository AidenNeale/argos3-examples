/* Include the controller definition */
#include "pipuck_foraging.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>

/****************************************/
/****************************************/

CPiPuckForaging::CPiPuckForaging() :
   pcWheels(NULL),
   pcGround(NULL),
   pcProximity(NULL),
   m_fWheelVelocity(2.5f) {}

/****************************************/
/****************************************/

void CPiPuckForaging::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the
    * XML tag of the device whose handle we want to have. For a list of
    * allowed values, type at the command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><epuck_Foraging><actuators> and
    * <controllers><epuck_Foraging><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
    pcWheels = GetActuator<CCI_PiPuckDifferentialDriveActuator>("pipuck_differential_drive");
    pcGround = GetSensor<CCI_PiPuckGroundColourSensor>("pipuck_ground_colour");
    pcProximity = GetSensor<CCI_PiPuckRangefindersSensor>("pipuck_rangefinders");

}

/****************************************/
/****************************************/

void CPiPuckForaging::ControlStep() {
  getGroundColorSensor();
  pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
  std::vector<Real> vecReadings;
  Real Readings[8];
  pcProximity->Visit([&vecReadings, &Readings] (const CCI_PiPuckRangefindersSensor::SInterface& s_interface)
  {
      vecReadings.emplace_back(s_interface.Proximity);
      Readings[s_interface.Label] = s_interface.Proximity;

      /*
      These output phrases are useful for debugging and understanding the
      example however cause the simulator to slow down to the sheer quantity of logging
      performed with 50 pipucks (8 messages per range finder sensor per robot)
      */

      // std::cout << "Label: " << s_interface.Label << std::endl;
      // std::cout << "Proximity: " << s_interface.Proximity << std::endl;

      // const CCI_PiPuckRangefindersSensor::TConfiguration configuration = s_interface.Configuration;
      // std::cout << "Configuration:" << std::endl;
      // std::cout << "Name: " << std::get<0>(configuration) << std::endl;
      // std::cout << "Position: " << std::get<1>(configuration) << std::endl;
      // std::cout << "Orientation: " << std::get<2>(configuration) << std::endl;
      // std::cout << "Range: " << std::get<3>(configuration) << std::endl;

      // std::cout << std::endl;
  });
  Real left = 0.5;
  Real right = 0.5;

  if(Readings[0] < 0.1)
  {
    if(Readings[7] < 0.1)
    {
      left = -0.2;
      right = 0.2;
    }
    else
    {
      if(Readings[1] < 0.1)
      {
        if(Readings[2] < 0.1)
        {
          left = -0.1;
          right = 0.2;
        }
        else
        {
          left = -0.1;
          right = 0.1;
        }
      }
      else
      {
        left = 0.0;
        right = 0.2;
      }
    }
  }
  else if(Readings[7] < 0.1)
  {
    if(Readings[6] < 0.1)
    {
      if(Readings[5] < 0.1)
      {
        left = 0.2;
        right = -0.1;
      }
      else
      {
        left = 0.1;
        right = -0.1;
      }
    }
    else
    {
      right = 0.0;
      left = 0.2;
    }
  }
  else
  {
    left = 0.5;
    right = 0.5;
  }


  pcWheels->SetLinearVelocity(left, right);
}

/****************************************/
/****************************************/

void CPiPuckForaging::readGroundColorSensor() {
  std::vector<CColor> vecReadings;
  setGroundColor(CColor());
  /* Access and retrieve ground colour sensor readings from all three ground sensors */
  pcGround->Visit([&vecReadings] (const CCI_PiPuckGroundColourSensor::SInterface& s_interface) {
    vecReadings.emplace_back(s_interface.cColor);
  });
  /* Only set the ground colour if all 3 sensors are over the same colour */
  if ((vecReadings[0] == vecReadings[1]) && (vecReadings[1] == vecReadings[2])) {
    setGroundColor(vecReadings[0]);
  }
  // std::cout << "1st: " << vecReadings[0] <<
  //             " 2nd: " << vecReadings[1] <<
  //             " 3rd: " << vecReadings[2] <<
  //             " groundColour: " << groundColor << std::endl;
}

/****************************************/
/****************************************/


/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CPiPuckForaging, "pipuck_foraging_controller")
