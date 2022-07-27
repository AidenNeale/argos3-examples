/* Include the controller definition */
#include "pipuck_obstacleavoidance.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>

/****************************************/
/****************************************/

CPiPuckObstacleAvoidance::CPiPuckObstacleAvoidance() :
   pcWheels(NULL),
   pcProximity(NULL),
   pcGroundColour(NULL),
   m_fWheelVelocity(2.5f) {}

/****************************************/
/****************************************/

void CPiPuckObstacleAvoidance::Init(TConfigurationNode& t_node) {
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
    * file at the <controllers><epuck_obstacleavoidance><actuators> and
    * <controllers><epuck_obstacleavoidance><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
    pcWheels = GetActuator<CCI_PiPuckDifferentialDriveActuator>("pipuck_differential_drive");
    pcProximity = GetSensor<CCI_PiPuckRangefindersSensor>("pipuck_rangefinders");
   /*
    * Parse the configuration file
    *
    * The user defines this part. Here, the algorithm accepts three
    * parameters and it's nice to put them in the config file so we don't
    * have to recompile if we want to try other settings.
    */
   GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
}

/****************************************/
/****************************************/

void CPiPuckObstacleAvoidance::ControlStep() {
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
REGISTER_CONTROLLER(CPiPuckObstacleAvoidance, "pipuck_obstacleavoidance_controller")
