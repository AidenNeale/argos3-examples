/* Include the controller definition */
#include "pipuck_foraging.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>

CPiPuckForaging::SZoneData::SZoneData() :
  hasZone(false),
  ZoneID(-1),
  ZoneLifetime(-1),
  ZoneType(-1) {}

void CPiPuckForaging::SZoneData::Reset() {
  hasZone = false;
  ZoneID = -1;
  ZoneLifetime = -1;
  ZoneType = -1;
}

CPiPuckForaging::SStateData::SStateData() :
  ProbRange(0.0f, 1.0f) {}

void CPiPuckForaging::SStateData::Reset() {
  State = STATE_RANDOM_WALK;
  InNest = true;
}
/****************************************/
/****************************************/

CPiPuckForaging::CPiPuckForaging() :
   pcWheels(NULL),
   pcGround(NULL),
   pcProximity(NULL),
   pcLight(NULL),
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
    pcLight = GetSensor<CCI_LightSensor>("light");

    m_sStateData.Reset();
}

/****************************************/
/****************************************/

void CPiPuckForaging::ControlStep() {
  switch (m_sStateData.State)
  {
  case SStateData::STATE_RETURN_TO_NEST:
    ReturnToNest();
    break;

  case SStateData::STATE_FOLLOW_PHEROMONE:
    FollowPheromoneTrial();
    break;

  case SStateData::STATE_RANDOM_WALK:
    RandomWalk();
    break;
  default:
    break;
  }
}

/****************************************/
/****************************************/

void CPiPuckForaging::ReturnToNest() {
  readGroundColorSensor();
  if (groundColor == CColor::BLUE) {
    m_sZoneData.hasZone = true;
    m_sZoneData.ZoneType = 1;
  }
  else if (groundColor == CColor::RED) {
    m_sZoneData.hasZone = true;
    m_sZoneData.ZoneType = 0;
  }
  else if (groundColor == CColor::ORANGE){
    m_sStateData.State = SStateData::STATE_RANDOM_WALK;
    m_sZoneData.Reset();
  }

  CRadians orientation = getCurrentOrientation();
  CRadians desiredOrientation = getDesiredOrientation();

  CRadians differenceInOrientation = desiredOrientation - orientation;

  std::cout << "My current orientation is: " << orientation <<
            ", My desired orientation is: " << desiredOrientation <<
            ", My difference in orientation is: " << differenceInOrientation << std::endl;


  std::vector<Real> vecReadings;
  Real Readings[8], left, right, totalReadings, workings;

  left = 0.0;
  right = 0.0;
  if (abs(ToDegrees(differenceInOrientation).GetValue() > 0.0)) {
    if (ToDegrees(differenceInOrientation).GetValue() > 0.0) {
      workings = (360 - abs(ToDegrees(differenceInOrientation).GetValue()));
    }
    else {
    workings = - (360 - abs(ToDegrees(differenceInOrientation).GetValue()));
    }
  }
  if (workings > 20.0 && workings < 340.0) {
    left = 0.2;
    right = 0.5;
  }
  else if (workings < -20.0 && workings > -340.0){
    left = 0.5;
    right = 0.2;
  }
  else {
    left = 0.5;
    right = 0.5;
  }



  readProximitySensor(vecReadings, Readings);

  totalReadings = 0;

  for (Real r : Readings) {
    totalReadings += r;
  }
  if (totalReadings < 0.79) {
    std::cout << totalReadings << std::endl;
    obstacleAvoidance(Readings, &left, &right);
  }
  std::cout << "L: " << left << " R: " << right << std::endl;
  pcWheels->SetLinearVelocity(left, right);
}

/****************************************/
/****************************************/

void CPiPuckForaging::FollowPheromoneTrial() {

}

/****************************************/
/****************************************/

void CPiPuckForaging::RandomWalk() {
  readGroundColorSensor();
  if (groundColor == CColor::BLUE) {
    m_sZoneData.hasZone = true;
    m_sZoneData.ZoneType = 1;
    m_sStateData.State = SStateData::STATE_RETURN_TO_NEST;
  }
  else if (groundColor == CColor::RED) {
    m_sZoneData.hasZone = true;
    m_sZoneData.ZoneType = 0;
    m_sStateData.State = SStateData::STATE_RETURN_TO_NEST;
  }
  else if (groundColor == CColor::ORANGE) {
    m_sZoneData.Reset();
  }

  std::vector<Real> vecReadings;
  Real Readings[8], left, right;

  readProximitySensor(vecReadings, Readings);

  obstacleAvoidance(Readings, &left, &right);

  pcWheels->SetLinearVelocity(left, right);
}

/****************************************/
/****************************************/

void CPiPuckForaging::obstacleAvoidance(Real* Readings, Real* left, Real* right) {
  if(Readings[0] < 0.1)
  {
    if(Readings[7] < 0.1)
    {
      *left = -0.2;
      *right = 0.2;
    }
    else
    {
      if(Readings[1] < 0.1)
      {
        if(Readings[2] < 0.1)
        {
          *left = -0.1;
          *right = 0.2;
        }
        else
        {
          *left = -0.1;
          *right = 0.1;
        }
      }
      else
      {
        *left = 0.0;
        *right = 0.2;
      }
    }
  }
  else if(Readings[7] < 0.1)
  {
    if(Readings[6] < 0.1)
    {
      if(Readings[5] < 0.1)
      {
        *left = 0.2;
        *right = -0.1;
      }
      else
      {
        *left = 0.1;
        *right = -0.1;
      }
    }
    else
    {
      *right = 0.0;
      *left = 0.2;
    }
  }
  else
  {
    *left = 0.5;
    *right = 0.5;
  }
}

/****************************************/
/****************************************/

void CPiPuckForaging::readProximitySensor(std::vector<Real> vecReadings, Real* Readings) {
  pcProximity->Visit([&vecReadings, Readings] (const CCI_PiPuckRangefindersSensor::SInterface& s_interface)
  {
    vecReadings.emplace_back(s_interface.Proximity);
    Readings[s_interface.Label] = s_interface.Proximity;
  });
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
