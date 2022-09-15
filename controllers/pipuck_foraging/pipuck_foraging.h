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

#include <argos3/core/simulator/space/space.h>

#include <argos3/plugins/robots/generic/control_interface/ci_light_sensor.h>

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

  struct SZoneData {
    bool hasZone;
    int ZoneType; // 0 Food, 1 Water
    int ZoneID;
    int ZoneLifetime;

    SZoneData();
    void Reset();
  };



  /*
  * Contains all the state information about the controller.
  */
  struct SStateData {
    /* The three possible states in which the controller can be */
    enum EState {
        STATE_RETURN_TO_NEST = 0,
        STATE_FOLLOW_PHEROMONE = 1,
        STATE_RANDOM_WALK = 2
    } State;

    /* True when the robot is in the nest */
    bool InNest;

    // /* Initial probability to switch from resting to exploring */
    // Real InitialRestToExploreProb;
    // /* Current probability to switch from resting to exploring */
    // Real RestToExploreProb;
    // /* Initial probability to switch from exploring to resting */
    // Real InitialExploreToRestProb;
    // /* Current probability to switch from exploring to resting */
    // Real ExploreToRestProb;
    /* Used as a range for uniform number generation */
    CRange<Real> ProbRange;
    // /* The increase of ExploreToRestProb due to the food rule */
    // Real FoodRuleExploreToRestDeltaProb;
    // /* The increase of RestToExploreProb due to the food rule */
    // Real FoodRuleRestToExploreDeltaProb;
    // /* The increase of ExploreToRestProb due to the collision rule */
    // Real CollisionRuleExploreToRestDeltaProb;
    // /* The increase of RestToExploreProb due to the social rule */
    // Real SocialRuleRestToExploreDeltaProb;
    // /* The increase of ExploreToRestProb due to the social rule */
    // Real SocialRuleExploreToRestDeltaProb;
    // /* The minimum number of steps in resting state before the robots
    //     starts thinking that it's time to move */
    // size_t MinimumRestingTime;
    // /* The number of steps in resting state */
    // size_t TimeRested;
    // /* The number of exploration steps without finding food after which
    //     a foot-bot starts thinking about going back to the nest */
    // size_t MinimumUnsuccessfulExploreTime;
    // /* The number of exploration steps without finding food */
    // size_t TimeExploringUnsuccessfully;
    // /* If the robots switched to resting as soon as it enters the nest,
    //     there would be overcrowding of robots in the border between the
    //     nest and the rest of the arena. To overcome this issue, the robot
    //     spends some time looking for a place in the nest before finally
    //     settling. The following variable contains the minimum time the
    //     robot must spend in state 'return to nest' looking for a place in
    //     the nest before switching to the resting state. */
    // size_t MinimumSearchForPlaceInNestTime;
    // /* The time spent searching for a place in the nest */
    // size_t TimeSearchingForPlaceInNest;

    SStateData();
    // void Init(TConfigurationNode& t_node);
    void Reset();
  };


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
  virtual void Reset();

  /*
  * Called to cleanup what done by Init() when the experiment finishes.
  * In this example controller there is no need for clean anything up,
  * so the function could have been omitted. It's here just for
  * completeness.
  */
  virtual void Destroy() {}

  void RandomWalk();

  void ReturnToNest();

  void FollowPheromoneTrial();

  void readProximitySensor(std::vector<Real> vecReadings, Real* Readings);

  void readGroundColorSensor();

  void getGroundColorSensor(CColor*);

  void obstacleAvoidance(Real*, Real*, Real*);

  inline CColor getGroundColor() {
    return groundColor;
  }

  inline void setGroundColor(CColor color) {
    groundColor = color;
  }

  inline CVector2 getCurrentPosition() {
    return currentPosition;
  }

  inline void setCurrentPosition(CVector2 currentPos) {
    currentPosition = currentPos;
  }

  inline CRadians getCurrentOrientation() {
    return currentOrientiation;
  }

  inline void setCurrentOrientation(CRadians orientiation) {
    currentOrientiation = orientiation;
  }

  inline CRadians getDesiredOrientation() {
    return orientationToLight;
  }

  inline void setDesiredOrientation(CRadians orientiation) {
    orientationToLight = orientiation;
  }

  inline SStateData getStateData() {
    return m_sStateData;
  }

  inline void setCurrentPosition(SStateData stateData) {
    m_sStateData = stateData;
  }

  /*
  * Returns the food data
  */
  inline SZoneData& GetZoneData() {
    return m_sZoneData;
  }

private:

  /* Pointer to the differential steering actuator */
  CCI_PiPuckDifferentialDriveActuator* pcWheels;
  /* Pointer to the pi-puck proximity sensor */
  CCI_PiPuckRangefindersSensor* pcProximity;
  /* Pointer to the pi-puck ground colour sensor */
  CCI_PiPuckGroundColourSensor* pcGround;
  /* Pointer to the generic robot light sensor */
  CCI_LightSensor* pcLight;

  bool pheromonesOn;

  double lightLevel = 0;

  CColor groundColor;

  CVector2 currentPosition;

  CRadians currentOrientiation;

  CRadians orientationToLight;

  /* Wheel speed. */
  Real m_fWheelVelocity;

  SZoneData m_sZoneData;

  /* The controller state information */
  SStateData m_sStateData;
};

#endif
