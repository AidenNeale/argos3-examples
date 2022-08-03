#include "pipuck_foraging_loop_functions.h"
#include <argos3/plugins/robots/pi-puck/simulator/pipuck_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <controllers/pipuck_foraging/pipuck_foraging.h>

namespace argos {

  /****************************************/
  /****************************************/

CPiPuckForagingLoopFunctions::CPiPuckForagingLoopFunctions() :
  m_cForagingArenaSideX(-0.9f, 1.7f),
  m_cForagingArenaSideY(-1.7f, 1.7f),
  m_pcFloor(NULL),
  m_pcRNG(NULL)
  {}

  /****************************************/
  /****************************************/

void CPiPuckForagingLoopFunctions::Init(TConfigurationNode& t_node) {
  TConfigurationNode& tForaging = GetNode(t_node, "foraging");
  /* Get a pointer to the floor entity */
  m_pcFloor = &GetSpace().GetFloorEntity();
  /* Create a new RNG */
  m_pcRNG = CRandom::CreateRNG("argos");
  GetNodeAttribute(tForaging, "foodZones", numFoodZones);
  GetNodeAttribute(tForaging, "waterZones", numWaterZones);
  GetNodeAttribute(tForaging, "zoneRadius", zoneRadius);
  GetNodeAttribute(tForaging, "zoneLifetime", zoneLifetime);

  /* Setup default starting values for food and water position */
  for (int i = 0; i < numFoodZones; i++) {
    m_foodZones.push_back(SZoneData());
    m_foodZones[i].zoneId = i;
    m_foodZones[i].lifetime = zoneLifetime;
  }
  for (int i = 0; i < numWaterZones; i++) {
    m_waterZones.push_back(SZoneData());
    m_waterZones[i].zoneId = i;
    m_waterZones[i].lifetime = zoneLifetime;
  }
  /* Distribute uniformly the items in the environment */
  DistributeFoodSource(0, numFoodZones);
  DistributeWaterSource(0, numWaterZones);
  foodLocationDefined = false;
  waterLocationDefined = false;
}

  /****************************************/
  /****************************************/

void CPiPuckForagingLoopFunctions::DistributeWaterSource(int startIndex, int endIndex) {
  bool locationOverlap, foodOverlapDetected, waterOverlapDetected;
  CVector2 newPos;
  for (int waterIndex = startIndex; waterIndex < endIndex; waterIndex++) {
    locationOverlap = true;
    while (locationOverlap) {
      foodOverlapDetected = false;
      waterOverlapDetected = false;
      newPos = CVector2(m_pcRNG->Uniform(m_cForagingArenaSideX),
                        m_pcRNG->Uniform(m_cForagingArenaSideY));
      /* Loop through previous water zones and check for overlaps */
      for (int oldWaterIndex=0; oldWaterIndex < numWaterZones; oldWaterIndex++) {
        if (oldWaterIndex == waterIndex) {
          // Do nothing, don't check against itself
        }
        if ((newPos - m_waterZones[oldWaterIndex].zoneLocation).Length() <= (2 * zoneRadius)) {
          waterOverlapDetected = true;
          break;
        }
      }
      /* Loop through food zones and check for overlap */
      for (int foodIndex = 0; foodIndex < numFoodZones; foodIndex++) {
        if ((newPos - m_foodZones[foodIndex].zoneLocation).Length() <= (2 * zoneRadius)) {
          foodOverlapDetected = true;
          break;
        }
      }
      /* If no overlaps detected, break from loop */
      if (!foodOverlapDetected && !waterOverlapDetected) {
        locationOverlap = false;
      }
    }
    m_waterZones[waterIndex].zoneLocation.Set(newPos.GetX(), newPos.GetY());
  }
}

  /****************************************/
  /****************************************/

void CPiPuckForagingLoopFunctions::DistributeFoodSource(int startIndex, int endIndex) {
  bool foodOverlapDetected, waterOverlapDetected, locationOverlap;
  CVector2 newPos;
  for (int foodIndex = startIndex; foodIndex < endIndex; foodIndex++) {
    locationOverlap = true;
    while (locationOverlap) {
      foodOverlapDetected = false;
      waterOverlapDetected = false;
      newPos = CVector2(m_pcRNG->Uniform(m_cForagingArenaSideX),
                        m_pcRNG->Uniform(m_cForagingArenaSideY));
      /* Loop through previous food zones and check for overlaps */
      for (int oldFoodIndex=0; oldFoodIndex < numFoodZones; oldFoodIndex++) {
        if (oldFoodIndex == foodIndex) {
          // Do nothing, don't check against itself
        }
        else if ((newPos - m_foodZones[oldFoodIndex].zoneLocation).Length() <= (2 * zoneRadius)) {
          foodOverlapDetected = true;
          break;
        }
      }

      /* Loop through water zones and check for overlaps */
      for (int waterIndex=0; waterIndex< numWaterZones; waterIndex++) {
        if ((newPos - m_waterZones[waterIndex].zoneLocation).Length() <= (2 * zoneRadius)) {
          waterOverlapDetected = true;
          break;
        }
      }
      /* If no overlaps detected, break from loop */
      if (!waterOverlapDetected && !foodOverlapDetected) {
        locationOverlap = false;
      }
    }
    m_foodZones[foodIndex].zoneLocation.Set(newPos.GetX(), newPos.GetY());
  }
}

  /****************************************/
  /****************************************/

  // void setNewZone(int index, str typeOfZone) {

  // }

  /****************************************/
  /****************************************/

void CPiPuckForagingLoopFunctions::Reset() {
  for (int i = 0; i < numFoodZones; i++) {
    m_foodZones[i].zoneLocation.Set(1000, 1000);
  }
  for (int i = 0; i < numWaterZones; i++) {
    m_waterZones[i].zoneLocation.Set(1000, 1000);
  }
  DistributeFoodSource(0, numFoodZones);
  DistributeWaterSource(0, numWaterZones);
}

  /****************************************/
  /****************************************/

void CPiPuckForagingLoopFunctions::Destroy() {

}

  /****************************************/
  /****************************************/

CColor CPiPuckForagingLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) {
  if(c_position_on_plane.GetX() < -1.5f) {
    return CColor::ORANGE;
  }
  for(UInt32 i = 0; i < m_foodZones.size(); i++) {
    if((c_position_on_plane - m_foodZones[i].zoneLocation).Length() < zoneRadius) {
      return CColor::RED;
    }
  }
  for(UInt32 j = 0; j < m_waterZones.size(); j++) {
    if((c_position_on_plane - m_waterZones[j].zoneLocation).Length() < zoneRadius) {
      return CColor::BLUE;
    }
  }
  return CColor::GREEN;
}

  /****************************************/
  /****************************************/

void CPiPuckForagingLoopFunctions::PreStep() {
  CSpace::TMapPerType& m_cPipucks = GetSpace().GetEntitiesByType("pipuck");
  for(CSpace::TMapPerType::iterator it = m_cPipucks.begin(); it != m_cPipucks.end(); ++it) {
      /* Get handle to foot-bot entity and controller */
      CPiPuckEntity& cPipuck = *any_cast<CPiPuckEntity*>(it->second);
      CPiPuckForaging& cController = dynamic_cast<CPiPuckForaging&>(cPipuck.GetControllableEntity().GetController());

      std::cout << cController.getGroundColor() << std::endl;
  }
}

  /****************************************/
  /****************************************/

  REGISTER_LOOP_FUNCTIONS(CPiPuckForagingLoopFunctions, "pipuck_foraging_loop_functions");

}
