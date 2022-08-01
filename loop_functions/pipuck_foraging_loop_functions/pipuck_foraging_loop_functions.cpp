#include "pipuck_foraging_loop_functions.h"
#include <argos3/plugins/robots/pi-puck/simulator/pipuck_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>

namespace argos {

  /****************************************/
  /****************************************/

CPiPuckForagingLoopFunctions::CPiPuckForagingLoopFunctions() :
  m_cForagingArenaSideX(-0.9f, 1.7f),
  m_cForagingArenaSideY(-1.7f, 1.7f),
  m_pcFloor(NULL),
  m_pcRNG(NULL)
  // m_unCollectedFood(0),
  // m_nEnergy(0),
  // m_unEnergyPerFoodItem(1),
  // m_unEnergyPerWalkingRobot(1)
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

  /* Setup default starting values for food and water position */
  for (int i = 0; i < numFoodZones; i++) {
    m_cFoodPos.push_back(CVector2(1000, 1000));
  }
  for (int i = 0; i < numWaterZones; i++) {
    m_cWaterPos.push_back(CVector2(1000, 1000));
  }
  /* Distribute uniformly the items in the environment */
  DistributeFoodSource();
  DistributeWaterSource();
  foodLocationDefined = false;
  waterLocationDefined = false;


}

  /****************************************/
  /****************************************/

void CPiPuckForagingLoopFunctions::DistributeWaterSource() {
  bool locationOverlap, waterOverlapDetected;
  CVector2 newPos;
  for (int waterIndex = 0; waterIndex < numWaterZones; waterIndex++) {
    locationOverlap = true;
    while (locationOverlap) {
      waterOverlapDetected = false;
      newPos = CVector2(m_pcRNG->Uniform(m_cForagingArenaSideX),
                        m_pcRNG->Uniform(m_cForagingArenaSideY));
      /* Loop through previous water zones and check for overlaps */
      for (int oldWaterIndex=0; oldWaterIndex < waterIndex; oldWaterIndex++) {
        if ((newPos - m_cWaterPos[oldWaterIndex]).Length() <= (2 * zoneRadius)) {
          waterOverlapDetected = true;
          break;
        }
      }
      /* Loop through food zones and check for overlap */
      for (int foodIndex = 0; foodIndex < numFoodZones; foodIndex++) {
        if ((newPos - m_cFoodPos[foodIndex]).Length() <= (2 * zoneRadius)) {
          break;
        }
        if ((foodIndex == (numFoodZones - 1)) && !waterOverlapDetected) {
          locationOverlap = false;
        }
      }
      if (numFoodZones == 0 && !waterOverlapDetected) {
        locationOverlap = false;
      }
    }
    m_cWaterPos[waterIndex].Set(newPos.GetX(), newPos.GetY());

  }
}

  /****************************************/
  /****************************************/

void CPiPuckForagingLoopFunctions::DistributeFoodSource() {
  bool foodOverlapDetected, locationOverlap;
  CVector2 newPos;
  for (int foodIndex = 0; foodIndex < numFoodZones; foodIndex++) {
    locationOverlap = true;
    while (locationOverlap) {
    foodOverlapDetected = false;
      newPos = CVector2(m_pcRNG->Uniform(m_cForagingArenaSideX),
                        m_pcRNG->Uniform(m_cForagingArenaSideY));
      /* Loop through previous food zones and check for overlaps */
      for (int oldFoodIndex=0; oldFoodIndex < foodIndex; oldFoodIndex++) {
        if ((newPos - m_cFoodPos[oldFoodIndex]).Length() <= (2 * zoneRadius)) {
          foodOverlapDetected = true;
          break;
        }
      }

      /* Loop through water zones and check for overlaps */
      for (int waterIndex=0; waterIndex< numWaterZones; waterIndex++) {
        if ((newPos - m_cWaterPos[waterIndex]).Length() <= (2 * zoneRadius)) {
          break;
        }
        if ((waterIndex == (numWaterZones - 1)) && !foodOverlapDetected) {
          locationOverlap = false;
        }
      }
      if (numWaterZones == 0 && !foodOverlapDetected) {
        locationOverlap = false;
      }
    }
    m_cFoodPos[foodIndex].Set(newPos.GetX(), newPos.GetY());
  }
}

  /****************************************/
  /****************************************/

void CPiPuckForagingLoopFunctions::Reset() {
  for (int i = 0; i < numFoodZones; i++) {
    m_cFoodPos[i].Set(1000, 1000);
  }
  for (int i = 0; i < numWaterZones; i++) {
    m_cWaterPos[i].Set(1000, 1000);
  }
  DistributeFoodSource();
  DistributeWaterSource();
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
  for(UInt32 i = 0; i < m_cFoodPos.size(); i++) {
    if((c_position_on_plane - m_cFoodPos[i]).Length() < zoneRadius) {
      return CColor::RED;
    }
  }
  for(UInt32 j = 0; j < m_cWaterPos.size(); j++) {
    if((c_position_on_plane - m_cWaterPos[j]).Length() < zoneRadius) {
      return CColor::BLUE;
    }
  }
  return CColor::GREEN;
}

  /****************************************/
  /****************************************/

void CPiPuckForagingLoopFunctions::PreStep() {

}

  /****************************************/
  /****************************************/

  REGISTER_LOOP_FUNCTIONS(CPiPuckForagingLoopFunctions, "pipuck_foraging_loop_functions");

}
