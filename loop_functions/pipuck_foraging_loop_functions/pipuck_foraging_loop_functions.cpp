#include "pipuck_foraging_loop_functions.h"
#include <argos3/plugins/robots/pi-puck/simulator/pipuck_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>

namespace argos {

  /****************************************/
  /****************************************

    TODO:
    [X] Different colour pheromones for food/water
    [X] Different lifetime pheromones depending on current food/water zone lifetime
    [X] Different Alpha levels - strongest at zone, weaker as moves away
    [X] Robots control turning pheromones on/off
    Use of Light Sensor to return pipucks to nest
    Optimisation of drawing getFloorColor()
      Turn loop through floor into array of [widthxheight] and map each index to a CColor

   ****************************************/
  /****************************************/

CPiPuckForagingLoopFunctions::CPiPuckForagingLoopFunctions() :
  m_cForagingArenaSideX(-0.8f, 1.7f),
  m_cForagingArenaSideY(-1.7f, 1.7f),
  m_pcFloor(NULL),
  m_pcRNG(NULL),
  defaultColor(CColor())
  {}

  /****************************************/
  /****************************************/

void CPiPuckForagingLoopFunctions::Init(TConfigurationNode& t_node) {
  TConfigurationNode& tForaging = GetNode(t_node, "foraging");
  TConfigurationNode& tStigmergy = GetNode(t_node, "stigmergy");
  /* Get a pointer to the floor entity */
  m_pcFloor = &GetSpace().GetFloorEntity();
  /* Create a new RNG */
  m_pcRNG = CRandom::CreateRNG("argos");

  GetNodeAttribute(tForaging, "foodZones", numFoodZones);
  GetNodeAttribute(tForaging, "waterZones", numWaterZones);
  GetNodeAttribute(tForaging, "finiteZones", finiteZones);
  GetNodeAttribute(tForaging, "zoneRadius", zoneRadius);
  GetNodeAttribute(tForaging, "zoneLifetime", zoneLifetime);

  GetNodeAttribute(tStigmergy, "enabled", pheromoneTrialEnabled);
  GetNodeAttribute(tStigmergy, "pheromoneRadius", pheromoneTrialRadius);
  GetNodeAttribute(tStigmergy, "rateOfEvaporation", rateOfEvaporation);

  /* Setup default starting values for food and water position */
  for (int i = 0; i < numFoodZones; i++) {
    m_foodZones.push_back(SZoneData());
    m_foodZones[i].zoneId = i;
  }
  for (int i = 0; i < numWaterZones; i++) {
    m_waterZones.push_back(SZoneData());
    m_waterZones[i].zoneId = i;
  }
  /* Distribute uniformly the items in the environment */
  DistributeFoodSource(0, numFoodZones);
  DistributeWaterSource(0, numWaterZones);
  foodLocationDefined = false;
  waterLocationDefined = false;

}

  /****************************************/
  /****************************************/

/*
Allocates new non-overlapping locations for a water zone

Parameters:
-----------
int startIndex:
  The beginning index of the m_waterZones that requires changing
int endIndex:
  The final index of the m_waterZones that requires changing
*/
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
    m_waterZones[waterIndex].lifetime = zoneLifetime;
    m_waterZones[waterIndex].zoneLocation.Set(newPos.GetX(), newPos.GetY());
  }
}

  /****************************************/
  /****************************************/

/*
Allocates new non-overlapping locations for a food zone

Parameters:
-----------
int startIndex:
  The beginning index of the m_foodZones that requires changing
int endIndex:
  The final index of the m_foodZones that requires changing
*/
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
    m_foodZones[foodIndex].lifetime = zoneLifetime;
    m_foodZones[foodIndex].zoneLocation.Set(newPos.GetX(), newPos.GetY());
  }
}

  /****************************************/
  /****************************************/

  void CPiPuckForagingLoopFunctions::DeductZoneLifetime(CColor floorColor, int index) {
    if (floorColor == CColor::BLUE) {
      m_waterZones[index].lifetime -= 1;
      if (m_waterZones[index].lifetime < 1) {
        DistributeWaterSource(index, index + 1);
      }
    }
    else if (floorColor == CColor::RED) {
      m_foodZones[index].lifetime -= 1;
      if (m_foodZones[index].lifetime < 1) {
        DistributeFoodSource(index, index + 1);
      }
    }
    else {
      std::cout << "An error has occured" << std::endl;
    }
  }

  /****************************************/
  /****************************************/

void CPiPuckForagingLoopFunctions::Reset() {
  for (int i = 0; i < numFoodZones; i++) {
    m_foodZones[i].zoneLocation.Set(1000, 1000);
  }
  for (int i = 0; i < numWaterZones; i++) {
    m_waterZones[i].zoneLocation.Set(1000, 1000);
  }
  m_pheromones.clear();
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
  /* Returns Orange if the floor position is within the "nest" */

  // std::cout << c_position_on_plane <<std::endl;
  if(c_position_on_plane.GetX() < -1.5f) {
    return CColor::ORANGE;
  }
  /* Returns Red if the floor position is a food zone*/
  for(UInt32 i = 0; i < m_foodZones.size(); i++) {
    if((c_position_on_plane - m_foodZones[i].zoneLocation).Length() < zoneRadius) {
      return CColor::RED;
    }
  }
  /* Returns Blue if the floor position is a water zone */
  for(UInt32 j = 0; j < m_waterZones.size(); j++) {
    if((c_position_on_plane - m_waterZones[j].zoneLocation).Length() < zoneRadius) {
      return CColor::BLUE;
    }
  }
  /* Returns Magenta if the floor position is a pheromone */
  for (UInt32 k = 0; k < m_pheromones.size(); k++){
    if ((c_position_on_plane - m_pheromones[k].position).Length() < pheromoneTrialRadius) {
      if (m_pheromones[k].lifetime > 0) {
        if (m_pheromones[k].type == 0) {
          defaultColor.SetRed(255);
          defaultColor.SetGreen(0 + m_pheromones[k].lifetime);
          defaultColor.SetBlue(255);
        }
        else {
          defaultColor.SetRed(0 + m_pheromones[k].lifetime);
          defaultColor.SetGreen(200);
          defaultColor.SetBlue(255);
        }
        return defaultColor;
      }
    }
  }
  /* Returns Green otherwise (the wild) */
  return CColor::GREEN;
}

  /****************************************/
  /****************************************/

void CPiPuckForagingLoopFunctions::DrawPheromoneTrials(CVector2 cPos, CPiPuckForaging::SZoneData& sZoneData) {
  UInt32 newID = m_pheromones.size();
  m_pheromones.push_back(SVirtualPheromone());
  m_pheromones[newID].id = newID;
  m_pheromones[newID].lifetime = int((sZoneData.ZoneLifetime/zoneLifetime) * 255);
  m_pheromones[newID].position.Set(cPos.GetX(), cPos.GetY());

  if (sZoneData.ZoneType == 0) {
    m_pheromones[newID].type = 0;
  }
  else if (sZoneData.ZoneType == 1) {
    m_pheromones[newID].type = 1;
  }
}

  /****************************************/
  /****************************************/

void CPiPuckForagingLoopFunctions::DeductPheromoneLifetime() {
  for (UInt32 index = 0; index < m_pheromones.size(); index++) {
    m_pheromones[index].lifetime -= UInt8(rateOfEvaporation * 100);
  }
}

  /****************************************/
  /****************************************/

static bool IsLessThanZero(CPiPuckForagingLoopFunctions::SVirtualPheromone p) {
  if (p.lifetime < 0) {
    return true;
  } else {
    return false;
  }
}

void CPiPuckForagingLoopFunctions::EraseExpiredPheromones() {
  /* Had issues with latency during removal operations. When looking at differences
     between List and Vectors for removal operations, the difference seemed minimal
     however by utilising remove_if, items could be removed in a single pass

     Optised way to remove items in a pass: https://en.wikipedia.org/wiki/Erase%E2%80%93remove_idiom */
  m_pheromones.erase(std::remove_if(m_pheromones.begin(), m_pheromones.end(), IsLessThanZero), m_pheromones.end());
}

  /****************************************/
  /****************************************/

void CPiPuckForagingLoopFunctions::PreStep() {
  // Record start time
  // auto start = std::chrono::high_resolution_clock::now();

  CSpace::TMapPerType& m_cPipucks = GetSpace().GetEntitiesByType("pipuck");
  UInt32 SimulationTime = GetSpace().GetSimulationClock();
  for(CSpace::TMapPerType::iterator it = m_cPipucks.begin(); it != m_cPipucks.end(); ++it) {
    /* Get handle to pipuck entity and controller */
    CPiPuckEntity& cPipuck = *any_cast<CPiPuckEntity*>(it->second);
    CPiPuckForaging& cController = dynamic_cast<CPiPuckForaging&>(cPipuck.GetControllableEntity().GetController());
    CColor floorColor = cController.getGroundColor();
    CPiPuckForaging::SZoneData& SZoneData = cController.GetZoneData();



    /* Get the position of the pipuck on the ground as a CVector2 */
    CVector2 cPos;
    cPos.Set(cPipuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
              cPipuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

    if ((floorColor == CColor::GREEN ||
          (floorColor.GetRed() == 255 && floorColor.GetGreen() >= 0 && floorColor.GetBlue() == 255) ||
          (floorColor.GetRed() >= 0 && floorColor.GetGreen() == 200 && floorColor.GetBlue() == 255))
          && pheromoneTrialEnabled && SZoneData.hasZone) {
      // Robot is in the wild, pheromone trials can be drawn
      DrawPheromoneTrials(cPos, SZoneData);
    }
    else if (floorColor == CColor::ORANGE) {
      // Robot in Nest
    }
    else if (floorColor == CColor::BLUE) {
      // Robot is in a water zone
      for (int waterIndex = 0; waterIndex < m_waterZones.size(); waterIndex++) {
        // Loop through each zone to determine which zone the robot is within
        if ((cPos - m_waterZones[waterIndex].zoneLocation).Length() < zoneRadius) {
          if (finiteZones) {
            SZoneData.ZoneLifetime = m_waterZones[waterIndex].lifetime;
            DeductZoneLifetime(floorColor, waterIndex);
          }
        }
      }
    }
    else if (floorColor == CColor::RED) {
      // Robot is in a food zone
      for (int foodIndex = 0; foodIndex < m_foodZones.size(); foodIndex++) {
        // Loop through each zone to determine which zone the robot is within
        if ((cPos - m_foodZones[foodIndex].zoneLocation).Length() < zoneRadius) {
          if (finiteZones) {
            SZoneData.ZoneLifetime = m_foodZones[foodIndex].lifetime;
            DeductZoneLifetime(floorColor, foodIndex);
          }
        }
      }
    }
  }

  if (pheromoneTrialEnabled) {
    if (SimulationTime % 10 == 0) {
      std::cout << m_pheromones.size() << std::endl;
    }
    DeductPheromoneLifetime();
    EraseExpiredPheromones();
  }

  m_pcFloor->SetChanged();

  // Record start time
  // auto finish = std::chrono::high_resolution_clock::now();
  // std::chrono::duration<double> elapsed = finish - start;
  // std::cout << "Elapsed time: " << elapsed.count() << " s\n";
}

  /****************************************/
  /****************************************/

  REGISTER_LOOP_FUNCTIONS(CPiPuckForagingLoopFunctions, "pipuck_foraging_loop_functions");

}