#ifndef PIPUCK_FORAGING_LOOP_FUNCTIONS_H
#define PIPUCK_FORAGING_LOOP_FUNCTIONS_H

namespace argos {
   class CEmbodiedEntity;
}

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/plugins/simulator/entities/light_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <chrono>
#include <controllers/pipuck_foraging/pipuck_foraging.h>


namespace argos {

  class CPiPuckForagingLoopFunctions : public CLoopFunctions {

  public:

    struct SZoneData {
      int zoneId;
      int lifetime;
      CVector2 zoneLocation;

      /* Default Constructor */
      SZoneData() {
        zoneId = -1;
        lifetime = -1;
        zoneLocation = CVector2(1000, 1000);
      };

      void Reset();
    };

    struct SVirtualPheromone {
      int id;
      int lifetime;
      int type;
      CVector2 position;

      SVirtualPheromone() {
        id = -1;
        lifetime = -1;
        type = -1;
        position = CVector2(1000, 1000);
      }
      void Reset();
    };

    CPiPuckForagingLoopFunctions();

    virtual ~CPiPuckForagingLoopFunctions() {}

    virtual void Init(TConfigurationNode& t_tree);
    virtual void Reset();
    virtual void Destroy();
    void DistributeWaterSource(int, int);
    void DistributeFoodSource(int, int);
    void DeductZoneLifetime(CColor, int);
    void DrawPheromoneTrials(CVector2, CPiPuckForaging::SZoneData&);
    void DeductPheromoneLifetime();
    void EraseExpiredPheromones();
    void UpdateFloor();
    void ResetFloor();
    CVector2 clampToPixelValue(CVector2 c_position_on_plane);
    virtual CColor GetFloorColor(const CVector2& c_position_on_plane);
    virtual void PreStep();


  private:
    CRange<Real> m_cForagingArenaSideX, m_cForagingArenaSideY;
    CFloorEntity* m_pcFloor;
    CColor * arenaPixelArray;
    CVector3 arenaSize;
    UInt16 arenaFloorX, arenaFloorY;
    CRandom::CRNG* m_pcRNG;
    int pixelsPerMeter;

    std::vector<SVirtualPheromone> m_pheromones;
    std::vector<SZoneData> m_foodZones;
    std::vector<SZoneData> m_waterZones;
    UInt32 numFoodZones, numWaterZones;
    float zoneRadius, zoneLifetime;
    float pheromoneTrialRadius, rateOfEvaporation;
    bool finiteZones, pheromoneTrialEnabled;
    bool foodLocationDefined, waterLocationDefined;

    CColor defaultColor;

  };
}

#endif

