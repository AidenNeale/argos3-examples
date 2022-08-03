#ifndef PIPUCK_FORAGING_LOOP_FUNCTIONS_H
#define PIPUCK_FORAGING_LOOP_FUNCTIONS_H

namespace argos {
   class CEmbodiedEntity;
}

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>


namespace argos {

  class CPiPuckForagingLoopFunctions : public CLoopFunctions {

  public:

    CPiPuckForagingLoopFunctions();

    virtual ~CPiPuckForagingLoopFunctions() {}

    virtual void Init(TConfigurationNode& t_tree);
    virtual void Reset();
    virtual void Destroy();
    virtual void DistributeWaterSource(int, int);
    virtual void DistributeFoodSource(int, int);
    virtual CColor GetFloorColor(const CVector2& c_position_on_plane);
    virtual void PreStep();

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

  private:
    CRange<Real> m_cForagingArenaSideX, m_cForagingArenaSideY;
    CFloorEntity* m_pcFloor;
    CRandom::CRNG* m_pcRNG;

    std::vector<SZoneData> m_foodZones;
    std::vector<SZoneData> m_waterZones;
    UInt32 numFoodZones, numWaterZones;
    float zoneRadius, zoneLifetime;
    bool foodLocationDefined, waterLocationDefined;
  };
}

#endif

