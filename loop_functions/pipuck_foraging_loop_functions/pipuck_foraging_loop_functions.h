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
    virtual void DistributeWaterSource();
    virtual void DistributeFoodSource();
    virtual CColor GetFloorColor(const CVector2& c_position_on_plane);
    virtual void PreStep();

  private:
    CRange<Real> m_cForagingArenaSideX, m_cForagingArenaSideY;
    CFloorEntity* m_pcFloor;
    CRandom::CRNG* m_pcRNG;


    std::vector<CVector2> m_cFoodPos;
    std::vector<CVector2> m_cWaterPos;
    UInt32 numFoodZones, numWaterZones;
    float zoneRadius;
    bool foodLocationDefined, waterLocationDefined;
  };
}

#endif

