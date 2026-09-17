
#pragma once

namespace planning {
    struct PiecewiseJerkSpeedConfig {
        double accWeight;
        double jerkWeight;
        double dkappaPenaltyWeight;
        double refSWeight;
        double refVWeight;
        PiecewiseJerkSpeedConfig()
            : accWeight(1.0), jerkWeight(10.0), dkappaPenaltyWeight(1000.0), refSWeight(10.0), refVWeight(10.0) {
        }
    };

    struct HybridAStarConfig {
        double xyGridResolution;
        double yawGridResolution;
        int nextNodeNum;
        double stepSize;
        double trajForwardPenalty;
        double trajBackPenalty;
        double trajGearsWitchPenalty;
        double trajSteerPenalty;
        double trajSteerChangePenalty;

        double gridAStarXYResolution;
        double nodeRadius;
        double deltaT;

        PiecewiseJerkSpeedConfig sCurveConfig;
        HybridAStarConfig()
            : xyGridResolution(0.2), yawGridResolution(0.05), nextNodeNum(12), stepSize(0.5), trajForwardPenalty(0.0), trajBackPenalty(0.0), trajGearsWitchPenalty(10.0), trajSteerPenalty(100.0), trajSteerChangePenalty(10.0), gridAStarXYResolution(0.1), nodeRadius(0.3), deltaT(0.2) {
        }
    };

}  // end namespace