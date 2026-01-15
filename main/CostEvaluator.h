// CostEvaluator.h
#ifndef COST_EVALUATOR_H
#define COST_EVALUATOR_H

#include <vector>
#include <string>
#include <tuple>
#include <cmath>
#include <limits>
#include <algorithm>
#include <stdexcept>
#include "Road.h"
#include "DcpTree.h"  // 确保 Action 来自 DcpTree
#include "State.h"  // 👈 替代原来的 struct State 定义
#include "Constants.h"
// Define state structure (equivalent to MATLAB struct with x, y, vy, etc.)
/*
struct State {
    double x = 0.0;  // lateral position
    double y = 0.0;  // longitudinal position
    double vx = 0.0;
    double vy = 0.0; // longitudinal velocity
    double ax = 0.0;
    double ay = 0.0;
};
*/

using Trajectory = std::vector<State>; // 1×T
using ObstacleTrajectories = std::vector<Trajectory>; // Nobs × T
/*
// Action representation (lat only needed here)
struct Action {
    std::string lat; // "LK", "LCL", "LCR"
    // Could add more fields (e.g., acc, target_speed) later
};
*/
// Output breakdown
struct CostBreakdown {
    double safety = 0.0;
    double efficiency = 0.0;
    double comfort = 0.0;
    double lc = 0.0;
    double centering = 0.0;
};

class CostEvaluator {
private:
    // Weights
    double W_SAFETY = 30.0;
    double W_EFFICIENCY = 40.0;
    double W_COMFORT = 2.0;
    double W_LC = 0.8;
    double W_CENTERING = 0.3;

    // MOBIL parameters
    double politeness = 0.2;
    double mobil_threshold = 0.5;

    // Configurations (struct-like)
    struct Config {
        double desired_speed_tolerance = 1.0;
        double leading_distance_threshold = 120.0;
        double min_distance_ratio = 0.4;
        double ttc_threshold = 3.5;
        double min_lane_change_gap = 7.0;
    } cfg;

    // IDM parameters
    struct IDMParams {
        double kDesiredVelocity = 0.0;
        double kMinimumSpacing = 2.0;
        double kDesiredHeadwayTime = 1.0;
        double kAcceleration = 1.0;
        double kComfortableBrakingDeceleration = 1.67;
        double kHardBrakingDeceleration = 5.0;
        double kExponent = 4.0;
        double kVehicleLength = 4.7;
    } idm_param;

    double simDt = 0.2; // simulation time step

    // Helper methods (private)
    State findLeadingOnLaneIndex(const State& current, const std::vector<State>& obstacles,
                                 const Road& road, int laneIdx) const;
    State findFollowingOnLaneIndex(const State& current, const std::vector<State>& obstacles,
                                   const Road& road, int laneIdx) const;
    double getDesiredAccelerationUsingIdm(const State& rear, const State& front) const;
    std::tuple<double, double, double>
    getMobilAccChangesOnCurrentLane(const State& cur_vehicle,
                                    const State& leading_vehicle,
                                    const State& following_vehicle) const;
    std::tuple<bool, double, double, double>
    getMobilAccChangesOnTargetLane(const State& cur_vehicle,
                                   const State& leading_vehicle,
                                   const State& following_vehicle) const;

public:
    // Constructor
    CostEvaluator() = default;

    // Main evaluate function: returns (total_cost, breakdown)
    std::pair<double, CostBreakdown>
    evaluate(const Action& action,
             const Trajectory& egoLayerTraj,
             const ObstacleTrajectories& obsLayerTrajs,
             const Road& road,
             double desiredSpeed,
             const State& initialEgoState) const;

    // Individual cost computation (public for testing/debug)
    double computeSafetyCost(const Action& action,
                             const Trajectory& egoTraj,
                             const ObstacleTrajectories& obsTrajs,
                             const Road& road) const;

    std::pair<double, double>
    computeEfficiencyCost(const Action& action,
                          const State& initialEgo,
                          const State& finalEgo,
                          const ObstacleTrajectories& obsTrajs,
                          const Road& road,
                          double desiredSpeed) const;

    double computeComfortCost(const State& initialEgo, const Trajectory& egoTraj) const;
    double computeLaneChangeCost(const Action& action,
                                 const State& initialEgo,
                                 const ObstacleTrajectories& obsTrajs,
                                 const Road& road) const;
    double computeCenteringCost(const Action& action,
                                const State& initialEgo,
                                const State& finalEgo,
                                const Road& road) const;

    // Getters/setters (optional)
    double getSimDt() const { return simDt; }
    void setSimDt(double dt) { simDt = dt; }
};

#endif // COST_EVALUATOR_H