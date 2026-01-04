// CostEvaluator.cpp
#include "CostEvaluator.h"
#include <numeric>
#include <cmath>
#include <algorithm>
#include <limits>
#include <tuple>

// ---------------- Helper Methods ----------------

State CostEvaluator::findLeadingOnLaneIndex(const State& current,
                                            const std::vector<State>& obstacles,
                                            const Road& road,
                                            int laneIdx) const {
    double minDistance = std::numeric_limits<double>::infinity();
    State leadingVehicle{};
    bool found = false;

    for (const auto& veh : obstacles) {
        if (road.getLaneIndex(veh.x) == laneIdx) {
            double d = veh.y - current.y;
            if (d > 0 && d < minDistance) {
                minDistance = d;
                leadingVehicle = veh;
                found = true;
            }
        }
    }
    return found ? leadingVehicle : State{};
}

State CostEvaluator::findFollowingOnLaneIndex(const State& current,
                                              const std::vector<State>& obstacles,
                                              const Road& road,
                                              int laneIdx) const {
    double maxDistance = -std::numeric_limits<double>::infinity();
    State followingVehicle{};
    bool found = false;

    for (const auto& veh : obstacles) {
        if (road.getLaneIndex(veh.x) == laneIdx) {
            double d = veh.y - current.y;
            if (d < 0 && d > maxDistance) {
                maxDistance = d;
                followingVehicle = veh;
                found = true;
            }
        }
    }
    return found ? followingVehicle : State{};
}

double CostEvaluator::getDesiredAccelerationUsingIdm(const State& rear, const State& front) const {
    double v0 = std::max(0.1, idm_param.kDesiredVelocity);
    double v = std::max(0.0, rear.vy);
    double dv = v - std::max(0.0, front.vy);

    double s_alpha = std::max(1e-3, front.y - rear.y - idm_param.kVehicleLength);
    double term_brake = dv / std::max(1e-3, 2.0 * std::sqrt(
        idm_param.kAcceleration * idm_param.kComfortableBrakingDeceleration));

    double s_star = idm_param.kMinimumSpacing + std::max(0.0,
        v * idm_param.kDesiredHeadwayTime + v * term_brake);

    double acc = idm_param.kAcceleration * (
        1.0 - std::pow(v / v0, idm_param.kExponent)
              - std::pow(s_star / s_alpha, 2.0)
    );
    return acc;
}

std::tuple<double, double, double>
CostEvaluator::getMobilAccChangesOnCurrentLane(const State& cur_vehicle,
                                                const State& leading_vehicle,
                                                const State& following_vehicle) const {
    double acc_o = 0.0, acc_o_tilda = 0.0, acc_c = 0.0;

    // Set desired velocity for this evaluation
    IDMParams local_idm = idm_param;
    local_idm.kDesiredVelocity = std::max(0.1, cur_vehicle.vy);

    // Compute acc_c
    if (leading_vehicle.y == 0 && leading_vehicle.vy == 0) { // empty (default-constructed)
        // Use virtual far vehicle
        State virtual_front{0, cur_vehicle.y + 200, 0, cur_vehicle.vy, 0, 0};
        acc_c = getDesiredAccelerationUsingIdm(cur_vehicle, virtual_front);
    } else {
        acc_c = getDesiredAccelerationUsingIdm(cur_vehicle, leading_vehicle);
    }

    // Compute acc_o and acc_o_tilda if following vehicle exists
    if (!(following_vehicle.y == 0 && following_vehicle.vy == 0)) {
        if (leading_vehicle.y == 0 && leading_vehicle.vy == 0) {
            State virtual_front{0, following_vehicle.y + 200, 0, following_vehicle.vy, 0, 0};
            acc_o = getDesiredAccelerationUsingIdm(following_vehicle, cur_vehicle);
            acc_o_tilda = getDesiredAccelerationUsingIdm(following_vehicle, virtual_front);
        } else {
            acc_o = getDesiredAccelerationUsingIdm(following_vehicle, leading_vehicle);
            acc_o_tilda = getDesiredAccelerationUsingIdm(following_vehicle, cur_vehicle);
        }
    }
    return {acc_o, acc_o_tilda, acc_c};
}

std::tuple<bool, double, double, double>
CostEvaluator::getMobilAccChangesOnTargetLane(const State& cur_vehicle,
                                               const State& leading_vehicle,
                                               const State& following_vehicle) const {
    bool is_lc_safe = true;
    double acc_n = 0.0, acc_n_tilda = 0.0, acc_c_tilda = 0.0;

    IDMParams local_idm = idm_param;
    local_idm.kDesiredVelocity = std::max(0.1, cur_vehicle.vy);

    // Compute acc_c_tilda
    if (leading_vehicle.y == 0 && leading_vehicle.vy == 0) {
        State virtual_front{0, cur_vehicle.y + 200, 0, cur_vehicle.vy, 0, 0};
        acc_c_tilda = getDesiredAccelerationUsingIdm(cur_vehicle, virtual_front);
    } else {
        acc_c_tilda = getDesiredAccelerationUsingIdm(cur_vehicle, leading_vehicle);
    }

    // Handle following vehicle
    if (!(following_vehicle.y == 0 && following_vehicle.vy == 0)) {
        if (leading_vehicle.y == 0 && leading_vehicle.vy == 0) {
            State virtual_front{0, following_vehicle.y + 200, 0, following_vehicle.vy, 0, 0};
            acc_n = getDesiredAccelerationUsingIdm(following_vehicle, virtual_front);
        } else {
            acc_n = getDesiredAccelerationUsingIdm(following_vehicle, leading_vehicle);
        }
        acc_n_tilda = getDesiredAccelerationUsingIdm(following_vehicle, cur_vehicle);

        double rear_gap = cur_vehicle.y - following_vehicle.y;
        if (rear_gap < cfg.min_lane_change_gap) {
            is_lc_safe = false;
        }
    }
    return {is_lc_safe, acc_n, acc_n_tilda, acc_c_tilda};
}

// ---------------- Public Cost Functions ----------------

double CostEvaluator::computeSafetyCost(const Action& /*action*/,
                                        const Trajectory& egoTraj,
                                        const ObstacleTrajectories& obsTrajs,
                                        const Road& road) const {
    double cost_safety = 0.0;
    size_t T_ego = egoTraj.size();
    if (obsTrajs.empty() || T_ego == 0) return 0.0;

    size_t N_obs = obsTrajs.size();
    size_t T_obs = obsTrajs.empty() ? 0 : obsTrajs[0].size();
    size_t T = std::min(T_ego, T_obs);
    if (T == 0 || N_obs == 0) return 0.0;

    for (size_t step = 0; step < T; ++step) {
        const State& ego_state = egoTraj[step];
        int ego_lane = road.getLaneIndex(ego_state.x);

        for (size_t i = 0; i < N_obs; ++i) {
            if (step >= obsTrajs[i].size()) continue; // irregular trajectories
            const State& obs_state = obsTrajs[i][step];
            int obs_lane = road.getLaneIndex(obs_state.x);

            if (obs_lane == ego_lane) {
                double dist = obs_state.y - ego_state.y;      // >0: obs ahead
                double rel_vel = ego_state.vy - obs_state.vy; // >0: ego closing
                if (dist > 0 && rel_vel > 0) {
                    double ttc = dist / std::max(rel_vel, 1e-6);
                    if (ttc < cfg.ttc_threshold) {
                        cost_safety += std::pow(cfg.ttc_threshold - ttc, 2);
                    }
                }
            }
        }
    }
    return (T > 0) ? cost_safety / static_cast<double>(T) : 0.0;
}

std::pair<double, double>
CostEvaluator::computeEfficiencyCost(const Action& action,
                                     const State& initialEgo,
                                     const State& finalEgo,
                                     const ObstacleTrajectories& obsTrajs,
                                     const Road& road,
                                     double desiredSpeed) const {
    // Speed cost
    double cost_ego_speed = 0.0;
    if (finalEgo.vy < desiredSpeed - cfg.desired_speed_tolerance) {
        double dv = desiredSpeed - finalEgo.vy;
        cost_ego_speed = 0.1 * dv * dv;
    }

    // Leading vehicle blocking cost
    double cost_leading_blocked = 0.0;

    int curLane = road.getLaneIndex(initialEgo.x);
    int laneIdx = curLane;
    if (action.lat == "LCL") {
        laneIdx = std::max(1, curLane - 1);
    } else if (action.lat == "LCR") {
        laneIdx = std::min(road.getNumLanes(), curLane + 1);
    }

    // Extract t=0 obstacles (first state of each obstacle trajectory)
    std::vector<State> obstacles_t0;
    for (const auto& traj : obsTrajs) {
        if (!traj.empty()) obstacles_t0.push_back(traj[0]);
    }

    State leading = findLeadingOnLaneIndex(initialEgo, obstacles_t0, road, laneIdx);
    if (!(leading.y == 0 && leading.vy == 0)) { // non-empty
        double dist = leading.y - initialEgo.y;
        if (dist > 0 && dist < cfg.leading_distance_threshold) {
            double distance_factor = std::max(cfg.min_distance_ratio, 1.0 - dist / cfg.leading_distance_threshold);
            double speed_deficit = std::max(0.0, desiredSpeed - leading.vy);
            cost_leading_blocked = distance_factor * speed_deficit;
        }
    }
    return {cost_ego_speed, cost_leading_blocked};
}

double CostEvaluator::computeComfortCost(const State& initialEgo, const Trajectory& egoTraj) const {
    size_t T = egoTraj.size();
    if (T == 0) return 0.0;

    std::vector<double> vels;
    vels.reserve(T + 1);
    vels.push_back(initialEgo.vy);
    for (const auto& s : egoTraj) {
        vels.push_back(s.vy);
    }

    std::vector<double> accs;
    accs.reserve(T);
    for (size_t i = 1; i < vels.size(); ++i) {
        accs.push_back((vels[i] - vels[i-1]) / simDt);
    }

    if (accs.size() <= 1) {
        return accs.empty() ? 0.0 : std::abs(accs[0]);
    }

    std::vector<double> jerks;
    jerks.reserve(accs.size() - 1);
    for (size_t i = 1; i < accs.size(); ++i) {
        jerks.push_back((accs[i] - accs[i-1]) / simDt);
    }

    double mean_abs_acc = std::accumulate(accs.begin(), accs.end(), 0.0,
        [](double sum, double a){ return sum + std::abs(a); }) / static_cast<double>(accs.size());

    double mean_abs_jerk = std::accumulate(jerks.begin(), jerks.end(), 0.0,
        [](double sum, double j){ return sum + std::abs(j); }) / static_cast<double>(jerks.size());

    return mean_abs_acc + 0.5 * mean_abs_jerk;
}

double CostEvaluator::computeLaneChangeCost(const Action& action,
                                            const State& initialEgo,
                                            const ObstacleTrajectories& obsTrajs,
                                            const Road& road) const {
    double cost_lc = 0.0;
    if (action.lat == "LK") return cost_lc;

    int curLane = road.getLaneIndex(initialEgo.x);
    int targetLane = curLane;
    if (action.lat == "LCL") {
        targetLane = std::max(1, curLane - 1);
    } else if (action.lat == "LCR") {
        targetLane = std::min(road.getNumLanes(), curLane + 1);
    }
    cost_lc = std::abs(curLane - targetLane);

    // Extract t=0 obstacles
    std::vector<State> obstacles_t0;
    for (const auto& traj : obsTrajs) {
        if (!traj.empty()) obstacles_t0.push_back(traj[0]);
    }

    State lead_cur    = findLeadingOnLaneIndex(initialEgo, obstacles_t0, road, curLane);
    State foll_cur    = findFollowingOnLaneIndex(initialEgo, obstacles_t0, road, curLane);
    State lead_target = findLeadingOnLaneIndex(initialEgo, obstacles_t0, road, targetLane);
    State foll_target = findFollowingOnLaneIndex(initialEgo, obstacles_t0, road, targetLane);

    auto [acc_o, acc_o_tilda, acc_c] = getMobilAccChangesOnCurrentLane(initialEgo, lead_cur, foll_cur);
    auto [is_lc_safe, acc_n, acc_n_tilda, acc_c_tilda] = getMobilAccChangesOnTargetLane(initialEgo, lead_target, foll_target);

    if (!is_lc_safe) {
        return std::numeric_limits<double>::infinity();
    }

    double ego_benefit = acc_c_tilda - acc_c;
    double rear_disadvantage = acc_o - acc_o_tilda;
    double net_gain = ego_benefit - politeness * rear_disadvantage;

    if (net_gain > mobil_threshold) {
        cost_lc *= 0.5;
    } else {
        cost_lc *= 2.0;
    }

    // Gap-based penalty
    if (!(lead_target.y == 0 && lead_target.vy == 0)) {
        double gap_tgt = std::max(1e-3, lead_target.y - initialEgo.y);
        if (gap_tgt < cfg.min_lane_change_gap) {
            cost_lc += std::pow(cfg.min_lane_change_gap - gap_tgt, 2);
        }
    }
    return cost_lc;
}

double CostEvaluator::computeCenteringCost(const Action& action,
                                           const State& initialEgo,
                                           const State& finalEgo,
                                           const Road& road) const {
    int curLane = road.getLaneIndex(initialEgo.x);
    int targetLane = curLane;
    if (action.lat == "LCL") {
        targetLane = std::max(1, curLane - 1);
    } else if (action.lat == "LCR") {
        targetLane = std::min(road.getNumLanes(), curLane + 1);
    }

    double center_x = road.getLaneCenter(targetLane);
    double offset = std::abs(finalEgo.x - center_x);
    double base = offset * offset;

    return (action.lat == "LK") ? base : 0.2 * base;
}

// ---------------- Main evaluate ----------------

std::pair<double, CostBreakdown>
CostEvaluator::evaluate(const Action& action,
                        const Trajectory& egoLayerTraj,
                        const ObstacleTrajectories& obsLayerTrajs,
                        const Road& road,
                        double desiredSpeed,
                        const State& initialEgoState) const {
    CostBreakdown cb{};
    if (egoLayerTraj.empty()) {
        return {std::numeric_limits<double>::infinity(), cb};
    }

    const State& finalEgoState = egoLayerTraj.back();

    // 1) Safety
    cb.safety = computeSafetyCost(action, egoLayerTraj, obsLayerTrajs, road);
    if (cb.safety > 8.0) {
        return {std::numeric_limits<double>::infinity(), cb};
    }

    // 2) Efficiency
    auto [cost_speed, cost_block] = computeEfficiencyCost(
        action, initialEgoState, finalEgoState, obsLayerTrajs, road, desiredSpeed);
    cb.efficiency = cost_speed + cost_block;

    // 3) Comfort
    cb.comfort = computeComfortCost(initialEgoState, egoLayerTraj);

    // 4) Lane Change
    cb.lc = computeLaneChangeCost(action, initialEgoState, obsLayerTrajs, road);

    // 5) Centering
    cb.centering = computeCenteringCost(action, initialEgoState, finalEgoState, road);

    double total = W_SAFETY * cb.safety +
                   W_EFFICIENCY * cb.efficiency +
                   W_COMFORT * cb.comfort +
                   W_LC * cb.lc +
                   W_CENTERING * cb.centering;

    return {total, cb};
}