#include "ForwardSimulator.h"
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <algorithm>
#include <limits>
#include <tuple>

ForwardSimulator::ForwardSimulator() {
    // 初始化RSS配置
    rssConfig.response_time = 0.3;
    rssConfig.lon_acc_max = 2.0;
    rssConfig.lon_brake_min = -3.0;
    rssConfig.lon_brake_max = -6.0;
    rssConfig.lat_acc_max = 2.0;
    rssConfig.lat_brake_min = -3.0;
    rssConfig.lat_brake_max = -5.0;
}

std::tuple<std::vector<State>, std::vector<State>, bool> 
ForwardSimulator::simulateLayer(const Action& action, const Vehicle& egoVehicle, 
                               const std::vector<Vehicle>& obstacleVehicles, const Road& road) {
    // 复制车辆用于仿真
    Vehicle simEgo = copyVehicle(egoVehicle);
    std::vector<Vehicle> simObs;
    for (const auto& obs : obstacleVehicles) {
        simObs.push_back(copyVehicle(obs));
    }

    // 计算仿真步数
    int numSteps = std::max(1, static_cast<int>(std::ceil(action.t / simDt)));
    
    std::vector<State> egoLayerTraj;
    std::vector<State> obsLayerTrajs;
    bool isFeasible = true;

    // 检查换道安全性
    if (action.lat != "LK") {
        if (!checkLaneChangeSafety(action, simEgo, simObs, road)) {
            isFeasible = false;
            return std::make_tuple(egoLayerTraj, obsLayerTrajs, isFeasible);
        }
    }

    for (int i = 0; i < numSteps; ++i) {
        // 获取控制输入
        auto [acc, steer] = actionToControl(action, simEgo, road, simObs);
        
        // 执行仿真步
        simEgo.step(acc, steer, simDt);
        
        // 记录自车轨迹点
        State trajPoint = {simEgo.x, simEgo.y, simEgo.vx, simEgo.vy, simEgo.theta};
        egoLayerTraj.push_back(trajPoint);

        // 更新障碍物车辆状态
        std::vector<State> obsStates;
        std::vector<Vehicle> allVehicles = simObs;
        allVehicles.push_back(simEgo);  // 添加自车到车辆列表
        
        for (auto& simObsVehicle : simObs) {
            if (simObsVehicle.type == "dynamic") {
                auto leader = findLeadingVehicle(simObsVehicle, road, allVehicles);
                double obs_acc = calculateIDM(simObsVehicle, leader);
                simObsVehicle.step(obs_acc, 0.0, simDt);
            }
            
            State obsState = {
                static_cast<double>(simObsVehicle.id),
                simObsVehicle.x, 
                simObsVehicle.y, 
                simObsVehicle.vx, 
                simObsVehicle.vy, 
                simObsVehicle.theta
            };
            obsStates.push_back(obsState);
        }

        // 将当前步的障碍物状态添加到轨迹中
        obsLayerTrajs.insert(obsLayerTrajs.end(), obsStates.begin(), obsStates.end());

        // 每步检查RSS安全
        if (!checkRSSSafety(simEgo, simObs, road)) {
            isFeasible = false;
            break;
        }
    }

    return std::make_tuple(egoLayerTraj, obsLayerTrajs, isFeasible);
}

bool ForwardSimulator::checkRSSSafety(const Vehicle& ego, const std::vector<Vehicle>& obs, const Road& road) {
    for (const auto& obsVehicle : obs) {
        // 检查碰撞
        if (ego.isColliding(obsVehicle)) {
            return false;
        }

        // 纵向距离检查
        double dist = obsVehicle.y - ego.y;
        if (dist > 0) {  // 前车
            double rel_v = ego.vy - obsVehicle.vy;
            if (rel_v > 0) {
                double min_dist = rssConfig.response_time * ego.vy + 
                                 (ego.vy * ego.vy) / (2 * rssConfig.lon_brake_min) - 
                                 (obsVehicle.vy * obsVehicle.vy) / (2 * rssConfig.lon_acc_max);
                if (dist < min_dist) {
                    return false;
                }
            }
        } else if (dist < 0) {  // 后车
            double rel_v = obsVehicle.vy - ego.vy;
            if (rel_v > 0) {
                double min_dist = rssConfig.response_time * obsVehicle.vy + 
                                 (obsVehicle.vy * obsVehicle.vy) / (2 * rssConfig.lon_brake_min) - 
                                 (ego.vy * ego.vy) / (2 * rssConfig.lon_acc_max);
                if (-dist < min_dist) {
                    return false;
                }
            }
        }
    }
    return true;
}

bool ForwardSimulator::checkLaneChangeSafety(const Action& action, const Vehicle& ego, 
                                            const std::vector<Vehicle>& obstacles, const Road& road) {
    int currentLane = road.getLaneIndex(ego.x);

    // 确定目标车道
    int targetLane = currentLane;
    if (action.lat == "LCL") {
        targetLane = std::max(1, currentLane - 1);
    } else if (action.lat == "LCR") {
        targetLane = std::min(road.getNumLanes(), currentLane + 1);
    }

    if (targetLane == currentLane) {
        return true;
    }

    // 检查目标车道安全
    for (const auto& obs : obstacles) {
        int obsLane = road.getLaneIndex(obs.x);
        
        if (obsLane == targetLane) {
            double long_dist = obs.y - ego.y;
            
            // 前方车辆检查
            if (long_dist > 0) {
                double rel_v = ego.vy - obs.vy;
                if (rel_v > 0.1) {
                    double safe_dist_front = calculateRSSSafeLongitudinalDistance(ego.vy, obs.vy, true);
                    if (long_dist < safe_dist_front) {
                        return false;
                    }
                }
            } else {  // 后方车辆检查
                double rel_v = obs.vy - ego.vy;
                if (rel_v > 0.1) {
                    double safe_dist_rear = calculateRSSSafeLongitudinalDistance(obs.vy, ego.vy, false);
                    if (-long_dist < safe_dist_rear) {
                        return false;
                    }
                }
            }
        }
    }
    
    return true;
}

double ForwardSimulator::calculateRSSSafeLongitudinalDistance(double v_rear, double v_front, bool is_front) {
    double rt = rssConfig.response_time;
    double a_max = rssConfig.lon_acc_max;
    double b_min = rssConfig.lon_brake_min;
    double b_max = rssConfig.lon_brake_max;
    
    v_rear = std::max(0.0, v_rear);
    v_front = std::max(0.0, v_front);

    if (is_front) {
        // 前方安全距离
        double dist_response = v_rear * rt;
        double v_rear_after = v_rear;
        double dist_rear_brake = (v_rear_after * v_rear_after) / (2 * b_min);
        double dist_front_accel = ((v_front + a_max * rt) * (v_front + a_max * rt)) / (2 * a_max);
        
        return dist_response + dist_rear_brake + dist_front_accel;
    } else {
        // 后方安全距离
        double dist_response = v_rear * rt;
        double v_rear_after = v_rear + a_max * rt;
        double dist_rear_brake = (v_rear_after * v_rear_after) / (2 * b_min);
        double dist_ego_brake = (v_front * v_front) / (2 * b_max);
        
        return dist_response + dist_rear_brake - dist_ego_brake;
    }
}

Vehicle ForwardSimulator::copyVehicle(const Vehicle& oldVehicle) const {
    Vehicle newVehicle(oldVehicle.id, oldVehicle.x, oldVehicle.y, 
                      oldVehicle.vy, oldVehicle.vx, oldVehicle.type);
    newVehicle.targetSpeed = oldVehicle.targetSpeed;
    newVehicle.theta = oldVehicle.theta;
    newVehicle.vx = oldVehicle.vx;
    newVehicle.vy = oldVehicle.vy;
    
    return newVehicle;
}

std::pair<double, double> ForwardSimulator::actionToControl(const Action& action, const Vehicle& ego, 
                                                          const Road& road, const std::vector<Vehicle>& obstacles) {
    // 构建所有车辆列表
    std::vector<Vehicle> allVehicles = obstacles;
    allVehicles.push_back(ego);

    // 查找前车
    auto leader = findLeadingVehicle(ego, road, allVehicles);
    double base_acc = calculateIDM(ego, leader);

    // 纵向控制
    double acc = base_acc;
    if (action.lon == "ACC") {
        acc = std::min(2.0, base_acc + 0.3);
    } else if (action.lon == "DEC") {
        acc = std::max(-5.0, base_acc - 0.8);
    }

    // 横向控制
    int currentLaneIndex = road.getLaneIndex(ego.x);
    int targetLaneIndex = currentLaneIndex;
    
    if (action.lat == "LCL") {
        targetLaneIndex = std::max(1, currentLaneIndex - 1);
    } else if (action.lat == "LCR") {
        targetLaneIndex = std::min(road.getNumLanes(), currentLaneIndex + 1);
    }

    // Pure Pursuit控制
    double lookahead_dist = std::max(5.0, std::min(15.0, std::max(1.0, ego.vy) * 0.8));
    double target_point_x = road.getLaneCenter(targetLaneIndex);
    double target_point_y = ego.y + lookahead_dist;
    
    double dx = target_point_x - ego.x;
    double dy = target_point_y - ego.y;
    double alpha = std::atan2(dx, dy) - ego.theta;
    
    double steer = std::atan2(2 * ego.L * std::sin(alpha), lookahead_dist);
    double max_steer = M_PI / 6.0;
    steer = std::max(-max_steer, std::min(max_steer, steer));

    return std::make_pair(acc, steer);
}

std::vector<Vehicle> ForwardSimulator::findLeadingVehicle(const Vehicle& current, const Road& road, 
                                                        const std::vector<Vehicle>& allVehicles) {
    double minDist = std::numeric_limits<double>::infinity();
    std::vector<Vehicle> leader;
    
    int currentLane = road.getLaneIndex(current.x);
    
    for (const auto& other : allVehicles) {
        if (other.id == current.id) continue;
        
        int otherLane = road.getLaneIndex(other.x);
        if (otherLane == currentLane) {
            double dist = other.y - current.y;
            if (dist > 0 && dist < minDist) {
                minDist = dist;
                leader.clear();
                leader.push_back(other);
            }
        }
    }
    
    return leader;
}

double ForwardSimulator::calculateIDM(const Vehicle& ego, const std::vector<Vehicle>& leader) const {
    double a_max = 1.2;
    double b_comf = 1.5;
    double v_des = std::max(1.0, ego.targetSpeed);
    double T = 1.5;
    double s0 = 2.0;
    
    double v_ego = calculateVehicleSpeed(ego);

    if (leader.empty()) {
        return a_max * (1 - std::pow(v_ego / v_des, 4));
    } else {
        double v_leader = calculateVehicleSpeed(leader[0]);
        double delta_v = v_ego - v_leader;
        double s = std::max(0.1, leader[0].y - ego.y - ego.length);
        double s_star = s0 + std::max(0.0, v_ego * T + (v_ego * delta_v) / (2 * std::sqrt(a_max * b_comf)));
        double acc = a_max * (1 - std::pow(v_ego / v_des, 4) - std::pow(s_star / s, 2));
        return std::max(-5.0, std::min(a_max, acc));
    }
}

double ForwardSimulator::calculateVehicleSpeed(const Vehicle& vehicle) const {
    return std::max(0.0, std::sqrt(vehicle.vx * vehicle.vx + vehicle.vy * vehicle.vy));
}