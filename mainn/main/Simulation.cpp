// Simulation.cpp
#include "Simulation.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <thread>

Simulation::Simulation(const Road& road,
                       const Vehicle& egoVehicle,
                       const std::vector<Vehicle>& obstacleVehicles,
                       double dt)
    : road_(road),
      egoVehicle_(egoVehicle),
      obstacleVehicles_(obstacleVehicles),
      dt_(dt),
      visualizer_(std::make_unique<Visualizer>(road)) {
    stats_.startTime = std::chrono::high_resolution_clock::now();
    std::cout << "并行计算：EUDM 后台规划已启用（后台 async）\n"; // C++ 已内置后台规划
}

void Simulation::updateObstacleVehicles() {
    for (auto& obs : obstacleVehicles_) {
        if (obs.type == "dynamic") {
            // 重置出界车辆
            if (obs.y >= road_.getRoadLength() - 10 || obs.y < -10) {
                obs.y = 10.0 + static_cast<double>(rand()) / RAND_MAX * 50.0;
            }

            // IDM 跟驰
            std::vector<Vehicle> allVehicles = obstacleVehicles_;
            allVehicles.push_back(egoVehicle_);
            auto leader = findLeadingVehicle(obs, allVehicles);
            double acc = calculateIDM(obs, leader);
            obs.step(acc, 0.0, dt_);
        }
    }
}

std::optional<Vehicle> Simulation::findLeadingVehicle(
    const Vehicle& currentVehicle,
    const std::vector<Vehicle>& allVehicles) const {
    double minDist = std::numeric_limits<double>::infinity();
    std::optional<Vehicle> leader;

    int curLane = road_.getLaneIndex(currentVehicle.x);
    for (const auto& other : allVehicles) {
        if (other.id == currentVehicle.id) continue;
        if (road_.getLaneIndex(other.x) == curLane) {
            double dist = other.y - currentVehicle.y;
            if (dist > 0 && dist < minDist) {
                minDist = dist;
                leader = other;
            }
        }
    }
    return leader;
}

double Simulation::calculateIDM(const Vehicle& ego,
                                const std::optional<Vehicle>& leader) const {
    constexpr double a_max = 1.5;
    constexpr double b_comf = 2.0;
    double v_des = ego.targetSpeed;
    constexpr double T = 1.5;
    constexpr double s0 = 2.0;
    double v_ego = std::max(0.0, ego.vy);

    if (!leader.has_value()) {
        return a_max * (1.0 - std::pow(v_ego / v_des, 4.0));
    }

    const Vehicle& lv = *leader;
    double v_leader = std::max(0.0, lv.vy);
    double delta_v = v_ego - v_leader;
    double s = std::max(0.1, lv.y - ego.y - ego.length);
    double s_star = s0 + std::max(0.0, v_ego * T + (v_ego * delta_v) / (2.0 * std::sqrt(a_max * b_comf)));
    double acc = a_max * (1.0 - std::pow(v_ego / v_des, 4.0) - std::pow(s_star / std::max(s, s0), 2.0));
    return std::clamp(acc, -5.0, a_max);
}

bool Simulation::checkCollision() const {
    for (const auto& obs : obstacleVehicles_) {
        if (egoVehicle_.isColliding(obs)) {
            std::cout << "  与车辆 " << obs.id << " 发生碰撞！\n";
            return true;
        }
    }
    return false;
}

void Simulation::run(int numSteps) {
    std::vector<double> speeds;
    double last_y = egoVehicle_.y;
    int last_lane = road_.getLaneIndex(egoVehicle_.x);

    for (int i = 1; i <= numSteps; ++i) {
        double currentTime = i * dt_;

        // ===== 1. 终止条件 =====
        if (egoVehicle_.y >= road_.getRoadLength() - 50.0) {
            std::cout << "\n✓ 自车接近道路终点，仿真成功完成！\n";
            std::cout << "  行驶距离: " << std::fixed << std::setprecision(1)
                      << egoVehicle_.y << " m\n";
            showStats(speeds, currentTime);
            return;
        }

        // ===== 2. 更新障碍物 =====
        updateObstacleVehicles();

        // ===== 3. 规划 + 控制 =====
        auto [acc, steer] = eudmManager_.plan(egoVehicle_, obstacleVehicles_, road_);

        // ===== 4. 更新自车（关键！更新 lastSteer）=====
        egoVehicle_.step(acc, steer, dt_);
        eudmManager_.getMutableDebugInfo().lastSteer = steer; // ✅ 解决 const 问题

        // ===== 5. 碰撞检查 =====
        if (checkCollision()) {
            stats_.collisions++;
            std::cout << "时间 " << std::fixed << std::setprecision(1)
                      << currentTime << "s: 碰撞发生！仿真结束。\n";
            break;
        }

        // ===== 6. 统计更新 =====
        stats_.totalDistance += (egoVehicle_.y - last_y);
        last_y = egoVehicle_.y;
        speeds.push_back(egoVehicle_.vy);

        int current_lane = road_.getLaneIndex(egoVehicle_.x);
        if (current_lane != last_lane) {
            stats_.laneChanges++;
            last_lane = current_lane;
        }

        // ===== 7. 可视化 =====
        if (enableVisualization_) {
            visualizer_->draw(egoVehicle_, obstacleVehicles_, currentTime, eudmManager_, drawIntervalSec_);
        }
    }

    showStats(speeds, numSteps * dt_);
}

void Simulation::showStats(const std::vector<double>& speeds, double totalTime) const {
    std::cout << "\n========== 仿真统计 ==========\n";
    std::cout << "仿真总时长: " << std::fixed << std::setprecision(1) << totalTime << " s\n";
    std::cout << "总行驶距离: " << stats_.totalDistance << " m\n";
    std::cout << "碰撞次数: " << stats_.collisions << "\n";
    std::cout << "换道次数: " << stats_.laneChanges << "\n";

    if (!speeds.empty()) {
        double mean_v = std::accumulate(speeds.begin(), speeds.end(), 0.0) / speeds.size();
        double max_v = *std::max_element(speeds.begin(), speeds.end());
        double min_v = *std::min_element(speeds.begin(), speeds.end());
        double std_v = 0.0;
        for (double v : speeds) std_v += (v - mean_v) * (v - mean_v);
        std_v = std::sqrt(std_v / speeds.size());

        std::cout << "平均速度: " << mean_v * 3.6 << " km/h\n";
        std::cout << "最高速度: " << max_v * 3.6 << " km/h\n";
        std::cout << "最低速度: " << min_v * 3.6 << " km/h\n";
        std::cout << "速度标准差: " << std::fixed << std::setprecision(2) << std_v << " m/s\n";
    }

    if (totalTime > 0) {
        double lc_rate = static_cast<double>(stats_.laneChanges) / totalTime * 60.0;
        std::cout << "换道频率: " << std::fixed << std::setprecision(2) << lc_rate << " 次/分钟\n";
    }

    std::cout << "================================\n";
}