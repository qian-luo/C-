// Simulation.h
#ifndef SIMULATION_H
#define SIMULATION_H

#include <vector>
#include <chrono>
#include <string>
#include <memory>

#include "Road.h"
#include "Vehicle.h"
#include "EudmManager.h"
#include "Visualizer.h"

struct SimulationStats {
    int collisions = 0;
    int laneChanges = 0;
    double totalDistance = 0.0;
    std::chrono::high_resolution_clock::time_point startTime;
};

class Simulation {
private:
    // ========== 属性（对应 MATLAB properties）==========
    Road road_;
    Vehicle egoVehicle_;
    std::vector<Vehicle> obstacleVehicles_;
    double dt_;

    EudmManager eudmManager_;
    std::unique_ptr<Visualizer> visualizer_; // 拥有 Visualizer 生命周期

    SimulationStats stats_;

    // 可视化控制
    bool enableVisualization_ = true;
    double drawIntervalSec_ = 0.3;
    double lastDrawTime_ = 0.0;

    // ========== 私有辅助方法 ==========
    void updateObstacleVehicles();
    std::optional<Vehicle> findLeadingVehicle(const Vehicle& currentVehicle,
                                              const std::vector<Vehicle>& allVehicles) const;
    double calculateIDM(const Vehicle& ego, const std::optional<Vehicle>& leader) const;
    bool checkCollision() const;

public:
    // 构造函数
    Simulation(const Road& road,
               const Vehicle& egoVehicle,
               const std::vector<Vehicle>& obstacleVehicles,
               double dt);

    // 主仿真接口
    void run(int numSteps);

    // 统计输出
    void showStats(const std::vector<double>& speeds, double totalTime) const;

    // Getter（调试用）
    const Road& getRoad() const { return road_; }
    const Vehicle& getEgoVehicle() const { return egoVehicle_; }
    const std::vector<Vehicle>& getObstacleVehicles() const { return obstacleVehicles_; }
};

#endif // SIMULATION_H