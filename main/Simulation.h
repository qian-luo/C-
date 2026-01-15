// Simulation.h - 仿真类头文件
#ifndef SIMULATION_H
#define SIMULATION_H

#include <vector>
#include <memory>
#include <chrono>
#include <iostream>
#include "Road.h"
#include "Vehicle.h"
#include "EudmManager.h"
#include "Visualizer.h"

// 仿真统计数据结构
struct SimulationStats {
    size_t collisions = 0;
    size_t laneChanges = 0;
    double totalDistance = 0.0;
    double totalTime = 0.0;
    double startTime = 0.0;
    
    // 性能统计
    size_t planCount = 0;
    double totalPlanTime = 0.0;
    double avgPlanTime = 0.0;
};

class Simulation {
private:
    // 仿真组件
    Road road_;
    Vehicle egoVehicle_;
    std::vector<Vehicle> obstacleVehicles_;
    double dt_;
    
    // 管理器和可视化器
    std::unique_ptr<EudmManager> eudmManager_;
    std::unique_ptr<Visualizer> visualizer_;
    
    // 仿真状态
    SimulationStats stats_;
    bool isRunning_;
    
    // 可视化控制
    bool enableVisualization_;
    double drawInterval_;
    std::chrono::time_point<std::chrono::high_resolution_clock> lastDrawTime_;
    
    // 私有方法
    void updateObstacleVehicles();
    std::optional<Vehicle> findLeadingVehicle(const Vehicle& current,
                                             const std::vector<Vehicle>& allVehicles) const;
    double calculateIDM(const Vehicle& ego, const std::optional<Vehicle>& leader) const;
    bool checkCollision() const;
    void showStats() const;

public:
    // 构造函数
    Simulation(const Road& road, const Vehicle& ego,
               const std::vector<Vehicle>& obstacles, double dt);
    
    // 初始化
    bool initialize(bool enableViz = true);
    
    // 运行仿真
    void run(int numSteps);
    
    // 单步运行（用于交互式控制）
    bool step();
    
    // 获取统计信息
    const SimulationStats& getStats() const { return stats_; }
    
    // 设置可视化间隔
    void setDrawInterval(double interval) { drawInterval_ = interval; }
    
    // 检查是否运行中
    bool isRunning() const { return isRunning_; }
    
    // 停止仿真
    void stop() { isRunning_ = false; }
};

#endif // SIMULATION_H