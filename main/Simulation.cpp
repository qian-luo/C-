// Simulation.cpp - 仿真类实现
#include "Simulation.h"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <random>

Simulation::Simulation(const Road& road, const Vehicle& ego,
                       const std::vector<Vehicle>& obstacles, double dt)
    : road_(road),
      egoVehicle_(ego),
      obstacleVehicles_(obstacles),
      dt_(dt),
      eudmManager_(std::make_unique<EudmManager>()),
      visualizer_(std::make_unique<Visualizer>(road)),
      isRunning_(false),
      enableVisualization_(true),
      drawInterval_(0.03) {  // ~30 FPS
}

bool Simulation::initialize(bool enableViz) {
    enableVisualization_ = enableViz;
    
    if (enableVisualization_) {
        if (!visualizer_->initialize()) {
            std::cerr << "警告：无法初始化可视化，将以无可视化模式运行\n";
            enableVisualization_ = false;
        }
    }
    
    // 初始化统计
    stats_.startTime = std::chrono::duration_cast<std::chrono::duration<double>>(
        std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    stats_.totalDistance = egoVehicle_.y;
    
    isRunning_ = true;
    return true;
}

void Simulation::run(int numSteps) {
    if (!isRunning_) return;
    
    std::cout << "开始仿真...\n";
    std::cout << "最大步数: " << numSteps << " (" << numSteps * dt_ << "秒)\n";
    
    double lastY = egoVehicle_.y;
    int lastLane = road_.getLaneIndex(egoVehicle_.x);
    std::vector<double> speeds;
    
    // 主仿真循环
    for (int i = 0; i < numSteps && isRunning_; ++i) {
        double currentTime = i * dt_;
        
        // 检查终止条件
        if (egoVehicle_.y >= road_.getRoadLength() - 50) {
            std::cout << "\n✓ 自车接近道路终点，仿真成功完成！\n";
            std::cout << "  行驶距离: " << egoVehicle_.y << " m\n";
            stats_.totalTime = currentTime;
            showStats();
            return;
        }
        
        // 更新障碍物
        updateObstacleVehicles();
        
        // 执行规划计算
        auto planStart = std::chrono::high_resolution_clock::now();
        auto [acc, steer] = eudmManager_->plan(egoVehicle_, obstacleVehicles_, road_);
        auto planEnd = std::chrono::high_resolution_clock::now();
        
        // 更新规划统计
        double planTime = std::chrono::duration<double>(planEnd - planStart).count();
        stats_.planCount++;
        stats_.totalPlanTime += planTime;
        stats_.avgPlanTime = stats_.totalPlanTime / stats_.planCount;
        
        // 更新自车状态
        egoVehicle_.step(acc, steer, dt_);
        
        // 碰撞检查
        if (checkCollision()) {
            stats_.collisions++;
            std::cout << "时间 " << currentTime << "s: 碰撞发生！仿真结束。\n";
            stats_.totalTime = currentTime;
            showStats();
            return;
        }
        
        // 更新统计
        stats_.totalDistance += egoVehicle_.y - lastY;
        lastY = egoVehicle_.y;
        speeds.push_back(egoVehicle_.vy);
        
        int currentLane = road_.getLaneIndex(egoVehicle_.x);
        if (currentLane != lastLane) {
            stats_.laneChanges++;
            lastLane = currentLane;
        }
        
        // 更新可视化
        if (enableVisualization_ && visualizer_->isOpen()) {
            auto now = std::chrono::high_resolution_clock::now();
            double timeSinceLastDraw = std::chrono::duration<double>(now - lastDrawTime_).count();
            
            if (timeSinceLastDraw >= drawInterval_) {
                // 处理窗口事件
                if (!visualizer_->handleEvents()) {
                    isRunning_ = false;
                    break;
                }
                
                // 绘制当前状态
                visualizer_->draw(egoVehicle_, obstacleVehicles_, currentTime, *eudmManager_);
                lastDrawTime_ = now;
            }
        }
    }
    
    stats_.totalTime = numSteps * dt_;
    showStats();
}

bool Simulation::step() {
    if (!isRunning_) return false;
    
    static int stepCount = 0;
    double currentTime = stepCount * dt_;
    
    // 更新障碍物
    updateObstacleVehicles();
    
    // 执行规划
    auto [acc, steer] = eudmManager_->plan(egoVehicle_, obstacleVehicles_, road_);
    
    // 更新自车状态
    egoVehicle_.step(acc, steer, dt_);
    
    // 碰撞检查
    if (checkCollision()) {
        stats_.collisions++;
        std::cout << "碰撞发生！\n";
        isRunning_ = false;
        return false;
    }
    
    // 更新统计
    if (stepCount > 0) {
        stats_.totalDistance += egoVehicle_.y - stats_.totalDistance;
    }
    
    // 更新可视化
    if (enableVisualization_) {
        if (!visualizer_->handleEvents()) {
            isRunning_ = false;
            return false;
        }
        visualizer_->draw(egoVehicle_, obstacleVehicles_, currentTime, *eudmManager_);
    }
    
    stepCount++;
    return true;
}

// Simulation.cpp - 修改updateObstacleVehicles函数
void Simulation::updateObstacleVehicles() {
    // 构建所有车辆列表（包括自车）
    std::vector<Vehicle> allVehicles = obstacleVehicles_;
    allVehicles.push_back(egoVehicle_);
    
    for (auto& obs : obstacleVehicles_) {
        if (obs.type == "dynamic") {
            // 特别处理车辆103
            if (obs.id == 103) {
                // 确保车辆103能检测到自车作为前车
                auto leader = findLeadingVehicle(obs, allVehicles);
                if (leader.has_value()) {
                    // 如果前车是自车，采取更激进的减速
                    if (leader->id == egoVehicle_.id) {
                        double dist = egoVehicle_.y - obs.y;
                        double relSpeed = obs.vy - egoVehicle_.vy;
                        
                        if (dist < 30.0 && relSpeed > 2.0) {
                            // 紧急减速
                            double emergencyDecel = -5.0;  // 紧急减速度
                            obs.step(emergencyDecel, 0.0, dt_);
                            continue;  // 跳过常规IDM
                        }
                    }
                }
            }
            
            // 常规IDM更新
            auto leader = findLeadingVehicle(obs, allVehicles);
            double acc = calculateIDM(obs, leader);
            obs.step(acc, 0.0, dt_);
        }
    }
}

std::optional<Vehicle> Simulation::findLeadingVehicle(
    const Vehicle& current, const std::vector<Vehicle>& allVehicles) const {
    
    double minDistance = std::numeric_limits<double>::infinity();
    std::optional<Vehicle> leader;
    
    int currentLane = road_.getLaneIndex(current.x);
    
    for (const auto& other : allVehicles) {
        if (other.id == current.id) continue;
        
        int otherLane = road_.getLaneIndex(other.x);
        if (otherLane == currentLane) {
            double dist = other.y - current.y;
            if (dist > 0 && dist < minDistance) {
                minDistance = dist;
                leader = other;
            }
        }
    }
    
    return leader;
}

double Simulation::calculateIDM(const Vehicle& ego, const std::optional<Vehicle>& leader) const {
    const double a_max = 1.5;      // 最大加速度 (m/s²)
    const double b_comf = 2.0;     // 舒适减速度 (m/s²)
    double v_des = ego.targetSpeed;
    const double T = 1.5;          // 车头时距 (s)
    const double s0 = 2.0;         // 最小间距 (m)
    
    double v_ego = std::max(0.0, ego.vy);
    
    if (!leader.has_value()) {
        // 无前车
        return a_max * (1.0 - std::pow(v_ego / v_des, 4.0));
    }
    
    // 有前车
    const Vehicle& lv = *leader;
    double v_leader = std::max(0.0, lv.vy);
    double delta_v = v_ego - v_leader;
    double s = std::max(0.1, lv.y - ego.y - ego.length);
    double s_star = s0 + std::max(0.0, v_ego * T + (v_ego * delta_v) / (2.0 * std::sqrt(a_max * b_comf)));
    
    double acc = a_max * (1.0 - std::pow(v_ego / v_des, 4.0) - std::pow(s_star / s, 2.0));
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

void Simulation::showStats() const {
    std::cout << "\n========== 仿真统计 ==========\n";
    std::cout << "仿真总时长: " << stats_.totalTime << " s\n";
    std::cout << "总行驶距离: " << stats_.totalDistance << " m\n";
    std::cout << "碰撞次数: " << stats_.collisions << "\n";
    std::cout << "换道次数: " << stats_.laneChanges << "\n";
    
    // 计算平均速度（需要记录历史速度）
    if (stats_.totalTime > 0) {
        double avgSpeed = stats_.totalDistance / stats_.totalTime;
        std::cout << "平均速度: " << avgSpeed * 3.6 << " km/h\n";
    }
    
    if (stats_.planCount > 0) {
        std::cout << "规划次数: " << stats_.planCount << "\n";
        std::cout << "平均规划时间: " << stats_.avgPlanTime * 1000.0 << " ms\n";
        
        if (stats_.totalTime > 0) {
            double planRate = stats_.planCount / stats_.totalTime;
            std::cout << "规划频率: " << planRate << " Hz\n";
        }
    }
    
    std::cout << "================================\n";
}