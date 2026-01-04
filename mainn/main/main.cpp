// main.cpp
#include <iostream>
#include <vector>
#include <random>
#include <cmath>

// 已记录/生成的类头文件
#include "Road.h"
#include "Vehicle.h"
#include "Simulation.h"

int main() {
    std::cout << "=============================================\n";
    std::cout << "EUDM 行为规划仿真 V3.5 (C++ 移植版)\n";
    std::cout << "=============================================\n\n";

    // ========== 仿真参数 ==========
    const double simTime = 300.0;   // s
    const double dt = 0.1;          // s
    const int numSteps = static_cast<int>(simTime / dt);

    std::cout << "初始化仿真环境...\n";

    // ========== 创建道路 ==========
    Road road(3, 3.5, 2500.0);  // 3车道, 3.5m宽, 2500m长

    // ========== 创建自车 ==========
    // MATLAB: Vehicle(0, road.getLaneCenter(3), 10, 5/3.6, 0, 'ego')
    // 注意：Vehicle 构造函数为 (id, x, y, vy, vx, type)
    double ego_x = road.getLaneCenter(3);  // 右侧车道（1:左, 2:中, 3:右）
    Vehicle egoVehicle(0, ego_x, 10.0, 5.0 / 3.6, 0.0, "ego");
    egoVehicle.targetSpeed = 40.0 / 3.6;  // 40 km/h → m/s

    // ========== 创建障碍物（20个）==========
    std::vector<Vehicle> obstacleVehicles;
    obstacleVehicles.reserve(20);

    // Helper: 添加障碍车
    auto addObstacle = [&](int id, double x, double y, double vy, double vx,
                           const std::string& type, double targetSpeed) {
        Vehicle obs(id, x, y, vy, vx, type);
        obs.targetSpeed = targetSpeed;
        obstacleVehicles.push_back(obs);
    };

    // 场景1: 右侧车道前方慢车(30km/h)
    addObstacle(101, road.getLaneCenter(3), 210.0, 8.33, 0.0, "dynamic", 8.33);

    // 场景2: 中间车道静态施工车
    addObstacle(102, road.getLaneCenter(2), 220.0, 0.0, 0.0, "static", 0.0);

    // 场景3: 左侧车道快速车(55km/h)
    addObstacle(103, road.getLaneCenter(1), 260.0, 15.28, 0.0, "dynamic", 15.28);

    // 场景4: 中间车道中速车(40km/h)
    addObstacle(104, road.getLaneCenter(2), 300.0, 11.11, 0.0, "dynamic", 11.11);

    // 场景5: 右侧车道路边静止车辆
    addObstacle(105, road.getLaneCenter(3), 320.0, 0.0, 0.0, "static", 0.0);

    // 场景6: 左侧车道慢车(30km/h)
    addObstacle(106, road.getLaneCenter(1), 360.0, 8.33, 0.0, "dynamic", 8.33);

    // 场景7: 中间车道拥堵慢车(35km/h)
    addObstacle(107, road.getLaneCenter(2), 400.0, 9.72, 0.0, "dynamic", 9.72);

    // 场景8: 右侧车道中速车(40km/h)
    addObstacle(108, road.getLaneCenter(3), 420.0, 11.11, 0.0, "dynamic", 11.11);

    // 场景9: 左侧车道较快车(50km/h)
    addObstacle(109, road.getLaneCenter(1), 460.0, 13.89, 0.0, "dynamic", 13.89);

    // 场景10: 中间车道静态障碍
    addObstacle(110, road.getLaneCenter(2), 500.0, 0.0, 0.0, "static", 0.0);

    // 场景11: 右侧车道慢车(30km/h)
    addObstacle(111, road.getLaneCenter(3), 520.0, 8.33, 0.0, "dynamic", 8.33);

    // 场景12: 左侧车道静态障碍
    addObstacle(112, road.getLaneCenter(1), 560.0, 0.0, 0.0, "static", 0.0);

    // 场景13: 中间车道较快车(45km/h)
    addObstacle(113, road.getLaneCenter(2), 600.0, 12.5, 0.0, "dynamic", 12.5);

    // 场景14: 右侧车道中速车(40km/h)
    addObstacle(114, road.getLaneCenter(3), 640.0, 11.11, 0.0, "dynamic", 11.11);

    // 场景15: 左侧车道中等偏慢(36km/h)
    addObstacle(115, road.getLaneCenter(1), 680.0, 10.0, 0.0, "dynamic", 10.0);

    // 场景16: 中间车道静态施工区
    addObstacle(116, road.getLaneCenter(2), 720.0, 0.0, 0.0, "static", 0.0);

    // 场景17: 右侧车道较快车(50km/h)
    addObstacle(117, road.getLaneCenter(3), 760.0, 13.89, 0.0, "dynamic", 13.89);

    // 场景18: 左侧车道中速车(40km/h)
    addObstacle(118, road.getLaneCenter(1), 800.0, 11.11, 0.0, "dynamic", 11.11);

    // 场景19: 中间车道慢车(35km/h)
    addObstacle(119, road.getLaneCenter(2), 840.0, 9.72, 0.0, "dynamic", 9.72);

    // 场景20: 右侧车道静态障碍
    addObstacle(120, road.getLaneCenter(3), 880.0, 0.0, 0.0, "static", 0.0);

    std::cout << "创建障碍物场景（20 个动静态障碍）...\n";

    // ========== 启动仿真 ==========
    std::cout << "开始仿真（最大仿真时间：" << simTime << "秒）...\n\n";
    
    Simulation simulation(road, egoVehicle, obstacleVehicles, dt);
    simulation.run(numSteps);

    std::cout << "\n仿真完成！\n";
    return 0;
}