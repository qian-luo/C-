// main.cpp - 主程序
#include <iostream>
#include <vector>
#include <memory>

#include "Road.h"
#include "Vehicle.h"
#include "Simulation.h"

int main() {
    std::cout << "=============================================\n";
    std::cout << "EUDM 行为规划仿真 V3.5 (C++ 移植版)\n";
    std::cout << "=============================================\n\n";

    try {
        // ========== 仿真参数 ==========
        const double simTime = 300.0;   // 总仿真时间 (s)
        const double dt = 0.1;          // 时间步长 (s)
        const int numSteps = static_cast<int>(simTime / dt);

        std::cout << "初始化仿真环境...\n";

        // ========== 创建道路 ==========
        Road road(3, 3.5, 2500.0);  // 3车道, 3.5m宽, 2500m长

        // ========== 创建自车 ==========
        double ego_x = road.getLaneCenter(3);  // 起始在右侧车道
        Vehicle egoVehicle(0, ego_x, 10.0, 5.0 / 3.6, 0.0, "ego");
        egoVehicle.targetSpeed = 40.0 / 3.6;  // 40 km/h -> m/s

        std::cout << "自车初始位置: 车道" << road.getLaneIndex(ego_x) 
                  << ", 速度" << egoVehicle.vy * 3.6 << " km/h\n";

        // ========== 创建障碍物 ==========
        std::vector<Vehicle> obstacleVehicles;
        obstacleVehicles.reserve(20);

        // Helper lambda 函数添加障碍物
        auto addObstacle = [&](int id, int lane, double y, double speed_kmh, 
                              const std::string& type) {
            double speed = speed_kmh / 3.6;
            Vehicle obs(id, road.getLaneCenter(lane), y, speed, 0.0, type);
            obs.targetSpeed = speed;
            obstacleVehicles.push_back(obs);
            
            std::string typeStr = (type == "static") ? "静态" : "动态";
            std::cout << "  障碍物" << id << ": " << typeStr 
                      << ", 车道" << lane << ", 位置" << y 
                      << "m, 速度" << speed_kmh << "km/h\n";
        };

        std::cout << "创建障碍物...\n";
        
        // 场景配置（车道: 1=左, 2=中, 3=右）
        addObstacle(101, 3, 210.0, 30.0, "dynamic");  // 右侧车道慢车
        addObstacle(102, 2, 220.0, 0.0, "static");    // 中间车道静态车
        addObstacle(103, 1, 260.0, 55.0, "dynamic");  // 左侧车道快车
        addObstacle(104, 2, 300.0, 40.0, "dynamic");
        addObstacle(105, 3, 320.0, 0.0, "static");
        addObstacle(106, 1, 360.0, 30.0, "dynamic");
        addObstacle(107, 2, 400.0, 35.0, "dynamic");
        addObstacle(108, 3, 420.0, 40.0, "dynamic");
        addObstacle(109, 1, 460.0, 50.0, "dynamic");
        addObstacle(110, 2, 500.0, 0.0, "static");
        addObstacle(111, 3, 520.0, 30.0, "dynamic");
        addObstacle(112, 1, 560.0, 0.0, "static");
        addObstacle(113, 2, 600.0, 45.0, "dynamic");
        addObstacle(114, 3, 640.0, 40.0, "dynamic");
        addObstacle(115, 1, 680.0, 36.0, "dynamic");
        addObstacle(116, 2, 720.0, 0.0, "static");
        addObstacle(117, 3, 760.0, 50.0, "dynamic");
        addObstacle(118, 1, 800.0, 40.0, "dynamic");
        addObstacle(119, 2, 840.0, 35.0, "dynamic");
        addObstacle(120, 3, 880.0, 0.0, "static");

        std::cout << "共创建 " << obstacleVehicles.size() << " 个障碍物\n\n";

        // ========== 创建并运行仿真 ==========
        std::cout << "创建仿真实例...\n";
        Simulation simulation(road, egoVehicle, obstacleVehicles, dt);
        
        // 询问是否启用可视化
        char enableViz;
        std::cout << "是否启用可视化？(y/n): ";
        std::cin >> enableViz;
        
        bool useVisualization = (enableViz == 'y' || enableViz == 'Y');
        
        if (!simulation.initialize(useVisualization)) {
            std::cerr << "仿真初始化失败\n";
            return 1;
        }

        std::cout << "\n开始仿真...\n";
        std::cout << "按ESC键退出仿真，空格键暂停/继续\n";
        std::cout << "按'+'放大，'-'缩小\n\n";

        // 运行仿真
        simulation.run(numSteps);

        if (useVisualization) {
            std::cout << "\n按任意键退出...\n";
            std::cin.ignore();
            std::cin.get();
        }

    } catch (const std::exception& e) {
        std::cerr << "仿真运行错误: " << e.what() << "\n";
        return 1;
    }

    std::cout << "\n仿真完成！\n";
    return 0;
}