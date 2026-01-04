#pragma once
#include "Vehicle.h"
#include "Road.h"
#include <vector>
#include <array>
#include <string>
#include <utility>
#include <memory>
#include "State.h"
#include "DcpTree.h" 
/*
// 动作结构体，对应MATLAB中的action
struct Action {
    std::string lat;  // 横向动作: "LK", "LCL", "LCR"
    std::string lon;  // 纵向动作: "ACC", "DEC", "MAINTAIN"
    double t;         // 持续时间
};
*/
// 轨迹点结构体，对应MATLAB中的trajPoint
/*struct TrajectoryPoint {
    double id;
    double x, y, vx, vy, theta;
};
*/
// RSS配置结构体
struct RSSConfig {
    double response_time = 0.3;
    double lon_acc_max = 2.0;
    double lon_brake_min = -3.0;
    double lon_brake_max = -6.0;
    double lat_acc_max = 2.0;
    double lat_brake_min = -3.0;
    double lat_brake_max = -5.0;
};

class ForwardSimulator {
public:
    // 属性
    double simDt = 0.1;  // 仿真步长 0.1s
    RSSConfig rssConfig;

    // 构造函数
    ForwardSimulator();

    // 方法声明
    std::tuple<std::vector<State>, std::vector<State>, bool> 
    simulateLayer(const Action& action, const Vehicle& egoVehicle, 
                  const std::vector<Vehicle>& obstacleVehicles, const Road& road);

    bool checkRSSSafety(const Vehicle& ego, const std::vector<Vehicle>& obs, const Road& road);

    bool checkLaneChangeSafety(const Action& action, const Vehicle& ego, 
                              const std::vector<Vehicle>& obstacles, const Road& road);

    double calculateRSSSafeLongitudinalDistance(double v_rear, double v_front, bool is_front);

    Vehicle copyVehicle(const Vehicle& oldVehicle) const;

    std::pair<double, double> actionToControl(const Action& action, const Vehicle& ego, 
                                             const Road& road, const std::vector<Vehicle>& obstacles);

    std::vector<Vehicle> findLeadingVehicle(const Vehicle& current, const Road& road, 
                                           const std::vector<Vehicle>& allVehicles);

    double calculateIDM(const Vehicle& ego, const std::vector<Vehicle>& leader) const;

private:
    // 私有辅助方法
    double calculateVehicleSpeed(const Vehicle& vehicle) const;
};