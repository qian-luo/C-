// Visualizer.h
#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <string>
#include <vector>
#include <memory>
#include <chrono>

// 依赖模块
#include "Road.h"
#include "Vehicle.h"
#include "EudmManager.h"

// 假设使用 matplotlib-cpp（需提前安装）
// GitHub: https://github.com/lava/matplotlib-cpp
// 编译需链接 Python + matplotlib
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

class Visualizer {
private:
    // ========== 属性（对应 MATLAB properties）==========
    std::string figName_;          // figure title
    Road road_;

    // 图形句柄（用字符串 ID 或 shared_ptr 管理；此处用 string + plt 内部名）
    // matplotlib-cpp 无 native handle，用 label + hold 管理更新
    std::string egoPlotLabel_ = "ego_vehicle";
    std::string plannedTrajLabel_ = "planned_trajectory";
    std::string egoTrailLabel_ = "ego_trail";
    std::vector<std::string> obsPlotLabels_;  // dynamic size

    bool figureInitialized_ = false;
    double lastDrawTime_ = 0.0;

    // ========== 私有辅助方法 ==========
    void drawRoad();
    void ensureFigure();
    void clearObsoleteObstacles(size_t currentNumObs);

public:
    // 构造函数
    explicit Visualizer(const Road& road);

    // 主绘制接口（const，不修改自身状态）
    void draw(const Vehicle& egoVehicle,
              const std::vector<Vehicle>& obstacleVehicles,
              double time,
              const EudmManager& eudmManager,
              double drawIntervalSec = 0.3);

    // 析构时自动关闭 figure（RAII）
    ~Visualizer() = default;
};

#endif // VISUALIZER_H