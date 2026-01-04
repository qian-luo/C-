// Visualizer.cpp
#include "Visualizer.h"
#include <cmath>
#include <iomanip>
#include <sstream>
#include <algorithm>

Visualizer::Visualizer(const Road& road)
    : figName_("EUDM 行为规划仿真"), road_(road) {
    ensureFigure();
    drawRoad();

    // 设置坐标轴
    double left_bound = road_.getLaneCenters().front() - road_.getLaneWidth();
    double right_bound = road_.getLaneCenters().back() + road_.getLaneWidth();
    plt::xlim(left_bound, right_bound);
    plt::ylim(0.0, road_.getRoadLength());
    plt::xlabel("横向位置 (m)");
    plt::ylabel("纵向位置 (m)");
    plt::title(figName_);
    plt::grid(true);
    plt::axis("equal");

    // 初始化图形（空数据）
    plt::plot({0}, {0}, "b", {{"label", "自车"}, {"alpha", "0.8"}});
    plt::plot({0}, {0}, "g--", {{"label", "规划轨迹"}, {"linewidth", "1.5"}});
    plt::plot({0}, {0}, "c-", {{"label", "历史轨迹"}});

    // 预留障碍物标签（动态填充）
    obsPlotLabels_.clear();
}

void Visualizer::ensureFigure() {
    if (!figureInitialized_) {
        plt::figure_size(800, 800);
        plt::figure(1); // 固定 figure 1
        figureInitialized_ = true;
    }
}

void Visualizer::drawRoad() {
    double y_min = -100.0;
    double y_max = road_.getRoadLength() + 100.0;

    // 道路边界
    double left_boundary = road_.getLaneBoundaries(1).second;  // .second = left
    double right_boundary = road_.getLaneBoundaries(road_.getNumLanes()).first; // .first = right

    plt::plot({left_boundary, left_boundary}, {y_min, y_max}, "k-", {{"linewidth", "2"}});
    plt::plot({right_boundary, right_boundary}, {y_min, y_max}, "k-", {{"linewidth", "2"}});

    // 车道分隔线
    for (int i = 1; i < road_.getNumLanes(); ++i) {
        double boundary = road_.getLaneBoundaries(i).first; // right of lane i = left of i+1
        plt::plot({boundary, boundary}, {y_min, y_max}, "k--", {{"linewidth", "1"}});
    }
}

void Visualizer::clearObsoleteObstacles(size_t currentNumObs) {
    // matplotlib-cpp 无法直接删除 handle，改用：重绘时只更新现有标签
    // 实际通过“覆盖同名标签”实现更新，无需显式清除
}

void Visualizer::draw(const Vehicle& egoVehicle,
                      const std::vector<Vehicle>& obstacleVehicles,
                      double time,
                      const EudmManager& eudmManager,
                      double drawIntervalSec) {
    double currentTime = time;
    if (currentTime - lastDrawTime_ < drawIntervalSec) {
        return; // 限频
    }
    lastDrawTime_ = currentTime;

    ensureFigure();

    // ========== 1. 更新规划轨迹 ==========
    const auto& traj = eudmManager.getPlannedTrajectory();
    if (!traj.empty()) {
        std::vector<double> x_traj, y_traj;
        x_traj.reserve(traj.size());
        y_traj.reserve(traj.size());
        for (const auto& pt : traj) {
            x_traj.push_back(pt.x);
            y_traj.push_back(pt.y);
        }
        plt::plot(x_traj, y_traj, "g--", {{"label", "规划轨迹"}, {"linewidth", "1.5"}});
    } else {
        plt::plot({0}, {0}, "g--", {{"label", "规划轨迹"}});
    }

    // ========== 2. 更新历史轨迹 ==========
    const auto& trail = egoVehicle.getTrajectory();
    if (!trail.empty()) {
        std::vector<double> x_trail, y_trail;
        x_trail.reserve(trail.size());
        y_trail.reserve(trail.size());
        for (const auto& p : trail) {
            x_trail.push_back(p[0]);
            y_trail.push_back(p[1]);
        }
        plt::plot(x_trail, y_trail, "c-", {{"label", "历史轨迹"}});
    } else {
        plt::plot({0}, {0}, "c-", {{"label", "历史轨迹"}});
    }

    // ========== 3. 更新自车图形 ==========
    auto egoShape = egoVehicle.getShape(); // 4×2 array
    std::vector<double> ego_x = {
        egoShape[0][0], egoShape[1][0], egoShape[2][0], egoShape[3][0], egoShape[0][0]
    };
    std::vector<double> ego_y = {
        egoShape[0][1], egoShape[1][1], egoShape[2][1], egoShape[3][1], egoShape[0][1]
    };
    plt::fill(ego_x, ego_y, "b", {{"alpha", "0.8"}, {"edgecolor", "w"}, {"linewidth", "1.5"}, {"label", "自车"}});

    // ========== 4. 更新障碍物 ==========
    // 删除旧障碍物标签（重绘覆盖）
    clearObsoleteObstacles(obstacleVehicles.size());

    size_t obsCount = obstacleVehicles.size();
    for (size_t i = 0; i < obsCount; ++i) {
        const auto& obs = obstacleVehicles[i];
        auto obsShape = obs.getShape();
        std::vector<double> obs_x = {
            obsShape[0][0], obsShape[1][0], obsShape[2][0], obsShape[3][0], obsShape[0][0]
        };
        std::vector<double> obs_y = {
            obsShape[0][1], obsShape[1][1], obsShape[2][1], obsShape[3][1], obsShape[0][1]
        };
        std::string color = (obs.type == "static") ? "#808080" : "r"; // gray for static
        std::string label = (i == 0) ? "动态障碍物" : ""; // 避免重复图例
        if (obs.type == "static" && i == 0) label = "静态障碍物";

        plt::fill(obs_x, obs_y, color,
                  {{"alpha", "0.7"}, {"edgecolor", "w"}, {"label", label}});
    }

    // ========== 5. 更新视图中心（跟随自车）==========
    double viewCenterY = egoVehicle.y + 25.0;
    double left_bound = road_.getLaneCenters().front() - road_.getLaneWidth();
    double right_bound = road_.getLaneCenters().back() + road_.getLaneWidth();
    plt::xlim(left_bound, right_bound);
    plt::ylim(viewCenterY - 40.0, viewCenterY + 60.0);

    // ========== 6. 构建信息文本 ==========
    int current_lane = road_.getLaneIndex(egoVehicle.x);
    std::string currentActionStr = "当前行动: 无";
    const auto& debug = eudmManager.getDebugInfo();
    if (!debug.currentAction.lon.empty() && !debug.currentAction.lat.empty()) {
        currentActionStr = "当前行动: [" + debug.currentAction.lon + "," + debug.currentAction.lat + "]";
    }

    std::string decisionStr = "决策序列: ";
    const auto& seq = eudmManager.getCurrentActionSequence(); // 👈 需在 EudmManager.h 添加 getter
    if (!seq.empty()) {
        size_t n = std::min<size_t>(3, seq.size());
        for (size_t k = 0; k < n; ++k) {
            decisionStr += "[" + seq[k].lon + "," + seq[k].lat + "] ";
        }
        if (seq.size() > 3) decisionStr += "...";
    }

    std::ostringstream infoStream;
    infoStream << std::fixed << std::setprecision(1);
    infoStream << "时间: " << time << "s\n"
               << "速度: " << egoVehicle.vy * 3.6 << " km/h\n"
               << "车道: " << current_lane << "\n"
               << "航向角: " << (egoVehicle.theta * 180.0 / M_PI) << "°\n"
               << "位置: (" << egoVehicle.x << ", " << egoVehicle.y << ")m\n"
               << currentActionStr << "\n"
               << decisionStr;

    // matplotlib-cpp 不支持 text box with background directly
    // → 改用 annotate + bbox（需 matplotlib ≥ 3.1）
    double text_x = road_.getLaneCenters().front() - road_.getLaneWidth() * 0.9;
    double text_y = viewCenterY + 50.0;
    plt::annotate(infoStream.str(),
                  {{"xy", std::vector<double>{text_x, text_y}},
                   {"xytext", std::vector<double>{text_x + 2, text_y}},
                   {"textcoords", "data"},
                   {"bbox", "dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.9, edgecolor='black')"},
                   {"fontsize", "10"},
                   {"va", "top"}});

    // ========== 7. 图例 ==========
    plt::legend({{"loc", "center left"}, {"bbox_to_anchor", "(1, 0.5)"}});

    // ========== 8. 刷新 ==========
    plt::pause(0.001); // non-blocking draw; equivalent to drawnow limitrate
}