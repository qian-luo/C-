#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <SFML/Graphics.hpp>
#include <vector>
#include <string>
#include <memory>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <iostream>

#include "Road.h"
#include "Vehicle.h"
#include "State.h"
#include "EudmManager.h"

class Visualizer {
private:
    // SFML核心对象
    sf::RenderWindow window_;
    std::unique_ptr<sf::Font> font_;
    
    // 道路常量引用
    const Road& road_;
    
    // 绘图图形对象
    sf::VertexArray egoShape_;          // 自车：改用VertexArray绘制旋转四边形，解决失真
    std::vector<sf::VertexArray> obsShapes_; // 障碍物：同样用VertexArray，完美匹配MATLAB fill
    sf::VertexArray plannedTraj_;       // 规划轨迹
    sf::VertexArray egoTrail_;          // 自车历史轨迹
    sf::Text infoText_;                 // 状态文本
    sf::RectangleShape infoBox_;        // 文本背景框

    // 可视化配置参数 (严格对齐MATLAB)
    float viewWidth_;
    float viewHeight_;
    float scale_;          // 像素/米 5px/m
    size_t egoTrailLength_;// 历史轨迹最大长度，防止卡顿
    float viewOffsetY_;    // 视图跟随偏移量 = MATLAB的+25m

    // ✅ 颜色常量：完全对齐MATLAB的RGB+透明度 (0-255映射0-1)
    const sf::Color BG_COLOR           = sf::Color(243, 243, 243);    // MATLAB [0.95,0.95,0.95]
    const sf::Color ROAD_COLOR         = sf::Color(200, 200, 200);    // 道路灰色
    const sf::Color LANE_LINE_COLOR    = sf::Color::Black;            // 车道线黑色
    const sf::Color EGO_COLOR          = sf::Color(0, 0, 255, 204);   // 蓝色 FaceAlpha=0.8
    const sf::Color EGO_OUTLINE_COLOR  = sf::Color::White;            // 自车白色描边
    const sf::Color DYN_OBS_COLOR      = sf::Color(255, 0, 0, 189);   // 红色 FaceAlpha=0.7
    const sf::Color STAT_OBS_COLOR     = sf::Color(128, 128, 128, 189);//灰色 FaceAlpha=0.7
    const sf::Color OBS_OUTLINE_COLOR  = sf::Color::White;            // 障碍物白色描边
    const sf::Color PLANNED_TRAJ_COLOR = sf::Color(0, 255, 0);        // 规划轨迹-绿色
    const sf::Color EGO_TRAIL_COLOR    = sf::Color(0, 255, 255);      // 历史轨迹-青色
    const sf::Color INFO_BG_COLOR      = sf::Color(255, 255, 230, 220);//信息框 米白半透
    const sf::Color INFO_TEXT_COLOR    = sf::Color::Black;            // 信息文本黑色

    // 私有工具方法
    void drawRoad();
    void updateView(const Vehicle& egoVehicle);
    sf::Vector2f worldToScreen(float x, float y) const;
    std::string buildInfoString(const Vehicle& egoVehicle, double time, const EudmManager& manager) const;
    sf::VertexArray createRotatedShape(const std::array<std::array<double, 2>, 4>& worldShape) const; // 绘制旋转车辆形状

public:
    explicit Visualizer(const Road& road);
    bool initialize();
    void draw(const Vehicle& egoVehicle, const std::vector<Vehicle>& obstacleVehicles, double time, const EudmManager& manager);
    bool handleEvents();
    bool isOpen() const { return window_.isOpen(); }
    sf::RenderWindow& getWindow() { return window_; }
    void setScale(float scale) { scale_ = std::max(1.0f, scale); }
};

#endif // VISUALIZER_H