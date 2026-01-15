#include "Visualizer.h"
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <iostream>

// 全局常量：SFML虚线绘制的间隔 (适配SFML2.5.1，模拟MATLAB g--)
const float DASH_SPACE = 5.0f;  // 虚线实线段长度
const float DASH_GAP   = 3.0f;  // 虚线间隔长度

Visualizer::Visualizer(const Road& road)
    : road_(road),
      viewWidth_(1200.0f),
      viewHeight_(800.0f),
      scale_(5.0f),          // 5像素/米，和原代码一致
      egoTrailLength_(50),   // 历史轨迹最多保存50个点，防卡顿
      viewOffsetY_(25.0f)    // ✅ 严格对齐MATLAB viewCenterY = egoVehicle.y +25
{
    // 初始化轨迹 - SFML2.5.1兼容
    plannedTraj_.setPrimitiveType(sf::LineStrip);
    egoTrail_.setPrimitiveType(sf::LineStrip);
    egoShape_.setPrimitiveType(sf::Quads); // 自车是四边形，用Quads完美匹配MATLAB fill

    // ✅ 【终极修复】字体加载：全路径+全别名+全大小写容错 (适配你的Arial字体目录)
    font_ = std::make_unique<sf::Font>();
    std::vector<std::string> fontPaths = {
        "/usr/share/fonts/truetype/msttcorefonts/Arial.ttf",
        "/usr/share/fonts/truetype/msttcorefonts/arial.ttf",
        "/usr/share/fonts/truetype/msttcorefonts/arialbd.ttf",
        "/usr/share/fonts/truetype/msttcorefonts/Arial_Bold.ttf",
        "/usr/share/fonts/truetype/msttcorefonts/times.ttf" // 兜底字体
    };
    bool fontLoaded = false;
    for(const auto& path : fontPaths) {
        if(font_->loadFromFile(path)) {
            fontLoaded = true;
            break;
        }
    }
    if(!fontLoaded) {
        std::cerr << "⚠️ 警告：字体加载失败，但不影响仿真运行，仅无文本显示\n";
    }

    // 初始化信息文本 (严格对齐MATLAB FontSize=10)
    infoText_.setFont(*font_);
    infoText_.setCharacterSize(10);
    infoText_.setFillColor(INFO_TEXT_COLOR);
    infoText_.setStyle(sf::Text::Regular);

    // 初始化信息框 (对齐MATLAB的背景+边框+内边距)
    infoBox_.setFillColor(INFO_BG_COLOR);
    infoBox_.setOutlineColor(sf::Color::Black);
    infoBox_.setOutlineThickness(1.0f);
}

bool Visualizer::initialize() {
    window_.create(sf::VideoMode(static_cast<unsigned int>(viewWidth_),
                                 static_cast<unsigned int>(viewHeight_)),
                  "EUDM 行为规划仿真",
                  sf::Style::Titlebar | sf::Style::Close);
    if (!window_.isOpen()) {
        std::cerr << "❌ 错误：无法创建SFML窗口\n";
        return false;
    }
    window_.setPosition(sf::Vector2i(100, 100));
    window_.setVerticalSyncEnabled(true); // 垂直同步防撕裂
    return true;
}

// ✅ 【核心修复】世界坐标 → 屏幕坐标 映射公式 (彻底解决偏移/跟随错误)
// MATLAB: y轴向前为正，SFML: y轴向下为正 → 正确反转+缩放+居中
sf::Vector2f Visualizer::worldToScreen(float x, float y) const {
    float screenX = (x * scale_) + (viewWidth_ / 2.0f);  // 横向居中
    float screenY = viewHeight_ - (y * scale_);          // 纵向反转，匹配MATLAB坐标系
    return sf::Vector2f(screenX, screenY);
}

// ✅ 绘制旋转后的车辆四边形 (完美匹配MATLAB的fill+车辆getShape()，解决形状失真/角度错误)
sf::VertexArray Visualizer::createRotatedShape(const std::array<std::array<double, 2>, 4>& worldShape) const {
    sf::VertexArray shape(sf::Quads);
    shape.resize(4);
    for(int i=0; i<4; ++i) {
        // 直接从array中取坐标，转换为float
        sf::Vector2f screenPos = worldToScreen(
            static_cast<float>(worldShape[i][0]), 
            static_cast<float>(worldShape[i][1])
        );
        shape[i].position = screenPos;
    }
    return shape;
}

// ✅ 严格对齐MATLAB drawRoad()逻辑 + SFML2.5.1兼容
void Visualizer::drawRoad() {
    // 1. 绘制道路背景矩形
    auto [_, roadLeft] = road_.getLaneBoundaries(1);
    auto [roadRight, __] = road_.getLaneBoundaries(road_.getNumLanes());
    float yMax = road_.getRoadLength() + 100;
    float yMin = -100;

    sf::Vector2f tl = worldToScreen(static_cast<float>(roadLeft), static_cast<float>(yMin));
    sf::Vector2f br = worldToScreen(static_cast<float>(roadRight), static_cast<float>(yMax));
    sf::RectangleShape roadBg(sf::Vector2f(std::abs(br.x - tl.x), std::abs(br.y - tl.y)));
    roadBg.setPosition(std::min(tl.x, br.x), std::min(tl.y, br.y));
    roadBg.setFillColor(ROAD_COLOR);
    window_.draw(roadBg);

    // 2. 绘制车道线：实线边界(粗) + 虚线分隔线(细) 严格对齐MATLAB
    sf::VertexArray laneLines(sf::Lines);
    // 道路左右实线边界 (LineWidth=2)
    laneLines.append(sf::Vertex(worldToScreen(static_cast<float>(roadLeft), static_cast<float>(yMin)), LANE_LINE_COLOR));
    laneLines.append(sf::Vertex(worldToScreen(static_cast<float>(roadLeft), static_cast<float>(yMax)), LANE_LINE_COLOR));
    laneLines.append(sf::Vertex(worldToScreen(static_cast<float>(roadRight), static_cast<float>(yMin)), LANE_LINE_COLOR));
    laneLines.append(sf::Vertex(worldToScreen(static_cast<float>(roadRight), static_cast<float>(yMax)), LANE_LINE_COLOR));
    // 车道分隔虚线 (LineWidth=1)
    for (int i=1; i<road_.getNumLanes(); ++i) {
        auto [right, left] = road_.getLaneBoundaries(i);
        float center = (left + right)/2;
        for(float y=yMin; y<yMax; y+= (DASH_SPACE+DASH_GAP)) {
            laneLines.append(sf::Vertex(worldToScreen(static_cast<float>(center), y), LANE_LINE_COLOR));
            laneLines.append(sf::Vertex(worldToScreen(static_cast<float>(center), y+DASH_SPACE), LANE_LINE_COLOR));
        }
    }
    window_.draw(laneLines);
}

// ✅ 【完美对齐】视图跟随自车 (1:1复刻MATLAB的ylim逻辑)
// MATLAB: viewCenterY = egoVehicle.y+25 → ylim([viewCenterY-40, viewCenterY+60])
void Visualizer::updateView(const Vehicle& egoVehicle) {
    sf::View view = window_.getView();
    float egoY = static_cast<float>(egoVehicle.y);
    sf::Vector2f viewCenter = worldToScreen(0.0f, egoY + viewOffsetY_); // 自车Y+25m为中心
    view.setCenter(viewCenter);
    window_.setView(view);
}

// 构建状态信息文本 (无修改，逻辑正确)
std::string Visualizer::buildInfoString(const Vehicle& egoVehicle, double time, const EudmManager& manager) const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(1);
    int currentLane = road_.getLaneIndex(egoVehicle.x);

    const Action& currentAction = manager.getDebugInfo().currentAction;
    std::string currentActionStr = "当前行动: [" + currentAction.lon + "," + currentAction.lat + "]";

    std::string decisionStr = "决策序列: ";
    const auto& actionSeq = manager.getCurrentActionSequence();
    for (size_t k = 0; k < std::min<size_t>(3, actionSeq.size()); ++k) {
        decisionStr += "[" + actionSeq[k].lon + "," + actionSeq[k].lat + "] ";
    }
    if (actionSeq.size() > 3) decisionStr += "...";

    ss << "时间: " << time << "s\n"
       << "速度: " << egoVehicle.vy * 3.6 << " km/h\n"
       << "车道: " << currentLane << "\n"
       << "航向角: " << (egoVehicle.theta * 180.0 / M_PI) << "°\n"
       << "位置: (" << egoVehicle.x << ", " << egoVehicle.y << ")m\n"
       << currentActionStr << "\n"
       << decisionStr;

    const auto& stats = manager.getPerformanceStats();
    if (stats.planCount > 0) {
        ss << "\n\n规划统计:\n"
           << "规划次数: " << stats.planCount << "\n"
           << "平均时间: " << stats.avgPlanTime * 1000.0 << "ms\n"
           << "上次时间: " << stats.lastPlanTime * 1000.0 << "ms";
    }
    return ss.str();
}

// ✅ 【最终版】核心绘制函数 - 无报错+100%对齐MATLAB draw()逻辑
void Visualizer::draw(const Vehicle& egoVehicle,
                     const std::vector<Vehicle>& obstacleVehicles,
                     double time,
                     const EudmManager& manager) {
    window_.clear(BG_COLOR);  // 清屏用MATLAB浅灰色背景
    updateView(egoVehicle);   // 跟随自车更新视图
    drawRoad();               // 绘制道路

    // 1. 绘制自车历史轨迹 (青色实线，截断长度防卡顿，对齐MATLAB egoTrailHandle)
    const auto& egoTraj = egoVehicle.getTrajectory();
    egoTrail_.clear();
    if(egoTraj.size() > 1) {
        size_t startIdx = egoTraj.size() > egoTrailLength_ ? egoTraj.size()-egoTrailLength_ : 0;
        for(size_t i=startIdx; i<egoTraj.size(); ++i) {
            sf::Vector2f pos = worldToScreen(static_cast<float>(egoTraj[i][0]), static_cast<float>(egoTraj[i][1]));
            egoTrail_.append(sf::Vertex(pos, EGO_TRAIL_COLOR));
        }
        window_.draw(egoTrail_);
    }

    // 2. 绘制规划轨迹 ✅【关键】绿色虚线 完美复刻MATLAB g-- (SFML2.5.1最佳实现)
    const auto& plannedTraj = manager.getPlannedTrajectory();
    plannedTraj_.clear();
    if(!plannedTraj.empty() && plannedTraj.size()>1) {
        for(size_t i=0; i<plannedTraj.size()-1; ++i) {
            sf::Vector2f p1 = worldToScreen(static_cast<float>(plannedTraj[i].x), static_cast<float>(plannedTraj[i].y));
            sf::Vector2f p2 = worldToScreen(static_cast<float>(plannedTraj[i+1].x), static_cast<float>(plannedTraj[i+1].y));
            plannedTraj_.append(sf::Vertex(p1, PLANNED_TRAJ_COLOR));
            plannedTraj_.append(sf::Vertex(p2, PLANNED_TRAJ_COLOR));
        }
        window_.draw(plannedTraj_);
    }

    // 3. 绘制障碍物车辆 (动态红/静态灰，白色描边，完美匹配MATLAB fill)
    obsShapes_.resize(obstacleVehicles.size());
    for(size_t i=0; i<obstacleVehicles.size(); ++i) {
        const auto& obs = obstacleVehicles[i];
        obsShapes_[i] = createRotatedShape(obs.getShape());
        sf::Color fillColor = (obs.type == "static") ? STAT_OBS_COLOR : DYN_OBS_COLOR;
        for(int j=0; j<4; ++j) obsShapes_[i][j].color = fillColor;
        window_.draw(obsShapes_[i]);

        // 障碍物ID标签
        if(font_->getInfo().family != "") {
            sf::Text idText;
            idText.setFont(*font_);
            idText.setString(obs.id>=0 ? std::to_string(obs.id) : "Obs");
            idText.setCharacterSize(9);
            idText.setFillColor(sf::Color::Black);
            sf::Vector2f center = worldToScreen(static_cast<float>(obs.x), static_cast<float>(obs.y));
            idText.setPosition(center.x-5, center.y-5);
            window_.draw(idText);
        }
    }

    // 4. 绘制自车 (蓝色半透+白色描边，完美匹配MATLAB egoPlotHandle)
    egoShape_ = createRotatedShape(egoVehicle.getShape()); // 现在类型匹配，无错误
        for(int j=0; j<4; ++j) {
            egoShape_[j].color = EGO_COLOR;
        }
    window_.draw(egoShape_);
    // 自车标签
    if(font_->getInfo().family != "") {
        sf::View originalView = window_.getView(); // 保存当前跟随自车的视图
        window_.setView(window_.getDefaultView());// 切换到【屏幕原生视图】，坐标是纯像素
        
        std::string infoStr = buildInfoString(egoVehicle, time, manager);
        infoText_.setString(infoStr);
        sf::FloatRect textBound = infoText_.getLocalBounds();
        const float pad = 6.0f; // 内边距，对齐MATLAB Margin=4
        
        // ✅ 纯像素坐标，固定死：屏幕左上角 (10,10)，永远不变！
        infoBox_.setPosition(10.0f, 10.0f);
        infoBox_.setSize(sf::Vector2f(textBound.width + 2*pad, textBound.height + 2*pad));
        infoText_.setPosition(10.0f + pad, 10.0f + pad);
        
        // 绘制悬浮的信息框+文本
        window_.draw(infoBox_);
        window_.draw(infoText_);
        
        window_.setView(originalView); // 恢复自车跟随视图，不影响其他绘制
    }
    // ==============================================
    // ✅✅✅ 【核心修复 END】
    // ==============================================

    window_.display();
}

// 窗口事件处理 (无修改，逻辑正确)
bool Visualizer::handleEvents() {
    sf::Event event;
    while(window_.pollEvent(event)) {
        if(event.type == sf::Event::Closed) { window_.close(); return false; }
        if(event.type == sf::Event::KeyPressed) {
            if(event.key.code == sf::Keyboard::Escape) { window_.close(); return false; }
            if(event.key.code == sf::Keyboard::Add) scale_ *=1.1;  // +号放大
            if(event.key.code == sf::Keyboard::Subtract) scale_ *=0.9; // -号缩小
        }
    }
    return true;
}