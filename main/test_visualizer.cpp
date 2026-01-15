// VisualizerTest_Windows.cpp - Windows SFML 2.5.1可视化器测试程序
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <memory>
#include <random>
#include <array>
#include <SFML/Graphics.hpp>
// ==================== 模拟辅助类定义 ====================

// State结构
struct State {
    double x;
    double y;
    
    operator std::array<double, 2>() const {
        return {x, y};
    }
};

// Action结构
struct Action {
    std::string lon;
    std::string lat;
    
    Action() : lon("Maintain"), lat("Keep") {}
    Action(const std::string& l, const std::string& t) : lon(l), lat(t) {}
};

// 轨迹类型别名
using Trajectory = std::vector<State>;

// ==================== 模拟EudmManager类 ====================
class MockEudmManager {
private:
    struct DebugInfo {
        Action currentAction;
    };
    
    struct PerformanceStats {
        int planCount = 0;
        double avgPlanTime = 0.0;
        double lastPlanTime = 0.0;
    };
    
    DebugInfo debugInfo_;
    std::vector<Action> actionSeq_;
    std::vector<State> plannedTraj_;
    PerformanceStats stats_;
    
public:
    MockEudmManager() {
        // 初始化模拟数据
        debugInfo_.currentAction = {"Maintain", "Keep"};
        
        // 设置动作序列
        actionSeq_ = {
            {"Maintain", "Keep"},
            {"Accelerate", "Left"},
            {"Decelerate", "Keep"},
            {"Maintain", "Right"}
        };
        
        // 设置性能统计
        stats_.planCount = 15;
        stats_.avgPlanTime = 0.018;
        stats_.lastPlanTime = 0.014;
        
        // 生成模拟轨迹（正弦波形状）
        for (int i = 0; i < 80; ++i) {
            double t = i * 0.2;
            State state;
            state.x = 0.5 * std::sin(t * 0.3);  // 轻微的横向摆动
            state.y = t * 5.0;  // 向前运动
            plannedTraj_.push_back(state);
        }
    }
    
    // 模拟真实EudmManager的接口
    const DebugInfo& getDebugInfo() const { return debugInfo_; }
    const std::vector<Action>& getCurrentActionSequence() const { return actionSeq_; }
    const std::vector<State>& getPlannedTrajectory() const { return plannedTraj_; }
    const PerformanceStats& getPerformanceStats() const { return stats_; }
};

// ==================== 简单模拟Road类 ====================
class Road {
private:
    int numLanes_;
    double laneWidth_;
    double roadLength_;
    std::vector<double> laneCenters_;
    
public:
    Road(int numLanes, double laneWidth, double roadLength)
        : numLanes_(numLanes), laneWidth_(laneWidth), roadLength_(roadLength) {
        // 计算车道中心线位置
        double totalWidth = numLanes * laneWidth;
        double startX = -totalWidth / 2 + laneWidth / 2;
        
        for (int i = 0; i < numLanes; ++i) {
            laneCenters_.push_back(startX + i * laneWidth);
        }
    }
    
    double getLaneCenter(int laneIndex) const {
        if (laneIndex < 1 || laneIndex > numLanes_) {
            return laneCenters_[numLanes_ / 2];  // 默认为中间车道
        }
        return laneCenters_[laneIndex - 1];
    }
    
    int getLaneIndex(double x) const {
        // 简单实现：找到最近的车道
        int bestLane = 1;
        double minDist = std::abs(x - laneCenters_[0]);
        
        for (int i = 1; i < numLanes_; ++i) {
            double dist = std::abs(x - laneCenters_[i]);
            if (dist < minDist) {
                minDist = dist;
                bestLane = i + 1;
            }
        }
        return bestLane;
    }
    
    std::pair<double, double> getLaneBoundaries(int laneIndex) const {
        if (laneIndex < 1 || laneIndex > numLanes_) {
            laneIndex = numLanes_ / 2 + 1;  // 默认为中间车道
        }
        
        double center = getLaneCenter(laneIndex);
        double left = center - laneWidth_ / 2;
        double right = center + laneWidth_ / 2;
        
        return std::make_pair(right, left);  // 注意：与MATLAB一致，右边界在前
    }
    
    int getNumLanes() const { return numLanes_; }
    double getLaneWidth() const { return laneWidth_; }
    double getRoadLength() const { return roadLength_; }
    const std::vector<double>& getLaneCenters() const { return laneCenters_; }
};

// ==================== 简单模拟Vehicle类 ====================
class Vehicle {
public:
    int id;
    double x, y, vx, vy, theta;
    double length = 4.7;
    double width = 1.8;
    std::string type;
    std::vector<std::array<double, 2>> trajectory;
    size_t maxTrajectoryLength = 200;
    
    Vehicle(int id, double x, double y, double vy, double vx, const std::string& type)
        : id(id), x(x), y(y), vy(vy), vx(vx), type(type), theta(0.0) {
        // 初始化轨迹
        trajectory.push_back({x, y});
    }
    
    std::array<std::array<double, 2>, 4> getShape() const {
        // 计算车辆四个角点的坐标（考虑车辆朝向）
        double halfLength = length / 2;
        double halfWidth = width / 2;
        
        std::array<std::array<double, 2>, 4> shape;
        
        // 前右
        shape[0] = {
            x + halfLength * std::cos(theta) - halfWidth * std::sin(theta),
            y + halfLength * std::sin(theta) + halfWidth * std::cos(theta)
        };
        
        // 前左
        shape[1] = {
            x + halfLength * std::cos(theta) + halfWidth * std::sin(theta),
            y + halfLength * std::sin(theta) - halfWidth * std::cos(theta)
        };
        
        // 后左
        shape[2] = {
            x - halfLength * std::cos(theta) + halfWidth * std::sin(theta),
            y - halfLength * std::sin(theta) - halfWidth * std::cos(theta)
        };
        
        // 后右
        shape[3] = {
            x - halfLength * std::cos(theta) - halfWidth * std::sin(theta),
            y - halfLength * std::sin(theta) + halfWidth * std::cos(theta)
        };
        
        return shape;
    }
    
    const std::vector<std::array<double, 2>>& getTrajectory() const {
        return trajectory;
    }
    
    void updateTrajectory() {
        trajectory.push_back({x, y});
        if (trajectory.size() > maxTrajectoryLength) {
            trajectory.erase(trajectory.begin());
        }
    }
};

// ==================== 简化版Visualizer类 ====================
class SimpleVisualizer {
private:
    sf::RenderWindow window_;
    sf::Font font_;
    
    // 常量定义
    const sf::Color ROAD_COLOR = sf::Color(200, 200, 200);
    const sf::Color LANE_LINE_COLOR = sf::Color::White;
    const sf::Color EGO_COLOR = sf::Color::Blue;
    const sf::Color DYN_OBS_COLOR = sf::Color::Red;
    const sf::Color STAT_OBS_COLOR = sf::Color(128, 128, 128);
    const sf::Color PLANNED_TRAJ_COLOR = sf::Color::Green;
    const sf::Color EGO_TRAIL_COLOR = sf::Color::Cyan;
    
    float viewWidth_;
    float viewHeight_;
    float scale_;
    Road& road_;
    
public:
    SimpleVisualizer(Road& road) 
        : viewWidth_(1200.0f), viewHeight_(800.0f), scale_(5.0f), road_(road) {
    }
    
    bool initialize() {
        // 创建窗口
        window_.create(sf::VideoMode(static_cast<unsigned int>(viewWidth_),
                                     static_cast<unsigned int>(viewHeight_)),
                      "EUDM 行为规划仿真 - SFML 2.5.1",
                      sf::Style::Titlebar | sf::Style::Close);
        
        if (!window_.isOpen()) {
            std::cerr << "错误：无法创建SFML窗口\n";
            return false;
        }
        
        // 设置窗口位置
        window_.setPosition(sf::Vector2i(100, 100));
        
        // 设置垂直同步
        window_.setVerticalSyncEnabled(true);
        
        // 加载字体
        if (!font_.loadFromFile("arial.ttf")) {
            std::cerr << "警告：无法加载arial.ttf，尝试使用默认字体\n";
            // 尝试创建默认字体
            if (!font_.loadFromFile("C:/Windows/Fonts/arial.ttf")) {
                std::cerr << "无法加载任何字体，文本可能无法显示\n";
            }
        }
        
        return true;
    }
    
    // 世界坐标转屏幕坐标
    sf::Vector2f worldToScreen(float x, float y) const {
        float screenX = (x + viewWidth_ / (2 * scale_)) * scale_;
        float screenY = viewHeight_ - y * scale_;  // 反转Y轴
        return sf::Vector2f(screenX, screenY);
    }
    
    // 绘制道路
    void drawRoad() {
        // 绘制道路背景
        float roadLeft = road_.getLaneBoundaries(1).second;  // 最左侧边界
        float roadRight = road_.getLaneBoundaries(road_.getNumLanes()).first;  // 最右侧边界
        
        sf::RectangleShape roadBackground;
        roadBackground.setFillColor(ROAD_COLOR);
        
        sf::Vector2f topLeft = worldToScreen(roadLeft, road_.getRoadLength());
        sf::Vector2f bottomRight = worldToScreen(roadRight, 0);
        
        roadBackground.setPosition(topLeft);
        roadBackground.setSize(sf::Vector2f(bottomRight.x - topLeft.x,
                                           bottomRight.y - topLeft.y));
        window_.draw(roadBackground);
        
        // 绘制车道线
        sf::VertexArray laneLines(sf::Lines);
        for (int i = 1; i <= road_.getNumLanes(); ++i) {
            auto boundaries = road_.getLaneBoundaries(i);
            double rightBoundary = boundaries.first;
            double leftBoundary = boundaries.second;
            
            // 车道中心虚线（如果是车道分隔线）
            if (i > 1) {
                float centerX = (leftBoundary + rightBoundary) / 2;
                for (float y = 0; y <= road_.getRoadLength(); y += 20.0f) {
                    sf::Vector2f start = worldToScreen(centerX, y);
                    sf::Vector2f end = worldToScreen(centerX, y + 10.0f);
                    
                    laneLines.append(sf::Vertex(start, LANE_LINE_COLOR));
                    laneLines.append(sf::Vertex(end, LANE_LINE_COLOR));
                }
            }
            
            // 道路边界实线
            sf::Vector2f leftStart = worldToScreen(leftBoundary, road_.getRoadLength());
            sf::Vector2f leftEnd = worldToScreen(leftBoundary, 0);
            sf::Vector2f rightStart = worldToScreen(rightBoundary, road_.getRoadLength());
            sf::Vector2f rightEnd = worldToScreen(rightBoundary, 0);
            
            laneLines.append(sf::Vertex(leftStart, sf::Color::Black));
            laneLines.append(sf::Vertex(leftEnd, sf::Color::Black));
            laneLines.append(sf::Vertex(rightStart, sf::Color::Black));
            laneLines.append(sf::Vertex(rightEnd, sf::Color::Black));
        }
        
        window_.draw(laneLines);
    }
    
    // 绘制车辆
    void drawVehicle(const Vehicle& vehicle, bool isEgo = false) {
        auto shape = vehicle.getShape();
        
        // 创建凸多边形表示车辆
        sf::ConvexShape convex;
        convex.setPointCount(4);
        
        for (int i = 0; i < 4; ++i) {
            sf::Vector2f point = worldToScreen(shape[i][0], shape[i][1]);
            convex.setPoint(i, point);
        }
        
        // 设置颜色
        if (isEgo) {
            convex.setFillColor(EGO_COLOR);
        } else {
            convex.setFillColor(vehicle.type == "static" ? STAT_OBS_COLOR : DYN_OBS_COLOR);
        }
        
        convex.setOutlineColor(sf::Color::White);
        convex.setOutlineThickness(1.0f);
        
        window_.draw(convex);
        
        // 绘制车辆ID
        sf::Text idText;
        idText.setFont(font_);
        idText.setString(isEgo ? "Ego" : std::to_string(vehicle.id));
        idText.setCharacterSize(12);
        idText.setFillColor(sf::Color::White);
        idText.setStyle(sf::Text::Bold);
        
        // 将ID放在车辆中心
        sf::Vector2f center = worldToScreen(vehicle.x, vehicle.y);
        idText.setPosition(center.x - 10, center.y - 10);
        
        window_.draw(idText);
    }
    
    // 绘制轨迹
    void drawTrajectory(const std::vector<std::array<double, 2>>& trajectory, 
                       const sf::Color& color, float thickness = 2.0f) {
        if (trajectory.size() < 2) return;
        
        sf::VertexArray lines(sf::LineStrip);
        
        for (const auto& point : trajectory) {
            sf::Vector2f screenPoint = worldToScreen(point[0], point[1]);
            lines.append(sf::Vertex(screenPoint, color));
        }
        
        window_.draw(lines);
    }
    
    // 绘制规划轨迹
    void drawPlannedTrajectory(const std::vector<State>& trajectory) {
        if (trajectory.empty()) return;
        
        sf::VertexArray lines(sf::LineStrip);
        
        for (const auto& point : trajectory) {
            sf::Vector2f screenPoint = worldToScreen(point.x, point.y);
            lines.append(sf::Vertex(screenPoint, PLANNED_TRAJ_COLOR));
        }
        
        window_.draw(lines);
    }
    
    // 绘制信息面板
    void drawInfoPanel(const Vehicle& egoVehicle, double time, const MockEudmManager& manager) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(1);
        
        int currentLane = road_.getLaneIndex(egoVehicle.x);
        
        // 构建信息字符串
        ss << "EUDM 行为规划仿真 - SFML 2.5.1\n";
        ss << "==============================\n";
        ss << "时间: " << time << "s\n";
        ss << "速度: " << egoVehicle.vy * 3.6 << " km/h\n";
        ss << "车道: " << currentLane << "\n";
        ss << "位置: (" << egoVehicle.x << ", " << egoVehicle.y << ")m\n";
        ss << "\n当前行动: [" << manager.getDebugInfo().currentAction.lon 
           << "," << manager.getDebugInfo().currentAction.lat << "]\n";
        
        // 决策序列
        ss << "决策序列: ";
        const auto& actionSeq = manager.getCurrentActionSequence();
        for (size_t k = 0; k < std::min<size_t>(3, actionSeq.size()); ++k) {
            ss << "[" << actionSeq[k].lon << "," << actionSeq[k].lat << "] ";
        }
        
        // 创建文本
        sf::Text infoText;
        infoText.setFont(font_);
        infoText.setString(ss.str());
        infoText.setCharacterSize(14);
        infoText.setFillColor(sf::Color::Black);
        infoText.setStyle(sf::Text::Bold);
        infoText.setPosition(10, 10);
        
        // 创建背景框
        sf::FloatRect textBounds = infoText.getGlobalBounds();
        sf::RectangleShape background;
        background.setFillColor(sf::Color(255, 255, 230, 200));
        background.setOutlineColor(sf::Color::Black);
        background.setOutlineThickness(1.0f);
        background.setPosition(textBounds.left - 5, textBounds.top - 5);
        background.setSize(sf::Vector2f(textBounds.width + 10, textBounds.height + 10));
        
        window_.draw(background);
        window_.draw(infoText);
    }
    
    // 绘制函数
    void draw(const Vehicle& egoVehicle,
              const std::vector<Vehicle>& obstacleVehicles,
              double time,
              const MockEudmManager& manager) {
        // 清空窗口
        window_.clear(sf::Color(240, 240, 240));  // 浅灰色背景
        
        // 绘制道路
        drawRoad();
        
        // 绘制自车轨迹
        drawTrajectory(egoVehicle.getTrajectory(), EGO_TRAIL_COLOR, 1.5f);
        
        // 绘制规划轨迹
        drawPlannedTrajectory(manager.getPlannedTrajectory());
        
        // 绘制障碍物
        for (const auto& vehicle : obstacleVehicles) {
            drawVehicle(vehicle);
        }
        
        // 绘制自车
        drawVehicle(egoVehicle, true);
        
        // 绘制信息面板
        drawInfoPanel(egoVehicle, time, manager);
        
        // 显示绘制结果
        window_.display();
    }
    
    // 处理窗口事件
    bool handleEvents() {
        sf::Event event;
        while (window_.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window_.close();
                return false;
            }
            
            // 处理键盘事件
            if (event.type == sf::Event::KeyPressed) {
                switch (event.key.code) {
                    case sf::Keyboard::Escape:
                        window_.close();
                        return false;
                    case sf::Keyboard::Space:
                        std::cout << "暂停/继续仿真\n";
                        break;
                    case sf::Keyboard::Add:
                        scale_ *= 1.1f;  // 放大
                        std::cout << "缩放: " << scale_ << " pixels/m\n";
                        break;
                    case sf::Keyboard::Subtract:
                        scale_ *= 0.9f;  // 缩小
                        std::cout << "缩放: " << scale_ << " pixels/m\n";
                        break;
                    default:
                        break;
                }
            }
        }
        return true;
    }
    
    // 检查窗口是否打开
    bool isOpen() const { return window_.isOpen(); }
};

// ==================== 主测试函数 ====================
int main() {
    std::cout << "EUDM行为规划仿真 - Visualizer测试程序\n";
    std::cout << "SFML版本: 2.5.1\n";
    std::cout << "编译时间: " << __DATE__ << " " << __TIME__ << "\n";
    std::cout << "=====================================\n\n";
    
    try {
        // 1. 创建道路
        Road road(3, 3.75, 500.0);  // 3车道，每道宽3.75米，道路长500米
        std::cout << "道路创建成功: " << road.getNumLanes() << "车道，长度" << road.getRoadLength() << "米\n";
        
        // 2. 创建可视化器
        SimpleVisualizer visualizer(road);
        if (!visualizer.initialize()) {
            std::cerr << "可视化器初始化失败!\n";
            return 1;
        }
        std::cout << "可视化器初始化成功\n";
        
        // 3. 创建模拟管理器
        MockEudmManager manager;
        std::cout << "模拟管理器创建成功\n";
        
        // 4. 创建车辆
        // 自车 - 在中间车道
        Vehicle egoVehicle(0, road.getLaneCenter(2), 50.0, 20.0, 0.0, "ego");
        egoVehicle.theta = 0.0;  // 朝向前方
        
        // 障碍物车辆
        std::vector<Vehicle> obstacles;
        
        // 前车（同车道）
        obstacles.push_back(Vehicle(1, road.getLaneCenter(2), 100.0, 15.0, 0.0, "dynamic"));
        
        // 左侧车道车辆
        obstacles.push_back(Vehicle(2, road.getLaneCenter(1), 80.0, 22.0, 0.0, "dynamic"));
        
        // 右侧车道车辆
        obstacles.push_back(Vehicle(3, road.getLaneCenter(3), 120.0, 18.0, 0.0, "dynamic"));
        
        // 静态障碍物（施工区）
        obstacles.push_back(Vehicle(4, road.getLaneCenter(2) + 1.0, 180.0, 0.0, 0.0, "static"));
        
        // 更多车辆增加场景复杂度
        obstacles.push_back(Vehicle(5, road.getLaneCenter(1), 150.0, 25.0, 0.0, "dynamic"));
        obstacles.push_back(Vehicle(6, road.getLaneCenter(3), 70.0, 19.0, 0.0, "dynamic"));
        
        std::cout << "创建了" << obstacles.size() << "个障碍物车辆\n";
        
        // 5. 仿真循环
        double simulationTime = 0.0;
        const double dt = 0.1;  // 100ms时间步长
        const int maxSteps = 1000;  // 最多仿真1000步
        
        std::cout << "\n开始仿真循环...\n";
        std::cout << "控制指令:\n";
        std::cout << "  空格键 - 暂停/继续\n";
        std::cout << "  +/-   - 缩放视图\n";
        std::cout << "  ESC   - 退出程序\n";
        std::cout << "\n仿真进行中...\n";
        
        bool paused = false;
        int stepCount = 0;
        
        // 随机数生成器用于添加随机性
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(-0.5, 0.5);
        
        while (stepCount < maxSteps && visualizer.isOpen()) {
            // 处理窗口事件
            if (!visualizer.handleEvents()) {
                break;  // 窗口被关闭
            }
            
            // 检查空格键暂停/继续
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Space)) {
                paused = !paused;
                std::cout << (paused ? "仿真暂停" : "仿真继续") << "\n";
                std::this_thread::sleep_for(std::chrono::milliseconds(200));  // 防抖
            }
            
            if (!paused) {
                // 更新自车位置
                egoVehicle.y += egoVehicle.vy * dt;
                
                // 模拟轻微的方向变化
                egoVehicle.x = road.getLaneCenter(2) + 0.5 * std::sin(simulationTime * 0.3);
                egoVehicle.updateTrajectory();
                
                // 更新障碍物位置
                for (auto& obs : obstacles) {
                    if (obs.type == "dynamic") {
                        // 向前运动
                        obs.y += obs.vy * dt;
                        
                        // 添加一些随机横向运动
                        double laneChange = 0.1 * std::sin(simulationTime * 0.2 + obs.id);
                        obs.x += laneChange * dt;
                        
                        // 确保车辆在道路边界内
                        int currentLane = road.getLaneIndex(obs.x);
                        double laneCenter = road.getLaneCenter(currentLane);
                        obs.x = laneCenter + 0.3 * std::sin(simulationTime * 0.1 * obs.id);
                        
                        // 更新轨迹
                        obs.updateTrajectory();
                    }
                }
                
                // 当车辆超出道路末端时，重置到起点（循环道路）
                if (egoVehicle.y > road.getRoadLength() - 50) {
                    egoVehicle.y = 50.0;
                    // 清除轨迹重新开始
                    egoVehicle.trajectory.clear();
                }
                
                for (auto& obs : obstacles) {
                    if (obs.y > road.getRoadLength() - 50) {
                        obs.y = 50.0 + (obs.id * 30);  // 错开位置
                        // 清除轨迹重新开始
                        obs.trajectory.clear();
                    }
                }
                
                simulationTime += dt;
                stepCount++;
                
                // 显示进度
                if (stepCount % 100 == 0) {
                    std::cout << "仿真时间: " << std::fixed << std::setprecision(1) 
                              << simulationTime << "s, 步骤: " << stepCount << "/" << maxSteps;
                    if (paused) std::cout << " [暂停]";
                    std::cout << "\n";
                }
            }
            
            // 绘制当前状态
            visualizer.draw(egoVehicle, obstacles, simulationTime, manager);
            
            // 控制帧率（约20 FPS）
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        
        std::cout << "\n仿真结束\n";
        std::cout << "总仿真时间: " << simulationTime << "s\n";
        std::cout << "总步数: " << stepCount << "\n";
        
        // 等待用户关闭窗口
        std::cout << "\n按任意键退出...";
        std::cin.get();
        
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "程序发生异常: " << e.what() << std::endl;
        return 1;
    }
}