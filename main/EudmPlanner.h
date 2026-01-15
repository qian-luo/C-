#ifndef EUDM_PLANNER_H
#define EUDM_PLANNER_H

#include <vector>
#include <string>
#include <limits>
#include <cmath>
#include <iomanip>
#include <thread>
#include <future>
#include <algorithm>
#include <functional>
#include <iostream> 
#include <iomanip>

// 依赖模块头文件（保持你的原有顺序）
#include "DcpTree.h"
#include "ForwardSimulator.h"
#include "CostEvaluator.h"
#include "Road.h"
#include "Vehicle.h"
#include "State.h"
// 规划结果输出结构体（不变）
struct PlanningResult {
    std::vector<std::vector<Action>> actionScripts;  // 所有动作序列
    std::vector<double> costs;                       // 序列对应代价
    std::vector<Trajectory> trajectories;            // 序列对应轨迹
    int bestIndex = -1;                              // 最优序列索引，-1=无可行解
};

class EudmPlanner {
private:
    DcpTree dcpTree_;               // 决策树
    ForwardSimulator simulator_;    // 前向仿真器
    CostEvaluator costEvaluator_;   // 代价评估器
    bool useParallel_ = true;       // 是否启用并行
    bool verbose_ = false;          // 日志打印开关

    // ===== 私有核心辅助函数 =====
    // 1. 动作序列转字符串 (对齐MATLAB: 仅显示前3层+...)
    std::string sequenceToString(const std::vector<Action>& seq) const;
    // 2. 生成默认轨迹 (LK直线保持车道，对齐MATLAB逻辑)
    Trajectory generateDefaultTrajectory(const Vehicle& ego, const Road& road) const;
    // 3. 评估单个动作序列【核心复用函数】，替换原有全局静态函数，消除资源竞争
    std::tuple<double, Trajectory, bool> evaluateSingleSequence(
        const std::vector<Action>& sequence,
        const Vehicle& egoInit,
        const std::vector<Vehicle>& obsInit,
        const Road& road,
        double targetSpeed,
        double discountFactor) const;

public:
    // 构造函数 (对齐MATLAB: 初始化参数完全一致)
    EudmPlanner();

    // ===== 对外规划接口 =====
    PlanningResult plan(const Vehicle& egoVehicle,
                        const std::vector<Vehicle>& obstacleVehicles,
                        const Road& road);

    PlanningResult planParallel(const Vehicle& egoVehicle,
                                const std::vector<Vehicle>& obstacleVehicles,
                                const Road& road);

    // ===== Setter/Getter 配置接口 =====
    void setUseParallel(bool enable) { useParallel_ = enable; }
    void setVerbose(bool enable) { verbose_ = enable; }
    bool getUseParallel() const { return useParallel_; }
    bool getVerbose() const { return verbose_; }

    // 调试用只读访问器
    const DcpTree& getDcpTree() const { return dcpTree_; }
    DcpTree& getDcpTree() { return dcpTree_; }
    const ForwardSimulator& getSimulator() const { return simulator_; }
    const CostEvaluator& getCostEvaluator() const { return costEvaluator_; }
};

#endif // EUDM_PLANNER_H