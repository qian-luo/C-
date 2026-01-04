// EudmPlanner.h
#ifndef EUDM_PLANNER_H
#define EUDM_PLANNER_H

#include <vector>
#include <string>
#include <memory>
#include <future>
#include <chrono>
#include <iostream>
#include <cmath>

// 依赖模块头文件
#include "DcpTree.h"
#include "ForwardSimulator.h"
#include "CostEvaluator.h"
#include "Road.h"
#include "Vehicle.h"

// 输出结构体：规划结果
struct PlanningResult {
    std::vector<std::vector<Action>> actionScripts;  // 所有序列（或最优序列）
    std::vector<double> costs;                       // 对应成本
    std::vector<Trajectory> trajectories;            // 对应 ego 轨迹
    int bestIndex = -1;                              // 最优序列索引（-1 表示无可行解）
};

class EudmPlanner {
private:
    DcpTree dcpTree_;
    ForwardSimulator simulator_;
    CostEvaluator costEvaluator_;

    bool useParallel_ = true;
    bool verbose_ = false;

    // Helper: 将 action 序列转为可读字符串（仅前3层）
    static std::string sequenceToString(const std::vector<Action>& seq);

    // Helper: 生成默认轨迹（LK 直线）
    Trajectory generateDefaultTrajectory(const Vehicle& ego, const Road& road) const;

public:
    // 构造函数
    EudmPlanner();

    // 主规划接口（串行）
    PlanningResult plan(const Vehicle& egoVehicle,
                        const std::vector<Vehicle>& obstacleVehicles,
                        const Road& road);

    // 主规划接口（并行）
    PlanningResult planParallel(const Vehicle& egoVehicle,
                                const std::vector<Vehicle>& obstacleVehicles,
                                const Road& road);

    // Getter / Setter
    void setUseParallel(bool enable) { useParallel_ = enable; }
    void setVerbose(bool enable) { verbose_ = enable; }
    bool getUseParallel() const { return useParallel_; }
    bool getVerbose() const { return verbose_; }

    // 只读访问组件（调试用）
    const DcpTree& getDcpTree() const { return dcpTree_; }
     DcpTree& getDcpTree() { return dcpTree_; }
    const ForwardSimulator& getSimulator() const { return simulator_; }
    const CostEvaluator& getCostEvaluator() const { return costEvaluator_; }
};

#endif // EUDM_PLANNER_H