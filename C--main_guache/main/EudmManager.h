// EudmManager.h
#ifndef EUDM_MANAGER_H
#define EUDM_MANAGER_H

#include <vector>
#include <string>
#include <memory>
#include <future>
#include <chrono>
#include <cmath>
#include <limits>
#include <tuple>
#include <iostream>
#include <optional>  // C++17 起标准库

// 复用已记录模块
#include "Road.h"
#include "Vehicle.h"
#include "DcpTree.h"
#include "CostEvaluator.h"
#include "EudmPlanner.h"
#include "ForwardSimulator.h"
#include "State.h"

// ========== 轻量结构体定义（对应 MATLAB struct）==========
struct Context {
    bool isValid = false;
    double seqStartTime = -std::numeric_limits<double>::infinity();
    std::vector<Action> actionSeq;
};

struct DebugInfo {
    Action currentAction{};
    int targetLane = -1;
    double lastSteer = 0.0;  // 用于转向速率限制
    bool isLastSteerInitialized = false;
};

struct PerformanceStats {
    size_t planCount = 0;
    double totalPlanTime = 0.0;
    double avgPlanTime = 0.0;
    double lastPlanTime = 0.0;
};

struct PlanCache {
    bool valid = false;
    double timestamp = -std::numeric_limits<double>::infinity();
    std::vector<std::vector<Action>> actionScripts;
    std::vector<double> costs;
    std::vector<Trajectory> trajectories;
    double expirationTime = 0.8;  // s
};

// 后台规划结果
struct BackgroundResult {
    std::vector<std::vector<Action>> actionScripts;
    std::vector<double> costs;
    std::vector<Trajectory> trajectories;
    bool success = false;
};

class EudmManager {
private:
    // ========== 属性（严格按 MATLAB 顺序）==========
    EudmPlanner planner_;
    Context context_;
    Trajectory plannedTrajectory_;
    Trajectory lastValidTrajectory_;
    std::vector<Action> currentActionSequence_;
    double simulationTime_ = 0.0;
    double dt_ = 0.1;
    DebugInfo debugInfo_;
    double replanHorizon_ = 1.0;
    double layerTime_ = 1.0;

    // 并行计算优化
    std::future<BackgroundResult> backgroundPlanResult_;
    bool isBackgroundPlanning_ = false;
    double lastBackgroundPlanTime_ = -std::numeric_limits<double>::infinity();
    double backgroundPlanInterval_ = 0.5;
    PlanCache planCache_;
    bool useParallel_ = true;  // MATLAB: useParallel = true
    PerformanceStats performanceStats_;
    bool verboseMode_ = false;
    double printInterval_ = 5.0;
    double lastPrintTime_ = 0.0;

    // ========== 私有辅助方法 ==========
    std::tuple<Action, bool> getReplanDesiredAction(double currentTime) const;
    void updateContext(const std::vector<Action>& actionSeq, double currentTime);
    std::vector<Action> createEmergencySequence() const;
    void updatePlanCache(const std::vector<std::vector<Action>>& actionScripts,
                         const std::vector<double>& costs,
                         const std::vector<Trajectory>& trajectories,
                         double timestamp);
    void updateDebugInfo(const Action& currentAction, const Vehicle& egoVehicle, const Road& road);
    void printStatus(double currentTime, const Action& currentAction,
                     const Vehicle& egoVehicle, const Road& road) const;
    bool shouldTriggerBackgroundPlan(double currentTime) const;
    void triggerBackgroundPlanning(const Vehicle& egoVehicle,
                                   const std::vector<Vehicle>& obstacleVehicles,
                                   const Road& road,
                                   double currentTime);
    bool checkBackgroundPlanComplete() const;
    void applyBackgroundPlanResult(double currentTime);

        // ✅ 新增：actionToControl 声明
    std::pair<double, double> actionToControl(const Action& action,
                                             const Vehicle& egoVehicle,
                                             const Road& road,
                                             const std::vector<Vehicle>& obstacleVehicles) ;

    // 其他已有的私有函数...
    int getTargetLaneFromAction(const Action& action,
                                const Road& road,
                                const Vehicle& egoVehicle) const;
    std::optional<Vehicle> findLeadingVehicle(const Vehicle& currentVehicle,
                                        const Road& road,
                                        const std::vector<Vehicle>& allVehicles) const;
    double calculateIDM(const Vehicle& ego, const std::optional<Vehicle>& leader) const;
public:
    // ========== 公有方法 ==========
    EudmManager();
    ~EudmManager() = default;

    void updateLastSteer(double steer) { 
        debugInfo_.lastSteer = steer; 
    }

    std::pair<double, double> plan(const Vehicle& egoVehicle,
                                   const std::vector<Vehicle>& obstacleVehicles,
                                   const Road& road);

    // Getter / Setter（保持接口一致）
    void setUseParallel(bool enable) { useParallel_ = enable; }
    void setVerbose(bool enable) { verboseMode_ = enable; }
    void setDt(double dt) { dt_ = dt; }
    void setReplanHorizon(double t) { replanHorizon_ = t; }
    void setLayerTime(double t) { layerTime_ = t; }
    void setBackgroundPlanInterval(double t) { backgroundPlanInterval_ = t; }

    // 只读访问（调试用）
    double getSimulationTime() const { return simulationTime_; }
    const Context& getContext() const { return context_; }
    const Trajectory& getPlannedTrajectory() const { return plannedTrajectory_; }
    const DebugInfo& getDebugInfo() const { return debugInfo_; }
    const PerformanceStats& getPerformanceStats() const { return performanceStats_; }
    // 新增：获取当前动作序列（供 Visualizer 使用）
    const std::vector<Action>& getCurrentActionSequence() const { return currentActionSequence_; }

    // 新增：非 const debugInfo 访问（解决 steer 更新问题）
    DebugInfo& getMutableDebugInfo() { return debugInfo_; }
};


#endif // EUDM_MANAGER_H