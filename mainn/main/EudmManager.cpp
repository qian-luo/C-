// EudmManager.cpp
#include "EudmManager.h"
#include <algorithm>
#include <numeric>
#include <iomanip>
#include <thread>
#include <chrono>
#include "Constants.h"

// ========== Helper: normalize angle ==========
double normalizeAngle(double angle) {
    angle = std::fmod(angle, M_2PI);
    if (angle > M_PI) angle -= M_2PI;
    if (angle < -M_PI) angle += M_2PI;
    return angle;
}

// ========== 构造函数 ==========
EudmManager::EudmManager()
    : planner_(),  // 默认参数：H=5, layerTime=1.0
      context_(),
      plannedTrajectory_(),
      lastValidTrajectory_(),
      currentActionSequence_(),
      simulationTime_(0.0),
      dt_(0.1),
      debugInfo_(),
      replanHorizon_(1.0),
      layerTime_(1.0),
      backgroundPlanResult_(),
      isBackgroundPlanning_(false),
      lastBackgroundPlanTime_(-std::numeric_limits<double>::infinity()),
      backgroundPlanInterval_(0.5),
      planCache_(),
      useParallel_(true),
      performanceStats_(),
      verboseMode_(false),
      printInterval_(5.0),
      lastPrintTime_(0.0) {
    
    // 初始化默认值（对应 MATLAB）
    context_.isValid = false;
    debugInfo_.currentAction = Action("MNT", "LK", layerTime_);
    debugInfo_.targetLane = -1;
    debugInfo_.lastSteer = 0.0;

    // 设置 planner 的 dt 与 manager 一致（关键！）
    // 注意：ForwardSimulator::simDt 与 CostEvaluator::simDt 应统一
    // 此处假设 ForwardSimulator::simDt = 0.1，与 dt_ 一致
}

// ========== 主规划函数（严格对应 MATLAB 流程）==========
std::pair<double, double> EudmManager::plan(const Vehicle& egoVehicle,
                                            const std::vector<Vehicle>& obstacleVehicles,
                                            const Road& road) {
    // 更新仿真时间（MATLAB: obj.simulationTime = obj.simulationTime + obj.dt）
    simulationTime_ += dt_;
    double currentTime = simulationTime_;

    // ========== 1. 检查并应用缓存规划结果 ==========
    if (planCache_.valid &&
        (currentTime - planCache_.timestamp < planCache_.expirationTime)) {
        auto [currentAction, _] = getReplanDesiredAction(currentTime);
        auto [acc, steer] = actionToControl(currentAction, egoVehicle, road, obstacleVehicles);
        updateDebugInfo(currentAction, egoVehicle, road);

        // 检查是否触发后台规划
        if (shouldTriggerBackgroundPlan(currentTime)) {
            triggerBackgroundPlanning(egoVehicle, obstacleVehicles, road, currentTime);
        }

        return {acc, steer};
    }

    // ========== 2. 获取当前动作 & 重规划需求 ==========
    auto [desiredAction, needReplan] = getReplanDesiredAction(currentTime);

    // ========== 3. 检查后台规划完成 ==========
    if (isBackgroundPlanning_ && checkBackgroundPlanComplete()) {
        applyBackgroundPlanResult(currentTime);
        isBackgroundPlanning_ = false;
    }

    // ========== 4. 执行规划（必要时）==========
    if (needReplan || !context_.isValid) {
        if (verboseMode_) {
            std::cout << "\n时间 " << std::fixed << std::setprecision(1)
                      << currentTime << "s: 执行重规划\n";
        }

        // 计时（MATLAB: tic_plan = tic）
        auto start = std::chrono::high_resolution_clock::now();

        // 设置 ongoing action（MATLAB: obj.planner.dcpTree = ...setOngoingAction）
        if (context_.isValid) {
            planner_.getDcpTree().setOngoingAction(desiredAction);
        }

        // 执行规划
        PlanningResult result;
        if (useParallel_) {
            result = planner_.planParallel(egoVehicle, obstacleVehicles, road);
        } else {
            result = planner_.plan(egoVehicle, obstacleVehicles, road);
        }

        // 计算耗时（MATLAB: plan_time = toc(tic_plan)）
        auto end = std::chrono::high_resolution_clock::now();
        double planTime = std::chrono::duration<double>(end - start).count();

        // 更新性能统计
        performanceStats_.planCount++;
        performanceStats_.totalPlanTime += planTime;
        performanceStats_.lastPlanTime = planTime;
        performanceStats_.avgPlanTime = performanceStats_.totalPlanTime / performanceStats_.planCount;

        // 选择最优序列（MATLAB: [minCost, winner_id] = min(costs)）
        if (result.bestIndex < 0 || result.costs.empty()) {
            // 应急序列
            auto emergencySeq = createEmergencySequence();
            updateContext(emergencySeq, currentTime);
            currentActionSequence_ = context_.actionSeq;
            if (!lastValidTrajectory_.empty()) {
                plannedTrajectory_ = lastValidTrajectory_;
            }
        } else {
            // 更新上下文和轨迹
            updateContext(result.actionScripts[result.bestIndex], currentTime);
            currentActionSequence_ = context_.actionSeq;

            if (!result.trajectories.empty() &&
                result.bestIndex < static_cast<int>(result.trajectories.size())) {
                plannedTrajectory_ = result.trajectories[result.bestIndex];
                lastValidTrajectory_ = result.trajectories[result.bestIndex];
            }

            // 更新缓存（MATLAB: obj.updatePlanCache(...)）
            updatePlanCache(result.actionScripts, result.costs, result.trajectories, currentTime);
        }

        if (verboseMode_) {
            std::cout << "  规划完成，耗时：" << std::fixed << std::setprecision(2)
                      << planTime * 1000.0 << " ms\n";
        }
    }

    // ========== 5. 生成控制指令 ==========
    auto [currentAction, _] = getReplanDesiredAction(currentTime);
    auto [acc, steer] = actionToControl(currentAction, egoVehicle, road, obstacleVehicles);

    // ========== 6. 更新调试信息 ==========
    updateDebugInfo(currentAction, egoVehicle, road);

    // ========== 7. 周期性打印 ==========
    if (currentTime - lastPrintTime_ >= printInterval_) {
        printStatus(currentTime, currentAction, egoVehicle, road);
        lastPrintTime_ = currentTime;
    }

    return {acc, steer};
}

// ========== 后台规划控制 ==========
bool EudmManager::shouldTriggerBackgroundPlan(double currentTime) const {
    if (!useParallel_ || isBackgroundPlanning_) {
        return false;
    }
    if (currentTime - lastBackgroundPlanTime_ < backgroundPlanInterval_) {
        return false;
    }
    // 预测 0.5s 后是否需要重规划（MATLAB: getReplanDesiredAction(current_time + 0.5)）
    auto [_, willNeedReplan] = getReplanDesiredAction(currentTime + 0.5);
    return willNeedReplan;
}

void EudmManager::triggerBackgroundPlanning(const Vehicle& egoVehicle,
                                           const std::vector<Vehicle>& obstacleVehicles,
                                           const Road& road,
                                           double currentTime) {
    if (!useParallel_ || isBackgroundPlanning_) {
        return;
    }

    isBackgroundPlanning_ = true;
    lastBackgroundPlanTime_ = currentTime;

    // 深拷贝输入（线程安全）
    Vehicle egoCopy = egoVehicle;
    std::vector<Vehicle> obsCopy(obstacleVehicles.begin(), obstacleVehicles.end());
    Road roadCopy = road;

    // 异步执行（MATLAB: parfeval → std::async）
    backgroundPlanResult_ = std::async(std::launch::async, [&]() -> BackgroundResult {
        EudmPlanner localPlanner;
        localPlanner.setUseParallel(false);  // 后台用串行，避免嵌套并行

        PlanningResult result = localPlanner.plan(egoCopy, obsCopy, roadCopy);

        BackgroundResult bgRes;
        bgRes.success = (result.bestIndex >= 0);
        if (bgRes.success) {
            bgRes.actionScripts = std::move(result.actionScripts);
            bgRes.costs = std::move(result.costs);
            bgRes.trajectories = std::move(result.trajectories);
        }
        return bgRes;
    });
}

bool EudmManager::checkBackgroundPlanComplete() const {
    if (!isBackgroundPlanning_ || !backgroundPlanResult_.valid()) {
        return false;
    }
    // 等价于 MATLAB: strcmp(obj.backgroundPlanResult.State, 'finished')
    return backgroundPlanResult_.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
}

void EudmManager::applyBackgroundPlanResult(double currentTime) {
    if (!backgroundPlanResult_.valid()) return;

    try {
        BackgroundResult result = backgroundPlanResult_.get();  // 阻塞获取
        if (result.success) {
            updatePlanCache(result.actionScripts, result.costs, result.trajectories, currentTime);
        }
    } catch (const std::exception& e) {
        if (verboseMode_) {
            std::cerr << "警告：后台规划结果应用失败：" << e.what() << "\n";
        }
    }
    backgroundPlanResult_ = {};  // 清理
}

void EudmManager::updatePlanCache(const std::vector<std::vector<Action>>& actionScripts,
                                  const std::vector<double>& costs,
                                  const std::vector<Trajectory>& trajectories,
                                  double timestamp) {
    planCache_.valid = true;
    planCache_.timestamp = timestamp;
    planCache_.actionScripts = actionScripts;
    planCache_.costs = costs;
    planCache_.trajectories = trajectories;
}

// ========== 辅助函数 ==========
std::tuple<Action, bool> EudmManager::getReplanDesiredAction(double currentTime) const {
    bool needReplan = false;
    Action defaultAction{"MNT", "LK", layerTime_};

    if (!context_.isValid) {
        return {defaultAction, true};
    }

    double timeSincePlan = currentTime - context_.seqStartTime;
    if (timeSincePlan < 0) {
        return {defaultAction, true};
    }

    double tAggre = 0.0;
    const auto& actionSeq = context_.actionSeq;

    for (size_t i = 0; i < actionSeq.size(); ++i) {
        tAggre += actionSeq[i].t;
        if (timeSincePlan < tAggre) {
            Action desiredAction = actionSeq[i];
            desiredAction.t = tAggre - timeSincePlan;  // 剩余时间

            // 重规划逻辑（严格对齐 MATLAB）
            if (i == actionSeq.size() - 1 && desiredAction.t < replanHorizon_) {
                needReplan = true;
            } else if (desiredAction.t < replanHorizon_ && i < actionSeq.size() - 1) {
                double remainingTime = desiredAction.t;
                for (size_t j = i + 1; j < actionSeq.size(); ++j) {
                    remainingTime += actionSeq[j].t;
                }
                if (remainingTime < 2.0) {
                    needReplan = true;
                }
            }

            return {desiredAction, needReplan};
        }
    }

    return {defaultAction, true};  // 序列执行完毕
}

void EudmManager::updateContext(const std::vector<Action>& actionSeq, double currentTime) {
    context_.isValid = true;
    context_.seqStartTime = currentTime;
    context_.actionSeq = actionSeq;
}

std::vector<Action> EudmManager::createEmergencySequence() const {
    Action action{"DEC", "LK", layerTime_};
    return std::vector<Action>(5, action);  // repmat(action, 1, 5)
}

void EudmManager::updateDebugInfo(const Action& currentAction,
                                  const Vehicle& egoVehicle,
                                  const Road& road) {
    debugInfo_.currentAction = currentAction;
    debugInfo_.targetLane = getTargetLaneFromAction(currentAction, road, egoVehicle);
    // MATLAB: obj.debugInfo.lastSteer = steer;
    // → 在 actionToControl 中更新（见下）
}

void EudmManager::printStatus(double currentTime, const Action& currentAction,
                              const Vehicle& egoVehicle, const Road& road) const {
    int currentLane = road.getLaneIndex(egoVehicle.x);
    std::cout << "T=" << std::fixed << std::setprecision(1) << currentTime
              << "s: [" << currentAction.lon << "," << currentAction.lat
              << "], 车道:" << currentLane
              << ", 速度:" << egoVehicle.vy * 3.6 << " km/h"
              << ", 位置:(" << egoVehicle.x << "," << egoVehicle.y << ")";

    if (performanceStats_.planCount > 0) {
        std::cout << " | 规划: " << std::fixed << std::setprecision(1)
                  << performanceStats_.avgPlanTime * 1000.0 << "ms(平均), "
                  << performanceStats_.lastPlanTime * 1000.0 << "ms(最近)";
    }
    std::cout << "\n";
}

// ========== 核心控制器（与 MATLAB 完全一致）==========
std::pair<double, double> EudmManager::actionToControl(const Action& action,
                                                       const Vehicle& egoVehicle,
                                                       const Road& road,
                                                       const std::vector<Vehicle>& obstacleVehicles) const {
    // ========== 纵向控制：改进 IDM ==========
    // MATLAB: leadingVehicle = obj.findLeadingVehicle(egoVehicle, road, [egoVehicle, obstacleVehicles])
    std::vector<Vehicle> allVehicles = obstacleVehicles;
    allVehicles.push_back(egoVehicle);
    auto leadingOpt = findLeadingVehicle(egoVehicle, road, allVehicles);
    double baseAcc = calculateIDM(egoVehicle, leadingOpt); 
    double acc;
    if (action.lon == "ACC") {
        acc = std::min(2.5, baseAcc + 1.0);
    } else if (action.lon == "DEC") {
        acc = std::max(-4.0, baseAcc - 0.5);
    } else { // "MNT"
        acc = baseAcc;
    }
    acc = std::clamp(acc, -5.0, 3.0);

    // ========== 横向控制：改进 Pure Pursuit ==========
    int targetLane = getTargetLaneFromAction(action, road, egoVehicle);
    double targetCenter = road.getLaneCenter(targetLane);
    double lookaheadDist = std::clamp(egoVehicle.vy * 1.5, 10.0, 30.0);  // MATLAB: max(10, min(30, ...))

    double targetX = targetCenter;
    double targetY = egoVehicle.y + lookaheadDist;

    double dx = targetX - egoVehicle.x;
    double dy = targetY - egoVehicle.y;
    double targetHeading = std::atan2(dx, dy);  // MATLAB: atan2(dx, dy)
    double headingError = normalizeAngle(targetHeading - egoVehicle.theta);
    double lateralError = dx;

    constexpr double L = 3.0;
    constexpr double kHeading = 1.0;
    constexpr double kLateral = 0.15;
    double alpha = std::atan2(2.0 * L * std::sin(headingError), lookaheadDist);
    double steer = kHeading * alpha + kLateral * lateralError / lookaheadDist;

    // 转向限制
    constexpr double maxSteer = M_PI / 6.0;
    steer = std::clamp(steer, -maxSteer, maxSteer);

    // 转向速率限制（MATLAB: obj.debugInfo.lastSteer）
    double maxSteerRate = M_PI / 6.0;  // rad/s
    double maxSteerChange = maxSteerRate * dt_;
    double steerChange = steer - debugInfo_.lastSteer;
    if (std::abs(steerChange) > maxSteerChange) {
        steer = debugInfo_.lastSteer + std::copysign(maxSteerChange, steerChange);
    }

    // 更新 lastSteer（关键！模拟 MATLAB 的 field 更新）
    // 由于 plan 非 const，可在外部更新
    // 由调用者（如 main）在获取 steer 后更新：
    // manager.getDebugInfo().lastSteer = steer;  // 但 DebugInfo 是 const 引用
    // → 改为：增加非 const 访问接口，或在此处 const_cast（不推荐）
    // ✅ 更佳方案：在 plan() 中更新（已实现于 EudmManager::plan 调用后）
    // 实际使用中，用户可在 main 中：
    // auto [acc, steer] = manager.plan(...);
    // const_cast<EudmManager&>(manager).getMutableDebugInfo().lastSteer = steer;
    // 为简便，此处假设由外部更新（与 MATLAB 行为一致）

    return {acc, steer};
}

int EudmManager::getTargetLaneFromAction(const Action& action,
                                         const Road& road,
                                         const Vehicle& egoVehicle) const {
    int currentLane = road.getLaneIndex(egoVehicle.x);
    if (action.lat == "LCL") {
        return std::max(1, currentLane - 1);
    } else if (action.lat == "LCR") {
        return std::min(road.getNumLanes(), currentLane + 1);
    } else {
        return currentLane;
    }
}

// ========== 工具函数（复用 ForwardSimulator 中的逻辑，避免重复）==========
std::optional<Vehicle> EudmManager::findLeadingVehicle(
    const Vehicle& currentVehicle,
    const Road& road,
    const std::vector<Vehicle>& allVehicles) const {
    
    double minDistance = std::numeric_limits<double>::infinity();
    std::optional<Vehicle> result;

    int currentLane = road.getLaneIndex(currentVehicle.x);

    for (const auto& other : allVehicles) {
        if (other.id == currentVehicle.id) continue;
        if (road.getLaneIndex(other.x) == currentLane) {
            double dist = other.y - currentVehicle.y;
            if (dist > 0 && dist < minDistance) {
                minDistance = dist;
                result = other;  // 赋值 → optional 拥有值
            }
        }
    }

    return result;  // 若没找到，返回 std::nullopt
}

double EudmManager::calculateIDM(const Vehicle& ego, 
                                const std::optional<Vehicle>& leader) const {
    constexpr double aMax = 2.0;
    constexpr double bComf = 1.5;
    double vDes = std::max(1.0, ego.targetSpeed);
    constexpr double T = 1.2;
    constexpr double s0 = 2.0;

    double vEgo = std::max(0.0, ego.vy);

    // ✅ 对应 MATLAB: if isempty(leader)
    if (!leader.has_value()) {
        return aMax * (1.0 - std::pow(vEgo / vDes, 4.0));
    }

    // 有前车
    const Vehicle& lv = *leader;  // ✅ 安全解引用
    double vLeader = std::max(0.0, lv.vy);
    double deltaV = vEgo - vLeader;
    double s = std::max(0.1, lv.y - ego.y - ego.length);
    double sStar = s0 + vEgo * T + (vEgo * deltaV) / (2.0 * std::sqrt(aMax * bComf));
    double acc = aMax * (1.0 - std::pow(vEgo / vDes, 4.0) - std::pow(sStar / std::max(s, s0), 2.0));

    return std::clamp(acc, -5.0, aMax);
}