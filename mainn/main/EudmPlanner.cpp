// EudmPlanner.cpp
#include "EudmPlanner.h"
#include <algorithm>
#include <limits>
#include <cmath>
#include <iomanip>
#include <thread>
#include <future>

// Helper: 序列转字符串
std::string EudmPlanner::sequenceToString(const std::vector<Action>& seq) {
    if (seq.empty()) return "[]";
    std::string res;
    size_t n = std::min<size_t>(3, seq.size());
    for (size_t i = 0; i < n; ++i) {
        res += "[" + seq[i].lon + "," + seq[i].lat + "]";
    }
    if (seq.size() > 3) {
        res += "...";
    }
    return res;
}

// Helper: 默认轨迹生成
Trajectory EudmPlanner::generateDefaultTrajectory(const Vehicle& ego, const Road& road) const {
    int laneIdx = road.getLaneIndex(ego.x);
    double laneCenter = road.getLaneCenter(laneIdx);

    // 计算总时长
    double totalT = dcpTree_.getOngoingAction().t;
    if (dcpTree_.getTreeHeight() > 2) {
        totalT += (dcpTree_.getTreeHeight() - 2) * dcpTree_.getLayerTime();
    }
    if (dcpTree_.getTreeHeight() > 1) {
        totalT += dcpTree_.getLastLayerTime();
    }

    size_t numPoints = static_cast<size_t>(std::ceil(totalT / simulator_.simDt)) + 1;
    Trajectory traj;
    traj.reserve(numPoints);

    for (size_t i = 0; i < numPoints; ++i) {
        double t = i * simulator_.simDt;
        State pt{};
        pt.x = laneCenter;
        pt.y = ego.y + t * ego.vy;
        pt.vx = 0.0;
        pt.vy = ego.vy;
        pt.theta = 0.0;  // 假设直线行驶 theta≈0（北向）
        traj.push_back(pt);
    }
    return traj;
}

// 构造函数
EudmPlanner::EudmPlanner()
    : dcpTree_(5, 1.0, 1.0),  // treeHeight=5, layerTime=1.0, lastLayerTime=1.0
      simulator_(),
      costEvaluator_() {
    // 对齐 simDt（关键！避免 comfort cost 计算偏差）
    costEvaluator_.setSimDt(simulator_.simDt);
}

// 评估单个序列的内部函数（供串行/并行复用）
static std::tuple<double, Trajectory, bool>
evaluateSequence(
    const std::vector<Action>& sequence,
    const Vehicle& egoInit,
    const std::vector<Vehicle>& obsInit,
    const Road& road,
     ForwardSimulator& simulator,
    const CostEvaluator& evaluator,
    double targetSpeed,
    int treeHeight,
    double discountFactor = 0.9) {

    Vehicle simEgo = simulator.copyVehicle(egoInit);
    std::vector<Vehicle> simObs(obsInit.begin(), obsInit.end()); // 深拷贝

    double totalCost = 0.0;
    Trajectory fullTrajectory;
    bool feasible = true;

    for (int layer = 0; layer < treeHeight; ++layer) { // 0-based
        const Action& action = sequence[layer];

        // 仿真该层
        auto [egoLayerTraj, obsLayerTrajs, isLayerFeasible] =
            simulator.simulateLayer(action, simEgo, simObs, road);

        if (!isLayerFeasible || egoLayerTraj.empty()) {
            feasible = false;
            break;
        }

        // 构建障碍物轨迹（Nobs × T）
        ObstacleTrajectories obsTrajsForCost;
        size_t numObs = simObs.size();
        if (!obsLayerTrajs.empty()) {
            // obsLayerTrajs: 长度 = T × Nobs（每步 append 所有 obs）
            size_t T = obsLayerTrajs.size() / numObs;
            obsTrajsForCost.resize(numObs);
            for (size_t i = 0; i < numObs; ++i) {
                obsTrajsForCost[i].reserve(T);
                for (size_t t = 0; t < T; ++t) {
                    size_t idx = t * numObs + i;
                    if (idx < obsLayerTrajs.size()) {
                        // TrajectoryPoint → State
                        State s{};
                        s.x = obsLayerTrajs[idx].x;
                        s.y = obsLayerTrajs[idx].y;
                        s.vx = obsLayerTrajs[idx].vx;
                        s.vy = obsLayerTrajs[idx].vy;
                        s.theta = obsLayerTrajs[idx].theta;
                        obsTrajsForCost[i].push_back(s);
                    }
                }
            }
        }

        // 评估该层成本
        State initState{};
        initState.x = simEgo.x; initState.y = simEgo.y;
        initState.vx = simEgo.vx; initState.vy = simEgo.vy;
        initState.theta = simEgo.theta;

        auto [layerCost, _] = evaluator.evaluate(
            action, egoLayerTraj, obsTrajsForCost, road, targetSpeed, initState);

        totalCost += std::pow(discountFactor, layer) * layerCost;
        fullTrajectory.insert(fullTrajectory.end(), egoLayerTraj.begin(), egoLayerTraj.end());

        // 更新状态（取最后一帧）
        if (!egoLayerTraj.empty()) {
            const auto& last = egoLayerTraj.back();
            simEgo.x = last.x; simEgo.y = last.y;
            simEgo.vx = last.vx; simEgo.vy = last.vy;
            simEgo.theta = last.theta;
        }

        // 更新障碍物（按列提取）
        if (!obsLayerTrajs.empty()) {
            size_t T = obsLayerTrajs.size() / numObs;
            for (size_t k = 0; k < numObs; ++k) {
                size_t lastIdx = (T - 1) * numObs + k;
                if (lastIdx < obsLayerTrajs.size()) {
                    const auto& lastObs = obsLayerTrajs[lastIdx];
                    simObs[k].x = lastObs.x;
                    simObs[k].y = lastObs.y;
                    simObs[k].vx = lastObs.vx;
                    simObs[k].vy = lastObs.vy;
                    simObs[k].theta = lastObs.theta;
                }
            }
        }
    }

    return std::make_tuple(totalCost, fullTrajectory, feasible);
}

// 串行规划
PlanningResult EudmPlanner::plan(const Vehicle& egoVehicle,
                                 const std::vector<Vehicle>& obstacleVehicles,
                                 const Road& road) {
    auto actionScripts = dcpTree_.generateActionScript(); // vector<vector<Action>>
    size_t numScripts = actionScripts.size();

    std::vector<double> costs(numScripts, std::numeric_limits<double>::infinity());
    std::vector<Trajectory> trajectories(numScripts);
    int validCount = 0;

    if (verbose_) {
        std::cout << "===== EUDM Planner (串行) =====\n";
        int lane = road.getLaneIndex(egoVehicle.x);
        std::cout << "  自车: 速度=" << std::fixed << std::setprecision(1)
                  << egoVehicle.vy * 3.6 << " km/h, 车道=" << lane
                  << ", 位置=(" << egoVehicle.x << ", " << egoVehicle.y << ")\n";
        const auto& ongoing = dcpTree_.getOngoingAction();
        std::cout << "  Ongoing: [" << ongoing.lon << "," << ongoing.lat
                  << "], t=" << std::setprecision(2) << ongoing.t << "s\n";
    }

    const double discountFactor = 0.9;
    const double targetSpeed = egoVehicle.targetSpeed;
    const int treeHeight = dcpTree_.getTreeHeight();

    for (size_t i = 0; i < numScripts; ++i) {
        auto [cost, traj, feasible] = evaluateSequence(
            actionScripts[i], egoVehicle, obstacleVehicles, road,
            simulator_, costEvaluator_, targetSpeed, treeHeight, discountFactor);

        if (feasible) {
            costs[i] = cost;
            trajectories[i] = std::move(traj);
            ++validCount;
        }
    }

    if (verbose_) {
        std::cout << "  评估了 " << numScripts << " 个序列，" << validCount << " 个可行\n";
    }

    // 全失败 → 默认序列
    PlanningResult result;
    if (std::all_of(costs.begin(), costs.end(),
                    [](double c) { return c == std::numeric_limits<double>::infinity(); })) {
        if (verbose_) {
            std::cout << "  警告：所有序列都不可行，生成默认LK序列\n";
        }

        // 构建默认序列
        std::vector<Action> defaultSeq(treeHeight);
        for (int i = 0; i < treeHeight; ++i) {
            defaultSeq[i].lon = "MNT";
            defaultSeq[i].lat = "LK";
            defaultSeq[i].t = dcpTree_.getLayerTime();
        }
        defaultSeq[0].t = dcpTree_.getOngoingAction().t;
        defaultSeq[treeHeight - 1].t = dcpTree_.getLastLayerTime();

        result.actionScripts = {defaultSeq};  // 单序列
        result.costs = {1000.0};
        result.trajectories = {generateDefaultTrajectory(egoVehicle, road)};
        result.bestIndex = 0;
    } else {
        // 找最优
        auto minIt = std::min_element(costs.begin(), costs.end());
        result.bestIndex = static_cast<int>(std::distance(costs.begin(), minIt));

        result.actionScripts = std::move(actionScripts);
        result.costs = std::move(costs);
        result.trajectories = std::move(trajectories);

        if (verbose_ && result.bestIndex >= 0) {
            double minCost = *minIt;
            std::string seqStr = sequenceToString(result.actionScripts[result.bestIndex]);
            std::cout << "  最优: 序列" << (result.bestIndex + 1) 
                      << " " << seqStr << ", 成本=" << std::fixed << std::setprecision(2)
                      << minCost << "\n";
        }
    }

    return result;
}

// 并行规划
PlanningResult EudmPlanner::planParallel(const Vehicle& egoVehicle,
                                         const std::vector<Vehicle>& obstacleVehicles,
                                         const Road& road) {
    auto actionScripts = dcpTree_.generateActionScript();
    size_t numScripts = actionScripts.size();

    if (verbose_) {
        std::cout << "===== EUDM Planner (并行) =====\n";
        int lane = road.getLaneIndex(egoVehicle.x);
        std::cout << "  自车: 速度=" << std::fixed << std::setprecision(1)
                  << egoVehicle.vy * 3.6 << " km/h, 车道=" << lane
                  << ", 位置=(" << egoVehicle.x << ", " << egoVehicle.y << ")\n";
        std::cout << "  评估 " << numScripts << " 个动作序列...\n";
    }

    const double discountFactor = 0.9;
    const double targetSpeed = egoVehicle.targetSpeed;
    const int treeHeight = dcpTree_.getTreeHeight();

    // 启动异步任务
    std::vector<std::future<std::tuple<double, Trajectory, bool>>> futures;
    futures.reserve(numScripts);

    for (size_t i = 0; i < numScripts; ++i) {
        futures.emplace_back(std::async(std::launch::async,
    [](const std::vector<Action>& seq,
       const Vehicle& ego,
       const std::vector<Vehicle>& obs,
       const Road& rd,
       double speed,
       int height,
       double factor) -> std::tuple<double, Trajectory, bool> {
        // 每个线程独立创建仿真器和评估器
        ForwardSimulator local_sim;
        CostEvaluator local_eval;
        local_eval.setSimDt(local_sim.simDt); // 对齐 dt

        // 调用 evaluateSequence（现在传入的是局部对象）
        return evaluateSequence(seq, ego, obs, rd, local_sim, local_eval, speed, height, factor);
    },
    actionScripts[i], egoVehicle, obstacleVehicles, road,
    targetSpeed, treeHeight, discountFactor));
    }

    // 收集结果
    std::vector<double> costs(numScripts, std::numeric_limits<double>::infinity());
    std::vector<Trajectory> trajectories(numScripts);
    int validCount = 0;

    for (size_t i = 0; i < numScripts; ++i) {
        auto [cost, traj, feasible] = futures[i].get();
        if (feasible) {
            costs[i] = cost;
            trajectories[i] = std::move(traj);
            ++validCount;
        }
    }

    if (verbose_) {
        std::cout << "  并行评估完成，" << validCount << " 个序列可行\n";
    }

    // 全失败 → 默认序列（同串行）
    PlanningResult result;
    if (std::all_of(costs.begin(), costs.end(),
                    [](double c) { return c == std::numeric_limits<double>::infinity(); })) {
        if (verbose_) {
            std::cout << "  警告：所有序列都不可行，生成默认LK序列\n";
        }

        std::vector<Action> defaultSeq(treeHeight);
        for (int i = 0; i < treeHeight; ++i) {
            defaultSeq[i].lon = "MNT";
            defaultSeq[i].lat = "LK";
            defaultSeq[i].t = dcpTree_.getLayerTime();
        }
        defaultSeq[0].t = dcpTree_.getOngoingAction().t;
        defaultSeq[treeHeight - 1].t = dcpTree_.getLastLayerTime();

        result.actionScripts = {defaultSeq};
        result.costs = {1000.0};
        result.trajectories = {generateDefaultTrajectory(egoVehicle, road)};
        result.bestIndex = 0;
    } else {
        auto minIt = std::min_element(costs.begin(), costs.end());
        result.bestIndex = static_cast<int>(std::distance(costs.begin(), minIt));

        result.actionScripts = std::move(actionScripts);
        result.costs = std::move(costs);
        result.trajectories = std::move(trajectories);

        if (verbose_ && result.bestIndex >= 0) {
            std::string seqStr = sequenceToString(result.actionScripts[result.bestIndex]);
            std::cout << "  最优: 序列" << (result.bestIndex + 1) 
                      << " " << seqStr << ", 成本=" << std::fixed << std::setprecision(2)
                      << *minIt << "\n";
        }
    }

    return result;
}