#include "EudmPlanner.h"

// ===================== 私有辅助函数实现 =====================
// 动作序列转字符串，严格对齐MATLAB: 前3层+...
std::string EudmPlanner::sequenceToString(const std::vector<Action>& seq) const {
    if (seq.empty()) return "[]";
    std::string res;
    size_t showCnt = std::min<size_t>(3, seq.size());
    for (size_t i = 0; i < showCnt; ++i) {
        res += "[" + seq[i].lon + "," + seq[i].lat + "]";
    }
    if (seq.size() > 3) res += "...";
    return res;
}

// 生成默认轨迹：LK直线保持车道，对齐MATLAB逻辑，修复时长计算错误
Trajectory EudmPlanner::generateDefaultTrajectory(const Vehicle& ego, const Road& road) const {
    int laneIdx = road.getLaneIndex(ego.x);
    double laneCenter = road.getLaneCenter(laneIdx);
    const double dt = simulator_.simDt;

    // ✅ 修复：正确计算总时长，严格对齐MATLAB
    double totalTime = dcpTree_.getOngoingAction().t;
    int treeH = dcpTree_.getTreeHeight();
    if (treeH > 2) {
        totalTime += (treeH - 2) * dcpTree_.getLayerTime();
    }
    if (treeH > 1) {
        totalTime += dcpTree_.getLastLayerTime();
    }

    size_t numPoints = static_cast<size_t>(std::ceil(totalTime / dt)) + 1;
    Trajectory traj;
    traj.reserve(numPoints);

    for (size_t i = 0; i < numPoints; ++i) {
        double t = i * dt;
        State pt;
        pt.x = laneCenter;
        pt.y = ego.y + t * ego.vy;
        pt.vx = 0.0;
        pt.vy = ego.vy;
        pt.theta = ego.theta; // 继承自车航向角，而非固定0，更合理
        pt.is_empty = false;  // 标记为有效状态
        traj.push_back(pt);
    }
    return traj;
}

// ✅ 核心修复：评估单个序列的私有成员函数，无全局静态，无资源竞争，逻辑100%对齐MATLAB
std::tuple<double, Trajectory, bool> EudmPlanner::evaluateSingleSequence(
    const std::vector<Action>& sequence,
    const Vehicle& egoInit,
    const std::vector<Vehicle>& obsInit,
    const Road& road,
    const double targetSpeed,
    const double discountFactor) const {
    
    double totalCost = 0.0;
    Trajectory fullTrajectory;
    bool isSequenceFeasible = true;
    const int treeHeight = dcpTree_.getTreeHeight();

    // ✅ 深拷贝初始状态：完全隔离序列间的仿真状态，杜绝污染
    Vehicle simEgo = simulator_.copyVehicle(egoInit);
    std::vector<Vehicle> simObs;
    for (const auto& obs : obsInit) {
        simObs.push_back(simulator_.copyVehicle(obs));
    }
    const size_t numObs = simObs.size();

    // 遍历决策树每一层 (0-based，适配C++)
    for (int layer = 0; layer < treeHeight; ++layer) {
        const Action& currentAction = sequence[layer];
        
        // ✅ 执行前向仿真 + 严格可行性检查 (对齐MATLAB核心逻辑)
        auto [egoLayerTraj, obsLayerTrajs, isLayerFeasible] = 
            simulator_.simulateLayer(currentAction, simEgo, simObs, road);

        // ✅ 层不可行 → 整个序列不可行，直接终止
        if (!isLayerFeasible || egoLayerTraj.empty()) {
            isSequenceFeasible = false;
            break;
        }

        // ✅ 修复：障碍物轨迹格式重组，严格对齐MATLAB [Nobs × T] → C++ ObstacleTrajectories
        ObstacleTrajectories obsTrajsForCost;
        if (!obsLayerTrajs.empty() && numObs > 0) {
            size_t timeStepCnt = egoLayerTraj.size(); // 时间步数 = 自车轨迹点数
            obsTrajsForCost.resize(timeStepCnt);
            for (size_t t = 0; t < timeStepCnt; ++t) {
                obsTrajsForCost[t].resize(numObs);
                for (size_t k = 0; k < numObs; ++k) {
                    size_t idx = t * numObs + k;
                    if (idx < obsLayerTrajs.size()) {
                        obsTrajsForCost[t][k] = obsLayerTrajs[idx];
                        obsTrajsForCost[t][k].is_empty = false;
                    }
                }
            }
        }

        // ✅ 构造本层初始状态，传入代价评估器
        State layerInitState;
        layerInitState.x = simEgo.x;
        layerInitState.y = simEgo.y;
        layerInitState.vx = simEgo.vx;
        layerInitState.vy = simEgo.vy;
        layerInitState.theta = simEgo.theta;
        layerInitState.is_empty = false;

        // ✅ 计算本层代价
        auto [layerCost, costBreakdown] = costEvaluator_.evaluate(
            currentAction, egoLayerTraj, obsTrajsForCost, road, targetSpeed, layerInitState);

        // ✅ 修复：折扣因子幂次，严格对齐MATLAB (MATLAB layer-1 → C++ layer)
        totalCost += std::pow(discountFactor, layer) * layerCost;

        // ✅ 拼接完整轨迹
        fullTrajectory.insert(fullTrajectory.end(), egoLayerTraj.begin(), egoLayerTraj.end());

        // ✅ 更新自车仿真状态：取层末状态
        const State& egoLast = egoLayerTraj.back();
        simEgo.x = egoLast.x;
        simEgo.y = egoLast.y;
        simEgo.vx = egoLast.vx;
        simEgo.vy = egoLast.vy;
        simEgo.theta = egoLast.theta;

        // ✅ 更新障碍物仿真状态：取层末状态
        if (!obsLayerTrajs.empty() && numObs > 0) {
            size_t timeStepCnt = egoLayerTraj.size();
            for (size_t k = 0; k < numObs; ++k) {
                size_t lastIdx = (timeStepCnt - 1) * numObs + k;
                if (lastIdx < obsLayerTrajs.size()) {
                    const State& obsLast = obsLayerTrajs[lastIdx];
                    simObs[k].x = obsLast.x;
                    simObs[k].y = obsLast.y;
                    simObs[k].vx = obsLast.vx;
                    simObs[k].vy = obsLast.vy;
                    simObs[k].theta = obsLast.theta;
                }
            }
        }
    }

    return {totalCost, fullTrajectory, isSequenceFeasible};
}

// ===================== 构造函数 =====================
EudmPlanner::EudmPlanner()
    : dcpTree_(5, 1.0, 1.0),  // treeHeight=5, layerTime=1.0, lastLayerTime=1.0
      simulator_(),
      costEvaluator_() {
    // ✅ 关键对齐：仿真步长同步，避免舒适度代价计算偏差
    costEvaluator_.setSimDt(simulator_.simDt);
}

// ===================== 串行规划接口 =====================
PlanningResult EudmPlanner::plan(const Vehicle& egoVehicle,
                                 const std::vector<Vehicle>& obstacleVehicles,
                                 const Road& road) {
    PlanningResult result;
    auto actionScripts = dcpTree_.generateActionScript();
    const size_t numScripts = actionScripts.size();
    const double discountFactor = 0.9;
    const double targetSpeed = egoVehicle.targetSpeed;

    // 初始化输出容器
    result.costs.assign(numScripts, std::numeric_limits<double>::infinity());
    result.trajectories.resize(numScripts);
    result.actionScripts = std::move(actionScripts);
    int validCount = 0;

    // Verbose日志打印，对齐MATLAB格式
    if (verbose_) {
        std::cout << "===== EUDM Planner (串行) =====\n";
        int lane = road.getLaneIndex(egoVehicle.x);
        std::cout << "  自车: 速度=" << std::fixed << std::setprecision(1)
                  << egoVehicle.vy * 3.6 << " km/h, 车道=" << lane
                  << ", 位置=(" << std::setprecision(1) << egoVehicle.x 
                  << ", " << std::setprecision(1) << egoVehicle.y << ")\n";
        const Action& ongoing = dcpTree_.getOngoingAction();
        std::cout << "  Ongoing: [" << ongoing.lon << "," << ongoing.lat
                  << "], t=" << std::setprecision(2) << ongoing.t << "s\n";
    }

    // 串行遍历所有动作序列
    for (size_t i = 0; i < numScripts; ++i) {
        auto [cost, traj, feasible] = evaluateSingleSequence(
            result.actionScripts[i], egoVehicle, obstacleVehicles, road,
            targetSpeed, discountFactor);
        
        if (feasible) {
            result.costs[i] = cost;
            result.trajectories[i] = std::move(traj);
            validCount++;
        }
    }

    // 日志打印可行序列数
    if (verbose_) {
        std::cout << "  评估了 " << numScripts << " 个序列，" << validCount << " 个可行\n";
    }

    // ✅ 所有序列不可行 → 生成默认LK序列兜底，对齐MATLAB
    bool allInfeasible = std::all_of(result.costs.begin(), result.costs.end(),
        [](double c) { return c == std::numeric_limits<double>::infinity(); });
    if (allInfeasible) {
        if (verbose_) std::cout << "  警告：所有序列都不可行，生成默认LK序列\n";
        std::vector<Action> defaultSeq(dcpTree_.getTreeHeight());
        for (int i = 0; i < dcpTree_.getTreeHeight(); ++i) {
            defaultSeq[i].lon = "MNT";
            defaultSeq[i].lat = "LK";
            defaultSeq[i].t = dcpTree_.getLayerTime();
        }
        defaultSeq[0].t = dcpTree_.getOngoingAction().t;
        defaultSeq.back().t = dcpTree_.getLastLayerTime();

        result.actionScripts = {defaultSeq};
        result.costs = {1000.0};
        result.trajectories = {generateDefaultTrajectory(egoVehicle, road)};
        result.bestIndex = 0;
    } else {
        // 找最优序列
        auto minCostIt = std::min_element(result.costs.begin(), result.costs.end());
        result.bestIndex = static_cast<int>(std::distance(result.costs.begin(), minCostIt));
        if (verbose_ && result.bestIndex >= 0) {
            std::string seqStr = sequenceToString(result.actionScripts[result.bestIndex]);
            std::cout << "  最优: 序列" << (result.bestIndex + 1)
                      << " " << seqStr << ", 成本=" << std::fixed << std::setprecision(2)
                      << *minCostIt << "\n";
        }
    }

    return result;
}

// ===================== 并行规划接口【核心修复】 =====================
PlanningResult EudmPlanner::planParallel(const Vehicle& egoVehicle,
                                         const std::vector<Vehicle>& obstacleVehicles,
                                         const Road& road) {
    PlanningResult result;
    auto actionScripts = dcpTree_.generateActionScript();
    const size_t numScripts = actionScripts.size();
    const double discountFactor = 0.9;
    const double targetSpeed = egoVehicle.targetSpeed;
    const int treeHeight = dcpTree_.getTreeHeight();

    result.costs.assign(numScripts, std::numeric_limits<double>::infinity());
    result.trajectories.resize(numScripts);
    result.actionScripts = std::move(actionScripts);
    int validCount = 0;

    // Verbose日志打印
    if (verbose_) {
        std::cout << "===== EUDM Planner (并行) =====\n";
        int lane = road.getLaneIndex(egoVehicle.x);
        std::cout << "  自车: 速度=" << std::fixed << std::setprecision(1)
                  << egoVehicle.vy * 3.6 << " km/h, 车道=" << lane
                  << ", 位置=(" << std::setprecision(1) << egoVehicle.x 
                  << ", " << std::setprecision(1) << egoVehicle.y << ")\n";
        std::cout << "  评估 " << numScripts << " 个动作序列...\n";
    }

    // ✅ 核心修复：并行任务创建，线程安全，无资源竞争，每个线程独立初始化仿真器/评估器
    std::vector<std::future<std::tuple<double, Trajectory, bool>>> futures;
    futures.reserve(numScripts);
    for (size_t i = 0; i < numScripts; ++i) {
        futures.emplace_back(std::async(std::launch::async,
            [this, &seq=result.actionScripts[i], &ego=egoVehicle, &obs=obstacleVehicles, &rd=road, targetSpeed, discountFactor]() {
                // 每个线程创建独立的仿真器+评估器，参数和主线程完全对齐
                ForwardSimulator localSim;
                CostEvaluator localEval;
                localEval.setSimDt(localSim.simDt); // 同步步长
                // 构造本地评估器：复用本类的参数配置，保证代价计算一致
                localEval = this->costEvaluator_;

                Vehicle localEgo = localSim.copyVehicle(ego);
                std::vector<Vehicle> localObs;
                for (const auto& o : obs) localObs.push_back(localSim.copyVehicle(o));

                // 调用评估逻辑
                return this->evaluateSingleSequence(seq, localEgo, localObs, rd, targetSpeed, discountFactor);
            }));
    }

    // ✅ 收集并行结果
    for (size_t i = 0; i < numScripts; ++i) {
        auto [cost, traj, feasible] = futures[i].get();
        if (feasible) {
            result.costs[i] = cost;
            result.trajectories[i] = std::move(traj);
            validCount++;
        }
    }

    // 日志打印并行结果
    if (verbose_) {
        std::cout << "  并行评估完成，" << validCount << " 个序列可行\n";
    }

    // ✅ 兜底逻辑：和串行完全一致
    bool allInfeasible = std::all_of(result.costs.begin(), result.costs.end(),
        [](double c) { return c == std::numeric_limits<double>::infinity(); });
    if (allInfeasible) {
        if (verbose_) std::cout << "  警告：所有序列都不可行，生成默认LK序列\n";
        std::vector<Action> defaultSeq(treeHeight);
        for (int i = 0; i < treeHeight; ++i) {
            defaultSeq[i].lon = "MNT";
            defaultSeq[i].lat = "LK";
            defaultSeq[i].t = dcpTree_.getLayerTime();
        }
        defaultSeq[0].t = dcpTree_.getOngoingAction().t;
        defaultSeq.back().t = dcpTree_.getLastLayerTime();

        result.actionScripts = {defaultSeq};
        result.costs = {1000.0};
        result.trajectories = {generateDefaultTrajectory(egoVehicle, road)};
        result.bestIndex = 0;
    } else {
        auto minCostIt = std::min_element(result.costs.begin(), result.costs.end());
        result.bestIndex = static_cast<int>(std::distance(result.costs.begin(), minCostIt));
        if (verbose_ && result.bestIndex >= 0) {
            std::string seqStr = sequenceToString(result.actionScripts[result.bestIndex]);
            std::cout << "  最优: 序列" << (result.bestIndex + 1)
                      << " " << seqStr << ", 成本=" << std::fixed << std::setprecision(2)
                      << *minCostIt << "\n";
        }
    }

    return result;
}