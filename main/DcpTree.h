#ifndef DCP_TREE_H
#define DCP_TREE_H

#include <string>
#include <vector>
#include <array>
#include <algorithm>
#include <stdexcept>

struct Action {
    std::string lon;
    std::string lat;
    double t;

    Action(std::string l = "", std::string lt = "", double time = 0.0)
        : lon(std::move(l)), lat(std::move(lt)), t(time) {}
};

class DcpTree {
public:
    DcpTree(int treeHeight, double layerTime, double lastLayerTime = -1.0);

    DcpTree& setOngoingAction(const Action& action);

    // 返回：vector of sequence，每个 sequence 是 vector<Action>，长度 = treeHeight
    std::vector<std::vector<Action>> generateActionScript() const;

    double planningHorizon() const;

    // 只读访问器（调试用）
    int getTreeHeight() const { return treeHeight_; }
    double getLayerTime() const { return layerTime_; }
    double getLastLayerTime() const { return lastLayerTime_; }
    const Action& getOngoingAction() const { return ongoingAction_; }

// 替换原来的 kLonActions / kLatActions 定义：
private:
    static constexpr std::array<const char*, 3> kLonActions = {"MNT", "ACC", "DEC"};
    static constexpr std::array<const char*, 3> kLatActions = {"LK", "LCL", "LCR"};

    int treeHeight_;
    double layerTime_;
    double lastLayerTime_;
    Action ongoingAction_;
};

#endif // DCP_TREE_HPP