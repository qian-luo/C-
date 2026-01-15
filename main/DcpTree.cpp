#include "DcpTree.h"
#include <cmath>

DcpTree::DcpTree(int treeHeight, double layerTime, double lastLayerTime)
    : treeHeight_(treeHeight),
      layerTime_(layerTime),
      lastLayerTime_(lastLayerTime < 0 ? layerTime : lastLayerTime),
      ongoingAction_("MNT", "LK", layerTime)  // 初始化列表中完成默认赋值
{
    if (treeHeight_ < 1) {
        throw std::invalid_argument("treeHeight must be >= 1");
    }
    if (layerTime_ <= 0) {
        throw std::invalid_argument("layerTime must be > 0");
    }
    if (lastLayerTime_ <= 0) {
        throw std::invalid_argument("lastLayerTime must be > 0");
    }
}

DcpTree& DcpTree::setOngoingAction(const Action& action) {
    ongoingAction_ = action;

    // MATLAB: if ~isfield(...) || isempty(...)
    // C++: 若 t <= 0（包含 NaN、0、负数），视为无效，重置为 layerTime_
    if (ongoingAction_.t <= 0 || std::isnan(ongoingAction_.t)) {
        ongoingAction_.t = layerTime_;
    }

    // 时间裁剪：不能小于 0.1
    if (ongoingAction_.t < 0.1) {
        ongoingAction_.t = 0.1;
    }

    return *this;
}

std::vector<std::vector<Action>> DcpTree::generateActionScript() const {
    std::vector<std::vector<Action>> sequences;

    // 提前校验 treeHeight_
    if (treeHeight_ < 1) {
        return sequences; // 空
    }

    std::string ongoingLat = ongoingAction_.lat;

    // 构建 latCandidates：排除 ongoingLat 的横向动作
    std::vector<std::string> latCandidates;
    for (const auto& lat : kLatActions) {
        if (lat != ongoingLat) {
            latCandidates.push_back(lat);
        }
    }

    // 遍历所有纵向动作
    for (const std::string lonAction : kLonActions) {
        // 构造 base_seq：长度 = treeHeight_
        std::vector<Action> base_seq;
        base_seq.reserve(treeHeight_);

        for (int i = 1; i <= treeHeight_; ++i) { // i: 1-based，与 MATLAB 一致
            double t_val;
            if (i == 1) {
                t_val = ongoingAction_.t;
            } else if (i == treeHeight_) {
                t_val = lastLayerTime_;
            } else {
                t_val = layerTime_;
            }
            base_seq.emplace_back(lonAction, ongoingLat, t_val);
        }

        // 先添加基础序列（全程不切换横向）
        sequences.push_back(base_seq);

        // 生成横向切换分支：从第2层（i=2）开始切换
        if (treeHeight_ >= 2 && !latCandidates.empty()) {
            for (int h = 2; h <= treeHeight_; ++h) {         // h: 从第 h 层开始切换（1-based）
                for (const std::string& latAction : latCandidates) {
                    auto seq = base_seq; // 深拷贝
                    // 从第 h 层到最后一层，改为 latAction
                    for (int k = h - 1; k < treeHeight_; ++k) { // C++ 0-based 索引
                        seq[k].lat = latAction;
                    }
                    sequences.push_back(std::move(seq));
                }
            }
        }
    }

    return sequences;
}

double DcpTree::planningHorizon() const {
    // MATLAB: if isempty(obj.ongoingAction)
    // 注意：MATLAB 中 struct 若字段全空可能被视为 empty，但此处 ongoingAction 总被初始化
    // 我们认为：只要 treeHeight_ >=1 且 ongoingAction.t > 0 即有效
    if (treeHeight_ < 1 || ongoingAction_.t <= 0) {
        return 0.0;
    }

    double horizon = ongoingAction_.t; // 第1层

    if (treeHeight_ > 2) {
        horizon += static_cast<double>(treeHeight_ - 2) * layerTime_;
    }

    if (treeHeight_ > 1) {
        horizon += lastLayerTime_;
    }

    return horizon;
}