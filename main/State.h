#ifndef STATE_H
#define STATE_H

#include <vector>
#include <string>

struct State {
    // 所有成员变量 + 新增：空状态标记位
    double x = 0.0;     // lateral position
    double y = 0.0;     // longitudinal position
    double vx = 0.0;
    double vy = 0.0;    // longitudinal velocity
    double ax = 0.0;
    double ay = 0.0;
    double theta = 0.0; // heading angle (rad), 0=North, +90°=East
    double id = -1.0;   // ✅ 初始化id=-1表示空状态，有效车辆id≥0
    bool is_empty = true; // ✅ 最优方案：显式空状态标记，绝对可靠

    // ✅ 完整构造函数：全部成员初始化，含空状态/有效状态双版本
    State() : x(0), y(0), vx(0), vy(0), ax(0), ay(0), theta(0), id(-1), is_empty(true) {}
    State(double x_, double y_, double vx_, double vy_, double ax_, double ay_, double theta_, double id_ = -1) 
        : x(x_), y(y_), vx(vx_), vy(vy_), ax(ax_), ay(ay_), theta(theta_), id(id_), is_empty(false) {}
    // ✅ 简化构造函数（常用）
    State(double x_, double y_, double vx_, double vy_, double theta_) 
        : x(x_), y(y_), vx(vx_), vy(vy_), ax(0), ay(0), theta(theta_), id(-1), is_empty(false) {}
};

using Trajectory = std::vector<State>;
using ObstacleTrajectories = std::vector<Trajectory>;

#endif // STATE_H