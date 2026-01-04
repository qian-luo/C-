// State.h
#ifndef STATE_H
#define STATE_H

#include <vector>
#include <string>

struct State {
    double x = 0.0;     // lateral position
    double y = 0.0;     // longitudinal position
    double vx = 0.0;
    double vy = 0.0;    // longitudinal velocity
    double ax = 0.0;
    double ay = 0.0;
    double theta = 0.0; // heading angle (rad), 0=North, +90°=East
    double id;

    // 可选：构造函数
    State(double x_ = 0.0, double y_ = 0.0, double vx_ = 0.0, double vy_ = 0.0,
          double ax_ = 0.0, double ay_ = 0.0, double theta_ = 0.0)
        : x(x_), y(y_), vx(vx_), vy(vy_), ax(ax_), ay(ay_), theta(theta_) {}
};

using Trajectory = std::vector<State>;
using ObstacleTrajectories = std::vector<Trajectory>;

#endif // STATE_H