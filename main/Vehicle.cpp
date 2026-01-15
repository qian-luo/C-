// Vehicle.cpp
#include "Vehicle.h"
#include <algorithm>
#include <limits>
#include "Constants.h"

// === Constructor
Vehicle::Vehicle(int id, double x, double y, double vy, double vx, const std::string& type)
    : id(id), x(x), y(y), vx(vx), vy(vy), type(type), targetSpeed(vy) {
    // Match MATLAB: theta = atan2(vx, vy)
    // (0° = north, 90° = east → x=sin, y=cos convention)
    if (std::abs(vx) < 0.01 && std::abs(vy) < 0.01) {
        theta = 0.0;
    } else {
        theta = std::atan2(vx, vy);  // ⚠️ Order: vx, vy (as in MATLAB)
    }
    curvature = 0.0;
    trajectory.push_back({x, y});
}

// === Step function
void Vehicle::step(double acc, double steer, double dt) {
    // 1. Clamp steering angle to [-π/6, π/6]
    constexpr double max_steer = M_PI / 6.0;
    steer = std::clamp(steer, -max_steer, max_steer);

    // 2. Steer rate limiting (from curvature)
    double current_steer = std::atan(L * curvature);
    double steer_rate = (steer - current_steer) / dt;
    steer_rate = std::clamp(steer_rate, -max_steer_rate, max_steer_rate);
    steer = current_steer + steer_rate * dt;

    // 3. Update speed
    double v = std::hypot(vx, vy);
    double v_new = std::max(0.0, v + acc * dt);

    // 4. Kinematic update
    if (std::abs(steer) < 1e-6) {
        // Straight motion
        x += v_new * std::sin(theta) * dt;
        y += v_new * std::cos(theta) * dt;
        // theta unchanged
    } else {
        // Bicycle model
        double beta = std::atan(lr / L * std::tan(steer));
        x += v_new * std::sin(theta + beta) * dt;
        y += v_new * std::cos(theta + beta) * dt;

        double omega = v_new / L * std::tan(steer);
        theta += omega * dt;
        theta = normalizeAngle(theta);  // Keep theta bounded
    }

    // 5. Update velocity components
    vx = v_new * std::sin(theta);
    vy = v_new * std::cos(theta);

    // 6. Update curvature
    curvature = std::tan(steer) / L;

    // 7. Update trajectory (with length limit)
   trajectory.push_back({x, y});     // ✅ 同样正确
    if (trajectory.size() > maxTrajectoryLength) {
        trajectory.erase(trajectory.begin());
    }
}

// === Get vehicle shape (4 corners in world frame)
std::array<std::array<double, 2>, 4> Vehicle::getShape() const {
    double hl = length / 2.0;
    double hw = width / 2.0;

    // Body-frame corners: [rear-left, rear-right, front-right, front-left]
    std::array<std::array<double, 2>, 4> corners_body = {{
        {-hw, -hl},
        { hw, -hl},
        { hw,  hl},
        {-hw,  hl}
    }};

    double cos_t = std::cos(theta);
    double sin_t = std::sin(theta);

    std::array<std::array<double, 2>, 4> world_corners;
    for (size_t i = 0; i < 4; ++i) {
        double xb = corners_body[i][0];
        double yb = corners_body[i][1];
        // 修正后：车体坐标 → 世界坐标 旋转公式，与MATLAB R' 完全等价
        world_corners[i][0] = xb * cos_t + yb * sin_t + x;
        world_corners[i][1] = -xb * sin_t + yb * cos_t + y;
    }
    return world_corners;
}

// === AABB collision detection with safety margin
bool Vehicle::isColliding(const Vehicle& other) const {
    constexpr double safety_margin = 0.2;

    auto self_shape = getShape();
    auto other_shape = other.getShape();

    // Extract x/y bounds (min/max)
    auto self_x = self_shape[0][0], self_y = self_shape[0][1];
    double self_min_x = self_x, self_max_x = self_x;
    double self_min_y = self_y, self_max_y = self_y;

    for (const auto& p : self_shape) {
        self_min_x = std::min(self_min_x, p[0]);
        self_max_x = std::max(self_max_x, p[0]);
        self_min_y = std::min(self_min_y, p[1]);
        self_max_y = std::max(self_max_y, p[1]);
    }

    double other_min_x = other_shape[0][0], other_max_x = other_shape[0][0];
    double other_min_y = other_shape[0][1], other_max_y = other_shape[0][1];
    for (const auto& p : other_shape) {
        other_min_x = std::min(other_min_x, p[0]);
        other_max_x = std::max(other_max_x, p[0]);
        other_min_y = std::min(other_min_y, p[1]);
        other_max_y = std::max(other_max_y, p[1]);
    }

    // Apply safety margin
    self_min_x -= safety_margin;  self_max_x += safety_margin;
    self_min_y -= safety_margin;  self_max_y += safety_margin;
    other_min_x -= safety_margin; other_max_x += safety_margin;
    other_min_y -= safety_margin; other_max_y += safety_margin;

    // AABB separation check
    bool separated = (self_max_x < other_min_x) ||
                     (self_min_x > other_max_x) ||
                     (self_max_y < other_min_y) ||
                     (self_min_y > other_max_y);

    return !separated;
}


