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
    trailerTheta = theta; // Initialize trailer heading with tractor heading
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

    // 5a. Update trailer heading if exists
    if (hasTrailer) {
        // Trailer kinematic model: d(theta_t)/dt = (v / trailerL) * sin(theta - theta_t)
        double d_theta_t = (v_new / trailerL) * std::sin(theta - trailerTheta);
        trailerTheta += d_theta_t * dt;
        trailerTheta = normalizeAngle(trailerTheta);
    }

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

// === Get trailer shape (4 corners in world frame)
std::array<std::array<double, 2>, 4> Vehicle::getTrailerShape() const {
    if (!hasTrailer) return {{{0,0}, {0,0}, {0,0}, {0,0}}};

    // 1. Calculate hitch point in world frame
    double x_h = x - hitchOffset * std::sin(theta);
    double y_h = y - hitchOffset * std::cos(theta);

    // 2. Calculate trailer center in world frame
    double x_t = x_h - (trailerLength / 2.0) * std::sin(trailerTheta);
    double y_t = y_h - (trailerLength / 2.0) * std::cos(trailerTheta);

    // 3. Calculate corners in trailer body frame
    double hl = trailerLength / 2.0;
    double hw = trailerWidth / 2.0;
    std::array<std::array<double, 2>, 4> corners_body = {{
        {-hw, -hl}, {hw, -hl}, {hw, hl}, {-hw, hl}
    }};

    // 4. Transform to world frame
    double cos_t = std::cos(trailerTheta);
    double sin_t = std::sin(trailerTheta);
    std::array<std::array<double, 2>, 4> world_corners;
    for (size_t i = 0; i < 4; ++i) {
        double xb = corners_body[i][0];
        double yb = corners_body[i][1];
        world_corners[i][0] = xb * cos_t + yb * sin_t + x_t;
        world_corners[i][1] = -xb * sin_t + yb * cos_t + y_t;
    }
    return world_corners;
}

// === AABB collision detection with safety margin
bool Vehicle::isColliding(const Vehicle& other) const {
    constexpr double safety_margin = 0.2;

    auto checkShapeCollision = [&](const std::array<std::array<double, 2>, 4>& shape1, 
                                   const std::array<std::array<double, 2>, 4>& shape2) {
        // Extract x/y bounds (min/max) for shape1
        double min1_x = shape1[0][0], max1_x = shape1[0][0];
        double min1_y = shape1[0][1], max1_y = shape1[0][1];
        for (const auto& p : shape1) {
            min1_x = std::min(min1_x, p[0]); max1_x = std::max(max1_x, p[0]);
            min1_y = std::min(min1_y, p[1]); max1_y = std::max(max1_y, p[1]);
        }

        // Extract x/y bounds (min/max) for shape2
        double min2_x = shape2[0][0], max2_x = shape2[0][0];
        double min2_y = shape2[0][1], max2_y = shape2[0][1];
        for (const auto& p : shape2) {
            min2_x = std::min(min2_x, p[0]); max2_x = std::max(max2_x, p[0]);
            min2_y = std::min(min2_y, p[1]); max2_y = std::max(max2_y, p[1]);
        }

        // Apply safety margin
        min1_x -= safety_margin; max1_x += safety_margin;
        min1_y -= safety_margin; max1_y += safety_margin;
        min2_x -= safety_margin; max2_x += safety_margin;
        min2_y -= safety_margin; max2_y += safety_margin;

        // AABB separation check
        return !(max1_x < min2_x || min1_x > max2_x || max1_y < min2_y || min1_y > max2_y);
    };

    // 1. Tractor-Tractor collision
    if (checkShapeCollision(getShape(), other.getShape())) return true;

    // 2. Self-Trailer vs Other-Tractor
    if (hasTrailer && checkShapeCollision(getTrailerShape(), other.getShape())) return true;

    // 3. Self-Tractor vs Other-Trailer
    if (other.hasTrailer && checkShapeCollision(getShape(), other.getTrailerShape())) return true;

    // 4. Self-Trailer vs Other-Trailer
    if (hasTrailer && other.hasTrailer && checkShapeCollision(getTrailerShape(), other.getTrailerShape())) return true;

    return false;
}


