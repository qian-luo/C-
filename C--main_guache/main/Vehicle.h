// Vehicle.h
#pragma once
#include <vector>
#include <array>
#include <string>
#include <cmath>

class Vehicle {
public:
    // === Properties (public for direct access, as in MATLAB)
    int id;
    double x, y, vx, vy, theta;
    double length = 4.7;
    double width = 1.8;
    std::string type;
    double targetSpeed;
    double curvature = 0.0;  // initial curvature

    // Trailer Properties
    bool hasTrailer = false;
    double trailerTheta = 0.0;
    double trailerLength = 6.0;
    double trailerWidth = 2.0;
    double trailerL = 5.0;      // hitch to trailer axle distance
    double hitchOffset = 2.35;  // tractor center to hitch distance (half length)

    double lr = 1.4;   // rear axle to CG
    double lf = 1.6;   // front axle to CG
    double L = 3.0;    // wheelbase

    std::vector<std::array<double, 2>> trajectory;
    size_t maxTrajectoryLength = 200;

    double max_steer_rate = 0.39;  // rad/s

    // === Constructor
    Vehicle(int id, double x, double y, double vy, double vx, const std::string& type);

    // === Methods
    void step(double acc, double steer, double dt);
    std::array<std::array<double, 2>, 4> getShape() const;
    std::array<std::array<double, 2>, 4> getTrailerShape() const;
    bool isColliding(const Vehicle& other) const;

    // >>> 新增：轨迹只读访问器 <<<
    const std::vector<std::array<double, 2>>& getTrajectory() const { return trajectory; }
    size_t getMaxTrajectoryLength() const { return maxTrajectoryLength; }

};