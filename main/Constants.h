#ifndef UTILS_H
#define UTILS_H

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_2PI
#define M_2PI 2 * M_PI
#endif

// 声明+实现 归一化角度函数，全局可用无重复定义
inline double normalizeAngle(double angle) {
    angle = std::fmod(angle, M_2PI);
    if (angle > M_PI) angle -= M_2PI;
    if (angle < -M_PI) angle += M_2PI;
    return angle;
}

#endif // UTILS_H