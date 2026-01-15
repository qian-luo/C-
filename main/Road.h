// Road.h        头文件(.h) ≈ 产品的说明书/接口文档（告诉别人怎么用）
#ifndef ROAD_H   //#ifndef ROAD_H：如果ROAD_H没有被定义过，则编译下面的代码
#define ROAD_H   //#define ROAD_H：定义ROAD_H宏
   //标准库包含：
#include <vector>    //#include ≈ 导入工具包（告诉程序需要哪些功能模块）
#include <cmath>     //数学函数库 包含常用数学函数：sqrt(), sin(), cos(), pow()等
#include <stdexcept> //标准异常处理库 包含异常类：std::runtime_error, std::logic_error等
#include <algorithm> // for std::min_element  算法库 包含通用算法：排序、查找、最小/最大值等

class Road {
private:               //private表示接下来的成员是私有的，只能在类的内部访问，不能从类的外部直接访问。
     // 私有成员变量
    int numLanes;      //车道的数量，整数类型
    double laneWidth;  // 车道宽度，双精度浮点数类型
    double roadLength; // 道路长度，双精度浮点数类型
    std::vector<double> laneCenters;// 一个double类型的动态数组（向量），用于存储每个车道的中心位置

public:                 //公有成员在类的外部可以被访问
    // 构造函数：初始化道路  构造函数是在创建类的新对象时自动调用。它用于初始化对象的状态。这个构造函数接受三个参数：车道数量、车道宽度和道路长度
    Road(int numLanes, double laneWidth, double roadLengthInput);

    // 获取指定车道的中心位置（车道索引从 1 开始）
    double getLaneCenter(int laneIndex) const; //const表示这个函数不会修改类的成员变量

    // 根据横向位置 x_pos 获取最邻近车道索引（从 1 开始）
    int getLaneIndex(double x_pos) const;

    // 获取指定车道的左右边界（注意：返回顺序为 [right, left]，与 MATLAB 一致）
    std::pair<double, double> getLaneBoundaries(int laneIndex) const; //std::pair<double, double>  一个对/元组：可以存储两个值
     //std::pair 是C++标准库中的一个模板类，它可以保存两个值，这两个值可以是不同的类型
    // 可选：提供 getter（非必须，但便于调试/扩展）； Getter函数 提供只读访问：允许外部代码获取私有数据，但不能修改
    int getNumLanes() const { return numLanes; }
    double getLaneWidth() const { return laneWidth; }
    double getRoadLength() const { return roadLength; }
    const std::vector<double>& getLaneCenters() const { return laneCenters; }
};

#endif // ROAD_H