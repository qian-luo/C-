// Road.cpp        源文件(.cpp) ≈ 产品的具体实现（内部工作原理）
#include "Road.h"  //包含自定义头文件：使用双引号" "；作用：导入类声明，让编译器知道Road类的存在；必需步骤：实现类之前必须包含对应的头文件
#include <vector>  //重复包含标准库
#include <cmath>   
#include <stdexcept>
#include <algorithm>

Road::Road(int numLanes, double laneWidth, double roadLengthInput)          //构造函数
    : numLanes(numLanes), laneWidth(laneWidth), roadLength(roadLengthInput) //初始化列表，其作用是在进入构造函数体之前，对类的成员变量进行初始化。
{
    // 参数校验（MATLAB 版未做，我们补上更安全）
    if (numLanes <= 0) {
        throw std::invalid_argument("numLanes must be positive");//throw 关键字用于抛出异常。
    }
    if (laneWidth <= 0) {
        throw std::invalid_argument("laneWidth must be positive");//std::invalid_argument 是标准库中定义的一种异常类型，用于表示无效的参数。
    }
    if (roadLengthInput <= 0) {
        throw std::invalid_argument("roadLength must be positive");
    }

    // 等效于 MATLAB: linspace(-totalWidth/2 + laneWidth/2, totalWidth/2 - laneWidth/2, numLanes)
    double totalWidth = numLanes * laneWidth;            //计算总宽度
    double start = -totalWidth / 2.0 + laneWidth / 2.0;  //计算起始位置
    double end   =  totalWidth / 2.0 - laneWidth / 2.0;  //计算结束位置

    laneCenters.resize(numLanes);//用于计算每个车道的中心位置，并将结果存储在成员变量 laneCenters 中。
    if (numLanes == 1) {        //单车道特殊处理，原因：下面的公式当 numLanes=1 时，(numLanes - 1) = 0，导致除以零错误！
        laneCenters[0] = 0.0;  // 特殊处理：单车道居中（与 linspace 一致）
    } else {
        for (int i = 0; i < numLanes; ++i) {    //多车道计算；++i：i自增1，等价于MATLAB的i = i + 1
            // 线性插值：第 i 个点 = start + (end - start) * i / (n-1)
            laneCenters[i] = start + (end - start) * static_cast<double>(i) / (numLanes - 1);
        }
    }
}

double Road::getLaneCenter(int laneIndex) const {  //getLaneCenter 获取指定车道的中心位置
    if (laneIndex < 1 || laneIndex > numLanes) {
        throw std::out_of_range("Invalid lane index: must be between 1 and " + std::to_string(numLanes));
    }
    // MATLAB 索引从 1 开始，C++ vector 从 0 开始 → 减 1
    return laneCenters[laneIndex - 1];
}

int Road::getLaneIndex(double x_pos) const { //getLaneIndex 根据横向位置获取车道索引
    if (laneCenters.empty()) {
        throw std::logic_error("No lanes defined");
    }

    // 计算所有 |x_pos - center|，找最小值对应索引
    // 使用 std::min_element + 自定义比较
    auto it = std::min_element(laneCenters.begin(), laneCenters.end(),
        [x_pos](double a, double b) {   //[x_pos](double a, double b) { ... }是一个lambda表达式（匿名函数）。[x_pos]是捕获列表，表示lambda内部可以使用外部变量x_pos
            return std::abs(x_pos - a) < std::abs(x_pos - b);//参数a和b是向量中的两个元素（车道中心位置）
        });                                                  //函数体计算x_pos到a和b的绝对距离，并返回哪个距离更小。  

    // 返回索引（从 1 开始）
    int index0 = static_cast<int>(std::distance(laneCenters.begin(), it));//static_cast<int>将距离转换为整数。
    return index0 + 1;              //std::distance：计算从起始迭代器（laneCenters.begin()）到找到的迭代器（it）之间的距离。
}

std::pair<double, double> Road::getLaneBoundaries(int laneIndex) const {  //getLaneBoundaries 获取指定车道的左右边界
    double center = getLaneCenter(laneIndex);//车道中心
    double left  = center - laneWidth / 2.0;//左边界计算
    double right = center + laneWidth / 2.0;//右边界计算

    // 注意：MATLAB 返回 [right, left]，所以 pair.first = right, .second = left
    return std::make_pair(right, left);
}