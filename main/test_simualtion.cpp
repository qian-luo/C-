// test_simulation.cpp - Simulation 测试程序
#include <iostream>
#include <vector>
#include <memory>
#include "Road.h"
#include "Vehicle.h"
#include "Simulation.h"
#include <cassert>
#include <iomanip>
#include <fstream>

// 测试用例枚举
enum TestCase {
    TEST_BASIC,           // 基础测试：无障碍物场景
    TEST_STATIC_OBSTACLE, // 静态障碍物测试
    TEST_DYNAMIC_TRAFFIC, // 动态交通流测试
    TEST_LANE_CHANGE,     // 换道测试
    TEST_COLLISION,       // 碰撞测试
    TEST_PERFORMANCE,     // 性能测试
    TEST_VISUALIZATION    // 可视化测试
};

// 辅助函数：创建测试道路
Road createTestRoad(TestCase test) {
    switch (test) {
        case TEST_BASIC:
        case TEST_PERFORMANCE:
            return Road(1000.0, 3, 3.75); // 1000米长，3车道
        case TEST_STATIC_OBSTACLE:
            return Road(800.0, 3, 3.75);
        case TEST_DYNAMIC_TRAFFIC:
            return Road(1500.0, 3, 3.75);
        case TEST_LANE_CHANGE:
            return Road(1200.0, 3, 3.75);
        case TEST_COLLISION:
            return Road(500.0, 2, 3.5);
        case TEST_VISUALIZATION:
            return Road(1000.0, 3, 3.75);
        default:
            return Road(1000.0, 3, 3.75);
    }
}

// 辅助函数：创建自车
Vehicle createEgoVehicle(TestCase test) {
    switch (test) {
        case TEST_BASIC:
            return Vehicle(0, 7.0, 0.0, 20.0, 0.0, "ego");
        case TEST_STATIC_OBSTACLE:
            return Vehicle(0, 7.0, 0.0, 15.0, 0.0, "ego");
        case TEST_DYNAMIC_TRAFFIC:
            return Vehicle(0, 7.0, 0.0, 25.0, 0.0, "ego");
        case TEST_LANE_CHANGE:
            return Vehicle(0, 3.75, 0.0, 18.0, 0.0, "ego");
        case TEST_COLLISION:
            return Vehicle(0, 3.75, 0.0, 20.0, 0.0, "ego");
        case TEST_PERFORMANCE:
            return Vehicle(0, 7.0, 0.0, 20.0, 0.0, "ego");
        case TEST_VISUALIZATION:
            return Vehicle(0, 7.0, 0.0, 20.0, 0.0, "ego");
        default:
            return Vehicle(0, 7.0, 0.0, 20.0, 0.0, "ego");
    }
}

// 辅助函数：创建障碍物车辆
std::vector<Vehicle> createObstacleVehicles(TestCase test) {
    std::vector<Vehicle> obstacles;
    
    switch (test) {
        case TEST_BASIC:
            // 无障碍物
            break;
            
        case TEST_STATIC_OBSTACLE:
            // 在自车前方100米处放置一个静态障碍物
            obstacles.push_back(Vehicle(1, 7.0, 100.0, 0.0, 0.0, "static"));
            break;
            
        case TEST_DYNAMIC_TRAFFIC:
            // 创建动态交通流
            obstacles.push_back(Vehicle(1, 3.75, 50.0, 15.0, 0.0, "dynamic"));
            obstacles.push_back(Vehicle(2, 7.0, 80.0, 12.0, 0.0, "dynamic"));
            obstacles.push_back(Vehicle(3, 10.25, 120.0, 18.0, 0.0, "dynamic"));
            obstacles.push_back(Vehicle(4, 3.75, 200.0, 20.0, 0.0, "dynamic"));
            break;
            
        case TEST_LANE_CHANGE:
            // 前方慢车，测试换道能力
            obstacles.push_back(Vehicle(1, 3.75, 80.0, 10.0, 0.0, "dynamic"));
            obstacles.push_back(Vehicle(2, 7.0, 150.0, 25.0, 0.0, "dynamic"));
            break;
            
        case TEST_COLLISION:
            // 在前方放置障碍物，预期会发生碰撞
            obstacles.push_back(Vehicle(1, 3.75, 50.0, 0.0, 0.0, "static"));
            break;
            
        case TEST_PERFORMANCE:
            // 多个障碍物测试性能
            for (int i = 1; i <= 10; i++) {
                obstacles.push_back(Vehicle(i, 3.75 + (i % 3) * 3.75, i * 40.0, 15.0 + (i % 5), 0.0, "dynamic"));
            }
            break;
            
        case TEST_VISUALIZATION:
            // 创建可视化测试场景
            obstacles.push_back(Vehicle(1, 3.75, 100.0, 18.0, 0.0, "dynamic"));
            obstacles.push_back(Vehicle(2, 7.0, 150.0, 15.0, 0.0, "dynamic"));
            obstacles.push_back(Vehicle(3, 10.25, 200.0, 22.0, 0.0, "dynamic"));
            break;
    }
    
    return obstacles;
}

// 辅助函数：打印测试标题
void printTestHeader(TestCase test) {
    std::cout << "\n========================================\n";
    std::cout << "测试用例: ";
    
    switch (test) {
        case TEST_BASIC:
            std::cout << "基础测试 - 无障碍物场景";
            break;
        case TEST_STATIC_OBSTACLE:
            std::cout << "静态障碍物测试";
            break;
        case TEST_DYNAMIC_TRAFFIC:
            std::cout << "动态交通流测试";
            break;
        case TEST_LANE_CHANGE:
            std::cout << "换道能力测试";
            break;
        case TEST_COLLISION:
            std::cout << "碰撞检测测试";
            break;
        case TEST_PERFORMANCE:
            std::cout << "性能测试 - 多障碍物场景";
            break;
        case TEST_VISUALIZATION:
            std::cout << "可视化功能测试";
            break;
    }
    
    std::cout << "\n========================================\n";
}

// 测试函数：运行特定测试用例
void runTestCase(TestCase test, bool enableVisualization = false) {
    printTestHeader(test);
    
    // 创建仿真组件
    Road road = createTestRoad(test);
    Vehicle ego = createEgoVehicle(test);
    std::vector<Vehicle> obstacles = createObstacleVehicles(test);
    
    // 创建仿真对象
    Simulation sim(road, ego, obstacles, 0.1); // 0.1秒时间步长
    
    // 初始化仿真
    if (!sim.initialize(enableVisualization && test == TEST_VISUALIZATION)) {
        std::cerr << "仿真初始化失败！\n";
        return;
    }
    
    // 设置合适的步数
    int numSteps;
    switch (test) {
        case TEST_BASIC:
            numSteps = 500; // 50秒
            break;
        case TEST_STATIC_OBSTACLE:
            numSteps = 600; // 60秒
            break;
        case TEST_DYNAMIC_TRAFFIC:
            numSteps = 1000; // 100秒
            break;
        case TEST_LANE_CHANGE:
            numSteps = 800; // 80秒
            break;
        case TEST_COLLISION:
            numSteps = 300; // 30秒
            break;
        case TEST_PERFORMANCE:
            numSteps = 2000; // 200秒
            sim.setDrawInterval(0.05); // 更快绘制以测试性能
            break;
        case TEST_VISUALIZATION:
            numSteps = 1000; // 100秒
            break;
        default:
            numSteps = 500;
    }
    
    // 运行仿真
    auto startTime = std::chrono::high_resolution_clock::now();
    sim.run(numSteps);
    auto endTime = std::chrono::high_resolution_clock::now();
    
    // 计算总运行时间
    auto duration = std::chrono::duration<double>(endTime - startTime).count();
    
    // 获取统计信息
    SimulationStats stats = sim.getStats();
    
    // 打印详细结果
    std::cout << "\n测试结果:\n";
    std::cout << "----------------------------------------\n";
    std::cout << "仿真时间: " << stats.totalTime << " 秒\n";
    std::cout << "行驶距离: " << std::fixed << std::setprecision(2) << stats.totalDistance << " 米\n";
    std::cout << "碰撞次数: " << stats.collisions << "\n";
    std::cout << "换道次数: " << stats.laneChanges << "\n";
    std::cout << "规划次数: " << stats.planCount << "\n";
    std::cout << "平均规划时间: " << std::fixed << std::setprecision(3) << stats.avgPlanTime * 1000.0 << " 毫秒\n";
    std::cout << "测试运行时间: " << std::fixed << std::setprecision(2) << duration << " 秒\n";
    std::cout << "实时因子: " << std::fixed << std::setprecision(2) << stats.totalTime / duration << "\n";
    
    // 输出测试结论
    std::cout << "\n测试结论: ";
    switch (test) {
        case TEST_BASIC:
            if (stats.collisions == 0 && stats.totalDistance > 100) {
                std::cout << "✓ 通过 - 自车成功行驶\n";
            } else {
                std::cout << "✗ 失败 - 未达到预期\n";
            }
            break;
            
        case TEST_STATIC_OBSTACLE:
            if (stats.collisions == 0) {
                std::cout << "✓ 通过 - 成功避开静态障碍物\n";
            } else {
                std::cout << "✗ 失败 - 与静态障碍物发生碰撞\n";
            }
            break;
            
        case TEST_DYNAMIC_TRAFFIC:
            if (stats.collisions == 0 && stats.laneChanges > 0) {
                std::cout << "✓ 通过 - 在动态交通中成功导航\n";
            } else if (stats.collisions == 0) {
                std::cout << "⚠ 警告 - 成功避碰但未换道\n";
            } else {
                std::cout << "✗ 失败 - 发生碰撞\n";
            }
            break;
            
        case TEST_LANE_CHANGE:
            if (stats.laneChanges >= 1) {
                std::cout << "✓ 通过 - 成功完成换道\n";
            } else {
                std::cout << "✗ 失败 - 未执行换道\n";
            }
            break;
            
        case TEST_COLLISION:
            if (stats.collisions > 0) {
                std::cout << "✓ 通过 - 正确检测到碰撞\n";
            } else {
                std::cout << "✗ 失败 - 未检测到预期碰撞\n";
            }
            break;
            
        case TEST_PERFORMANCE:
            if (stats.avgPlanTime < 0.05) { // 50毫秒内完成规划
                std::cout << "✓ 通过 - 性能达标\n";
            } else {
                std::cout << "⚠ 警告 - 规划时间较长: " << stats.avgPlanTime * 1000.0 << "ms\n";
            }
            break;
            
        case TEST_VISUALIZATION:
            std::cout << "✓ 完成 - 可视化测试已执行\n";
            break;
    }
}

// 单元测试：验证 Simulation 类的关键功能
void unitTests() {
    std::cout << "\n========================================\n";
    std::cout << "单元测试\n";
    std::cout << "========================================\n";
    
    // 测试1: 创建仿真对象
    std::cout << "测试1: 创建仿真对象... ";
    Road road(1000.0, 3, 3.75);
    Vehicle ego(0, 7.0, 0.0, 20.0, 0.0, "ego");
    std::vector<Vehicle> obstacles;
    
    Simulation sim(road, ego, obstacles, 0.1);
    std::cout << "✓ 通过\n";
    
    // 测试2: 仿真初始化
    std::cout << "测试2: 仿真初始化... ";
    bool initResult = sim.initialize(false);
    assert(initResult == true);
    std::cout << "✓ 通过\n";
    
    // 测试3: 检查运行状态
    std::cout << "测试3: 检查运行状态... ";
    assert(sim.isRunning() == true);
    std::cout << "✓ 通过\n";
    
    // 测试4: 单步仿真
    std::cout << "测试4: 单步仿真... ";
    bool stepResult = sim.step();
    assert(stepResult == true);
    std::cout << "✓ 通过\n";
    
    // 测试5: 停止仿真
    std::cout << "测试5: 停止仿真... ";
    sim.stop();
    assert(sim.isRunning() == false);
    std::cout << "✓ 通过\n";
    
    std::cout << "\n所有单元测试通过！\n";
}

// 性能基准测试
void benchmarkTests() {
    std::cout << "\n========================================\n";
    std::cout << "性能基准测试\n";
    std::cout << "========================================\n";
    
    // 测试不同场景下的性能
    std::vector<std::pair<TestCase, std::string>> benchmarkScenarios = {
        {TEST_BASIC, "无障碍物"},
        {TEST_STATIC_OBSTACLE, "单个静态障碍物"},
        {TEST_DYNAMIC_TRAFFIC, "4个动态障碍物"},
        {TEST_PERFORMANCE, "10个动态障碍物"}
    };
    
    std::vector<std::vector<double>> results;
    
    for (const auto& scenario : benchmarkScenarios) {
        printTestHeader(scenario.first);
        
        // 创建仿真
        Road road = createTestRoad(scenario.first);
        Vehicle ego = createEgoVehicle(scenario.first);
        std::vector<Vehicle> obstacles = createObstacleVehicles(scenario.first);
        
        Simulation sim(road, ego, obstacles, 0.1);
        sim.initialize(false);
        
        // 运行较短时间以进行基准测试
        auto start = std::chrono::high_resolution_clock::now();
        sim.run(200); // 20秒仿真
        auto end = std::chrono::high_resolution_clock::now();
        
        double duration = std::chrono::duration<double>(end - start).count();
        SimulationStats stats = sim.getStats();
        
        // 记录结果
        results.push_back({
            duration,
            stats.avgPlanTime * 1000.0, // 转换为毫秒
            static_cast<double>(stats.planCount) / duration // 规划频率
        });
        
        std::cout << "场景: " << scenario.second << "\n";
        std::cout << "  运行时间: " << duration << " 秒\n";
        std::cout << "  平均规划时间: " << stats.avgPlanTime * 1000.0 << " 毫秒\n";
        std::cout << "  规划频率: " << stats.planCount / duration << " Hz\n\n";
    }
    
    // 输出基准测试摘要
    std::cout << "性能基准测试摘要:\n";
    std::cout << "---------------------------------------------------\n";
    std::cout << "场景                | 运行时间(s) | 规划时间(ms) | 频率(Hz)\n";
    std::cout << "---------------------------------------------------\n";
    
    for (size_t i = 0; i < benchmarkScenarios.size(); i++) {
        std::cout << std::left << std::setw(20) << benchmarkScenarios[i].second << " | "
                  << std::right << std::setw(10) << std::fixed << std::setprecision(2) << results[i][0] << " | "
                  << std::setw(12) << std::fixed << std::setprecision(2) << results[i][1] << " | "
                  << std::setw(8) << std::fixed << std::setprecision(1) << results[i][2] << "\n";
    }
    std::cout << "---------------------------------------------------\n";
}

// 交互式测试菜单
void interactiveTestMenu() {
    int choice;
    bool exit = false;
    
    while (!exit) {
        std::cout << "\n========================================\n";
        std::cout << "Simulation 测试程序 - 主菜单\n";
        std::cout << "========================================\n";
        std::cout << "1. 运行所有测试用例\n";
        std::cout << "2. 运行单元测试\n";
        std::cout << "3. 运行性能基准测试\n";
        std::cout << "4. 运行特定测试用例\n";
        std::cout << "5. 运行可视化演示\n";
        std::cout << "6. 生成测试报告\n";
        std::cout << "0. 退出\n";
        std::cout << "请选择: ";
        
        std::cin >> choice;
        
        switch (choice) {
            case 1:
                std::cout << "\n运行所有测试用例...\n";
                for (int i = TEST_BASIC; i <= TEST_VISUALIZATION; i++) {
                    runTestCase(static_cast<TestCase>(i), i == TEST_VISUALIZATION);
                }
                break;
                
            case 2:
                unitTests();
                break;
                
            case 3:
                benchmarkTests();
                break;
                
            case 4:
                std::cout << "\n选择测试用例:\n";
                std::cout << "1. 基础测试\n";
                std::cout << "2. 静态障碍物测试\n";
                std::cout << "3. 动态交通流测试\n";
                std::cout << "4. 换道能力测试\n";
                std::cout << "5. 碰撞检测测试\n";
                std::cout << "6. 性能测试\n";
                std::cout << "7. 可视化测试\n";
                std::cout << "请选择: ";
                int testChoice;
                std::cin >> testChoice;
                if (testChoice >= 1 && testChoice <= 7) {
                    runTestCase(static_cast<TestCase>(testChoice-1), testChoice == 7);
                } else {
                    std::cout << "无效选择！\n";
                }
                break;
                
            case 5:
                std::cout << "\n启动可视化演示...\n";
                runTestCase(TEST_VISUALIZATION, true);
                break;
                
            case 6:
                std::cout << "\n生成测试报告...\n";
                // 这里可以添加生成详细测试报告的功能
                std::cout << "测试报告功能暂未实现\n";
                break;
                
            case 0:
                exit = true;
                std::cout << "退出测试程序。\n";
                break;
                
            default:
                std::cout << "无效选择，请重试！\n";
        }
    }
}

// 主函数
int main(int argc, char* argv[]) {
    std::cout << "========================================\n";
    std::cout << "   Simulation 测试程序\n";
    std::cout << "========================================\n";
    
    // 解析命令行参数
    bool runAllTests = false;
    bool runInteractive = false;
    
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--all" || arg == "-a") {
            runAllTests = true;
        } else if (arg == "--interactive" || arg == "-i") {
            runInteractive = true;
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "用法: " << argv[0] << " [选项]\n";
            std::cout << "选项:\n";
            std::cout << "  -a, --all         运行所有测试\n";
            std::cout << "  -i, --interactive 交互式菜单\n";
            std::cout << "  -h, --help        显示帮助信息\n";
            return 0;
        }
    }
    
    if (runAllTests) {
        // 运行所有测试用例
        std::cout << "\n运行所有测试用例...\n";
        for (int i = TEST_BASIC; i <= TEST_PERFORMANCE; i++) {
            runTestCase(static_cast<TestCase>(i));
        }
        std::cout << "\n所有测试完成！\n";
    } else if (runInteractive) {
        // 启动交互式菜单
        interactiveTestMenu();
    } else {
        // 默认运行核心测试
        std::cout << "\n运行核心测试用例...\n";
        runTestCase(TEST_BASIC);
        runTestCase(TEST_STATIC_OBSTACLE);
        runTestCase(TEST_DYNAMIC_TRAFFIC);
        runTestCase(TEST_LANE_CHANGE);
        runTestCase(TEST_PERFORMANCE);
        
        std::cout << "\n核心测试完成！\n";
        std::cout << "使用 --interactive 启动交互式菜单\n";
        std::cout << "使用 --all 运行所有测试（包括可视化）\n";
    }
    
    return 0;
}