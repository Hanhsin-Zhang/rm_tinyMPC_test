// test/main.cpp

#include <iostream>
#include <memory>
#include <vector>

// 包含重构后的求解器头文件
#include "tiny_mpc/TinyMpcSolver.h"

// 使用 hitcrt 命名空间
using namespace hitcrt;

// ---- 问题参数定义 ----
// 系统和MPC维度
constexpr int N_STATES = 4;
constexpr int N_INPUTS = 2;
constexpr int N_HORIZON = 10;

// ADMM 惩罚参数
constexpr TinyType RHO_VALUE = 5.0;

// 状态转移矩阵 A [4x4]
const TinyType ADYN_DATA[N_STATES * N_STATES] = {
    1.0, 0.0, 0.25, 0.0, 
    0.0, 1.0, 0.0,  0.25,
    0.0, 0.0, 1.0,  0.0,
    0.0, 0.0, 0.0,  1.0};

// 输入矩阵 B [4x2]
const TinyType BDYN_DATA[N_STATES * N_INPUTS] = {
    0.03125, 0.0,
    0.0,     0.03125,
    0.25,    0.0,
    0.0,     0.25};

// 状态代价权重 Q (对角线元素) [4x1]
const TinyType Q_DATA[N_STATES] = {20.0, 20.0, 10.0, 10.0};

// 输入代价权重 R (对角线元素) [2x1]
const TinyType R_DATA[N_INPUTS] = {10.0, 10.0};

// 参考轨迹 Xref [4x10] - 只取前10个点
const TinyType XREF_DATA[N_STATES * N_HORIZON] = {
    100.0,    100.0,   0.0,      0.0,
    100.216,  99.9638, 2.5532,  -0.4246,
    101.6516, 99.73,   9.60856, -1.54185,
    105.3231, 99.1536, 20.2998, -3.1303,
    112.0367, 98.1426, 33.8143, -4.98675,
    122.4021, 96.6534, 49.3926, -6.92613,
    136.846,  94.6866, 66.3291, -8.78152,
    155.6253, 92.2821, 83.9714, -10.4042,
    178.8405, 89.5148, 101.721, -11.6634,
    206.4491, 86.4901, 119.032, -12.4469};

/**
 * @brief 主函数，演示TinyMpcSolver的使用
 * @param argc
 * @param argv
 * @return int
 * @author <Your Name>
 * @mail <your.email@example.com>
 */
int main(int argc, char** argv) {
    // ---- 1. 数据准备 ----
    // 使用 Eigen::Map 将C数组映射为Eigen矩阵，避免数据复制
    Eigen::Map<const TinyMatrix> adyn(ADYN_DATA, N_STATES, N_STATES);
    Eigen::Map<const TinyMatrix> bdyn(BDYN_DATA, N_STATES, N_INPUTS);
    Eigen::Map<const TinyVector> q_diag(Q_DATA, N_STATES);
    
    Eigen::Map<const TinyVector> r_diag(R_DATA, N_INPUTS);


    // 定义状态和输入的约束
    TinyMatrix x_min = TinyMatrix::Constant(N_STATES, N_HORIZON, -1500.0);
    TinyMatrix x_max = TinyMatrix::Constant(N_STATES, N_HORIZON, 1500.0);
    TinyMatrix u_min = TinyMatrix::Constant(N_INPUTS, N_HORIZON - 1, -600.0);
    TinyMatrix u_max = TinyMatrix::Constant(N_INPUTS, N_HORIZON - 1, 600.0);
    
    // 定义参考轨迹
    Eigen::Map<const TinyMatrix> x_ref(XREF_DATA, N_STATES, N_HORIZON);
    // 此示例中输入参考轨迹为0
    TinyMatrix u_ref = TinyMatrix::Zero(N_INPUTS, N_HORIZON - 1);

    std::cout << "======== TinyMPC C++ Test Case ========" << std::endl;
    std::cout << "System: " << N_STATES << " states, " << N_INPUTS << " inputs." << std::endl;
    std::cout << "Horizon: " << N_HORIZON << " steps." << std::endl;
    std::cout << "---------------------------------------" << std::endl;

    try {
        // ---- 2. 初始化求解器 ----
        // 使用智能指针管理求解器实例
        auto solverPtr = std::make_unique<TinyMpcSolver>(
            adyn, bdyn, q_diag.asDiagonal(), r_diag.asDiagonal(), 
            x_min, x_max, u_min, u_max,
            N_STATES, N_INPUTS, N_HORIZON, 
            RHO_VALUE, true // 启用详细日志
        );

        // ---- 3. 设置问题并求解 ----
        // 设置初始状态（使用参考轨迹的第一个点）
        solverPtr->setInitialState(x_ref.col(0));
        
        // 设置参考轨迹
        solverPtr->setStateReference(x_ref);
        // solverPtr->setInputReference(u_ref);

        // 更新求解器设置（可选，此处使用默认值）
        // solverPtr->updateSettings(1e-3, 1e-3, 100, 1, true, true);
        
        std::cout << "\nSolving MPC problem..." << std::endl;
        int status = solverPtr->solve();

        // ---- 4. 获取并打印结果 ----
        const TinySolution& solution = solverPtr->getSolution();

        if (solution.solved) {
            std::cout << "\nSUCCESS: Problem solved in " << solution.iter << " iterations." << std::endl;
        } else {
            std::cerr << "\nFAILURE: Solver did not converge after " << solution.iter << " iterations." << std::endl;
        }

        Eigen::IOFormat fmt(4, 0, ", ", "\n", "[", "]");
        std::cout << "\nInitial State:\n" << x_ref.col(0).transpose().format(fmt) << std::endl;
        std::cout << "\nOptimal Control Input (first 5 steps):\n" << solution.u.leftCols(5).format(fmt) << std::endl;
        std::cout << "\nResulting State Trajectory (first 5 steps):\n" << solution.x.leftCols(5).format(fmt) << std::endl;
        
        return 0;

    } catch (const std::invalid_argument& e) {
        std::cerr << "\nERROR: An exception occurred during setup: " << e.what() << std::endl;
        return 1;
    }
}