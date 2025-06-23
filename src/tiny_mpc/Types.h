// Types.h

#pragma once

#include <Eigen/Dense>

namespace hitcrt {

// 使用
// using
// 定义类型别名
using TinyType = double;
using TinyMatrix = Eigen::Matrix<TinyType, Eigen::Dynamic, Eigen::Dynamic>;
using TinyVector = Eigen::Matrix<TinyType, Eigen::Dynamic, 1>;

/**
 * @brief
 * 存储求解结果
 */
struct TinySolution {
    int iter;
    bool solved;
    TinyMatrix x;  // 状态轨迹 [nx, N]
    TinyMatrix u;  // 控制输入轨迹 [nu, N-1]
};

/**
 * @brief
 * 存储预计算的矩阵和缓存数据
 */
struct TinyCache {
    TinyType rho;
    TinyMatrix kInf;    // LQR增益矩阵 [nu, nx]
    TinyMatrix pInf;    // Riccati方程解 [nx, nx]
    TinyMatrix quuInv;  // (R + B' P B)^-1 [nu, nu]
    TinyMatrix amBKt;   // (A - B K)' [nx, nx]
};

/**
 * @brief
 * 用户可配置的求解器设置
 */
struct TinySettings {
    TinyType absPriTol;
    TinyType absDuaTol;
    int maxIter;
    int checkTermination;
    bool enStateBound;
    bool enInputBound;
};

/**
 * @brief
 * 存储求解过程中的所有工作变量
 */
struct TinyWorkspace {
    int nx;  // 状态维度
    int nu;  // 输入维度
    int N;   // 时间窗长度

    // 状态和输入变量
    TinyMatrix x;  // [nx, N]
    TinyMatrix u;  // [nu, N-1]

    // 线性代价项
    TinyMatrix q;  // [nx, N]
    TinyMatrix r;  // [nu, N-1]

    // Riccati反向传播项
    TinyMatrix p;  // [nx, N]
    TinyMatrix d;  // [nu, N-1]

    // 辅助变量
    TinyMatrix v;     // [nx, N]
    TinyMatrix vnew;  // [nx, N]
    TinyMatrix z;     // [nu, N-1]
    TinyMatrix znew;  // [nu, N-1]

    // 对偶变量
    TinyMatrix g;  // [nx, N]
    TinyMatrix y;  // [nu, N-1]

    // 用户提供的系统和代价矩阵
    TinyVector Q;     // [nx, 1]
    TinyVector R;     // [nu, 1]
    TinyMatrix adyn;  // [nx, nx]
    TinyMatrix bdyn;  // [nx, nu]

    // 状态和输入的边界
    TinyMatrix xMin;  // [nx, N]
    TinyMatrix xMax;  // [nx, N]
    TinyMatrix uMin;  // [nu, N-1]
    TinyMatrix uMax;  // [nu, N-1]

    // 参考轨迹
    TinyMatrix xRef;  // [nx, N]
    TinyMatrix uRef;  // [nu, N-1]

    // 求解状态跟踪
    TinyType primalResidualState;
    TinyType primalResidualInput;
    TinyType dualResidualState;
    TinyType dualResidualInput;
    int status;
    int iter;
};

}  // namespace hitcrt
   // hitcrt