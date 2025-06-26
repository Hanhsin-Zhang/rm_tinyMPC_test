/**
 * @file GimbalController.h
 * @brief 声明了基于TinyMPC的双轴（Pitch & Yaw）云台控制器类
 * @author hitcrt (hitcrt@xxx.com)
 * @date 2024-05-20
 * @copyright Copyright (C) 2024, HITCRT_VISION, all rights reserved.
 *
 * @par 修改日志:
 * <table>
 * <tr><th>Date       <th>Author   <th>Description
 * <tr><td>2024-05-16 <td>hitcrt   <td>1. 创建初始版本。
 * <tr><td>2024-05-20 <td>hitcrt   <td>2. 根据HIT-CRT代码规范进行全面重构。
 * </table>
 */
#pragma once

#include <Eigen/Dense>
#include <chrono>
#include <memory>
#include <stdexcept>
#include "TinyMpcSolver.h"

namespace hitcrt {

/**
 * @brief 目标参数结构体 (适用于旋转目标)
 * @details 存储和计算旋转目标打击所需的所有参数。
 */
struct TargetParamsSpin {
    // --- 输入参数 ---
    double rCar;          ///< 当前装甲板所在圆周半径 (m)
    double rCarNext;      ///< 下一个装甲板所在圆周半径 (m)
    double deltaYNext;    ///< 下一个装甲板与当前装甲板的高度差 (m)
    double rotateSpeed;   ///< 旋转角速度 (rad/s)
    double posX, spdX;    ///< 目标位置X, 速度X (世界系)
    double posY, spdY;    ///< 目标位置Y, 速度Y (世界系)
    double posZ, spdZ;    ///< 目标位置Z, 速度Z (世界系)
    double posYaw;        ///< 目标姿态Yaw (世界系, rad)
    double cycAngle;      ///< 装甲板之间的角度差 (rad)
    double flySpeed;      ///< 弹速 (m/s)
    double actionTime;    ///< 动作延迟 (例如：发弹延迟) (s)
    bool isLargeArmor;    ///< 是否为大装甲板

    // --- 派生计算参数 ---
    double dist;          ///< 目标在XZ平面的距离 (m)
    double linearSpeed;   ///< 旋转线速度 (m/s)
    double yawToCam;      ///< 目标车中心-相机-目标装甲板 连线的夹角 (rad)
    double rCarMean;      ///< 装甲板半径均值，用于粗略计算 (m)

    /**
     * @brief 带参构造函数，进行输入验证和自动计算。
     * @throw std::invalid_argument 如果输入参数不符合物理或逻辑约束。
     */
    TargetParamsSpin(double rCar, double rCarNext, double deltaYNext, double rotateSpeed,
                     double posX, double spdX, double posY, double spdY, double posZ, double spdZ,
                     double posYaw, double cycAngle, double flySpeed, double actionTime,
                     bool isLargeArmor);
};

/**
 * @brief 目标参数结构体 (适用于非旋转目标)
 * @details 存储和计算非旋转目标打击所需的所有参数。
 */
struct TargetParams {
    // --- 输入参数 ---
    double posX, spdX;  ///< 目标位置X, 速度X (世界系)
    double posY, spdY;  ///< 目标位置Y, 速度Y (世界系)
    double posZ, spdZ;  ///< 目标位置Z, 速度Z (世界系)
    double flySpeed;    ///< 弹速 (m/s)
    double actionTime;  ///< 动作延迟 (s)
    bool isLargeArmor;  ///< 是否为大装甲板

    // --- 派生计算参数 ---
    double dist;  ///< 目标在XZ平面的距离 (m)

    /**
     * @brief 带参构造函数，进行输入验证和自动计算。
     * @throw std::invalid_argument 如果输入参数不符合物理或逻辑约束。
     */
    TargetParams(double posX, double spdX, double posY, double spdY, double posZ, double spdZ,
                 double flySpeed, double actionTime, bool isLargeArmor);
};

/**
 * @brief 用于从GimbalController返回控制结果的结构体
 */
struct ControlOutput {
    double positionGoalPitch;   ///< Pitch轴当前时刻的参考位置 (rad)
    double velocityGoalPitch;   ///< Pitch轴当前时刻的参考速度 (rad/s)
    double positionPredPitch;   ///< MPC预测的下一时刻(k+1)的Pitch位置 (rad)
    double velocityPredPitch;   ///< MPC预测的下一时刻(k+1)的Pitch速度 (rad/s)
    double accelerationPitch;   ///< MPC计算的当前时刻(k)的最优Pitch加速度 (rad/s^2)
    int iterationsPitch;        ///< Pitch轴MPC求解迭代次数

    double positionGoalYaw;     ///< Yaw轴当前时刻的参考位置 (rad)
    double velocityGoalYaw;     ///< Yaw轴当前时刻的参考速度 (rad/s)
    double positionPredYaw;     ///< MPC预测的下一时刻(k+1)的Yaw位置 (rad)
    double velocityPredYaw;     ///< MPC预测的下一时刻(k+1)的Yaw速度 (rad/s)
    double accelerationYaw;     ///< MPC计算的当前时刻(k)的最优Yaw加速度 (rad/s^2)
    int iterationsYaw;          ///< Yaw轴MPC求解迭代次数

    bool shootFlag;             ///< 开火决策标志
    double solveTimeMs;         ///< MPC求解总耗时 (ms)
};

/**
 * @brief 基于TinyMPC的双轴云台控制器类
 * @details 封装了Pitch和Yaw两个独立MPC求解器的初始化和循环求解过程，
 *          用于计算云台的最优控制指令。
 */
class GimbalController {
public:
    /**
     * @brief 构造函数，初始化两个MPC求解器 (Yaw & Pitch)
     * @param yawHorizon        Yaw轴预测时域
     * @param yawDtSolve        Yaw轴MPC求解时间步长
     * @param pitchHorizon      Pitch轴预测时域
     * @param pitchDtSolve      Pitch轴MPC求解时间步长
     * @param yawRho            Yaw轴ADMM算法的惩罚系数
     * @param pitchRho          Pitch轴ADMM算法的惩罚系数
     * @param yawQDiag          Yaw轴状态代价权重对角向量 (维度: s_YAW_STATES)
     * @param yawRDiag          Yaw轴控制代价权重对角向量 (维度: s_YAW_INPUTS)
     * @param yawXMinVec        Yaw轴状态量下限向量 (维度: s_YAW_STATES)
     * @param yawXMaxVec        Yaw轴状态量上限向量 (维度: s_YAW_STATES)
     * @param yawUMinVec        Yaw轴控制量下限向量 (维度: s_YAW_INPUTS)
     * @param yawUMaxVec        Yaw轴控制量上限向量 (维度: s_YAW_INPUTS)
     * @param pitchQDiag        Pitch轴状态代价权重对角向量 (维度: s_PITCH_STATES)
     * @param pitchRDiag        Pitch轴控制代价权重对角向量 (维度: s_PITCH_INPUTS)
     * @param pitchXMinVec      Pitch轴状态量下限向量 (维度: s_PITCH_STATES)
     * @param pitchXMaxVec      Pitch轴状态量上限向量 (维度: s_PITCH_STATES)
     * @param pitchUMinVec      Pitch轴控制量下限向量 (维度: s_PITCH_INPUTS)
     * @param pitchUMaxVec      Pitch轴控制量上限向量 (维度: s_PITCH_INPUTS)
     * @author hitcrt (hitcrt@xxx.com)
     */
    GimbalController(int yawHorizon, double yawDtSolve, int pitchHorizon, double pitchDtSolve,
                     double yawRho, double pitchRho, const Eigen::VectorXd& yawQDiag,
                     const Eigen::VectorXd& yawRDiag, const Eigen::VectorXd& yawXMinVec,
                     const Eigen::VectorXd& yawXMaxVec, const Eigen::VectorXd& yawUMinVec,
                     const Eigen::VectorXd& yawUMaxVec, const Eigen::VectorXd& pitchQDiag,
                     const Eigen::VectorXd& pitchRDiag, const Eigen::VectorXd& pitchXMinVec,
                     const Eigen::VectorXd& pitchXMaxVec, const Eigen::VectorXd& pitchUMinVec,
                     const Eigen::VectorXd& pitchUMaxVec);

    ~GimbalController() = default;

    /**
     * @brief 在每个控制循环中调用此方法来计算最优控制量
     * @tparam ParamType 目标参数类型 (TargetParamsSpin 或 TargetParams)
     * @param params 包含生成参考轨迹所需的所有动态参数的结构体
     * @param currentYawPos 当前Yaw轴的实际位置 (rad)
     * @param currentYawVel 当前Yaw轴的实际速度 (rad/s)
     * @param currentPitchPos 当前Pitch轴的实际位置 (rad)
     * @param currentPitchVel 当前Pitch轴的实际速度 (rad/s)
     * @return ControlOutput 包含控制指令、开火决策和性能指标的结构体
     * @author hitcrt (hitcrt@xxx.com)
     */
    template <typename ParamType>
    ControlOutput update(const ParamType& params, double currentYawPos, double currentYawVel,
                         double currentPitchPos, double currentPitchVel) {
        auto startTime = std::chrono::high_resolution_clock::now();

        // 1. 生成参考轨迹 (C++会自动根据params的类型调用正确的重载)
        TinyMatrix xRefYaw(s_YAW_STATES, m_yawHorizon);
        TinyMatrix xRefPitch(s_PITCH_STATES, m_pitchHorizon);
        generateReferenceTrajectory(xRefYaw, xRefPitch, params);

        // 2. 设置并运行Yaw轴MPC求解器
        TinyMatrix uRefYaw = TinyMatrix::Zero(s_YAW_INPUTS, m_yawHorizon - 1);
        TinyVector x0Yaw(s_YAW_STATES);
        x0Yaw << currentYawPos, currentYawVel;
        m_yawSolverPtr->setInitialState(x0Yaw);
        m_yawSolverPtr->setStateReference(xRefYaw);
        m_yawSolverPtr->setInputReference(uRefYaw);
        m_yawSolverPtr->solve();
        const TinySolution& yawSolution = m_yawSolverPtr->getSolution();

        // 3. 设置并运行Pitch轴MPC求解器
        TinyMatrix uRefPitch = TinyMatrix::Zero(s_PITCH_INPUTS, m_pitchHorizon - 1);
        TinyVector x0Pitch(s_PITCH_STATES);
        x0Pitch << currentPitchPos, currentPitchVel;
        m_pitchSolverPtr->setInitialState(x0Pitch);
        m_pitchSolverPtr->setStateReference(xRefPitch);
        m_pitchSolverPtr->setInputReference(uRefPitch);
        m_pitchSolverPtr->solve();
        const TinySolution& pitchSolution = m_pitchSolverPtr->getSolution();

        // 4. 根据参数类型，在编译时决定如何计算shoot_flag
        bool shootFlag;
        if constexpr (std::is_same_v<ParamType, TargetParamsSpin>) {
            shootFlag = calculateShootFlag(yawSolution.x, xRefYaw, pitchSolution.x, xRefPitch, params);
        } else {
            // 非旋转目标，可以根据误差决定是否开火，此处简化为不开火
            shootFlag = false; 
        }

        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);

        // 5. 填充并返回输出
        ControlOutput output;
        output.positionGoalYaw = xRefYaw(0, 0);
        output.velocityGoalYaw = xRefYaw(1, 0);
        output.positionPredYaw = yawSolution.x(0, 1);
        output.velocityPredYaw = yawSolution.x(1, 1);
        output.accelerationYaw = yawSolution.u(0, 0);
        output.iterationsYaw = yawSolution.iter;

        output.positionGoalPitch = xRefPitch(0, 0);
        output.velocityGoalPitch = xRefPitch(1, 0);
        output.positionPredPitch = pitchSolution.x(0, 1);
        output.velocityPredPitch = pitchSolution.x(1, 1);
        output.accelerationPitch = pitchSolution.u(0, 0);
        output.iterationsPitch = pitchSolution.iter;

        output.shootFlag = shootFlag;
        output.solveTimeMs = duration.count() / 1000.0;
        return output;
    }

private:
    // --- 私有成员变量 ---
    std::unique_ptr<TinyMpcSolver> m_yawSolverPtr;    ///< Yaw轴MPC求解器
    std::unique_ptr<TinyMpcSolver> m_pitchSolverPtr;  ///< Pitch轴MPC求解器

    // MPC维度参数 (固定为二阶系统)
    static constexpr int s_YAW_STATES = 2;
    static constexpr int s_YAW_INPUTS = 1;
    static constexpr int s_PITCH_STATES = 2;
    static constexpr int s_PITCH_INPUTS = 1;

    // MPC配置参数 (由构造函数初始化)
    const int m_yawHorizon;      ///< Yaw轴预测时域
    const double m_yawDtSolve;   ///< Yaw轴求解时间步长
    const int m_pitchHorizon;    ///< Pitch轴预测时域
    const double m_pitchDtSolve; ///< Pitch轴求解时间步长

    // --- 私有方法 ---
    /**
     * @brief 为旋转目标生成参考轨迹
     * @param[out] xRefOutYaw Yaw轴的参考状态轨迹矩阵
     * @param[out] xRefOutPitch Pitch轴的参考状态轨迹矩阵
     * @param[in] params 旋转目标的参数
     * @author hitcrt (hitcrt@xxx.com)
     */
    void generateReferenceTrajectory(TinyMatrix& xRefOutYaw, TinyMatrix& xRefOutPitch,
                                     const TargetParamsSpin& params);

    /**
     * @brief 为非旋转目标生成参考轨迹
     * @param[out] xRefOutYaw Yaw轴的参考状态轨迹矩阵
     * @param[out] xRefOutPitch Pitch轴的参考状态轨迹矩阵
     * @param[in] params 非旋转目标的参数
     * @author hitcrt (hitcrt@xxx.com)
     */
    void generateReferenceTrajectory(TinyMatrix& xRefOutYaw, TinyMatrix& xRefOutPitch,
                                     const TargetParams& params);

    /**
     * @brief 计算是否可以开火
     * @param predictedTrajectoryYaw MPC预测的Yaw轨迹
     * @param referenceTrajectoryYaw 参考的Yaw轨迹
     * @param predictedTrajectoryPitch MPC预测的Pitch轨迹
     * @param referenceTrajectoryPitch 参考的Pitch轨迹
     * @param params 旋转目标的参数
     * @return bool 如果预测误差在阈值内，返回true
     * @author hitcrt (hitcrt@xxx.com)
     */
    bool calculateShootFlag(const TinyMatrix& predictedTrajectoryYaw,
                            const TinyMatrix& referenceTrajectoryYaw,
                            const TinyMatrix& predictedTrajectoryPitch,
                            const TinyMatrix& referenceTrajectoryPitch, const TargetParamsSpin& params);

    /**
     * @brief 计算子弹飞行时间 (旋转目标)
     * @param params 旋转目标的参数
     * @param timeDelay 预测的未来时间点
     * @return double 飞行时间 (s)
     * @author hitcrt (hitcrt@xxx.com)
     */
    double calculateFlyTimeSpin(const TargetParamsSpin& params, double timeDelay);

    /**
     * @brief 计算子弹飞行时间 (非旋转目标)
     * @param params 非旋转目标的参数
     * @param timeDelay 预测的未来时间点
     * @return double 飞行时间 (s)
     * @author hitcrt (hitcrt@xxx.com)
     */
    double calculateFlyTimeNorm(const TargetParams& params, double timeDelay);

    /**
     * @brief 计算抬枪补偿后的Pitch角度
     * @param y 目标高度 (m)
     * @param dist 水平距离 (m)
     * @param flySpeed 弹速 (m/s)
     * @return double 需要的Pitch角度 (rad)
     * @author hitcrt (hitcrt@xxx.com)
     */
    double calculateFirePitchRad(double y, double dist, double flySpeed);
    
    /**
     * @brief 计算目标Yaw角度
     * @param x 目标X坐标 (m)
     * @param z 目标Z坐标 (m)
     * @return double 需要的Yaw角度 (rad)
     * @author hitcrt (hitcrt@xxx.com)
     */
    double calculateFireYawRad(double x, double z);

    /**
     * @brief 修正角度，处理-PI到+PI的跳变问题
     * @param lastYaw 上一时刻的角度 (rad)
     * @param thisYaw 当前时刻的角度 (rad)
     * @return double 修正后的角度差值 (rad)
     * @author hitcrt (hitcrt@xxx.com)
     */
    double fixAngle(double lastYaw, double thisYaw);
};

}  // namespace hitcrt