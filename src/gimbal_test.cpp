/**
 * @file main.cpp
 * @brief GimbalController类的使用示例与双轴（Pitch & Yaw）控制仿真
 * @author hitcrt (hitcrt@xxx.com)
 * @date 2024-05-20
 * @copyright Copyright (C) 2024, HITCRT_VISION, all rights reserved.
 *
 * @par 修改日志:
 * <table>
 * <tr><th>Date       <th>Author   <th>Description
 * <tr><td>2024-05-16 <td>hitcrt   <td>1. 创建初始版本。
 * <tr><td>2024-05-18 <td>hitcrt   <td>2. 扩展仿真以支持Pitch和Yaw双轴控制。
 * <tr><td>2024-05-20 <td>hitcrt   <td>3. 根据HIT-CRT代码规范进行全面重构。
 * </table>
 */

#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

#include "gimbal_control/GimbalController.h"
using namespace hitcrt;
// ---- 仿真参数 ----
/**
 * @brief 仿真器的离散时间步长 (s)
 */
const double SIMULATION_DT = 0.005;

/**
 * @brief 仿真用的二阶系统状态转移矩阵 A (适用于Pitch和Yaw)
 */
const Eigen::Matrix2d SIMULATION_A_DYN = (Eigen::Matrix2d() << 1.0, SIMULATION_DT, 0.0, 1.0).finished();

/**
 * @brief 仿真用的二阶系统输入矩阵 B (适用于Pitch和Yaw)
 */
const Eigen::Vector2d SIMULATION_B_DYN =
    (Eigen::Vector2d() << 0.5 * SIMULATION_DT * SIMULATION_DT, SIMULATION_DT).finished();

/**
 * @brief 主函数，运行云台控制器仿真
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return int 程序退出码
 * @author hitcrt (hitcrt@xxx.com)
 */
int main(int argc, char* argv[]) {
    std::cout << "======== GimbalController C++ Test Case (Pitch & Yaw) ========" << std::endl;

    // ---- 1. 定义控制器参数 ----
    // Yaw轴参数
    const int YAW_N_HORIZON = 30;
    const double YAW_DT_SOLVE = 0.015;
    const double YAW_RHO_VALUE = 5.0;
    Eigen::Vector2d yawQDiag;
    yawQDiag << 100000000.0, 0.001;
    Eigen::VectorXd yawRDiag(1);
    yawRDiag << 0.001;
    Eigen::Vector2d yawXMinVec;
    yawXMinVec << -10000.0, -20.0;  // Yaw位置几乎无限制
    Eigen::Vector2d yawXMaxVec;
    yawXMaxVec << 10000.0, 20.0;
    Eigen::VectorXd yawUMinVec(1);
    yawUMinVec << -60.0;
    Eigen::VectorXd yawUMaxVec(1);
    yawUMaxVec << 60.0;

    // Pitch轴参数
    const int PITCH_N_HORIZON = 8;
    const double PITCH_DT_SOLVE = 0.015;
    const double PITCH_RHO_VALUE = 5.0;
    Eigen::Vector2d pitchQDiag;
    pitchQDiag << 100000000.0, 0.001;
    Eigen::VectorXd pitchRDiag(1);
    pitchRDiag << 0.001;
    Eigen::Vector2d pitchXMinVec;
    pitchXMinVec << -30.0 * M_PI / 180.0, -30.0;  // Pitch位置限制为-30到60度
    Eigen::Vector2d pitchXMaxVec;
    pitchXMaxVec << 60.0 * M_PI / 180.0, 30.0;
    Eigen::VectorXd pitchUMinVec(1);
    pitchUMinVec << -180.0;
    Eigen::VectorXd pitchUMaxVec(1);
    pitchUMaxVec << 180.0;

    // ---- 2. 初始化控制器 ----
    std::unique_ptr<GimbalController> gimbalControllerPtr;
    try {
        gimbalControllerPtr = std::make_unique<GimbalController>(
            YAW_N_HORIZON, YAW_DT_SOLVE, PITCH_N_HORIZON, PITCH_DT_SOLVE, YAW_RHO_VALUE, PITCH_RHO_VALUE,
            yawQDiag, yawRDiag, yawXMinVec, yawXMaxVec, yawUMinVec, yawUMaxVec, pitchQDiag, pitchRDiag,
            pitchXMinVec, pitchXMaxVec, pitchUMinVec, pitchUMaxVec);
    } catch (const std::exception& e) {
        std::cerr << "Failed to start simulation: " << e.what() << std::endl;
        return 1;
    }

    // ---- 3. 仿真设置 ----
    // 初始化Yaw轴和Pitch轴的当前状态 [位置, 速度]
    Eigen::Vector2d currentYawState;
    currentYawState << 0.0, 0.0;  // Yaw初始位置0.0 rad, 速度0 rad/s

    Eigen::Vector2d currentPitchState;
    currentPitchState << 0.0, 0.0;  // Pitch初始位置0 rad, 速度0 rad/s

    // 目标动态参数
    double rCar = 0.3;
    double rCarNext = 0.4;
    double deltaYNext = 0.1;
    double rotateSpeed = 12.0;
    double posX = -1.0;
    double spdX = 1.0;
    double posY = -0.1;
    double spdY = 0.1;
    double posZ = 1.5;
    double spdZ = 0.0;
    double posYaw = 0.0;
    double cycAngle = M_PI / 2.0;
    double flySpeed = 20.0;  // 弹速单位m/s
    double actionTime = 0.1;
    bool isSpin = (rotateSpeed >= 2.0);  // 旋转速度阈值判断
    bool isLargeArmor = false;

    // 设置数据日志文件
    std::ofstream dataLog("gimbal_controller_log.csv");
    if (dataLog.is_open()) {
        // 写入新的CSV表头
        dataLog << "time,"
                << "pos_actual_yaw,vel_actual_yaw,accel_control_yaw,pos_goal_yaw,vel_goal_yaw,pos_predicted_"
                   "yaw,vel_predicted_yaw,iterations_yaw,"
                << "pos_actual_pitch,vel_actual_pitch,accel_control_pitch,pos_goal_pitch,vel_goal_pitch,pos_"
                   "predicted_pitch,vel_predicted_pitch,iterations_pitch,"
                << "shoot_flag,solve_time_ms\n";
    }

    // ---- 4. 仿真循环 ----
    for (int iter = 0; iter < 400; ++iter) {
        ControlOutput result;
        if (isSpin) {
            TargetParamsSpin targetParams(rCar, rCarNext, deltaYNext, rotateSpeed, posX, spdX, posY, spdY,
                                          posZ, spdZ, posYaw, cycAngle, flySpeed, actionTime, isLargeArmor);
            result = gimbalControllerPtr->update(targetParams, currentYawState(0), currentYawState(1),
                                                 currentPitchState(0), currentPitchState(1));
        } else {
            TargetParams targetParams(posX, spdX, posY, spdY, posZ, spdZ, flySpeed, actionTime, isLargeArmor);
            result = gimbalControllerPtr->update(targetParams, currentYawState(0), currentYawState(1),
                                                 currentPitchState(0), currentPitchState(1));
        }

        // 打印调试信息
        //  std::cout << "[Iter " << iter << "] "
        //            << "Solve Time: " << result.solveTimeMs << " ms, "
        //            << "Shoot: " << (result.shootFlag ? "YES" : "NO") << "\n"
        //            << "  Yaw  -> Iters: " << result.iterationsYaw << ", Accel: " << result.accelerationYaw
        //            << "\n"
        //            << "  Pitch-> Iters: " << result.iterationsPitch << ", Accel: " <<
        //            result.accelerationPitch
        //            << std::endl;

        // 记录数据到CSV文件
        if (dataLog.is_open()) {
            dataLog << iter * SIMULATION_DT << "," << currentYawState(0) << "," << currentYawState(1) << ","
                    << result.accelerationYaw << "," << result.positionGoalYaw << ","
                    << result.velocityGoalYaw << "," << result.positionPredYaw << ","
                    << result.velocityPredYaw << "," << result.iterationsYaw << "," << currentPitchState(0)
                    << "," << currentPitchState(1) << "," << result.accelerationPitch << ","
                    << result.positionGoalPitch << "," << result.velocityGoalPitch << ","
                    << result.positionPredPitch << "," << result.velocityPredPitch << ","
                    << result.iterationsPitch << "," << result.shootFlag << "," << result.solveTimeMs << "\n";
        }

        // ---- 仿真模型更新 ----
        // 使用MPC输出的控制量（加速度）来更新仿真模型的状态
        Eigen::VectorXd u0Yaw(1);
        u0Yaw << result.accelerationYaw;
        currentYawState = SIMULATION_A_DYN * currentYawState + SIMULATION_B_DYN * u0Yaw;

        Eigen::VectorXd u0Pitch(1);
        u0Pitch << result.accelerationPitch;
        currentPitchState = SIMULATION_A_DYN * currentPitchState + SIMULATION_B_DYN * u0Pitch;

        // 更新目标的位置和姿态，模拟目标的运动
        posYaw += rotateSpeed * SIMULATION_DT;
        posX += spdX * SIMULATION_DT;
        posY += spdY * SIMULATION_DT;
        posZ += spdZ * SIMULATION_DT;

        if (posYaw > M_PI / 2.0) {
            double swap = rCar;
            rCar = rCarNext;
            rCarNext = swap;
            posY += deltaYNext;
            deltaYNext = deltaYNext * -1.0;
            posYaw -= (M_PI / 2.0);
        }
    }

    if (dataLog.is_open()) {
        dataLog.close();
        std::cout << "\nSimulation data logged to gimbal_controller_log.csv" << std::endl;
    }

    return 0;
}
