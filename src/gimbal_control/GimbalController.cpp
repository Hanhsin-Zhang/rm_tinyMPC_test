/**
 * @file GimbalController.cpp
 * @brief 实现了GimbalController类
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
#include "GimbalController.h"

#include <cassert>
#include <cmath>
#include <iostream>

namespace hitcrt {

namespace {
/**
 * @brief (辅助函数) 给定距离，半径和yaw，计算目标车中心-本车中心-击打目标的夹角
 * @param angleToCam 目标装甲板-目标车中心-自身中心夹角 (rad)
 * @param r 装甲板到车中心的半径 (m)
 * @param d 自身到车中心的距离 (m)
 * @return double 云台需要转过的角度 (rad)
 * @author hitcrt (hitcrt@xxx.com)
 */
double getAngleToGimbal(double angleToCam, double r, double d) {
    return std::atan2(r * std::sin(angleToCam), d + r * std::cos(angleToCam));
}
}  // namespace

/**
 * @brief TargetParamsSpin 构造函数
 * @author hitcrt (hitcrt@xxx.com)
 */
TargetParamsSpin::TargetParamsSpin(double _rCar, double _rCarNext, double _deltaYNext,
                                   double _rotateSpeed, double _posX, double _spdX, double _posY,
                                   double _spdY, double _posZ, double _spdZ, double _posYaw,
                                   double _cycAngle, double _flySpeed, double _actionTime,
                                   bool _isLargeArmor)
    : rCar(_rCar),
      rCarNext(_rCarNext),
      deltaYNext(_deltaYNext),
      rotateSpeed(_rotateSpeed),
      posX(_posX),
      spdX(_spdX),
      posY(_posY),
      spdY(_spdY),
      posZ(_posZ),
      spdZ(_spdZ),
      posYaw(_posYaw),
      cycAngle(_cycAngle),
      flySpeed(_flySpeed),
      actionTime(_actionTime),
      isLargeArmor(_isLargeArmor) {
    constexpr double epsilon = 1e-6;
    if (rCar < 0.0) {
        throw std::invalid_argument("Input error: rCar cannot be negative.");
    }
    if (rCarNext < 0.0) {
        throw std::invalid_argument("Input error: rCarNext cannot be negative.");
    }
    if (flySpeed <= epsilon) {
        throw std::invalid_argument("Input error: flySpeed must be positive.");
    }
    if (actionTime < 0.0) {
        throw std::invalid_argument("Input error: actionTime cannot be negative.");
    }

    dist = std::hypot(posX, posZ);
    linearSpeed = rCar * rotateSpeed;
    yawToCam = posYaw - std::atan2(posX, posZ);
    rCarMean = (rCar + rCarNext) / 2.0;
}

/**
 * @brief TargetParams 构造函数
 * @author hitcrt (hitcrt@xxx.com)
 */
TargetParams::TargetParams(double _posX, double _spdX, double _posY, double _spdY, double _posZ,
                           double _spdZ, double _flySpeed, double _actionTime, bool _isLargeArmor)
    : posX(_posX),
      spdX(_spdX),
      posY(_posY),
      spdY(_spdY),
      posZ(_posZ),
      spdZ(_spdZ),
      flySpeed(_flySpeed),
      actionTime(_actionTime),
      isLargeArmor(_isLargeArmor) {
    constexpr double epsilon = 1e-6;
    if (flySpeed <= epsilon) {
        throw std::invalid_argument("Input error: flySpeed must be positive.");
    }
    if (actionTime < 0.0) {
        throw std::invalid_argument("Input error: actionTime cannot be negative.");
    }
    dist = std::hypot(posX, posZ);
}

/**
 * @brief GimbalController 构造函数
 * @author hitcrt (hitcrt@xxx.com)
 */
GimbalController::GimbalController(
    int yawHorizon, double yawDtSolve, int pitchHorizon, double pitchDtSolve, double yawRho,
    double pitchRho, const Eigen::VectorXd& yawQDiag, const Eigen::VectorXd& yawRDiag,
    const Eigen::VectorXd& yawXMinVec, const Eigen::VectorXd& yawXMaxVec,
    const Eigen::VectorXd& yawUMinVec, const Eigen::VectorXd& yawUMaxVec,
    const Eigen::VectorXd& pitchQDiag, const Eigen::VectorXd& pitchRDiag,
    const Eigen::VectorXd& pitchXMinVec, const Eigen::VectorXd& pitchXMaxVec,
    const Eigen::VectorXd& pitchUMinVec, const Eigen::VectorXd& pitchUMaxVec)
    : m_yawHorizon(yawHorizon),
      m_yawDtSolve(yawDtSolve),
      m_pitchHorizon(pitchHorizon),
      m_pitchDtSolve(pitchDtSolve) {
    assert(m_yawHorizon > 1 && "Yaw horizon is too small.");
    assert(m_pitchHorizon > 1 && "Pitch horizon is too small.");
    assert(yawQDiag.size() == s_YAW_STATES && "yawQDiag vector has incorrect dimensions.");
    assert(yawRDiag.size() == s_YAW_INPUTS && "yawRDiag vector has incorrect dimensions.");
    assert(yawXMinVec.size() == s_YAW_STATES && "yawXMinVec vector has incorrect dimensions.");
    assert(yawXMaxVec.size() == s_YAW_STATES && "yawXMaxVec vector has incorrect dimensions.");
    assert(yawUMinVec.size() == s_YAW_INPUTS && "yawUMinVec vector has incorrect dimensions.");
    assert(yawUMaxVec.size() == s_YAW_INPUTS && "yawUMaxVec vector has incorrect dimensions.");
    assert(pitchQDiag.size() == s_PITCH_STATES && "pitchQDiag vector has incorrect dimensions.");
    assert(pitchRDiag.size() == s_PITCH_INPUTS && "pitchRDiag vector has incorrect dimensions.");
    assert(pitchXMinVec.size() == s_PITCH_STATES && "pitchXMinVec vector has incorrect dimensions.");
    assert(pitchXMaxVec.size() == s_PITCH_STATES && "pitchXMaxVec vector has incorrect dimensions.");
    assert(pitchUMinVec.size() == s_PITCH_INPUTS && "pitchUMinVec vector has incorrect dimensions.");
    assert(pitchUMaxVec.size() == s_PITCH_INPUTS && "pitchUMaxVec vector has incorrect dimensions.");

    // 1. 数据准备 - Yaw轴
    TinyMatrix adynYaw(s_YAW_STATES, s_YAW_STATES);
    adynYaw << 1.0, m_yawDtSolve, 0.0, 1.0;
    TinyMatrix bdynYaw(s_YAW_STATES, s_YAW_INPUTS);
    bdynYaw << 0.5 * m_yawDtSolve * m_yawDtSolve, m_yawDtSolve;
    TinyMatrix QYaw = yawQDiag.asDiagonal();
    TinyMatrix RYaw = yawRDiag.asDiagonal();
    TinyMatrix xMinYaw = yawXMinVec.replicate(1, m_yawHorizon);
    TinyMatrix xMaxYaw = yawXMaxVec.replicate(1, m_yawHorizon);
    TinyMatrix uMinYaw = yawUMinVec.replicate(1, m_yawHorizon - 1);
    TinyMatrix uMaxYaw = yawUMaxVec.replicate(1, m_yawHorizon - 1);

    // 2. 数据准备 - Pitch轴
    TinyMatrix adynPitch(s_PITCH_STATES, s_PITCH_STATES);
    adynPitch << 1.0, m_pitchDtSolve, 0.0, 1.0;
    TinyMatrix bdynPitch(s_PITCH_STATES, s_PITCH_INPUTS);
    bdynPitch << 0.5 * m_pitchDtSolve * m_pitchDtSolve, m_pitchDtSolve;
    TinyMatrix QPitch = pitchQDiag.asDiagonal();
    TinyMatrix RPitch = pitchRDiag.asDiagonal();
    TinyMatrix xMinPitch = pitchXMinVec.replicate(1, m_pitchHorizon);
    TinyMatrix xMaxPitch = pitchXMaxVec.replicate(1, m_pitchHorizon);
    TinyMatrix uMinPitch = pitchUMinVec.replicate(1, m_pitchHorizon - 1);
    TinyMatrix uMaxPitch = pitchUMaxVec.replicate(1, m_pitchHorizon - 1);

    // 3. 初始化求解器
    try {
        m_yawSolverPtr = std::make_unique<TinyMpcSolver>(adynYaw, bdynYaw, QYaw, RYaw, xMinYaw,
                                                         xMaxYaw, uMinYaw, uMaxYaw, s_YAW_STATES,
                                                         s_YAW_INPUTS, m_yawHorizon, yawRho, false);
    } catch (const std::invalid_argument& e) {
        std::cerr << "ERROR: Failed to initialize Yaw GimbalController: " << e.what() << std::endl;
        throw;
    }
    try {
        m_pitchSolverPtr = std::make_unique<TinyMpcSolver>(
            adynPitch, bdynPitch, QPitch, RPitch, xMinPitch, xMaxPitch, uMinPitch, uMaxPitch,
            s_PITCH_STATES, s_PITCH_INPUTS, m_pitchHorizon, pitchRho, false);
    } catch (const std::invalid_argument& e) {
        std::cerr << "ERROR: Failed to initialize Pitch GimbalController: " << e.what() << std::endl;
        throw;
    }
}

// 显式实例化模板函数，使其定义可以保留在头文件中
template ControlOutput GimbalController::update<hitcrt::TargetParamsSpin>(
    const hitcrt::TargetParamsSpin&, double, double, double, double);
template ControlOutput GimbalController::update<hitcrt::TargetParams>(const hitcrt::TargetParams&,
                                                                       double, double, double,
                                                                       double);

/**
 * @brief generateReferenceTrajectory for TargetParamsSpin
 * @author hitcrt (hitcrt@xxx.com)
 */
void GimbalController::generateReferenceTrajectory(TinyMatrix& xRefOutYaw,
                                                   TinyMatrix& xRefOutPitch,
                                                   const TargetParamsSpin& params) {
    const int rotateDir = (params.rotateSpeed > 0) - (params.rotateSpeed < 0);
    const double cycAngleHalf = 0.5 * params.cycAngle;
    double lastYaw = 0.0;

    for (int i = 0; i < m_yawHorizon; ++i) {
        const double t = i * m_yawDtSolve;
        double flyTime = calculateFlyTimeSpin(params, t);
        const double targetPosX = params.posX + params.spdX * (t + flyTime);
        const double targetPosZ = params.posZ + params.spdZ * (t + flyTime);
        const double targetDistXZ = std::hypot(targetPosX, targetPosZ);

        double angleToCam = params.yawToCam + params.rotateSpeed * (flyTime + t);
        const int hitNumYaw =
            std::abs(std::floor((rotateDir * angleToCam + cycAngleHalf) / params.cycAngle));
        const double rCar = (hitNumYaw & 1) ? params.rCarNext : params.rCar;
        angleToCam = angleToCam - rotateDir * hitNumYaw * params.cycAngle;
        const double angleGoalPosBase = getAngleToGimbal(angleToCam, rCar, targetDistXZ);
        const double distToTargetArmorSq =
            rCar * rCar + targetDistXZ * targetDistXZ +
            2 * rCar * targetDistXZ * std::cos(angleToCam);
        const double distToTargetArmor = std::sqrt(distToTargetArmorSq);

        double angleGoalSpeed =
            params.linearSpeed * std::cos(angleToCam) / distToTargetArmor;
        angleGoalSpeed -= (params.posX * params.spdZ - params.posZ * params.spdX) /
                          (distToTargetArmor * distToTargetArmor);

        const double angleGoalPosFinal = angleGoalPosBase + std::atan2(targetPosX, targetPosZ);

        if (i == 0) {
            lastYaw = angleGoalPosFinal;
            xRefOutYaw(0, i) = angleGoalPosFinal;
        } else {
            double currentYaw = lastYaw + fixAngle(lastYaw, angleGoalPosFinal);
            xRefOutYaw(0, i) = currentYaw;
            lastYaw = currentYaw;
        }
        xRefOutYaw(1, i) = angleGoalSpeed;
    }

    for (int i = 0; i < m_pitchHorizon; ++i) {
        const double t = i * m_pitchDtSolve;
        double flyTime = calculateFlyTimeSpin(params, t);
        const double targetPosX = params.posX + params.spdX * (t + flyTime);
        const double targetPosZ = params.posZ + params.spdZ * (t + flyTime);
        const double targetDistXZ = std::hypot(targetPosX, targetPosZ);

        double angleToCam = params.yawToCam + params.rotateSpeed * (flyTime + t);
        const int hitNumYaw = std::abs(std::floor((rotateDir * angleToCam + cycAngleHalf) / params.cycAngle));
        angleToCam = angleToCam - rotateDir * hitNumYaw * params.cycAngle;
        const double rCar = (hitNumYaw & 1) ? params.rCarNext : params.rCar;
        
        const double armorY = (hitNumYaw & 1) ? (params.posY + params.deltaYNext) : params.posY;
        const double distXZReal = targetDistXZ - std::cos(angleToCam) * rCar;
        const double targetHeight = params.posY; // Y速度不稳，不用于预测
        const double targetSpdHRZ = (targetPosX * params.spdZ + targetPosZ * params.spdX) / targetDistXZ;
        const double velocityProjectionTerm = (-targetHeight * targetSpdHRZ);
        const double targetDistAll = std::hypot(targetDistXZ, targetHeight);
        const double angleGoalSpeed = velocityProjectionTerm / (targetDistAll * targetDistAll);
        
        const double angleGoalPosFinal = calculateFirePitchRad(armorY, distXZReal, params.flySpeed);
        xRefOutPitch(0, i) = angleGoalPosFinal;
        xRefOutPitch(1, i) = angleGoalSpeed;
    }
}

/**
 * @brief generateReferenceTrajectory for TargetParams
 * @author hitcrt (hitcrt@xxx.com)
 */
void GimbalController::generateReferenceTrajectory(TinyMatrix& xRefOutYaw,
                                                   TinyMatrix& xRefOutPitch,
                                                   const TargetParams& params) {
    double lastYaw = 0.0;
    for (int i = 0; i < m_yawHorizon; ++i) {
        const double t = i * m_yawDtSolve;
        double flyTime = calculateFlyTimeNorm(params, t);
        const double targetPosX = params.posX + params.spdX * (t + flyTime);
        const double targetPosZ = params.posZ + params.spdZ * (t + flyTime);
        const double targetDistXZ = std::hypot(targetPosX, targetPosZ);
        const double velocityProjectionTerm =
            (targetPosX * params.spdZ - targetPosZ * params.spdX);
        const double angleGoalSpeed = -1.0 * velocityProjectionTerm / (targetDistXZ * targetDistXZ);
        const double angleGoalPosFinal = std::atan2(targetPosX, targetPosZ);

        if (i == 0) {
            lastYaw = angleGoalPosFinal;
            xRefOutYaw(0, i) = angleGoalPosFinal;
        } else {
            double currentYaw = lastYaw + fixAngle(lastYaw, angleGoalPosFinal);
            xRefOutYaw(0, i) = currentYaw;
            lastYaw = currentYaw;
        }
        xRefOutYaw(1, i) = angleGoalSpeed;
    }
    for (int i = 0; i < m_pitchHorizon; ++i) {
        const double t = i * m_pitchDtSolve;
        double flyTime = calculateFlyTimeNorm(params, t);
        const double targetPosX = params.posX + params.spdX * (t + flyTime);
        const double targetPosZ = params.posZ + params.spdZ * (t + flyTime);
        const double targetDistXZ = std::hypot(targetPosX, targetPosZ);
        const double targetHeight = params.posY + params.spdY * (t + flyTime);
        const double targetSpdHRZ =
            (targetPosX * params.spdX + targetPosZ * params.spdZ) / targetDistXZ;
        const double velocityProjectionTerm =
            (targetDistXZ * params.spdY - targetHeight * targetSpdHRZ);
        const double targetDistAll = std::hypot(targetDistXZ, targetHeight);
        const double angleGoalSpeed = velocityProjectionTerm / (targetDistAll * targetDistAll);
        const double angleGoalPosFinal = calculateFirePitchRad(targetHeight, targetDistXZ, params.flySpeed);
        
        xRefOutPitch(0, i) = angleGoalPosFinal;
        xRefOutPitch(1, i) = angleGoalSpeed;
    }
}

/**
 * @brief calculateShootFlag implementation
 * @author hitcrt (hitcrt@xxx.com)
 */
bool GimbalController::calculateShootFlag(const TinyMatrix& predictedTrajectoryYaw,
                                          const TinyMatrix& referenceTrajectoryYaw,
                                          const TinyMatrix& predictedTrajectoryPitch,
                                          const TinyMatrix& referenceTrajectoryPitch,
                                          const TargetParamsSpin& params) {
    constexpr double LARGE_ARMOR_WIDTH = 0.230;
    constexpr double SMALL_ARMOR_WIDTH = 0.135;

    double flyTime = calculateFlyTimeSpin(params, 0);
    double timeToHit = params.actionTime + flyTime;

    if (timeToHit > (m_yawHorizon - 1) * m_yawDtSolve || timeToHit < 0) {
        std::cout<<"timeToHit is too long to predict!"<<std::endl;
        return false;
    }

    int kPred = static_cast<int>(std::round(timeToHit / m_yawDtSolve));
    kPred = std::max(0, std::min(kPred, m_yawHorizon - 1));

    const double predictedPos = predictedTrajectoryYaw(0, kPred);
    const double referencePos = referenceTrajectoryYaw(0, kPred);
    const double positionError = std::abs(predictedPos - referencePos);

    const int rotateDir = (params.rotateSpeed > 0) - (params.rotateSpeed < 0);
    double angleToCam = params.yawToCam + params.rotateSpeed * timeToHit;
    int hitNumYaw = std::abs(
        std::floor((rotateDir * angleToCam + (0.5 * params.cycAngle)) / params.cycAngle));
    angleToCam = angleToCam - rotateDir * hitNumYaw * params.cycAngle;
    const double rCar = (hitNumYaw % 2 == 1) ? params.rCarNext : params.rCar;
    double distToTargetArmor =
        std::sqrt(rCar * rCar + params.dist * params.dist + 2 * rCar * params.dist * std::cos(angleToCam));

    const double armorWidth = params.isLargeArmor ? LARGE_ARMOR_WIDTH : SMALL_ARMOR_WIDTH;
    const double armorProjectedLen = std::cos(angleToCam) * (armorWidth / 2.0);
    const double shootYawThreshold = std::atan(armorProjectedLen / distToTargetArmor) * 1.0;

    return positionError < shootYawThreshold;
}

/**
 * @brief calculateFlyTimeSpin implementation
 * @author hitcrt (hitcrt@xxx.com)
 */
double GimbalController::calculateFlyTimeSpin(const TargetParamsSpin& params, double timeDelay) {
    const double targetPosX = params.posX + params.spdX * timeDelay;
    const double targetPosY = params.posY;  // 假设整车不会上下跳
    const double targetPosZ = params.posZ + params.spdZ * timeDelay;

    constexpr double epsilon = 1e-6;
    double a = (params.spdX * params.spdX + params.spdY * params.spdY + params.spdZ * params.spdZ -
                params.flySpeed * params.flySpeed);
    double b = 2 * (targetPosX * params.spdX + targetPosY * params.spdY + targetPosZ * params.spdZ -
                params.rCarMean * params.flySpeed * std::abs(params.rotateSpeed));
    double c = targetPosX * targetPosX + targetPosY * targetPosY + targetPosZ * targetPosZ -
               params.rCarMean * params.rCarMean;

    double discriminant = b * b - 4 * a * c;
    if (discriminant < 0) {
        return params.dist / params.flySpeed; // 返回无解时的估算值
    }

    if (std::abs(a) < epsilon) {
        return (std::abs(b) < epsilon) ? (params.dist / params.flySpeed) : (-c / b);
    } else {
        double sqrtDiscriminant = std::sqrt(discriminant);
        double t1 = (-b + sqrtDiscriminant) / (2 * a);
        double t2 = (-b - sqrtDiscriminant) / (2 * a);
        if (t1 > epsilon && (t1 < t2 || t2 < epsilon)) {
            return t1;
        }
        if (t2 > epsilon) {
            return t2;
        }
    }
    return params.dist / params.flySpeed; // 返回无解时的估算值
}

/**
 * @brief calculateFlyTimeNorm implementation
 * @author hitcrt (hitcrt@xxx.com)
 */
double GimbalController::calculateFlyTimeNorm(const TargetParams& params, double timeDelay) {
    const double targetPosX = params.posX + params.spdX * timeDelay;
    const double targetPosY = params.posY + params.spdY * timeDelay;
    const double targetPosZ = params.posZ + params.spdZ * timeDelay;

    constexpr double epsilon = 1e-6;
    double a = (params.spdX * params.spdX + params.spdY * params.spdY + params.spdZ * params.spdZ -
                params.flySpeed * params.flySpeed);
    double b = 2 * (targetPosX * params.spdX + targetPosY * params.spdY + targetPosZ * params.spdZ);
    double c = targetPosX * targetPosX + targetPosY * targetPosY + targetPosZ * targetPosZ;

    double discriminant = b * b - 4 * a * c;
    if (discriminant < 0) {
        return params.dist / params.flySpeed; // 返回无解时的估算值
    }

    if (std::abs(a) < epsilon) {
       return (std::abs(b) < epsilon) ? (params.dist / params.flySpeed) : (-c / b);
    } else {
        double sqrtDiscriminant = std::sqrt(discriminant);
        double t1 = (-b + sqrtDiscriminant) / (2 * a);
        double t2 = (-b - sqrtDiscriminant) / (2 * a);
        if (t1 > epsilon && (t1 < t2 || t2 < epsilon)) {
            return t1;
        }
        if (t2 > epsilon) {
            return t2;
        }
    }
    return params.dist / params.flySpeed; // 返回无解时的估算值
}

/**
 * @brief calculateFirePitchRad implementation
 * @author hitcrt (hitcrt@xxx.com)
 */
double GimbalController::calculateFirePitchRad(double y, double dist, double flySpeed) {
    if (dist <= 0 || flySpeed <= 0) {
        return 0.0;
    }
    const double GRAVITY = 9.8;
    const double a = (GRAVITY * dist * dist) / (2.0 * flySpeed * flySpeed);
    const double b = -dist;
    const double c = y + a;
    const double delta = b * b - 4.0 * a * c;
    if (delta < 0) {
        return 0.0;  // 目标无法击中
    }
    const double tanTheta = (-b - std::sqrt(delta)) / (2.0 * a);
    return std::atan(tanTheta);
}

/**
 * @brief calculateFireYawRad implementation
 * @author hitcrt (hitcrt@xxx.com)
 */
double GimbalController::calculateFireYawRad(double x, double z) { 
    return std::atan2(x, z); 
}

/**
 * @brief fixAngle implementation
 * @author hitcrt (hitcrt@xxx.com)
 */
double GimbalController::fixAngle(double lastYaw, double thisYaw) {
    double deltaYaw = thisYaw - lastYaw;
    if (deltaYaw > M_PI) {
        deltaYaw -= 2 * M_PI;
    } else if (deltaYaw < -M_PI) {
        deltaYaw += 2 * M_PI;
    }
    return deltaYaw;
}

}  // namespace hitcrt