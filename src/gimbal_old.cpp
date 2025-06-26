#include <Eigen/Dense>  // 确保包含了Eigen头文件
#include <chrono>       // 新增：用于高精度计时
#include <cmath>        // 用于数学计算 (sin, cos, sqrt, etc.)
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>
// 包含重构后的求解器头文件
#include "tiny_mpc/TinyMpcSolver.h"

// 使用 hitcrt 命名空间
using namespace hitcrt;

// ---- 1. 参数定义 (根据MATLAB脚本) ----

// MPC维度
constexpr int N_STATES = 2;    // 状态量维度 [位置, 速度]
constexpr int N_INPUTS = 1;    // 控制量维度 [加速度]
constexpr int N_HORIZON = 30;  // 预测长度 (MATLAB: N = 19)

// ADMM 惩罚参数 (可根据实际情况调整)
constexpr TinyType RHO_VALUE = 5.0;

// 系统动力学参数
const TinyType DT_SIM =
    0.005;  // 被控系统（plant）的仿真步长 (MATLAB: dtsolve = dtreal*timefactor = 1/200 * 4 = 0.02)
const TinyType DT_SOLVE =
    0.015;  // MPC求解步长 (MATLAB: dtsolve = dtreal*timefactor = 1/200 * 4 = 0.02)

// 状态转移矩阵 A [2x2] (MATLAB: system.A2)
// A = [1, dt; 0, 1]
const TinyType ADYN_DATA[N_STATES * N_STATES] = {1.0, 0.0, DT_SOLVE, 1.0};
const TinyType ADYN_DATA_SIM[N_STATES * N_STATES] = {1.0, 0.0, DT_SIM, 1.0};

// 输入矩阵 B [2x1] (MATLAB: system.B2)
// B = [0.5*dt^2; dt]
const TinyType BDYN_DATA[N_STATES * N_INPUTS] = {0.5 * DT_SOLVE * DT_SOLVE, DT_SOLVE};
const TinyType BDYN_DATA_SIM[N_STATES * N_INPUTS] = {0.5 * DT_SIM * DT_SIM, DT_SIM};

// 状态代价权重 Q (对角线元素) [2x1] (MATLAB: Q = 10*eye(1) -> 惩罚第一个状态)
const TinyType Q_DATA[N_STATES] = {100000000.0, 0.001};

// 输入代价权重 R (对角线元素) [1x1] (MATLAB: R = 0*eye(1))
const TinyType R_DATA[N_INPUTS] = {0.001};

// 状态约束 (MATLAB: xmin, xmax)
const TinyType X_MIN_DATA[N_STATES] = {-1000.0, -20.0};  // [rad, rad/s]
const TinyType X_MAX_DATA[N_STATES] = {1000.0, 20.0};    // [rad, rad/s]

// 输入约束 (MATLAB: umin, umax)
const TinyType U_MIN_DATA[N_INPUTS] = {-60.0};  // [rad/s^2]
const TinyType U_MAX_DATA[N_INPUTS] = {60.0};   // [rad/s^2]

/**
 * @brief 用于生成参考轨迹的物理参数包
 */
struct TrajectoryParams {
    // 基础物理参数
    double Rcar;
    double RcarNext;
    double rotateSpeed;
    double PosX;    //来自滤波器
    double SpdX;    //来自滤波器
    double PosZ;    //来自滤波器
    double SpdZ;    //来自滤波器
    double PosYaw;  //来自滤波器
    double flyspeed;

    // 派生参数 (在构造函数中计算，以提高效率)
    double linearspeed;
    double cycangle;
    double flytime;
    double dist;
    double yawToCam;
    /**
     * @brief 构造函数，初始化所有参数
     * @param r 车辆半径 (m)
     * @param rs 车辆转速 (rad/s)
     * @param d 车辆距离 (m)
     * @param fs 弹速 (m/s)
     */
    TrajectoryParams(double r, double r2, double rs, double x, double z, double vx, double vz,
                     double yaw, double fs)
        : Rcar(r),
          RcarNext(r2),
          rotateSpeed(rs),
          PosX(x),
          PosZ(z),
          SpdX(vx),
          SpdZ(vz),
          PosYaw(yaw),
          flyspeed(fs) {
        // 计算派生参数
        dist = sqrt(pow(PosX, 2) + pow(PosZ, 2));
        linearspeed = Rcar * rotateSpeed;
        cycangle = M_PI / 2.0;
        flytime = (dist - (Rcar + RcarNext) / 2) / flyspeed;
        yawToCam = PosYaw - atan2(PosX, PosZ);
    }
};
//给定距离，半径和yaw，计算目标车中心-本车中心-击打目标的夹角
double getAngleToGimbal(double angleToCam, double r, double d) {
    return atan2(r * sin(angleToCam), d + r * cos(angleToCam));
};

/**
 * @brief 根据给定的物理参数生成参考轨迹
 * @param x_ref_out    生成的参考轨迹矩阵 (N_STATES x N_HORIZON)
 * @param params       包含所有物理参数的结构体
 * @param dt           MPC求解的时间步长
 */
void generate_reference_trajectory(TinyMatrix& x_ref_out, const TrajectoryParams& params,
                                   double dt) {
    const int rotateDir = (params.rotateSpeed > 0) - (params.rotateSpeed < 0);
    const double rotate_speed_flytime_offset = params.rotateSpeed * params.flytime;
    const double cycangle_half = 0.5 * params.cycangle;
    const double dist_sq = params.dist * params.dist;
    const double velocity_projection_term =
        (params.PosX * params.SpdZ - params.PosZ * params.SpdX) / params.dist;

    for (int i = 0; i < N_HORIZON; ++i) {
        const double t = i * dt;
        //angleToCam:目标装甲板-目标车中心-自身中心夹角
        double angleToCam = params.yawToCam + rotate_speed_flytime_offset + params.rotateSpeed * t;
        const int m_hitNumYaw =
            std::abs(floor((rotateDir * angleToCam + cycangle_half) / params.cycangle));
        const double Rcar = (m_hitNumYaw & 1) ? params.RcarNext : params.Rcar;
        //归到中央
        angleToCam = angleToCam - rotateDir * m_hitNumYaw * params.cycangle;
        //angleGoalPos_base:目标车中心-目标装甲板-自身中心夹角
        const double angleGoalPos_base = getAngleToGimbal(angleToCam, Rcar, params.dist);
        //到目标装甲板距离
        const double disttoTargetArmor_sq =
            Rcar * Rcar + dist_sq + 2 * Rcar * params.dist * cos(angleGoalPos_base);
        const double disttoTargetArmor = sqrt(disttoTargetArmor_sq);

        // 计算投影到中央的角速度
        double angleGoalSpeed =
            params.linearspeed *
            sin(asin(params.dist / disttoTargetArmor * sin(angleToCam)) + M_PI_2);
        angleGoalSpeed -= velocity_projection_term;  // 使用预计算的速度投影项
        angleGoalSpeed /= disttoTargetArmor;

        // 计算最终位置
        const double angleGoalPos_final =
            angleGoalPos_base + atan2(params.PosX + params.SpdX * t, params.PosZ + params.SpdZ * t);

        x_ref_out(0, i) = angleGoalPos_final;
        x_ref_out(1, i) = angleGoalSpeed;
    }
}
/**
 * @brief 根据预测轨迹与参考轨迹在打击时刻的误差，决定是否开火。
 * @param predicted_trajectory MPC求解器优化后的状态轨迹 (solution.x)
 * @param reference_trajectory 生成的参考状态轨迹 (x_ref)
 * @param traj_params          包含物理参数的结构体，用于计算flytime
 * @param actiontime           固定的动作延迟 (如打蛋延迟)
 * @param dt                   MPC预测时域的时间步长 (DT_SOLVE)
 * @param horizon_length       预测时域的长度 (N_HORIZON)
 * @return                     如果误差小于阈值，返回true (可以开火)，否则返回false
 */
bool calculate_ShootFlag(const TinyMatrix& predicted_trajectory,
                         const TinyMatrix& reference_trajectory, const TrajectoryParams& params,
                         double actiontime, double dt, int horizon_length, bool isLargeArmor) {
    double timetohit = actiontime + params.flytime;
    if (timetohit > (horizon_length - 1) * dt || timetohit < 0) {
        std::cout
            << "shootflag err, targrt too far!  (timetohit > (horizon_length-1)*dt || timetohit<0)"
            << std::endl;
        return false;
    }
    // 2. 计算打击时间对应的最近时间步索引
    // 使用 std::round 来找到最近的整数索引
    int kPred = static_cast<int>(std::round(timetohit / dt));  //这个不插值
    // int kReal = static_cast<int>(floor(timetohit / dt));       //这个插值
    // 3. 约束索引在有效范围内 [0, horizon_length - 1]
    kPred = std::max(0, std::min(kPred, horizon_length - 1));
    // kReal = std::max(0, std::min(kReal, horizon_length - 1));
    // 4. 从预测轨迹和参考轨迹中获取最近索引处的位置
    // double alpha = (timetohit - kReal * dt) / dt;
    // const double predicted_pos = predicted_trajectory(0, kReal) * (1.0 - alpha) +
    //                              predicted_trajectory(0, kReal + 1) * alpha; //插值预测的跟踪结果
    const double predicted_pos = predicted_trajectory(0, kPred);
    const double reference_pos = reference_trajectory(0, kPred);  // 预测的目标（参考）轨迹

    // 5. 计算位置误差的绝对值
    const double position_error = std::abs(predicted_pos - reference_pos);

    // ======================== 动态阈值 ========================
    const int rotateDir = (params.rotateSpeed > 0) - (params.rotateSpeed < 0);
    double angleToCam = params.yawToCam + params.rotateSpeed * (timetohit);
    int m_hitNumYaw =
        std::abs(floor((rotateDir * angleToCam + (0.5 * params.cycangle)) / params.cycangle));
    angleToCam = angleToCam - rotateDir * m_hitNumYaw * params.cycangle;
    const double Rcar = (m_hitNumYaw % 2 == 1) ? params.RcarNext : params.Rcar;
    double angleGoalPos = getAngleToGimbal(angleToCam, Rcar, params.dist);
    //计算目标车中心-本车中心-击打目标的夹角
    double disttoTargetArmor =
        sqrt(Rcar * Rcar + params.dist * params.dist + 2 * Rcar * params.dist * cos(angleGoalPos));
    const double armorWidth = isLargeArmor ? 0.23 : 0.135;
    const double armorProjectLen = cos(angleGoalPos + angleToCam) * (armorWidth / 2.0);
    // 通过距离和目标倾斜角度计算击打阈值yaw，可视情况放缩
    const double shootYawThreshold = atan(armorProjectLen / disttoTargetArmor) * 1.0;

    // 6. 比较误差和阈值，返回决策
    return position_error < shootYawThreshold;
}
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
    Eigen::Map<const TinyMatrix> adynSim(ADYN_DATA_SIM, N_STATES, N_STATES);
    // std::cout << "adyn = " << std::endl << adyn << std::endl;
    Eigen::Map<const TinyMatrix> bdyn(BDYN_DATA, N_STATES, N_INPUTS);
    Eigen::Map<const TinyMatrix> bdynSim(BDYN_DATA_SIM, N_STATES, N_INPUTS);
    // std::cout << "bdyn = " << std::endl << bdyn << std::endl;
    Eigen::Map<const TinyVector> q_diag(Q_DATA, N_STATES);
    Eigen::Map<const TinyVector> r_diag(R_DATA, N_INPUTS);

    // 定义状态和输入的约束
    Eigen::Map<const TinyVector> x_min_vec(X_MIN_DATA, N_STATES);
    Eigen::Map<const TinyVector> x_max_vec(X_MAX_DATA, N_STATES);
    Eigen::Map<const TinyVector> u_min_vec(U_MIN_DATA, N_INPUTS);
    Eigen::Map<const TinyVector> u_max_vec(U_MAX_DATA, N_INPUTS);

    TinyMatrix x_min = x_min_vec.replicate(1, N_HORIZON);
    TinyMatrix x_max = x_max_vec.replicate(1, N_HORIZON);
    TinyMatrix u_min = u_min_vec.replicate(1, N_HORIZON - 1);
    TinyMatrix u_max = u_max_vec.replicate(1, N_HORIZON - 1);

    std::cout << "======== TinyMPC C++ Test Case ========" << std::endl;
    std::cout << "System: " << N_STATES << " states, " << N_INPUTS << " inputs." << std::endl;
    std::cout << "Horizon: " << N_HORIZON << " steps." << std::endl;
    std::cout << "---------------------------------------" << std::endl;

    // 目标参数（初始值）
    double Rcar = 0.2;
    double RcarNext = 0.3;
    double rotateSpeed = 6;
    double PosX = 0;  //来自滤波器
    double SpdX = 0;
    double PosZ = 1;  //来自滤波器
    double SpdZ = 0;
    double PosYaw = 0;  //来自滤波器
    double flyspeed = 20;
    double actiontime = 0.1;  //打蛋延迟
    try {
        auto solverPtr = std::make_unique<TinyMpcSolver>(
            adyn, bdyn, q_diag.asDiagonal(), r_diag.asDiagonal(), x_min, x_max, u_min, u_max,
            N_STATES, N_INPUTS, N_HORIZON, RHO_VALUE, true  // 启用详细日志
        );

        // 被控系统初始化
        // 定义被控系统的当前状态（例如，云台的当前角度和角速度）
        TinyVector x_current(N_STATES);
        x_current << 0.0, 0.0;  // 假设初始位置和速度都为0

        // 数据记录初始化
        // 创建并打开一个CSV文件用于记录仿真数据
        std::ofstream data_log("mpc_log.csv");
        if (data_log.is_open()) {
            // 【修改】写入新的CSV表头
            data_log << "time,pos_actual,velocity_actual,pos_ref,velocity_ref,control_input,solve_"
                        "time_ms,shoot_flag,iterations\n ";
        }
        for (int iter = 0; iter < 400; iter++) {
            std::cout << "\n[Iteration " << iter << "] Solving MPC problem..." << std::endl;
            // 定义参考轨迹
            // 创建并初始化轨迹参数
            //计时开始
            auto start_time = std::chrono::high_resolution_clock::now();
            TrajectoryParams traj_params(Rcar, RcarNext, rotateSpeed, PosX, PosZ, SpdX, SpdZ,
                                         PosYaw, flyspeed);
            // 生成位置参考轨迹
            TinyMatrix x_ref(N_STATES, N_HORIZON);
            generate_reference_trajectory(x_ref, traj_params, DT_SOLVE);  // 将参数包传递给函数
            // 此示例中输入参考轨迹为0
            TinyMatrix u_ref = TinyMatrix::Zero(N_INPUTS, N_HORIZON - 1);

            // ---- 4. 设置问题并求解 ----
            TinyVector x0 = x_current;
            solverPtr->setInitialState(x0);
            solverPtr->setStateReference(x_ref);
            // std::cout << "x_ref = " << x_ref << std::endl;

            int status = solverPtr->solve();

            // ---- 5. 获取结果 ----
            const TinySolution& solution = solverPtr->getSolution();

            double timetohit = actiontime + traj_params.flytime;
            // 从状态轨迹第一维插值计算timetohit时的位置
            bool shoot_flag = calculate_ShootFlag(solution.x, x_ref, traj_params, actiontime,
                                                  DT_SOLVE, N_HORIZON, 0);
            // 计时结束并计算耗时
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration =
                std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            double solve_time_ms = duration.count() / 1000.0;
            // if (solution.solved) {
            //     std::cout << "\nSUCCESS: Problem solved in " << solution.iter << " iterations."
            //               << std::endl;
            // } else {
            //     std::cerr << "\nFAILURE: Solver did not converge after " << solution.iter
            //               << " iterations." << std::endl;
            // }
            // Eigen::IOFormat fmt(4, 0, ", ", "\n", "[", "]");
            // std::cout << "\nInitial State:\n" << x0.transpose().format(fmt) << std::endl;
            // std::cout << "\nReference Trajectory (first 5 steps):\n"
            //           << x_ref.leftCols(5).format(fmt) << std::endl;
            // std::cout << "\nOptimal Control Input (first 5 steps):\n"
            //           << solution.u.leftCols(5).format(fmt) << std::endl;
            // std::cout << "\nResulting State Trajectory (first 5 steps):\n"
            //           << solution.x.leftCols(5).format(fmt) << std::endl;
            //目标模拟
            PosYaw += rotateSpeed * DT_SIM;
            PosX += SpdX * DT_SIM;
            PosZ += SpdZ * DT_SIM;
            // std::cout << "PosX =" << PosX << "   PosZ =" << PosZ << std::endl;
            if (PosYaw > M_PI / 2.0) {
                double swap = Rcar;
                Rcar = RcarNext;
                RcarNext = swap;
                PosYaw = PosYaw - (M_PI / 2.0);
            }
            //被控系统模拟
            TinyVector u0 = solution.u.col(0);
            // std::cout << "solution.u = " << solution.u << std::endl;
            // std::cout << "solution.x = " << solution.x << std::endl;
            // 使用系统模型更新被控系统的状态：x(k+1) = A*x(k) + B*u(k)
            x_current = adynSim * x0 + bdynSim * u0;
            // std::cout << "x0 = " << x0 << std::endl;
            //记录新的数据到CSV
            if (data_log.is_open()) {
                data_log << iter * DT_SIM << ","    // 时间戳
                         << x0(0) << ","            // 当前时刻的实际位置
                         << x0(1) << ","            // 当前时刻的实际V
                         << x_ref(0, 0) << ","      // 当前时刻的参考位置
                         << x_ref(1, 0) << ","      // 当前时刻的参考速度
                         << u0(0) << ","            // 计算出的控制输入
                         << solve_time_ms << ","    // 求解耗时(ms)
                         << shoot_flag << ","       // 击打标志
                         << solution.iter << "\n";  // 迭代次数
            }
        }
        if (data_log.is_open()) {
            data_log.close();
            std::cout << "\nSimulation data logged to mpc_log.csv" << std::endl;
        }
        return 0;
    } catch (const std::invalid_argument& e) {
        std::cerr << "\nERROR: An exception occurred during setup: " << e.what() << std::endl;
        return 1;
    }
}