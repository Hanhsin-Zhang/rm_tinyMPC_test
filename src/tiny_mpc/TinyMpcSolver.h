// TinyMpcSolver.h

#pragma once

#include <memory>
#include <string>

#include "Constants.h"
#include "Types.h"

namespace hitcrt {

/**
 * @class TinyMpcSolver
 * @brief 一个基于ADMM算法的微型模型预测控制器（MPC）求解器
 * @author <Your Name>
 * @mail <your.email@example.com>
 */
class TinyMpcSolver {
public:
    /**
     * @brief 构造函数，初始化MPC问题
     * @param adyn 状态转移矩阵 A [nx, nx]
     * @param bdyn 输入矩阵 B [nx, nu]
     * @param Q 状态权重矩阵（对角线） [nx, nx]
     * @param R 输入权重矩阵（对角线） [nu, nu]
     * @param xMin 状态下界 [nx, N]
     * @param xMax 状态上界 [nx, N]
     * @param uMin 输入下界 [nu, N-1]
     * @param uMax 输入上界 [nu, N-1]
     * @param nx 状态维度
     * @param nu 输入维度
     * @param N 时间窗长度
     * @param rho ADMM惩罚参数
     * @param verbose 是否打印详细信息
     * @throw std::invalid_argument 如果矩阵维度不匹配
     */
    TinyMpcSolver(const TinyMatrix& adyn, const TinyMatrix& bdyn, const TinyMatrix& Q,
                  const TinyMatrix& R, const TinyMatrix& xMin, const TinyMatrix& xMax,
                  const TinyMatrix& uMin, const TinyMatrix& uMax, int nx, int nu, int N,
                  TinyType rho, bool verbose = false);

    /**
     * @brief 析构函数
     */
    ~TinyMpcSolver() = default;

    // 删除拷贝构造和赋值操作，确保对象的唯一所有权
    TinyMpcSolver(const TinyMpcSolver&) = delete;
    TinyMpcSolver& operator=(const TinyMpcSolver&) = delete;

    /**
     * @brief 更新求解器设置
     * @param absPriTol 原始残差容忍度
     * @param absDuaTol 对偶残差容忍度
     * @param maxIter 最大迭代次数
     * @param checkTermination 检查终止条件的频率
     * @param enStateBound 是否启用状态约束
     * @param enInputBound 是否启用输入约束
     * @return void
     */
    void updateSettings(TinyType absPriTol, TinyType absDuaTol, int maxIter,
                        int checkTermination, bool enStateBound, bool enInputBound);

    /**
     * @brief 设置初始状态
     * @param x0 初始状态向量 [nx, 1]
     * @return void
     */
    void setInitialState(const TinyVector& x0);

    /**
     * @brief 设置状态参考轨迹
     * @param xRef 状态参考轨迹 [nx, N]
     * @return void
     */
    void setStateReference(const TinyMatrix& xRef);

    /**
     * @brief 设置输入参考轨迹
     * @param uRef 输入参考轨迹 [nu, N-1]
     * @return void
     */
    void setInputReference(const TinyMatrix& uRef);

    /**
     * @brief 执行MPC求解
     * @return int 0表示成功，1表示失败（未收敛）
     */
    int solve();

    /**
     * @brief 获取求解结果
     * @return const TinySolution& 对求解结果的常量引用
     */
    const TinySolution& getSolution() const;

private:
    /**
     * @brief 检查矩阵维度是否正确
     * @param matrixName 矩阵名称，用于打印错误信息
     * @param rowsOrCols 是检查行还是列
     * @param actual 实际维度
     * @param expected 期望维度
     * @return void
     * @throw std::invalid_argument 如果维度不匹配
     */
    void checkDimension(const std::string& matrixName, const std::string& rowsOrCols,
                        int actual, int expected);

    /**
     * @brief 设置默认求解器参数
     * @return void
     */
    void setDefaultSettings();

    /**
     * @brief 预计算Riccati相关的缓存矩阵
     * @param adyn 状态转移矩阵 A
     * @param bdyn 输入矩阵 B
     * @param Q 状态权重向量
     * @param R 输入权重向量
     * @param rho ADMM惩罚参数
     * @param verbose 是否打印详细信息
     * @return void
     */
    void precomputeCache(const TinyMatrix& adyn, const TinyMatrix& bdyn,
                         const TinyMatrix& Q, const TinyMatrix& R, TinyType rho,
                         bool verbose);

    /**
     * @brief ADMM算法：更新原始变量（状态x和输入u）
     * @return void
     */
    void updatePrimal();

    /**
     * @brief ADMM算法：Riccati后向传播，计算梯度
     * @return void
     */
    void backwardPassGrad();

    /**
     * @brief ADMM算法：LQR前向传播，计算轨迹
     * @return void
     */
    void forwardPass();

    /**
     * @brief ADMM算法：更新辅助变量（z和v）
     * @return void
     */
    void updateSlack();

    /**
     * @brief ADMM算法：更新对偶变量（y和g）
     * @return void
     */
    void updateDual();

    /**
     * @brief ADMM算法：更新线性代价项（q和r）
     * @return void
     */
    void updateLinearCost();

    /**
     * @brief 检查终止条件
     * @return bool 如果满足终止条件则返回true
     */
    bool checkTerminationCondition();

    TinySolution m_solution;
    TinySettings m_settings;
    TinyCache m_cache;
    TinyWorkspace m_work;
    Eigen::IOFormat m_eigenFormat;
};

}  // namespace hitcrt