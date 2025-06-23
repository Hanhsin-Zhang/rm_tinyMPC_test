// TinyMpcSolver.cpp (已修复)

#include "TinyMpcSolver.h"

#include <iostream>
#include <stdexcept>
#include <vector>

namespace hitcrt {

// ... (构造函数和 checkDimension 函数保持不变) ...

/**

@brief 构造函数，初始化MPC问题

@author <Your Name>

@mail your.email@example.com
*/
TinyMpcSolver::TinyMpcSolver(const TinyMatrix& adyn, const TinyMatrix& bdyn, const TinyMatrix& Q,
                             const TinyMatrix& R, const TinyMatrix& xMin, const TinyMatrix& xMax,
                             const TinyMatrix& uMin, const TinyMatrix& uMax, int nx, int nu, int N,
                             TinyType rho, bool verbose)
    : m_eigenFormat(4, 0, ", ", "\n", "[", "]") {
    // 维度检查
    try {
        checkDimension("State transition matrix (A)", "rows", adyn.rows(), nx);
        checkDimension("State transition matrix (A)", "columns", adyn.cols(), nx);
        checkDimension("Input matrix (B)", "rows", bdyn.rows(), nx);
        checkDimension("Input matrix (B)", "columns", bdyn.cols(), nu);
        checkDimension("State stage cost (Q)", "rows", Q.rows(), nx);
        checkDimension("State stage cost (Q)", "columns", Q.cols(), nx);
        checkDimension("Input stage cost (R)", "rows", R.rows(), nu);
        checkDimension("Input stage cost (R)", "columns", R.cols(), nu);
        checkDimension("Lower state bounds (xMin)", "rows", xMin.rows(), nx);
        checkDimension("Lower state bounds (xMin)", "cols", xMin.cols(), N);
        checkDimension("Upper state bounds (xMax)", "rows", xMax.rows(), nx);
        checkDimension("Upper state bounds (xMax)", "cols", xMax.cols(), N);
        checkDimension("Lower input bounds (uMin)", "rows", uMin.rows(), nu);
        checkDimension("Lower input bounds (uMin)", "cols", uMin.cols(), N - 1);
        checkDimension("Upper input bounds (uMax)", "rows", uMax.rows(), nu);
        checkDimension("Upper input bounds (uMax)", "cols", uMax.cols(), N - 1);
    } catch (const std::invalid_argument& e) {
        std::cerr << "Error during TinyMpcSolver initialization: " << e.what() << std::endl;
        throw;
    }

    // 初始化设置
    setDefaultSettings();

    // 初始化工作区
    m_work.nx = nx;
    m_work.nu = nu;
    m_work.N = N;

    m_work.x = TinyMatrix::Zero(nx, N);
    m_work.u = TinyMatrix::Zero(nu, N - 1);
    m_work.q = TinyMatrix::Zero(nx, N);
    m_work.r = TinyMatrix::Zero(nu, N - 1);
    m_work.p = TinyMatrix::Zero(nx, N);
    m_work.d = TinyMatrix::Zero(nu, N - 1);
    m_work.v = TinyMatrix::Zero(nx, N);
    m_work.vnew = TinyMatrix::Zero(nx, N);
    m_work.z = TinyMatrix::Zero(nu, N - 1);
    m_work.znew = TinyMatrix::Zero(nu, N - 1);
    m_work.g = TinyMatrix::Zero(nx, N);
    m_work.y = TinyMatrix::Zero(nu, N - 1);

    m_work.Q = (Q + rho * TinyMatrix::Identity(nx, nx)).diagonal();
    m_work.R = (R + rho * TinyMatrix::Identity(nu, nu)).diagonal();
    m_work.adyn = adyn;
    m_work.bdyn = bdyn;

    m_work.xMin = xMin;
    m_work.xMax = xMax;
    m_work.uMin = uMin;
    m_work.uMax = uMax;

    m_work.xRef = TinyMatrix::Zero(nx, N);
    m_work.uRef = TinyMatrix::Zero(nu, N - 1);

    m_work.primalResidualState = 0;
    m_work.primalResidualInput = 0;
    m_work.dualResidualState = 0;
    m_work.dualResidualInput = 0;
    m_work.status = 0;  // 0 for unsolved
    m_work.iter = 0;

    // 预计算缓存
    precomputeCache(adyn, bdyn, m_work.Q.asDiagonal(), m_work.R.asDiagonal(), rho, verbose);
}

// ... (updateSettings, setInitialState, etc. 保持不变) ...
/**

@brief 更新求解器设置

@author <Your Name>

@mail your.email@example.com
*/
void TinyMpcSolver::updateSettings(TinyType absPriTol, TinyType absDuaTol, int maxIter,
                                   int checkTermination, bool enStateBound, bool enInputBound) {
    m_settings.absPriTol = absPriTol;
    m_settings.absDuaTol = absDuaTol;
    m_settings.maxIter = maxIter;
    m_settings.checkTermination = checkTermination;
    m_settings.enStateBound = enStateBound;
    m_settings.enInputBound = enInputBound;
}

/**

@brief 设置初始状态

@author <Your Name>

@mail your.email@example.com
*/
void TinyMpcSolver::setInitialState(const TinyVector& x0) {
    if (x0.rows() != m_work.nx) {
        throw std::invalid_argument("Initial state x0 has incorrect dimension.");
    }
    m_work.x.col(0) = x0;
}

/**

@brief 设置状态参考轨迹

@author <Your Name>

@mail your.email@example.com
*/
void TinyMpcSolver::setStateReference(const TinyMatrix& xRef) {
    checkDimension("State reference trajectory (xRef)", "rows", xRef.rows(), m_work.nx);
    checkDimension("State reference trajectory (xRef)", "columns", xRef.cols(), m_work.N);
    m_work.xRef = xRef;
}

/**

@brief 设置输入参考轨迹

@author <Your Name>

@mail your.email@example.com
*/
void TinyMpcSolver::setInputReference(const TinyMatrix& uRef) {
    checkDimension("Input reference trajectory (uRef)", "rows", uRef.rows(), m_work.nu);
    checkDimension("Input reference trajectory (uRef)", "columns", uRef.cols(), m_work.N - 1);
    m_work.uRef = uRef;
}

/**

@brief 执行MPC求解 (已修复和优化)

@author <Your Name>

@mail your.email@example.com
*/
int TinyMpcSolver::solve() {
    m_solution.solved = false;
    m_solution.iter = 0;
    m_work.status = 11;  // Unsolved
    m_work.iter = 0;

    // ======================= 优化点: 算法启动 =======================
    // 在主循环前，先进行一次代价更新和后向传播，为第一次前向传播提供有意义的梯度
    // 这有助于提高收敛速度和稳定性
    updateLinearCost();
    backwardPassGrad();
    // =============================================================

    for (int i = 0; i < m_settings.maxIter; ++i) {
        forwardPass();
        updateSlack();
        updateDual();
        updateLinearCost();  // 为下一次 backwardPass 准备代价

        m_work.iter++;

        if (checkTerminationCondition()) {
            m_work.status = 1;  // Solved
            m_solution.iter = m_work.iter;
            m_solution.solved = true;
            m_solution.x = m_work.vnew;  // 使用经过约束投影后的值作为解
            m_solution.u = m_work.znew;
            return 0;  // Success
        }

        // 为下一次迭代准备：v 和 z 在 checkTerminationCondition 中用于计算残差
        // backwardPassGrad 之前不需要更新，因为它不依赖 v 和 z
        m_work.v = m_work.vnew;
        m_work.z = m_work.znew;

        // 为下一次 forwardPass 准备梯度
        backwardPassGrad();
    }

    // 达到最大迭代次数，求解失败
    m_solution.iter = m_work.iter;
    m_solution.solved = false;
    m_solution.x = m_work.vnew;  // 即使失败，也返回当前最优解
    m_solution.u = m_work.znew;
    return 1;  // Failure
}

// ... (getSolution, checkDimension, setDefaultSettings, precomputeCache 保持不变) ...
const TinySolution& TinyMpcSolver::getSolution() const { return m_solution; }
void TinyMpcSolver::checkDimension(const std::string& matrixName, const std::string& rowsOrCols,
                                   int actual, int expected) {
    if (actual != expected) {
        throw std::invalid_argument(matrixName + " has " + std::to_string(actual) + " " +
                                    rowsOrCols + ". Expected " + std::to_string(expected) + ".");
    }
}
void TinyMpcSolver::setDefaultSettings() {
    m_settings.absPriTol = DEFAULT_ABS_PRI_TOL;
    m_settings.absDuaTol = DEFAULT_ABS_DUA_TOL;
    m_settings.maxIter = DEFAULT_MAX_ITER;
    m_settings.checkTermination = DEFAULT_CHECK_TERMINATION;
    m_settings.enStateBound = DEFAULT_EN_STATE_BOUND;
    m_settings.enInputBound = DEFAULT_EN_INPUT_BOUND;
}
void TinyMpcSolver::precomputeCache(const TinyMatrix& adyn, const TinyMatrix& bdyn,
                                    const TinyMatrix& Q, const TinyMatrix& R, TinyType rho,
                                    bool verbose) {
    TinyMatrix Q1 = Q + rho * TinyMatrix::Identity(m_work.nx, m_work.nx);
    TinyMatrix R1 = R + rho * TinyMatrix::Identity(m_work.nu, m_work.nu);

    if (verbose) {
        std::cout << "A = " << adyn.format(m_eigenFormat) << std::endl;
        std::cout << "B = " << bdyn.format(m_eigenFormat) << std::endl;
        std::cout << "Q_rho = " << Q1.format(m_eigenFormat) << std::endl;
        std::cout << "R_rho = " << R1.format(m_eigenFormat) << std::endl;
        std::cout << "rho = " << rho << std::endl;
    }

    TinyMatrix ktp1 = TinyMatrix::Zero(m_work.nu, m_work.nx);
    TinyMatrix ptp1 = rho * TinyMatrix::Ones(m_work.nx, 1).asDiagonal();
    TinyMatrix kinf = TinyMatrix::Zero(m_work.nu, m_work.nx);
    TinyMatrix pinf = TinyMatrix::Zero(m_work.nx, m_work.nx);

    for (int i = 0; i < 1000; ++i) {
        kinf = (R1 + bdyn.transpose() * ptp1 * bdyn).inverse() * bdyn.transpose() * ptp1 * adyn;
        pinf = Q1 + adyn.transpose() * ptp1 * (adyn - bdyn * kinf);
        if ((kinf - ktp1).cwiseAbs().maxCoeff() < 1e-5) {
            if (verbose) {
                std::cout << "Kinf converged after " << i + 1 << " iterations" << std::endl;
            }
            break;
        }
        ktp1 = kinf;
        ptp1 = pinf;
    }

    m_cache.rho = rho;
    m_cache.kInf = kinf;
    m_cache.pInf = pinf;
    m_cache.quuInv = (R1 + bdyn.transpose() * pinf * bdyn).inverse();
    m_cache.amBKt = (adyn - bdyn * kinf).transpose();

    if (verbose) {
        std::cout << "Kinf = " << m_cache.kInf.format(m_eigenFormat) << std::endl;
        std::cout << "Pinf = " << m_cache.pInf.format(m_eigenFormat) << std::endl;
        std::cout << "Quu_inv = " << m_cache.quuInv.format(m_eigenFormat) << std::endl;
        std::cout << "AmBKt = " << m_cache.amBKt.format(m_eigenFormat) << std::endl;
        std::cout << "\nPrecomputation finished!\n" << std::endl;
    }
}

// ... (backwardPassGrad, forwardPass, updateSlack, updateDual 保持不变) ...
void TinyMpcSolver::backwardPassGrad() {
    for (int i = m_work.N - 2; i >= 0; --i) {
        m_work.d.col(i).noalias() =
            m_cache.quuInv * (m_work.bdyn.transpose() * m_work.p.col(i + 1) + m_work.r.col(i));
        m_work.p.col(i).noalias() = m_work.q.col(i) +
                                    m_cache.amBKt.lazyProduct(m_work.p.col(i + 1)) -
                                    (m_cache.kInf.transpose()).lazyProduct(m_work.r.col(i));
    }
}
void TinyMpcSolver::forwardPass() {
    for (int i = 0; i < m_work.N - 1; ++i) {
        m_work.u.col(i).noalias() = -m_cache.kInf.lazyProduct(m_work.x.col(i)) - m_work.d.col(i);
        m_work.x.col(i + 1).noalias() =
            m_work.adyn.lazyProduct(m_work.x.col(i)) + m_work.bdyn.lazyProduct(m_work.u.col(i));
    }
}
void TinyMpcSolver::updateSlack() {
    m_work.znew = m_work.u + m_work.y;
    m_work.vnew = m_work.x + m_work.g;

    if (m_settings.enInputBound) {
        m_work.znew = m_work.uMax.cwiseMin(m_work.uMin.cwiseMax(m_work.znew));
    }

    if (m_settings.enStateBound) {
        m_work.vnew = m_work.xMax.cwiseMin(m_work.xMin.cwiseMax(m_work.vnew));
    }
}
void TinyMpcSolver::updateDual() {
    m_work.y += m_work.u - m_work.znew;
    m_work.g += m_work.x - m_work.vnew;
}

/**

@brief ADMM算法：更新线性代价项（q和r） (已修复)

@author <Your Name>

@mail your.email@example.com
*/
void TinyMpcSolver::updateLinearCost() {
    m_work.r = -(m_work.uRef.array().colwise() * m_work.R.array());
    m_work.r.noalias() -= m_cache.rho * (m_work.znew - m_work.y);

    m_work.q = -(m_work.xRef.array().colwise() * m_work.Q.array());
    m_work.q.noalias() -= m_cache.rho * (m_work.vnew - m_work.g);

    // ============== Bug 1 修复点 ==============
    // 修复终端代价梯度的计算
    // 原错误代码: m_work.p.col(m_work.N - 1) = -(m_work.xRef.col(m_work.N -
    // 1).transpose().lazyProduct(m_cache.pInf)); 错误原因: 维度不匹配，(1,nx)*(nx,nx) -> (1,nx)
    // 行向量，但需要 (nx,1) 列向量 正确做法: (nx,nx)*(nx,1) -> (nx,1) 列向量
    m_work.p.col(m_work.N - 1) = -m_cache.pInf.lazyProduct(m_work.xRef.col(m_work.N - 1));
    // ==========================================
    m_work.p.col(m_work.N - 1).noalias() -=
        m_cache.rho * (m_work.vnew.col(m_work.N - 1) - m_work.g.col(m_work.N - 1));
}

// ... (checkTerminationCondition 保持不变) ...
bool TinyMpcSolver::checkTerminationCondition() {
    if (m_work.iter % m_settings.checkTermination == 0) {
        m_work.primalResidualState = (m_work.x - m_work.vnew).cwiseAbs().maxCoeff();
        m_work.dualResidualState = (m_work.v - m_work.vnew).cwiseAbs().maxCoeff() * m_cache.rho;
        m_work.primalResidualInput = (m_work.u - m_work.znew).cwiseAbs().maxCoeff();
        m_work.dualResidualInput = (m_work.z - m_work.znew).cwiseAbs().maxCoeff() * m_cache.rho;

        if (m_work.primalResidualState < m_settings.absPriTol &&
            m_work.primalResidualInput < m_settings.absPriTol &&
            m_work.dualResidualState < m_settings.absDuaTol &&
            m_work.dualResidualInput < m_settings.absDuaTol) {
            return true;
        }
    }
    return false;
}

}  // namespace hitcrt