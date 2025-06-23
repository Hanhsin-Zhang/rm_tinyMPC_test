// Constants.h

#pragma once

namespace hitcrt {

// 默认求解器设置
constexpr double DEFAULT_ABS_PRI_TOL = 1e-03;
constexpr double DEFAULT_ABS_DUA_TOL = 1e-03;
constexpr int DEFAULT_MAX_ITER = 10;//速度要求
constexpr int DEFAULT_CHECK_TERMINATION = 1;
constexpr bool DEFAULT_EN_STATE_BOUND = true;
constexpr bool DEFAULT_EN_INPUT_BOUND = true;

}  // namespace hitcrt