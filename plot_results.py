# plot_results.py

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.style as style

def visualize_mpc_results(csv_file_path='bin/gimbal_controller_log.csv'):
    """
    读取并可视化Pitch和Yaw双轴MPC仿真的结果。
    采用3x2的网格布局来清晰展示各项性能指标。

    Args:
        csv_file_path (str): 包含仿真数据的CSV文件的路径。
    """
    # --- 1. 读取数据 ---
    try:
        # 新的C++代码会生成带表头的CSV，直接读取即可
        data = pd.read_csv(csv_file_path)
        # 清理列名中可能存在的前后空格
        data.columns = data.columns.str.strip()
    except FileNotFoundError:
        print(f"错误: 文件 '{csv_file_path}' 未找到。")
        print("请确保您已经运行了修改后的C++仿真程序，并且CSV文件路径正确。")
        return
    except Exception as e:
        print(f"读取CSV文件时出错: {e}")
        return

    # 检查数据是否为空
    if data.empty:
        print(f"警告: 文件 '{csv_file_path}' 为空或无法解析，无法生成图像。")
        return

    # --- 2. 创建图像 ---
    # 使用更美观的绘图风格
    style.use('seaborn-v0_8-whitegrid')
    
    # 创建一个3x2的子图网格，它们共享X轴以实现同步缩放
    fig, axes = plt.subplots(3, 2, figsize=(16, 12), sharex=True)
    fig.suptitle('Dual-Axis Gimbal MPC Simulation Analysis (Pitch & Yaw)', fontsize=20, y=0.98)

    # --- 3. 绘制每个子图 ---

    # --- 行 1: 位置和速度跟踪 ---
    
    # 子图 1: 位置 (Position Tracking)
    ax1 = axes[0, 0]
    ax1.plot(data['time'], data['pos_actual_yaw'], label='Yaw Actual', color='royalblue', linewidth=2)
    ax1.plot(data['time'], data['pos_goal_yaw'], label='Yaw Reference', color='skyblue', linestyle='--', linewidth=2)
    ax1.plot(data['time'], data['pos_actual_pitch'], label='Pitch Actual', color='firebrick', linewidth=2)
    ax1.plot(data['time'], data['pos_goal_pitch'], label='Pitch Reference', color='salmon', linestyle='--', linewidth=2)
    ax1.set_title('Position Tracking', fontsize=14)
    ax1.set_ylabel('Position (rad)')
    ax1.legend()
    
    # 子图 2: 速度 (Velocity Tracking)
    ax2 = axes[0, 1]
    ax2.plot(data['time'], data['vel_actual_yaw'], label='Yaw Actual', color='royalblue', linewidth=2)
    ax2.plot(data['time'], data['vel_goal_yaw'], label='Yaw Reference', color='skyblue', linestyle='--', linewidth=2)
    ax2.plot(data['time'], data['vel_actual_pitch'], label='Pitch Actual', color='firebrick', linewidth=2)
    ax2.plot(data['time'], data['vel_goal_pitch'], label='Pitch Reference', color='salmon', linestyle='--', linewidth=2)
    ax2.set_title('Velocity Tracking', fontsize=14)
    ax2.set_ylabel('Velocity (rad/s)')
    ax2.legend()

    # --- 行 2: 控制输入和求解器迭代 ---

    # 子图 3: 控制输入 (Control Input)
    ax3 = axes[1, 0]
    ax3.plot(data['time'], data['accel_control_yaw'], label='Yaw Acceleration', color='royalblue', linewidth=2)
    ax3.plot(data['time'], data['accel_control_pitch'], label='Pitch Acceleration', color='firebrick', linewidth=2)
    ax3.set_title('Control Input (Acceleration)', fontsize=14)
    ax3.set_ylabel('Acceleration (rad/s^2)')
    ax3.legend()

    # 子图 4: 求解器迭代次数 (Solver Iterations)
    ax4 = axes[1, 1]
    # 使用阶梯图(step plot)更适合表示离散的迭代次数
    ax4.step(data['time'], data['iterations_yaw'], where='post', label='Yaw Iterations', color='royalblue', linewidth=2)
    ax4.step(data['time'], data['iterations_pitch'], where='post', label='Pitch Iterations', color='firebrick', linewidth=2)
    ax4.set_title('Solver Iterations per Step', fontsize=14)
    ax4.set_ylabel('Iterations')
    ax4.legend()

    # --- 行 3: 全局性能和决策 ---

    # 子图 5: 求解耗时 (Solver Time)
    ax5 = axes[2, 0]
    ax5.plot(data['time'], data['solve_time_ms'], label='Total Solve Time', color='purple', linewidth=2)
    ax5.set_title('Total MPC Solve Time', fontsize=14)
    ax5.set_ylabel('Time (ms)')
    ax5.legend()
    ax5.set_xlabel('Time (s)')

    # 子图 6: 开火决策 (Shoot Flag)
    ax6 = axes[2, 1]
    # 使用阶梯图表示0/1的跳变信号
    ax6.step(data['time'], data['shoot_flag'], where='post', label='Shoot Flag', color='forestgreen', linewidth=2)
    ax6.set_title('Shoot Decision Output', fontsize=14)
    ax6.set_ylabel('Flag (1=Shoot, 0=Hold)')
    ax6.set_ylim(-0.1, 1.1)  # 设置Y轴范围以清晰显示0和1
    ax6.legend()
    ax6.set_xlabel('Time (s)')

    # --- 4. 显示图像 ---
    plt.tight_layout(rect=[0, 0, 1, 0.96]) # 调整布局以防止总标题重叠
    plt.show()

if __name__ == '__main__':
    # 尝试在几个可能的路径中寻找日志文件
    possible_paths = ['gimbal_controller_log.csv', 'bin/gimbal_controller_log.csv', 'build/gimbal_controller_log.csv']
    found_path = None
    for path in possible_paths:
        try:
            # 简单检查文件是否存在且非空
            import os
            if os.path.exists(path) and os.path.getsize(path) > 0:
                found_path = path
                print(f"找到日志文件: '{found_path}'")
                break
        except (FileNotFoundError, OSError):
            continue

    if found_path:
        visualize_mpc_results(found_path)
    else:
        print("错误: 在常见路径中未找到 'gimbal_controller_log.csv'。")
        print("请手动指定文件路径或将脚本放在正确的位置。")