# plot_results.py

import pandas as pd
import matplotlib.pyplot as plt

def visualize_mpc_results(csv_file_path='bin/mpc_log.csv'):
    """
    读取并可视化MPC仿真的结果，包含求解器性能分析。

    Args:
        csv_file_path (str): 包含仿真数据的CSV文件的路径。
    """
    # --- 1. 读取数据 ---
    try:
        # 显式指定列名，以防CSV为空或格式不正确
        col_names = ['time', 'pos_actual', 'velocity_actual','pos_ref','velocity_ref', 'control_input', 'solve_time_ms','shoot_flag' ,'iterations']
        data = pd.read_csv(csv_file_path, names=col_names, header=0)
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
    # 创建一个包含4个子图的Figure，它们共享X轴
    fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, figsize=(12, 15), sharex=True)
    fig.suptitle('MPC Simulation and Solver Performance', fontsize=18, y=0.99)

    # --- 3. 绘制每个子图 ---

    # 子图1: 位置 (Position)
    ax1.plot(data['time'], data['pos_actual'], label='Actual Position', color='b', linewidth=2)
    ax1.plot(data['time'], data['pos_ref'], label='Reference Position', color='r', linestyle='--', linewidth=2)
    ax1.set_ylabel('Position (rad)')
    ax1.set_title('Position Tracking')
    ax1.legend()
    ax1.grid(True, linestyle=':', alpha=0.6)

    # 子图2: 控制输入 (Control Input)
    ax2.plot(data['time'], data['control_input'], label='Control Input (u)', color='g', linewidth=2)
    ax2.set_ylabel('Input (rad/s^2)')
    ax2.set_title('Control Input Over Time')
    ax2.legend()
    ax2.grid(True, linestyle=':', alpha=0.6)

    # 子图3: 【新增】单步求解耗时 (Solver Time)
    ax3.plot(data['time'], data['solve_time_ms'], label='Solver Time per Step', color='purple', linewidth=2)
    ax3.set_ylabel('Time (ms)')
    ax3.set_title('MPC Solver Time')
    ax3.legend()
    ax3.grid(True, linestyle=':', alpha=0.6)

    # # 子图4: 【新增】求解迭代次数 (Solver Iterations)

    # ax4.plot(data['time'], data['velocity_actual'], label='Actual Velocity', color='orange', linewidth=2)
    # ax4.plot(data['time'], data['velocity_ref'], label='Reference Velocity', color='green', linestyle='--',linewidth=2)
    # ax4.set_ylabel('Velocity (rad/s)')
    # ax4.set_title('Speed Tracking')
    # ax4.legend()
    # ax4.grid(True, linestyle=':', alpha=0.6)
    ax4.plot(data['time'], data['shoot_flag'], label='Shoot Flag', color='m', linewidth=2)
    ax4.set_title('Shoot Output')
    ax4.legend()
    ax4.grid(True, linestyle=':', alpha=0.6)
    # 为共享的X轴设置标签
    ax4.set_xlabel('Time (s)')

    # --- 4. 显示图像 ---
    plt.tight_layout(rect=[0, 0, 1, 0.97]) # 调整布局以防止总标题重叠
    plt.show()

if __name__ == '__main__':
    # 尝试在几个可能的路径中寻找日志文件
    possible_paths = ['mpc_log.csv', 'bin/mpc_log.csv', 'build/mpc_log.csv']
    found_path = None
    for path in possible_paths:
        try:
            with open(path) as f:
                found_path = path
                print(f"找到日志文件: '{found_path}'")
                break
        except FileNotFoundError:
            continue

    if found_path:
        visualize_mpc_results(found_path)
    else:
        print("错误: 在常见路径中未找到 'mpc_log.csv'。")
        print("请手动指定文件路径或将脚本放在正确的位置。")