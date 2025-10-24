# -*- coding: utf-8 -*-
"""
DFS出口寻找与路径跟踪系统
不内置迷宫内部结构，使用DFS自动寻找出口并跟踪路径
"""
import asyncio
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from datetime import datetime
import threading
import sys
from collections import deque


class Maze4x4:
    def __init__(self, cell_size_cm=70):
        """
        初始化4*4迷宫
        迷宫有5*5=25个格点（从(0,0)到(4,4)）
        墙壁用两个坐标点之间的线段表示
        
        参数:
            cell_size_cm: 每个格子的边长（单位：厘米），默认70cm
        """
        # 水平墙壁列表 [起点, 终点]，每个点用(x, y)表示
        self.horizontal_walls = []
        # 垂直墙壁列表
        self.vertical_walls = []
        
        # 真实世界尺度
        self.cell_size_cm = cell_size_cm  # 每个格子边长（厘米）
        self.cell_size_m = cell_size_cm / 100  # 每个格子边长（米）
        
        # 创建外围墙壁（不内置内部墙）
        self._create_boundary_walls()
    
    def _create_boundary_walls(self):
        """仅创建外围墙壁，不内置任何内部结构"""
        # 外围墙壁（上边界）
        for i in range(4):
            self.horizontal_walls.append([(i, 4), (i+1, 4)])
        
        # 外围墙壁（下边界）
        for i in range(4):
            self.horizontal_walls.append([(i, 0), (i+1, 0)])
        
        # 外围墙壁（左边界）
        for i in range(4):
            self.vertical_walls.append([(0, i), (0, i+1)])
        
        # 外围墙壁（右边界）
        for i in range(4):
            self.vertical_walls.append([(4, i), (4, i+1)])
        
        # 不内置任何内部墙壁，内部结构未知，由运行时探索或算法决定
    
    def add_horizontal_wall(self, x1, y, x2):
        """
        添加水平墙壁
        x1, x2: x坐标范围
        y: y坐标
        """
        self.horizontal_walls.append([(x1, y), (x2, y)])
    
    def add_vertical_wall(self, x, y1, y2):
        """
        添加垂直墙壁
        x: x坐标
        y1, y2: y坐标范围
        """
        self.vertical_walls.append([(x, y1), (x, y2)])
    
    def grid_to_cm(self, x, y):
        """将网格坐标转换为真实世界坐标（厘米）"""
        return (x * self.cell_size_cm, y * self.cell_size_cm)
    
    def grid_to_m(self, x, y):
        """将网格坐标转换为真实世界坐标（米）"""
        return (x * self.cell_size_m, y * self.cell_size_m)
    
    def cm_to_grid(self, x_cm, y_cm):
        """将真实世界坐标（厘米）转换为网格坐标"""
        return (x_cm / self.cell_size_cm, y_cm / self.cell_size_cm)
    
    def m_to_grid(self, x_m, y_m):
        """将真实世界坐标（米）转换为网格坐标"""
        return (x_m / self.cell_size_m, y_m / self.cell_size_m)
    
    def get_maze_size_cm(self):
        """获取迷宫的真实尺寸（厘米）"""
        return (4 * self.cell_size_cm, 4 * self.cell_size_cm)
    
    def get_maze_size_m(self):
        """获取迷宫的真实尺寸（米）"""
        return (4 * self.cell_size_m, 4 * self.cell_size_m)
    
    def has_wall_between(self, cell1, cell2):
        """
        检查两个相邻格子之间是否有墙
        
        参数:
            cell1: (x1, y1) 第一个格子坐标
            cell2: (x2, y2) 第二个格子坐标
        返回:
            True: 有墙，False: 无墙
        """
        x1, y1 = cell1
        x2, y2 = cell2
        
        # 检查是否相邻
        if abs(x1 - x2) + abs(y1 - y2) != 1:
            return True  # 不相邻视为有墙
        
        # 检查水平墙壁
        if y1 == y2:  # 水平相邻
            # 墙壁在两个格子之间的边界上
            if x2 > x1:  # cell2在右边
                # 检查垂直墙壁 x=x2, y在[y1, y1+1]
                for wall in self.vertical_walls:
                    wx1, wy1 = wall[0]
                    wx2, wy2 = wall[1]
                    # 墙壁是垂直的
                    if wx1 == wx2 == x2:
                        if min(wy1, wy2) <= y1 < max(wy1, wy2):
                            return True
            else:  # cell2在左边
                # 检查垂直墙壁 x=x1, y在[y1, y1+1]
                for wall in self.vertical_walls:
                    wx1, wy1 = wall[0]
                    wx2, wy2 = wall[1]
                    if wx1 == wx2 == x1:
                        if min(wy1, wy2) <= y1 < max(wy1, wy2):
                            return True
        
        # 检查垂直墙壁
        if x1 == x2:  # 垂直相邻
            if y2 > y1:  # cell2在上边
                # 检查水平墙壁 y=y2, x在[x1, x1+1]
                for wall in self.horizontal_walls:
                    wx1, wy1 = wall[0]
                    wx2, wy2 = wall[1]
                    if wy1 == wy2 == y2:
                        if min(wx1, wx2) <= x1 < max(wx1, wx2):
                            return True
            else:  # cell2在下边
                # 检查水平墙壁 y=y1, x在[x1, x1+1]
                for wall in self.horizontal_walls:
                    wx1, wy1 = wall[0]
                    wx2, wy2 = wall[1]
                    if wy1 == wy2 == y1:
                        if min(wx1, wx2) <= x1 < max(wx1, wx2):
                            return True
        
        return False
    
    def find_exit_path(self, start):
        """
        使用DFS在未知内部结构的4x4网格中寻找从起点到任意边界"出口"的路径。
        说明：不内置任何内部墙，默认内部可通行；若未来对接真实传感器，可在 has_wall_between 回调中加入判定。
        
        参数:
            start: (x, y) 起点格子坐标（0-3）
        返回:
            路径列表 [(x1, y1), (x2, y2), ...] 或 None（无可达出口）
        """
        if not (0 <= start[0] < 4 and 0 <= start[1] < 4):
            print(f"起点 {start} 超出范围！")
            return None

        def is_exit(cell):
            x, y = cell
            # 任一边界即为出口，但不等于起点
            on_border = (x == 0 or x == 3 or y == 0 or y == 3)
            return on_border and cell != start

        stack = [start]
        visited = set([start])
        parent = {start: None}

        while stack:
            current = stack.pop()
            if is_exit(current):
                # 回溯路径
                path = []
                node = current
                while node is not None:
                    path.append(node)
                    node = parent[node]
                path.reverse()
                return path

            x, y = current
            neighbors = [
                (x+1, y),
                (x-1, y),
                (x, y+1),
                (x, y-1)
            ]
            for nx, ny in neighbors:
                if not (0 <= nx < 4 and 0 <= ny < 4):
                    continue
                nxt = (nx, ny)
                if nxt in visited:
                    continue
                if self.has_wall_between(current, nxt):
                    continue
                visited.add(nxt)
                parent[nxt] = current
                stack.append(nxt)

        return None
    
    def draw(self, show_real_scale=True, path=None, start=None, end=None):
        """
        绘制迷宫
        
        参数:
            show_real_scale: 是否在标题中显示真实尺度信息
            path: 路径列表 [(x1, y1), (x2, y2), ...]，如果提供则绘制路径
            start: 起点坐标 (x, y)
            end: 终点坐标 (x, y)
        """
        fig, ax = plt.subplots(figsize=(8, 8))
        
        # 绘制所有水平墙壁
        for wall in self.horizontal_walls:
            x_coords = [wall[0][0], wall[1][0]]
            y_coords = [wall[0][1], wall[1][1]]
            ax.plot(x_coords, y_coords, 'b-', linewidth=2)
        
        # 绘制所有垂直墙壁
        for wall in self.vertical_walls:
            x_coords = [wall[0][0], wall[1][0]]
            y_coords = [wall[0][1], wall[1][1]]
            ax.plot(x_coords, y_coords, 'b-', linewidth=2)
        
        # 绘制路径（如果提供）
        if path and len(path) > 1:
            # 将格子坐标转换为格子中心点坐标
            path_x = [p[0] + 0.5 for p in path]
            path_y = [p[1] + 0.5 for p in path]
            
            # 绘制路径线
            ax.plot(path_x, path_y, 'r-', linewidth=2.5, label='路径', zorder=5)
            
            # 绘制路径点
            ax.plot(path_x, path_y, 'ro', markersize=8, zorder=6)
            
            # 标注路径点序号
            for i, (x, y) in enumerate(zip(path_x, path_y)):
                ax.text(x, y, str(i), fontsize=8, ha='center', va='center',
                       color='white', fontweight='bold', zorder=7)
        
        # 绘制起点和终点（如果提供）
        if start is not None:
            start_x, start_y = start[0] + 0.5, start[1] + 0.5
            ax.plot(start_x, start_y, 'go', markersize=15, label='起点', zorder=8)
            ax.text(start_x, start_y - 0.3, 'S', fontsize=12, ha='center', 
                   fontweight='bold', color='green')
        
        if end is not None:
            end_x, end_y = end[0] + 0.5, end[1] + 0.5
            ax.plot(end_x, end_y, 'mo', markersize=15, label='出口', zorder=8)
            ax.text(end_x, end_y - 0.3, 'E', fontsize=12, ha='center',
                   fontweight='bold', color='magenta')
        
        # 设置坐标轴
        ax.set_xlim(-0.5, 4.5)
        ax.set_ylim(-0.5, 4.5)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3, linestyle='--')
        
        # 设置刻度
        ax.set_xticks(range(5))
        ax.set_yticks(range(5))
        
        # 设置标题和标签
        if show_real_scale:
            total_size_cm = 4 * self.cell_size_cm
            total_size_m = total_size_cm / 100
            title = f'4×4 迷宫 (DFS出口搜索)\n(格子边长: {self.cell_size_cm}cm, 总尺寸: {total_size_cm}cm × {total_size_cm}cm = {total_size_m:.1f}m × {total_size_m:.1f}m)'
        else:
            title = '4×4 迷宫 (DFS出口搜索)'
        
        if path:
            title += f'\n路径长度: {len(path)} 步'
        
        ax.set_title(title, fontsize=14, fontweight='bold')
        ax.set_xlabel('X 坐标 (网格)', fontsize=12)
        ax.set_ylabel('Y 坐标 (网格)', fontsize=12)
        
        # 在图上添加比例尺说明
        if show_real_scale:
            ax.text(0.02, 0.98, f'1 格 = {self.cell_size_cm} cm = {self.cell_size_m} m', 
                   transform=ax.transAxes, fontsize=10,
                   verticalalignment='top',
                   bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        # 添加图例（如果有路径或起终点）
        if path or start or end:
            ax.legend(loc='upper right', fontsize=10)
        
        plt.tight_layout()
        plt.show()


class DFSPathFollower:
    """DFS出口寻找与路径跟踪控制器"""
    
    def __init__(self, maze, cell_size_cm=70):
        """
        初始化路径跟踪控制器
        
        参数:
            maze: Maze4x4实例
            cell_size_cm: 格子边长（厘米）
        """
        self.maze = maze
        self.cell_size_cm = cell_size_cm
        self.robot = None
        
        # 当前状态
        self.current_pos_cm = [0.0, 0.0]  # 当前位置（厘米）
        self.current_angle = 0.0  # 当前朝向角度（度）
        self.target_path = []  # 目标路径
        self.path_index = 0
        
        # 控制参数
        self.position_tolerance_cm = 8  # 位置到达容差
        self.angle_tolerance = 10  # 角度容差
        self.max_consecutive_turns = 5  # 最大连续转向次数
        self.consecutive_turn_count = 0  # 当前连续转向计数
        self.angle_correction_threshold = 5  # 角度修正阈值（度）
        
        # 里程计累积
        self.total_distance_moved = 0
        self.odometry_buffer = []  # 里程计数据缓冲
        
        # 轨迹记录
        self.trajectory_log = []
        self.is_running = False
        self.user_stop_requested = False  # 用户请求停止标志
        
        # 动态可视化
        self.fig = None
        self.ax = None
        self.robot_marker = None
        self.trajectory_line = None
        self.enable_visualization = False
        self.trajectory_history = []  # [(x, y), ...]
        self.update_lock = threading.Lock()
    
    async def connect_robot(self, device_addr=None):
        """连接机器人"""
        from car import RobotController, DEFAULT_DEVICE_ADDR
        if device_addr is None:
            device_addr = DEFAULT_DEVICE_ADDR
        
        print(f"正在连接机器人 {device_addr}...")
        self.robot = RobotController(
            device_addr=device_addr,
            on_data=self.handle_odometry_data  # 设置数据回调
        )
        
        await self.robot.connect()
        await self.robot.start_notify()
        print("✓ 机器人连接成功")
        
        # 执行严格的初始化流程
        await self.initialize_robot()
    
    async def initialize_robot(self):
        """严格的机器人初始化流程（首次连接时）"""
        print("\n开始机器人初始化流程...")
        
        # 1. 初始化里程计 (0009)
        print("  [1/4] 初始化里程计...")
        await self.robot.distance_ini_print()
        await asyncio.sleep(0.5)
        
        # 2. 如果雷达开着，先关闭雷达 (a525)
        print("  [2/4] 关闭雷达（如果开着）...")
        await self.robot.stop_scan()
        await asyncio.sleep(0.5)
        
        # 3. 设置 motor_is_stop = 1 (0010) - 初始状态
        print("  [3/4] 设置 motor_is_stop = 1（初始状态）...")
        await self.robot.is_controll_1()
        await asyncio.sleep(0.5)
        
        # 4. 确保小车停止
        print("  [4/4] 确保小车停止...")
        await self.robot.stop()
        await asyncio.sleep(0.3)
        
        print("✓ 机器人初始化完成")
    
    async def prepare_for_movement(self):
        """准备移动前的状态设置"""
        # 1. 设置 motor_is_stop = 0 (0011)
        await self.robot.is_controll_0()
        await asyncio.sleep(0.1)
        
        # 2. 打开里程计 (0006)
        await self.robot.distance_controll()
        await asyncio.sleep(0.1)
    
    async def finish_movement(self):
        """移动完成后的状态清理"""
        # 1. 停止小车
        await self.robot.stop()
        await asyncio.sleep(0.2)
        
        # 2. ⭐ 获取里程计差值数据 (0009) - 关键步骤！
        await self.robot.distance_ini_print()
        await asyncio.sleep(0.3)  # 等待数据返回
        
        # 3. 关闭里程计 (0006)
        await self.robot.distance_controll()
        await asyncio.sleep(0.1)
        
        # 4. 设置 motor_is_stop = 1 (0010)
        await self.robot.is_controll_1()
        await asyncio.sleep(0.1)
    
    async def cleanup_robot(self):
        """清理机器人状态"""
        print("\n清理机器人状态...")
        
        # 1. 停止小车
        print("  [1/3] 停止小车...")
        await self.robot.stop()
        await asyncio.sleep(0.3)
        
        # 2. 关闭里程计 (0006)
        print("  [2/3] 关闭里程计...")
        await self.robot.distance_controll()
        await asyncio.sleep(0.5)
        
        # 3. 设置 motor_is_stop = 1 (0010)
        print("  [3/3] 设置 motor_is_stop = 1...")
        await self.robot.is_controll_1()
        await asyncio.sleep(0.5)
        
        print("✓ 机器人状态清理完成")
    
    async def disconnect_robot(self):
        """断开机器人连接"""
        if self.robot:
            await self.cleanup_robot()
            await self.robot.disconnect()
            print("✓ 机器人断开连接")
    
    async def monitor_user_input(self):
        """监听用户输入，按 'q' 或 'Q' 停止"""
        print("\n[提示] 在任何时候按 'q' + 回车 可以停止小车")
        
        loop = asyncio.get_event_loop()
        
        while self.is_running and not self.user_stop_requested:
            try:
                user_input = await asyncio.wait_for(
                    loop.run_in_executor(None, sys.stdin.readline),
                    timeout=0.5
                )
                
                if user_input.strip().lower() == 'q':
                    print("\n⚠ 用户请求停止！正在停止小车...")
                    self.user_stop_requested = True
                    break
            except asyncio.TimeoutError:
                continue
            except Exception as e:
                pass
            
            await asyncio.sleep(0.1)
    
    def handle_odometry_data(self, data):
        """处理里程计数据回调"""
        self.odometry_buffer.append(data)
        
        # 如果是里程计位置数据
        if 'x' in data and 'y' in data:
            self.current_pos_cm[0] = data['x']
            self.current_pos_cm[1] = data['y']
            
            if 'theta' in data:
                self.current_angle = data['theta']
            
            print(f"  [里程计] 位置更新: ({self.current_pos_cm[0]:.1f}, {self.current_pos_cm[1]:.1f})cm, 角度: {self.current_angle:.1f}°")
            self.update_visualization()
        
        # 如果是距离增量数据
        elif 'distance' in data and 'angle' not in data:
            distance_delta = data.get('distance', 0)
            self.total_distance_moved += distance_delta
            
            # 根据当前角度更新位置
            rad = math.radians(self.current_angle)
            self.current_pos_cm[0] += distance_delta * math.cos(rad)
            self.current_pos_cm[1] += distance_delta * math.sin(rad)
            
            print(f"  [里程计] 移动 {distance_delta:.1f}cm, 总计: {self.total_distance_moved:.1f}cm")
            self.update_visualization()
    
    def set_start_position(self, grid_x, grid_y, angle=0):
        """设置起始位置"""
        self.current_pos_cm = [
            (grid_x + 0.5) * self.cell_size_cm,
            (grid_y + 0.5) * self.cell_size_cm
        ]
        self.current_angle = angle
        self.total_distance_moved = 0
        
        print(f"起始位置: 格子({grid_x}, {grid_y}) = ({self.current_pos_cm[0]:.1f}, {self.current_pos_cm[1]:.1f})cm, 角度={angle}°")
    
    def set_path(self, path):
        """设置目标路径"""
        self.target_path = path
        self.path_index = 0
        print(f"路径设置: {len(path)} 个目标点")
    
    def init_visualization(self):
        """初始化动态可视化"""
        plt.ion()  # 开启交互模式
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        
        # 绘制迷宫墙壁
        for wall in self.maze.horizontal_walls:
            self.ax.plot([wall[0][0], wall[1][0]], [wall[0][1], wall[1][1]], 
                        'b-', linewidth=2)
        for wall in self.maze.vertical_walls:
            self.ax.plot([wall[0][0], wall[1][0]], [wall[0][1], wall[1][1]], 
                        'b-', linewidth=2)
        
        # 绘制规划路径
        if self.target_path:
            path_x = [(p[0] + 0.5) for p in self.target_path]
            path_y = [(p[1] + 0.5) for p in self.target_path]
            self.ax.plot(path_x, path_y, 'g--', linewidth=2, label='DFS出口路径', alpha=0.7)
            self.ax.plot(path_x, path_y, 'go', markersize=10, alpha=0.5)
            
            # 标记起点和出口
            self.ax.plot(path_x[0], path_y[0], 'gs', markersize=15, label='起点')
            self.ax.plot(path_x[-1], path_y[-1], 'gD', markersize=15, label='出口')
        
        # 初始化机器人标记
        robot_x = self.current_pos_cm[0] / self.cell_size_cm
        robot_y = self.current_pos_cm[1] / self.cell_size_cm
        self.robot_marker = Circle((robot_x, robot_y), 0.15, color='red', 
                                   fill=True, zorder=10, label='小车')
        self.ax.add_patch(self.robot_marker)
        
        # 初始化轨迹线
        self.trajectory_line, = self.ax.plot([], [], 'r-', linewidth=2.5, 
                                             label='实际轨迹', alpha=0.8)
        
        # 设置坐标轴
        self.ax.set_xlim(-0.5, 4.5)
        self.ax.set_ylim(-0.5, 4.5)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X (格子)', fontsize=12)
        self.ax.set_ylabel('Y (格子)', fontsize=12)
        self.ax.set_title('DFS出口寻找 - 实时路径跟踪', fontsize=14, fontweight='bold')
        self.ax.legend(loc='upper right', fontsize=10)
        
        plt.tight_layout()
        plt.show(block=False)
        plt.pause(0.1)
        
        self.enable_visualization = True
        print("✓ 动态可视化已启动")
    
    def update_visualization(self):
        """更新可视化"""
        if not self.enable_visualization or self.fig is None:
            return
        
        with self.update_lock:
            # 更新机器人位置
            robot_x = self.current_pos_cm[0] / self.cell_size_cm
            robot_y = self.current_pos_cm[1] / self.cell_size_cm
            self.robot_marker.center = (robot_x, robot_y)
            
            # 更新轨迹
            self.trajectory_history.append((robot_x, robot_y))
            if len(self.trajectory_history) > 1:
                traj_x = [p[0] for p in self.trajectory_history]
                traj_y = [p[1] for p in self.trajectory_history]
                self.trajectory_line.set_data(traj_x, traj_y)
            
            # 更新标题显示当前状态
            if self.path_index < len(self.target_path):
                target_grid = self.target_path[self.path_index]
                self.ax.set_title(
                    f'DFS出口寻找 - 目标点 {self.path_index}/{len(self.target_path)-1}: {target_grid}\n'
                    f'当前位置: ({self.current_pos_cm[0]:.1f}, {self.current_pos_cm[1]:.1f})cm, '
                    f'角度: {self.current_angle:.0f}°',
                    fontsize=12, fontweight='bold'
                )
            else:
                self.ax.set_title('DFS出口寻找 - 已到达出口！', fontsize=14, fontweight='bold', color='green')
            
            # 刷新图表
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            plt.pause(0.01)
    
    def close_visualization(self):
        """关闭可视化"""
        if self.enable_visualization and self.fig is not None:
            plt.ioff()
            self.enable_visualization = False
            print("✓ 动态可视化已关闭")
    
    def get_current_target(self):
        """获取当前目标点（厘米坐标）"""
        if self.path_index >= len(self.target_path):
            return None
        
        grid_x, grid_y = self.target_path[self.path_index]
        return [
            (grid_x + 0.5) * self.cell_size_cm,
            (grid_y + 0.5) * self.cell_size_cm
        ]
    
    def calculate_control_command(self, target_cm):
        """
        计算控制指令
        
        返回:
            ('forward'/'left'/'right'/'stop', 角度差, 距离)
        """
        # 计算距离和角度
        dx = target_cm[0] - self.current_pos_cm[0]
        dy = target_cm[1] - self.current_pos_cm[1]
        distance = math.sqrt(dx*dx + dy*dy)
        
        # 目标角度
        target_angle = math.degrees(math.atan2(dy, dx))
        
        # 角度差
        angle_diff = target_angle - self.current_angle
        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff < -180:
            angle_diff += 360
        
        # 决策逻辑
        if distance < self.position_tolerance_cm:
            return 'stop', angle_diff, distance
        
        # 如果角度偏差大，需要转向
        if abs(angle_diff) > self.angle_tolerance:
            if angle_diff > 0:
                return 'left', angle_diff, distance
            else:
                return 'right', angle_diff, distance
        
        # 角度OK，前进
        return 'forward', angle_diff, distance
    
    async def correct_angle_drift(self, target_angle):
        """实时修正角度偏移（小幅度调整）"""
        angle_diff = target_angle - self.current_angle
        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff < -180:
            angle_diff += 360
        
        # 如果角度偏移超过阈值，进行小幅度修正
        if abs(angle_diff) > self.angle_correction_threshold:
            print(f"  ⚠ 检测到角度偏移 {angle_diff:.1f}°，进行微调...")
            
            await self.prepare_for_movement()
            
            if angle_diff > 0:
                await self.robot.left()
                await asyncio.sleep(0.1)
                self.current_angle += angle_diff * 0.3
                print(f"  微调左转，角度修正")
            else:
                await self.robot.right()
                await asyncio.sleep(0.1)
                self.current_angle += angle_diff * 0.3
                print(f"  微调右转，角度修正")
            
            await self.finish_movement()
            self.current_angle %= 360
            self.update_visualization()
            
            return True
        
        return False
    
    async def execute_control_step(self):
        """执行一个控制步骤"""
        # 检查用户是否请求停止
        if self.user_stop_requested:
            print("\n✗ 用户已停止操作")
            return False
        
        target_cm = self.get_current_target()
        if target_cm is None:
            print("✓ 所有目标点已完成")
            return False
        
        # 计算控制指令
        command, angle_diff, distance = self.calculate_control_command(target_cm)
        
        target_grid = self.target_path[self.path_index]
        print(f"\n[目标 {self.path_index}] 格子{target_grid}, 距离={distance:.1f}cm, 角度差={angle_diff:.1f}°")
        print(f"  当前位置: ({self.current_pos_cm[0]:.1f}, {self.current_pos_cm[1]:.1f})cm, 角度={self.current_angle:.1f}°")
        print(f"  控制指令: {command}, 连续转向次数: {self.consecutive_turn_count}")
        
        # 执行指令
        if command == 'stop':
            await self.robot.stop()
            print(f"  ✓ 到达目标点 {self.path_index}")
            
            # 记录轨迹
            self.trajectory_log.append({
                'time': datetime.now(),
                'position': self.current_pos_cm.copy(),
                'angle': self.current_angle,
                'waypoint': self.path_index
            })
            
            self.path_index += 1
            self.consecutive_turn_count = 0
            self.update_visualization()
            
            return True
        
        elif command in ['left', 'right']:
            # 检查是否连续转向次数过多
            if self.consecutive_turn_count >= self.max_consecutive_turns:
                print(f"  ⚠ 连续转向{self.consecutive_turn_count}次，强制前进一小段...")
                
                await self.prepare_for_movement()
                await self.robot.forward()
                await asyncio.sleep(0.25)
                await self.finish_movement()
                
                # 更新位置
                move_distance = 10
                rad = math.radians(self.current_angle)
                self.current_pos_cm[0] += move_distance * math.cos(rad)
                self.current_pos_cm[1] += move_distance * math.sin(rad)
                
                self.consecutive_turn_count = 0
                print(f"  已前进{move_distance}cm，重置转向计数")
                self.update_visualization()
            else:
                await self.prepare_for_movement()
                
                if command == 'left':
                    await self.robot.left()
                    await asyncio.sleep(0.5)
                    self.current_angle += 90
                else:
                    await self.robot.right()
                    await asyncio.sleep(0.5)
                    self.current_angle -= 90
                
                self.current_angle %= 360
                await self.finish_movement()
                self.consecutive_turn_count += 1
                self.update_visualization()
        
        elif command == 'forward':
            # 在前进之前，检查并修正角度偏移
            target_cm = self.get_current_target()
            if target_cm:
                dx = target_cm[0] - self.current_pos_cm[0]
                dy = target_cm[1] - self.current_pos_cm[1]
                target_angle = math.degrees(math.atan2(dy, dx))
                
                corrected = await self.correct_angle_drift(target_angle)
                if corrected:
                    await asyncio.sleep(0.1)
            
            await self.prepare_for_movement()
            await self.robot.forward()
            await asyncio.sleep(0.43)
            await self.finish_movement()
            
            self.consecutive_turn_count = 0
            
            # 更新位置（如果没有里程计反馈，使用推算）
            if not self.odometry_buffer:
                move_distance = 15
                rad = math.radians(self.current_angle)
                self.current_pos_cm[0] += move_distance * math.cos(rad)
                self.current_pos_cm[1] += move_distance * math.sin(rad)
        
        self.update_visualization()
        return True
    
    async def follow_path_realtime(self, enable_viz=True):
        """实时路径跟踪（主循环）"""
        print("\n" + "="*60)
        print("开始DFS出口路径跟踪")
        print("="*60)
        
        # 初始化可视化
        if enable_viz:
            self.init_visualization()
        
        self.is_running = True
        self.user_stop_requested = False
        max_steps = 200
        step_count = 0
        
        # 启动用户输入监听任务
        monitor_task = asyncio.create_task(self.monitor_user_input())
        
        try:
            while self.is_running and step_count < max_steps:
                step_count += 1
                
                if self.user_stop_requested:
                    print("\n✗✗✗ 用户已取消路径跟踪")
                    break
                
                should_continue = await self.execute_control_step()
                
                if not should_continue:
                    break
                
                if self.path_index >= len(self.target_path):
                    print("\n✓✓✓ 已到达出口！路径跟踪完成！")
                    break
                
                await asyncio.sleep(0.2)
        
        finally:
            self.is_running = False
            monitor_task.cancel()
            try:
                await monitor_task
            except asyncio.CancelledError:
                pass
            
            await self.robot.stop()
            print("\n小车已停止")
        
        print("="*60)
        print(f"路径跟踪结束（{step_count} 步）")
        if self.user_stop_requested:
            print("原因：用户取消")
        print("="*60)
        
        self.print_summary()
        
        # 保持可视化窗口显示
        if self.enable_visualization:
            print("\n动态可视化窗口将继续显示...")
            print("关闭窗口以继续程序")
    
    def print_summary(self):
        """打印摘要"""
        print("\n轨迹摘要:")
        print(f"  总移动距离: {self.total_distance_moved:.1f}cm")
        print(f"  完成路径点: {len(self.trajectory_log)}/{len(self.target_path)}")
        
        for log in self.trajectory_log:
            print(f"    路径点 {log['waypoint']}: "
                  f"位置=({log['position'][0]:.1f}, {log['position'][1]:.1f})cm, "
                  f"角度={log['angle']:.1f}°")


async def main():
    """主程序"""
    print("="*60)
    print("DFS出口寻找与路径跟踪系统")
    print("说明：不内置迷宫内部结构，使用DFS自动寻找出口")
    print("="*60)
    
    # 创建迷宫（仅外围墙）
    maze = Maze4x4(cell_size_cm=70)
    
    # 显示迷宫信息
    print("\n迷宫信息:")
    print(f"- 格子边长: {maze.cell_size_cm} cm = {maze.cell_size_m} m")
    width_cm, height_cm = maze.get_maze_size_cm()
    width_m, height_m = maze.get_maze_size_m()
    print(f"- 迷宫总尺寸: {width_cm} cm × {height_cm} cm = {width_m} m × {height_m} m")
    print(f"- 内部结构: 未知（不内置墙壁）")
    
    # 输入起点
    try:
        start_input = input("\n请输入起点坐标（格式: x,y，例如: 1,1）: ").strip()
        start_x, start_y = map(int, start_input.split(','))
        start = (start_x, start_y)
        
        start_angle = float(input("请输入起始角度（0=右，90=上，180=左，270=下，默认0）: ").strip() or "0")
        
    except (ValueError, KeyError):
        print("输入错误，使用默认值")
        start, start_angle = (1, 1), 0
    
    # 使用DFS自动寻找出口路径
    print(f"\n使用DFS寻找出口路径，起点: {start}")
    path = maze.find_exit_path(start)
    
    if not path:
        print("✗ 无法找到可达出口！")
        return
    
    print(f"✓ 找到出口路径: {' -> '.join([str(p) for p in path])}")
    print(f"  路径长度: {len(path)} 步")
    print(f"  起点: {path[0]}")
    print(f"  出口: {path[-1]}")
    
    # 先显示静态路径图
    show_static = input("\n是否显示静态路径图？(y/n，默认y): ").strip().lower()
    if show_static != 'n':
        maze.draw(show_real_scale=True, path=path, start=start, end=path[-1])
    
    # 创建跟踪器
    follower = DFSPathFollower(maze, cell_size_cm=70)
    follower.set_start_position(start[0], start[1], start_angle)
    follower.set_path(path)
    
    # 询问模式
    mode = input("\n选择模式 (1=仿真, 2=真实小车，默认1): ").strip()
    
    if mode == '2':
        # 真实小车模式
        device_addr = input("请输入小车MAC地址（回车使用默认）: ").strip() or None
        
        try:
            await follower.connect_robot(device_addr)
            
            input("\n按回车开始路径跟踪...")
            
            await follower.follow_path_realtime(enable_viz=True)
            
            await follower.disconnect_robot()
            
            # 动态可视化已经显示，保持窗口
            if follower.enable_visualization:
                plt.show()  # 阻塞显示
        
        except Exception as e:
            print(f"\n错误: {e}")
            import traceback
            traceback.print_exc()
            
            if follower.robot:
                await follower.disconnect_robot()
    
    else:
        # 仿真模式
        print("\n=== 仿真模式 ===")
        
        # 创建模拟机器人
        class SimRobot:
            async def forward(self): pass
            async def backward(self): pass
            async def left(self): pass
            async def right(self): pass
            async def stop(self): pass
            async def distance_ini_print(self): pass
            async def stop_scan(self): pass
            async def is_controll_1(self): pass
            async def is_controll_0(self): pass
            async def distance_controll(self): pass
        
        follower.robot = SimRobot()
        
        await follower.follow_path_realtime(enable_viz=True)
        
        # 动态可视化已经显示，保持窗口
        if follower.enable_visualization:
            plt.show()  # 阻塞显示


if __name__ == "__main__":
    asyncio.run(main())

