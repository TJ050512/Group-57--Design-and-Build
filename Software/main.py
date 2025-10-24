#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
maze_explorer_control.py

交互式命令控制版本： (保持你现有逻辑)
...
"""
import asyncio
import json
import sys
import math
import threading
import queue
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np

from car import RobotController  # 你现有的 robot controller
from match import align_and_overwrite, align_and_get_object_coords
from transform import visualize_grid_classified, gridify_json_to_grid

# 全局控制开关
running = True
DEFAULT_DEVICE_ADDR = "C4:25:01:17:00:1B"

# 命令队列（线程 -> asyncio）
cmd_queue = queue.Queue()


class MazeExplorer:
    """
    迷宫探索控制器
    
    完整操作流程：
    【初始化阶段】
    1. 初始化里程计 (0x0009)
    2. 执行首次参考扫描
    3. 关闭雷达，设置 motor_is_stop=0 (0x0011)
    4. 打开里程计 (0x0006)
    
    【移动阶段】
    5. 里程计处于工作状态，执行各种移动操作 (前进/后退/左转/右转)
    
    【扫描阶段】（每次扫描时执行）
    6. 停止小车
    7. 停止里程计 (0x0006)
    8. 设置 motor_is_stop=1 (0x0010)
    9. 打开雷达扫描
    10. 输出里程计差值 (0x0009) - 获取两次扫描之间的位移和角度
    11. 恢复里程计工作状态：设置 motor_is_stop=0 (0x0011) + 打开里程计 (0x0006)
    """
    def __init__(self, device_addr=DEFAULT_DEVICE_ADDR):
        self.rc = RobotController(device_addr)
        self.current_position = [0.0, 0.0]  # x, y 米
        self.current_angle = 0.0  # 0 = 前方（度）
        self.grid = None
        self.grid_meta = None
        self.observed_mask = None
        self.step_distance = 0.5  # 每次前进假设移动距离（米）
        # 扫描/动作默认时长
        self.move_duration = 1.7
        self.turn_duration = 0.27
        self.scan_duration = 4.3
        # 自动建图模式开关
        self.auto_mapping = True  # 默认开启自动建图
        # matplotlib 非阻塞设置
        plt.ion()

    async def connect_and_init(self):
        await self.rc.connect()
        await self.rc.start_notify()
        print("连接成功，开始初始化流程...")
        
        # 1. 先输入0009初始化里程计
        print("→ 步骤1: 初始化里程计 (0x0009)")
        await self.rc.distance_ini_print()
        await asyncio.sleep(0.5)
        
        # 2. 打开雷达进行首次参考扫描
        print("→ 步骤2: 进行首次参考扫描 (o.json)...")
        saved = await self.perform_scan("o.json", scan_only=True, duration=self.scan_duration)
        print(f"首次参照扫描已保存: {saved}")
        
        # 3. 关闭雷达后，设置里程计为工作模式
        print("→ 步骤3: 设置 motor_is_stop=0 (0x0011)")
        await self.rc.is_controll_0()
        await asyncio.sleep(0.3)
        
        print("→ 步骤4: 打开里程计 (0x0006)")
        await self.rc.distance_controll()
        await asyncio.sleep(0.3)
        
        # 使用 reference 构建栅格
        self.grid, self.grid_meta, self.observed_mask = gridify_json_to_grid("o.json")
        print("✓ 初始化栅格完成，系统就绪！")
        print(f"\n自动建图模式: {'✓ 已启用' if self.auto_mapping else '✗ 已禁用'}")
        if self.auto_mapping:
            print("  → 每次移动后将自动执行扫描和里程计记录")
            print("  → 按 'm' 键可以切换到手动模式\n")

    async def perform_scan(self, filename="n.json", scan_only=False, duration=None):
        dur = self.scan_duration if duration is None else duration
        try:
            if not scan_only:
                await self.rc.stop()
                await asyncio.sleep(0.2)
            self.rc.start_recording()
            await self.rc.start_scan()
            print(f"开始扫描，持续 {dur} 秒...")
            await asyncio.sleep(dur)
            await self.rc.stop_scan()
            saved_file = self.rc.stop_recording_and_save(filename)
            print(f"扫描保存到: {saved_file}")
            return saved_file
        except Exception as e:
            print(f"扫描过程中出错: {e}")
            return None

    async def move_for(self, direction="forward", duration=None):
        """
        前进/后退移动
        注意：小车会在指定时间后自动停止，无需手动发送停止指令
        """
        dur = self.move_duration if duration is None else duration
        try:
            if direction == "forward":
                await self.rc.forward()
                sign = 1.0
            else:
                await self.rc.backward()
                sign = -1.0
            # 等待小车自动停止（小车会在dur秒后自动停止）
            await asyncio.sleep(dur)
            # 等待额外0.3秒确保小车完全停止
            await asyncio.sleep(0.3)
            
            rad = math.radians(self.current_angle)
            dx = math.cos(rad) * self.step_distance * sign
            dy = math.sin(rad) * self.step_distance * sign
            self.current_position[0] += dx
            self.current_position[1] += dy
            print(f"移动完成，当前位姿 pos={self.current_position}, angle={self.current_angle}")
            
            # 如果启用自动建图，则在移动后自动扫描
            if self.auto_mapping:
                print("\n[自动建图模式] 开始扫描和里程计记录...")
                await self.do_scan_sequence()
            
            return True
        except Exception as e:
            print(f"move_for 错误: {e}")
            return False

    async def turn_for(self, direction="left", duration=None):
        """
        左转/右转
        注意：小车会在指定时间后自动停止，无需手动发送停止指令
        """
        dur = self.turn_duration if duration is None else duration
        try:
            if direction == "left":
                await self.rc.left()
                delta = -90
            else:
                await self.rc.right()
                delta = 90
            # 等待小车自动停止
            await asyncio.sleep(dur)
            # 等待额外0.3秒确保小车完全停止
            await asyncio.sleep(0.3)
            
            self.current_angle = (self.current_angle + delta) % 360
            print(f"转向完成, angle={self.current_angle}")
            
            # 如果启用自动建图，则在转向后自动扫描
            if self.auto_mapping:
                print("\n[自动建图模式] 开始扫描和里程计记录...")
                await self.do_scan_sequence()
            
            return True
        except Exception as e:
            print(f"turn_for 错误: {e}")
            return False

    async def _restore_odometry_state(self):
        """
        恢复里程计工作状态的辅助方法
        步骤：
        1. 设置 motor_is_stop=0 (0x0011)
        2. 打开里程计 (0x0006)
        """
        print("→ 恢复里程计工作状态...")
        print("  - 设置 motor_is_stop=0 (0x0011)")
        await self.rc.is_controll_0()
        await asyncio.sleep(0.3)
        
        print("  - 打开里程计 (0x0006)")
        await self.rc.distance_controll()
        await asyncio.sleep(0.3)
        print("✓ 里程计已恢复工作状态")

    async def do_scan_sequence(self, rotate_first=False):
        """
        扫描 -> 对齐 -> 绘图（保存 PNG，不弹出阻塞窗口）
        兼容 align_and_get_object_coords 返回 dict 的情况。
        
        完整流程：
        1. 停止小车
        2. 停止里程计 (0x0006)
        3. 设置 motor_is_stop=1 (0x0010)
        4. 打开雷达扫描
        5. 输出里程计差值 (0x0009)
        6. 关闭雷达
        7. 设置 motor_is_stop=0 (0x0011)
        8. 打开里程计 (0x0006)
        """
        try:
            # 步骤1: 确保小车已停止（小车会自动停止，这里只是额外等待确保完全停止）
            print("→ 等待小车完全停止...")
            await asyncio.sleep(0.3)
            
            # 步骤2: 停止里程计
            print("→ 停止里程计 (0x0006)")
            await self.rc.distance_controll()
            await asyncio.sleep(0.3)
            
            # 步骤3: 设置 motor_is_stop=1
            print("→ 设置 motor_is_stop=1 (0x0010)")
            await self.rc.is_controll_1()
            await asyncio.sleep(0.3)
            
            # 步骤4: 执行扫描
            print("→ 开始雷达扫描...")
            saved = await self.perform_scan("n.json", scan_only=True, duration=self.scan_duration)
            if not saved:
                print("扫描未成功，跳过后续可视化。")
                # 即使扫描失败，也要恢复里程计状态
                await self._restore_odometry_state()
                return
            
            # 步骤5: 输出里程计差值
            print("→ 输出里程计差值 (0x0009)")
            await self.rc.distance_ini_print()
            await asyncio.sleep(0.5)
            
            # 步骤6-8: 恢复里程计工作状态
            await self._restore_odometry_state()

            print("对齐扫描数据 n.json -> o.json ...")
            try:
                align_result = align_and_overwrite(reference_path="o.json", path_to_align="n.json", show_plot=False)
                print(
                    f"对齐结果: rotation {align_result.get('rotation_deg')} deg, translation {align_result.get('translation')}, rms {align_result.get('rms_error')}")
            except Exception as e:
                print(f"align_and_overwrite 出错（可能非致命）: {e}")

            try:
                self.grid, self.grid_meta, self.observed_mask = gridify_json_to_grid("o.json")
            except Exception as e:
                print(f"gridify_json_to_grid 出错: {e}")

            # 调用 align_and_get_object_coords（它现在返回 dict）
            align_info = None
            try:
                align_info = align_and_get_object_coords(reference_path="o.json", path_to_align="n.json")
                if isinstance(align_info, dict):
                    print("align_and_get_object_coords 返回：")
                    for k in (
                    'rotation_deg', 'translation', 'object_origin_in_global', 'rms_error', 'num_points_object',
                    'num_points_global'):
                        if k in align_info:
                            print(f"  {k}: {align_info.get(k)}")
                else:
                    print("align_and_get_object_coords 返回非 dict（可能是点集），将在下方绘制该点集。")
            except Exception as e:
                print(f"align_and_get_object_coords 出错（可忽略）: {e}")
                align_info = None

            # 先尝试从 align_info（如果它是点集）绘制；否则从 n.json 读取点并绘制
            plotted = False
            try:
                if align_info and not isinstance(align_info, dict):
                    # 如果返回的是点集（旧版行为），直接绘制
                    self._show_scatter(align_info, title=f"Aligned scan {datetime.now().strftime('%H:%M:%S')}",
                                       origin=None)
                    plotted = True
            except Exception as e:
                print(f"绘制 align_info 作为点集失败: {e}")

            if not plotted:
                pts = None
                try:
                    pts = self._load_points_from_json("n.json")
                except Exception as e:
                    print(f"从 n.json 读取点失败: {e}")
                    pts = None

                if pts:
                    origin = None
                    if isinstance(align_info, dict) and 'object_origin_in_global' in align_info:
                        origin = align_info.get('object_origin_in_global')
                    self._show_scatter(pts, title=f"Raw/Aligned scan {datetime.now().strftime('%H:%M:%S')}",
                                       origin=origin)
                else:
                    print("没有可绘制的点云（align_info 不是点集，且从 n.json 未能提取点）。")

            # 生成栅格图：**不要**让 visualize_grid_classified 阻塞
            try:
                out_png = f"grid_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
                # 使用 show=False 以避免阻塞；函数应该仍保存 out_png
                visualize_grid_classified(self.grid, self.grid_meta, self.observed_mask, out_png=out_png, show=False)
                print(f"栅格图已保存: {out_png} (show=False, 未弹窗以避免阻塞)")
            except Exception as e:
                print(f"visualize_grid_classified 出错: {e}")

        except Exception as e:
            print(f"do_scan_sequence 错误: {e}")

    def _show_scatter(self, coords, title="scan", origin=None):
        """
        保存散点图为 PNG (不会弹出交互窗口)
        coords: Nx2 list/array
        origin: optional [x,y]，将在图上标注为星号
        """
        try:
            arr = np.array(coords)
            # 兼容错误输入（例如 dict）
            if arr.size == 0 or arr.ndim == 0:
                print("_show_scatter: 点集为空或格式不对，跳过绘图。")
                return
            if arr.ndim == 1:
                arr = arr.reshape(1, -1)
            if arr.shape[1] < 2:
                print("_show_scatter: 点数据维度不足，至少需要 x,y 。")
                return

            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            fname = f"scatter_{timestamp}.png"
            fig = plt.figure(figsize=(6, 6))
            ax = fig.add_subplot(1, 1, 1)
            ax.scatter(arr[:, 0], arr[:, 1], s=2, label='scan points')
            if origin and isinstance(origin, (list, tuple)) and len(origin) >= 2:
                try:
                    ox, oy = float(origin[0]), float(origin[1])
                    ax.scatter([ox], [oy], s=60, marker='*', label='object_origin', zorder=5)
                    ax.annotate("origin", (ox, oy))
                except Exception:
                    pass
            ax.set_xlabel("x (m)")
            ax.set_ylabel("y (m)")
            ax.set_title(title)
            ax.axis('equal')
            ax.grid(True)
            ax.legend()
            # 保存并关闭（避免 GUI 事件循环问题）
            fig.savefig(fname, bbox_inches='tight')
            plt.close(fig)
            print(f"散点图已保存: {fname} (未弹窗以避免阻塞)")
        except Exception as e:
            print(f"_show_scatter 出错: {e}")

    def _load_points_from_json(self, filename):
        """
        新增/增强的点云解析器。
        - 处理你的格式：列表 of {"angle":..., "distance":..., "quality":...}
        - 自动检测角度单位（度/弧度）和距离单位（mm/m）
        - 忽略 quality <= 0 的点（可调整）
        返回 Nx2 的 list 或 None
        """
        try:
            with open(filename, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except Exception as e:
            print(f"_load_points_from_json 读取失败: {e}")
            return None

        # 如果是简单的 list-of-[x,y]
        if isinstance(data, list):
            if len(data) == 0:
                return []
            first = data[0]
            # case: list of [x,y]
            if isinstance(first, (list, tuple)) and len(first) >= 2 and isinstance(first[0], (int, float)):
                return data
            # case: list of dicts with 'angle' & 'distance' (你的格式)
            if isinstance(first, dict) and ('angle' in first and 'distance' in first):
                pts = []
                angles = []
                dists = []
                qualities = []
                for rec in data:
                    try:
                        ang = rec.get('angle', None)
                        dist = rec.get('distance', None)
                        qual = rec.get('quality', None)
                        if ang is None or dist is None:
                            continue
                        # optional filter by quality: 跳过明显坏质量点
                        if qual is not None:
                            try:
                                if float(qual) <= 0:
                                    continue
                            except:
                                pass
                        angles.append(float(ang))
                        dists.append(float(dist))
                        qualities.append(qual)
                    except Exception:
                        continue
                if len(dists) == 0:
                    return []
                max_dist = max(dists)
                # 如果距离数值很大（>50），很可能单位是 mm -> 转为米
                if max_dist > 50:
                    scale = 1.0 / 1000.0
                else:
                    scale = 1.0
                max_angle = max([abs(a) for a in angles]) if angles else 0.0
                # 如果角度数值较大 (> 2*pi)，则按度处理
                angle_in_degrees = (max_angle > 2 * math.pi)
                for a, r in zip(angles, dists):
                    try:
                        rr = float(r) * scale
                        if angle_in_degrees:
                            rad = math.radians(a)
                        else:
                            rad = float(a)
                        x = math.cos(rad) * rr
                        y = math.sin(rad) * rr
                        pts.append([x, y])
                    except Exception:
                        continue
                return pts

        # 老的兼容分支（points / scans / ranges+angles）
        if isinstance(data, dict):
            if 'points' in data and isinstance(data['points'], list):
                return data['points']
            if 'scans' in data and isinstance(data['scans'], list):
                pts = []
                for s in data['scans']:
                    if isinstance(s, dict) and 'x' in s and 'y' in s:
                        pts.append([s['x'], s['y']])
                if pts:
                    return pts
            if 'ranges' in data and 'angles' in data:
                angles = data['angles']
                ranges = data['ranges']
                pts = []
                # 自动判断角度单位（度还是弧度）
                max_angle = max([abs(a) for a in angles]) if angles else 0.0
                angle_in_degrees = (max_angle > 2 * math.pi)
                max_range = max(ranges) if ranges else 0.0
                scale = (1.0/1000.0) if max_range > 50 else 1.0
                for a, r in zip(angles, ranges):
                    try:
                        rr = float(r) * scale
                        if angle_in_degrees:
                            rad = math.radians(a)
                        else:
                            rad = float(a)
                        pts.append([math.cos(rad) * rr, math.sin(rad) * rr])
                    except:
                        pass
                return pts

        print("无法从 json 自动提取点云，请检查文件格式（已尝试多种格式）。")
        return None

    async def process_command(self, cmd: str):
        c = cmd.strip().lower()
        if c == 'w':
            print("命令: 前进 1.2s")
            await self.move_for(direction="forward", duration=1.2)
        elif c == 's':
            print("命令: 后退 1.7s")
            await self.move_for(direction="back", duration=1.7)
        elif c == 'a':
            print("命令: 左转 0.27s")
            await self.turn_for(direction="left", duration=0.27)
        elif c == 'd':
            print("命令: 右转 0.27s")
            await self.turn_for(direction="right", duration=0.27)
        elif c == 'x':
            print("命令: 手动执行扫描序列...")
            await self.do_scan_sequence(rotate_first=False)
        elif c == 'm':
            # 切换自动建图模式
            self.auto_mapping = not self.auto_mapping
            status = "✓ 已启用" if self.auto_mapping else "✗ 已禁用"
            print(f"自动建图模式: {status}")
            if self.auto_mapping:
                print("  → 每次移动后将自动执行扫描和里程计记录")
            else:
                print("  → 需要手动按 'x' 键执行扫描")
        elif c == 'q':
            global running
            running = False
            print("收到退出命令，准备断开并退出...")
        else:
            print(f"未知命令: {cmd}. 有效命令: w/s/a/d/x/m/q")

    async def run_command_loop(self):
        await self.connect_and_init()
        try:
            while running:
                try:
                    cmd = cmd_queue.get_nowait()
                except queue.Empty:
                    cmd = None

                if cmd:
                    await self.process_command(cmd)
                else:
                    await asyncio.sleep(0.1)
        finally:
            print("正在关闭系统...")
            try:
                await self.rc.stop()
                await asyncio.sleep(0.2)
            except:
                pass
            try:
                # 停止里程计
                print("→ 停止里程计...")
                await self.rc.distance_controll()
                await asyncio.sleep(0.3)
            except:
                pass
            try:
                await self.rc.disconnect()
            except:
                pass
            print("✓ 已断开机器人连接，程序退出。")


def input_thread():
    print("\n" + "="*60)
    print("控制命令说明：")
    print("  w - 前进 1.2s")
    print("  s - 后退 1.7s")
    print("  a - 左转 0.27s")
    print("  d - 右转 0.27s")
    print("  x - 手动执行扫描和里程计记录")
    print("  m - 切换自动建图模式 (默认: 开启)")
    print("  q - 退出程序")
    print("="*60)
    print("\n提示：")
    print("  • 自动建图模式开启时，每次移动后会自动扫描和记录里程计")
    print("  • 小车会在指定时间后自动停止，无需手动停车")
    print("  • 里程计会自动记录两次扫描之间的位移和角度变化")
    print("="*60 + "\n")
    while running:
        try:
            line = input().strip()
        except EOFError:
            break
        if not line:
            continue
        for ch in line:
            cmd_queue.put(ch)
        if 'q' in line.lower():
            break


async def main():
    t = threading.Thread(target=input_thread, daemon=True)
    t.start()
    explorer = MazeExplorer()
    try:
        await explorer.run_command_loop()
    except KeyboardInterrupt:
        print("捕获 KeyboardInterrupt，退出。")
    finally:
        t.join(timeout=0.5)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except Exception as e:
        print(f"程序异常退出: {e}")
    finally:
        sys.exit(0)
