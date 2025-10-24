import asyncio
import json
import sys
from datetime import datetime
from car import RobotController  # 直接使用自带的通知处理函数
import numpy as np
import threading

# 导入工具函数
from match import align_and_overwrite, align_and_get_object_coords
from transform import visualize_grid_classified, gridify_json_to_grid


running = True
DEFAULT_DEVICE_ADDR = "C4:25:01:17:00:1B"  # 设备地址


class MazeExplorer:
    def __init__(self, device_addr=DEFAULT_DEVICE_ADDR):
        self.rc = RobotController(device_addr)
        # 移除自定义通知处理函数的重写，使用car.py中自带的_notification_handler
        self.current_position = [0, 0]
        self.current_angle = 0
        self.grid = None
        self.grid_meta = None
        self.observed_mask = None
        self.step_distance = 0.5

    async def initialize(self):
        await self.rc.connect()
        await self.rc.start_notify()
        print("进行首次扫描...")
        await self.perform_scan("o.json")
        self.grid, self.grid_meta, self.observed_mask = gridify_json_to_grid("o.json")
        print("初始化栅格完成")

    async def perform_scan(self, filename):
        await self.rc.stop()
        await asyncio.sleep(0.5)
        self.rc.start_recording()
        await self.rc.start_scan()
        await asyncio.sleep(3)  # 扫描时长
        await self.rc.stop_scan()
        saved_file = self.rc.stop_recording_and_save(filename)
        return saved_file

    async def move_step(self):
        try:
            await self.rc.forward()
            await asyncio.sleep(1)
            await self.rc.stop()
            return True
        except Exception as e:
            print(f"移动错误: {e}")
            await self.rc.stop()
            return False

    async def turn(self, direction):
        try:
            if direction == 'left':
                await self.rc.left()
            else:
                await self.rc.right()
            await asyncio.sleep(0.5)
            await self.rc.stop()
            self.current_angle = (self.current_angle + 90) % 360 if direction == 'right' else (self.current_angle - 90) % 360
            return True
        except Exception as e:
            print(f"转向错误: {e}")
            await self.rc.stop()
            return False

    async def explore_step(self):
        print("向前移动一步...")
        if not await self.move_step():
            return False

        print("进行新的扫描...")
        await self.perform_scan("n.json")

        print("对齐扫描数据...")
        align_result = align_and_overwrite(
            reference_path="o.json",
            path_to_align="n.json",
            show_plot=True
        )
        print(f"对齐结果: 旋转 {align_result['rotation_deg']}度, 平移 {align_result['translation']}, RMS误差 {align_result['rms_error']}")

        print("更新占据栅格...")
        self.grid, self.grid_meta, self.observed_mask = gridify_json_to_grid("o.json")

        viz_img = visualize_grid_classified(
            self.grid,
            self.grid_meta,
            self.observed_mask,
            out_png=f"grid_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png",
            show=True
        )

        if self.check_front_obstacle():
            print("前方有障碍，右转...")
            await self.turn('right')

        return True

    def check_front_obstacle(self):
        if self.grid is None:
            return False

        center_x, center_y = self.grid.shape[1] // 2, self.grid.shape[0] // 2
        if self.current_angle % 360 == 0:
            check_range = (center_y - 5, center_y + 5, center_x, center_x + 15)
        elif self.current_angle % 360 == 90:
            check_range = (center_y - 15, center_y, center_x - 5, center_x + 5)
        elif self.current_angle % 360 == 180:
            check_range = (center_y - 5, center_y + 5, center_x - 15, center_x)
        else:
            check_range = (center_y, center_y + 15, center_x - 5, center_x + 5)

        y_start, y_end, x_start, x_end = check_range
        for i in range(max(0, y_start), min(self.grid.shape[0], y_end)):
            for j in range(max(0, x_start), min(self.grid.shape[1], x_end)):
                if self.grid_meta.get('use_logodds', True) and self.grid[i, j] > 0:
                    return True
                elif not self.grid_meta.get('use_logodds', True) and self.grid[i, j] > 0.6:
                    return True
        return False

    async def start_exploration(self):
        try:
            await self.initialize()
            while running:
                success = await self.explore_step()
                if not success:
                    print("探索步骤失败，重试...")
                    await asyncio.sleep(1)
                await asyncio.sleep(0.5)
        except Exception as e:
            print(f"探索错误: {e}")
        finally:
            await self.rc.stop()
            await self.rc.disconnect()
            print("探索结束，断开连接")


def command_monitor():
    global running
    while running:
        cmd = input("输入 'q' 并回车以暂停探索: \n")
        if cmd.strip().lower() == 'q':
            print("收到暂停指令，停止探索...")
            running = False


async def main():
    monitor_thread = threading.Thread(target=command_monitor, daemon=True)
    monitor_thread.start()
    explorer = MazeExplorer()
    await explorer.start_exploration()
    monitor_thread.join()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("程序被手动中断")
    finally:
        sys.exit(0)