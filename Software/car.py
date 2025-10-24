# robot_controller.py
# -*- coding: utf-8 -*-
import asyncio
import select
from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
import aioconsole
import json
from datetime import datetime
from typing import Optional, Callable, Any, List

# 默认特征与设备地址（可在创建实例时覆盖）
DEFAULT_NOTIFICATION_CHARACTERISTIC = "0000ffe1-0000-1000-8000-00805f9b34fb"
DEFAULT_WRITE_CHARACTERISTIC = "0000ffe1-0000-1000-8000-00805f9b34fb"
DEFAULT_DEVICE_ADDR = "C4:25:01:17:00:1B"


class RobotController:
    """
    异步 BLE 小车控制器。
    用法示例：
        async with RobotController(addr) as rc:
            await rc.forward()
            await rc.start_scan()
            ...
    """
    # 指令常量
    CMD_STOP = bytearray([0x00, 0x00])
    CMD_FORWARD = bytearray([0x00, 0x01])
    CMD_BACKWARD = bytearray([0x00, 0x02])
    CMD_LEFT = bytearray([0x00, 0x03])
    CMD_RIGHT = bytearray([0x00, 0x04])
    CMD_SCAN = bytearray([0xA5, 0x20])
    CMD_SCAN_STOP = bytearray([0xA5, 0x25])
    CMD_IS_STOP_CONTROLL_1 = bytearray([0x00, 0x10])
    CMD_IS_STOP_CONTROLL_0 = bytearray([0x00, 0x11])
    CMD_INI_AND_PRINT_DIS = bytearray([0x00, 0x09])
    CMD_DIS_CONTROLL = bytearray([0x00, 0x06])

    def __init__(
        self,
        device_addr: str = DEFAULT_DEVICE_ADDR,
        write_char: str = DEFAULT_WRITE_CHARACTERISTIC,
        notify_char: str = DEFAULT_NOTIFICATION_CHARACTERISTIC,
        use_bdaddr: bool = False,
        on_data: Optional[Callable[[dict], Any]] = None
    ):
        """
        device_addr: 设备 MAC 地址或标识
        write_char: 写特征 UUID
        notify_char: 通知特征 UUID
        use_bdaddr: 传给 find_device_by_address 的选项（如果需要）
        on_data: 可选回调，每次解析到雷达点时调用，传入解析出的 dict
        """
        self.device_addr = device_addr
        self.write_char = write_char
        self.notify_char = notify_char
        self.use_bdaddr = use_bdaddr
        self.on_data = on_data

        # 运行时变量
        self.client: Optional[BleakClient] = None
        self._disconnected_event = asyncio.Event()

        # 数据记录
        self.is_recording = False
        self.radar_data_list: List[dict] = []
        self.scan_start_time: Optional[datetime] = None

    # -----------------------
    # 连接 / 通知 管理
    # -----------------------
    async def connect(self, timeout: Optional[float] = None):
        """
        连接到设备并准备就绪（不会自动开始通知）。
        """
        # 根据你的 Bleak 版本，find_device_by_address 的 cb/use_bdaddr 参数可能有所不同。
        # 我保留了原始代码使用的 cb 参数以兼容你的环境。
        device = await BleakScanner.find_device_by_address(
            self.device_addr, cb=dict(use_bdaddr=self.use_bdaddr)
        )
        if device is None:
            raise RuntimeError(f"Could not find device with address '{self.device_addr}'")

        self._disconnected_event.clear()

        def _disconnected_callback(client):
            # 标记断连
            print("Disconnected callback called!")
            self._disconnected_event.set()

        self.client = BleakClient(device, disconnected_callback=_disconnected_callback)
        await self.client.connect(timeout=timeout)
        print("Connected")

    async def disconnect(self):
        """断开连接并清理资源"""
        if self.client and self.client.is_connected:
            # 在断开前保证停止通知（若在通知中）
            try:
                await self.stop_notify()
            except Exception:
                pass
            await self.client.disconnect()
            print("Disconnected")
        self.client = None

    async def __aenter__(self):
        await self.connect()
        # 自动注册通知回调（默认开启通知）
        await self.start_notify()
        return self

    async def __aexit__(self, exc_type, exc, tb):
        await self.disconnect()

    async def start_notify(self):
        """开始接收通知（把通知回调和实例方法绑定）"""
        if not self.client or not self.client.is_connected:
            raise RuntimeError("Client not connected")
        await self.client.start_notify(self.notify_char, self._notification_handler)
        print("Started notify on", self.notify_char)

    async def stop_notify(self):
        if not self.client:
            return
        try:
            await self.client.stop_notify(self.notify_char)
            print("Stopped notify on", self.notify_char)
        except Exception:
            # 忽略停止通知时的异常（例如未曾注册）
            pass

    # -----------------------
    # 通知回调（解析并记录雷达数据）
    # -----------------------
    def _notification_handler(self, characteristic: BleakGATTCharacteristic, data: bytearray):
        """
        Bleak 通知回调 —— 尝试将收到的 bytes 解码为 utf-8 字符串，
        按行 (\r\n) 拆分，每行尝试 json.loads -> append 到 radar_data_list。
        同时可通过 on_data 回调通知外部。
        """
        # 打印原始数据以便调试
        print("rev data:", data)
        if not self.is_recording:
            return  # 只有在记录模式下才保存数据（与原逻辑一致）

        try:
            data_str = data.decode('utf-8')
            lines = data_str.strip().split('\r\n')
            for line in lines:
                if not line.strip():
                    continue
                try:
                    radar_point = json.loads(line)
                    self.radar_data_list.append(radar_point)
                    if self.on_data:
                        try:
                            self.on_data(radar_point)
                        except Exception:
                            pass
                except json.JSONDecodeError:
                    # 无法解析就跳过该行
                    pass
            print(f"  → 已记录数据点 #{len(self.radar_data_list)}")
        except Exception as e:
            print(f"  ⚠ 数据解析错误: {e}")

    # -----------------------
    # 发送指令封装（外部可直接调用这些方法）
    # -----------------------
    async def _write(self, data: bytearray):
        if not self.client or not self.client.is_connected:
            raise RuntimeError("Client not connected")
        await self.client.write_gatt_char(self.write_char, data)

    async def forward(self):
        await self._write(self.CMD_FORWARD)

    async def backward(self):
        await self._write(self.CMD_BACKWARD)

    async def left(self):
        await self._write(self.CMD_LEFT)

    async def right(self):
        await self._write(self.CMD_RIGHT)

    async def stop(self):
        await self._write(self.CMD_STOP)

    async def is_controll_1(self):
        await self._write(self.CMD_IS_STOP_CONTROLL_1)

    async def is_controll_0(self):
        await self._write(self.CMD_IS_STOP_CONTROLL_0)

    async def distance_controll(self):
        await self._write(self.CMD_DIS_CONTROLL)

    async def distance_ini_print(self):
        await self._write(self.CMD_INI_AND_PRINT_DIS)

    async def start_scan(self):
        """发送开始扫描指令（并可同时开启记录）"""
        await self._write(self.CMD_SCAN)

    async def stop_scan(self):
        await self._write(self.CMD_SCAN_STOP)

    # -----------------------
    # 数据记录管理
    # -----------------------
    def start_recording(self):
        """只切换记录状态（不会自动发送扫描指令）"""
        self.is_recording = True
        self.radar_data_list = []
        self.scan_start_time = datetime.now()
        print("✓ 已开始记录雷达数据...")

    def stop_recording_and_save(self, filename: Optional[str] = None) -> Optional[str]:
        """
        停止记录并保存已收集的数据到 JSON 文件。
        返回写入的文件名（如果有保存），否则返回 None。
        """
        self.is_recording = False
        if not self.radar_data_list:
            print("⚠ 没有收集到数据")
            return None

        if not filename:
            filename = f"radar_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        with open(filename, 'w', encoding='utf-8') as f:
            json.dump(self.radar_data_list, f, ensure_ascii=False, indent=2)

        count = len(self.radar_data_list)
        print(f"✓ 已保存 {count} 条数据到文件: {filename}")
        # 清空缓存
        self.radar_data_list = []
        return filename

    # -----------------------
    # 交互式手动控制（保留）
    # -----------------------
    async def manual_control(self):
        """保留交互式命令行控制（基于 aioconsole）"""
        if not self.client or not self.client.is_connected:
            raise RuntimeError("Client not connected")

        print("\n=== 小车手动控制 ===")
        print("控制说明：")
        print("  w - 前进")
        print("  s - 后退")
        print("  a - 左转")
        print("  d - 右转")
        print("  x - 停止")
        print("  i - 初始化里程计/发送里程计")
        print("  c - 控制里程计开关")
        print("  0 - is_stop=0")
        print("  1 - is_stop=1")
        print("  r - 扫描（发送扫描指令并开始记录）")
        print("  ss - 扫描停止（发送停止扫描指令并停止保存）")
        print("  q - 退出程序")
        print("==================\n")

        while True:
            try:
                cmd = await aioconsole.ainput("请输入指令 (w/s/a/d/x/i/c/0/1/r/ss/q): ")
                cmd = cmd.strip().lower()
                if cmd == 'w':
                    print("→ 前进")
                    await self.forward()
                elif cmd == 's':
                    print("→ 后退")
                    await self.backward()
                elif cmd == 'a':
                    print("→ 左转")
                    await self.left()
                elif cmd == 'd':
                    print("→ 右转")
                    await self.right()
                elif cmd == 'x':
                    print("→ 停止")
                    await self.stop()
                elif cmd == 'i':
                    print("→ 初始化里程计/发送里程计")
                    await self.distance_ini_print()
                elif cmd == 'c':
                    print("→ 控制里程计开关")
                    await self.distance_controll()
                elif cmd == '0':
                    print("→ is_stop=0")
                    await self.is_controll_0()
                elif cmd == '1':
                    print("→ is_stop=1")
                    await self.is_controll_1()
                elif cmd == 'r':
                    print("→ 扫描")
                    self.start_recording()
                    await self.start_scan()
                elif cmd == 'ss':
                    print("→ 扫描停止")
                    await self.stop_scan()
                    self.stop_recording_and_save()
                elif cmd == 'q':
                    print("退出程序...")
                    await self.stop()
                    break
                else:
                    print("无效指令，请重新输入")
            except Exception as e:
                print(f"发生错误: {e}")
                break


# -----------------------
# 直接运行脚本示例
# -----------------------
async def main():
    rc = RobotController(device_addr=DEFAULT_DEVICE_ADDR)
    try:
        # 使用异步上下文更安全：会自动 start_notify 并在退出时断开
        async with rc:
            # 进入交互式手动控制（和原脚本行为一致）
            await rc.manual_control()
    except Exception as e:
        print("错误:", e)
    finally:
        # 保证断开
        try:
            await rc.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    asyncio.run(main())
