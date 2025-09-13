#!/usr/bin/env python3
"""
串口通信类 - 含接收监控功能
提供简单接口通过串口发送和接收数据
"""

import serial
import threading
import time
import sys
from typing import Union, Callable, Optional

try:
    import serial.tools.list_ports
    HAS_TOOLS = True
except ImportError:
    HAS_TOOLS = False
    print("警告: 无法导入 serial.tools，列表端口功能将不可用")

class SerialSender:
    def __init__(self, port: str = None, baudrate: int = 9600, timeout: float = 1.0, 
                 auto_monitor: bool = True):
        """
        初始化串口发送器
        
        参数:
            port: 串口设备名 (如 'COM3' 或 '/dev/ttyUSB0')
            baudrate: 波特率 (默认: 9600)
            timeout: 超时时间(秒) (默认: 1.0)
            auto_monitor: 是否自动启动接收监控 (默认: True)
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.auto_monitor = auto_monitor
        self.ser = None
        self.is_connected = False
        
        # 接收监控相关属性
        self.receiving = False
        self.receive_thread = None
        self.receive_callback = None
        
        # 如果提供了端口，自动连接
        if port:
            self.connect()
    
    def connect(self) -> bool:
        """
        连接到串口设备
        
        返回:
            bool: 连接是否成功
        """
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            self.is_connected = True
            print(f"已连接到 {self.port}, 波特率 {self.baudrate}")
            
            # 如果设置了自动监控，启动接收监控
            if self.auto_monitor:
                self.start_receive_monitor()
            
            return True
        except serial.SerialException as e:
            print(f"无法打开串口 {self.port}: {e}")
            self.is_connected = False
            return False
    
    def disconnect(self):
        """断开串口连接"""
        # 停止接收监控
        self.stop_receive_monitor()
        
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.is_connected = False
            print(f"已断开与 {self.port} 的连接")
    
    def send(self, data: Union[str, bytes], encoding: str = 'utf-8') -> bool:
        """
        发送数据到串口
        
        参数:
            data: 要发送的数据，可以是字符串或字节
            encoding: 字符串编码方式 (默认: 'utf-8')
            
        返回:
            bool: 发送是否成功
        """
        if not self.is_connected:
            print("错误: 未连接到串口设备")
            return False
        
        try:
            # 转换数据为字节
            if isinstance(data, str):
                data_bytes = data.encode(encoding)
            else:
                data_bytes = data
            
            # 发送数据
            bytes_sent = self.ser.write(data_bytes)
            print(f"已发送 {bytes_sent} 字节: {data_bytes}")
            return True
            
        except Exception as e:
            print(f"发送错误: {e}")
            return False
    
    def send_hex(self, hex_data: str) -> bool:
        """
        发送十六进制数据
        
        参数:
            hex_data: 十六进制字符串 (如 "48 65 6C 6C 6F" 或 "48656C6C6F")
            
        返回:
            bool: 发送是否成功
        """
        try:
            # 清理十六进制字符串
            hex_data = hex_data.strip().replace(' ', '')
            # 转换为字节
            data_bytes = bytes.fromhex(hex_data)
            # 发送
            return self.send(data_bytes)
        except ValueError as e:
            print(f"错误: 无效的十六进制数据 - {e}")
            return False
    
    def list_ports(self):
        """列出所有可用的串口设备"""
        if not HAS_TOOLS:
            print("错误: serial.tools 不可用，无法列出端口")
            return
        
        ports = serial.tools.list_ports.comports()
        if not ports:
            print("未找到可用的串口设备")
            return
        
        print("可用的串口设备:")
        for i, port in enumerate(ports):
            print(f"{i+1}. {port.device} - {port.description}")
    
    def _receive_monitor(self):
        """接收监控线程函数"""
        print("开始监控串口接收...")
        while self.receiving and self.is_connected:
            try:
                # 检查是否有数据可读
                if self.ser.in_waiting > 0:
                    # 读取数据
                    data = self.ser.read(self.ser.in_waiting)
                    
                    # 打印接收到的数据
                    print(f"接收到 {len(data)} 字节: {data}")
                    
                    # 如果有回调函数，调用它
                    if self.receive_callback:
                        self.receive_callback(data)
                
                # 短暂休眠，避免CPU占用过高
                time.sleep(0.01)
                
            except Exception as e:
                print(f"接收数据错误: {e}")
                break
    
    def start_receive_monitor(self, callback: Optional[Callable] = None):
        """
        启动接收监控
        
        参数:
            callback: 可选的回调函数，当接收到数据时调用
        """
        if not self.is_connected:
            print("错误: 未连接到串口设备")
            return
        
        if self.receiving:
            print("接收监控已经在运行中")
            return
        
        self.receive_callback = callback
        self.receiving = True
        
        # 创建并启动接收线程
        self.receive_thread = threading.Thread(target=self._receive_monitor)
        self.receive_thread.daemon = True  # 设置为守护线程，主程序退出时自动结束
        self.receive_thread.start()
    
    def stop_receive_monitor(self):
        """停止接收监控"""
        self.receiving = False
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=1.0)  # 等待线程结束，最多1秒
        print("已停止监控串口接收")
    
    def __del__(self):
        """析构函数，确保连接被关闭"""
        self.disconnect()
    
    def __enter__(self):
        """支持上下文管理器"""
        if not self.is_connected and self.port:
            self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """支持上下文管理器"""
        self.disconnect()


# 示例回调函数
def my_receive_callback(data):
    """自定义接收回调函数示例"""
    try:
        # 尝试将数据解码为字符串
        text = data.decode('utf-8')
        print(f"接收到文本: {text}")
    except UnicodeDecodeError:
        # 如果不能解码为文本，显示十六进制
        hex_str = ' '.join(f'{b:02X}' for b in data)
        print(f"接收到十六进制数据: {hex_str}")


# 示例使用
if __name__ == "__main__":
    # 创建串口发送器实例，设置 auto_monitor=False 避免自动启动
    # 请将 'COM3' 替换为您的实际串口设备
    sender = SerialSender(port='COM6', baudrate=115200, auto_monitor=False)
    
    # 如果连接成功，发送数据并监控接收
    if sender.is_connected:
        # 启动接收监控，使用自定义回调函数
        sender.start_receive_monitor(callback=my_receive_callback)
        
        # 发送一些测试数据
        sender.send("Hello, World!")
        time.sleep(1)  # 等待1秒
        
        sender.send_hex("48 65 6C 6C 6F 20 57 6F 72 6C 64 21")
        time.sleep(1)  # 等待1秒
        
        # 保持程序运行一段时间，以便观察接收
        print("程序运行中，按 Ctrl+C 退出...")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n用户中断程序")
    
    # 断开连接
    sender.disconnect()