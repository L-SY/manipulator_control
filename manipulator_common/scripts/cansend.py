import can
import argparse
import time

def send_can_frame(interface, can_id, frame_type):
    # 配置CAN接口
    bus = can.interface.Bus(channel=interface, bustype='socketcan')

    # 定义不同类型的CAN帧数据
    frame_data = {
        'close': [0xFF] * 7 + [0xFD],
        'start': [0xFF] * 7 + [0xFC],
        'setZero': [0xFF] * 7 + [0xFE]
    }

    # 处理setAllZero特殊情况
    if frame_type == 'setAllZero':
        set_all_zero(bus)
        return

    if frame_type not in frame_data:
        raise ValueError(f"Invalid frame type: {frame_type}. Valid types are 'close', 'start', 'setZero', 'setAllZero'.")

    can_data = frame_data[frame_type]

    # 创建CAN消息
    msg = can.Message(arbitration_id=can_id,
                      data=can_data,
                      is_extended_id=False)

    # 发送CAN消息
    bus.send(msg)
    print(f"Sent {frame_type} to ID 0x{can_id:03X}")
    print(f"message: {msg}")

def set_all_zero(bus):
    """对所有8个电机设置零点"""
    zero_data = [0xFF] * 7 + [0xFE]  # setZero命令的数据

    for motor_id in range(1, 9):  # 电机ID从1到8
        msg = can.Message(
            arbitration_id=motor_id,
            data=zero_data,
            is_extended_id=False
        )
        bus.send(msg)
        print(f"Sent setZero to motor ID 0x{motor_id:03X}")
        print(f"message: {msg}")
        time.sleep(0.01)  # 短暂延时，避免总线拥堵

    print("All motors set to zero position")

if __name__ == "__main__":
    # 示例： python cansend.py can0 0x001 setZero
    # 或者： python cansend.py can0 0x000 setAllZero
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='Send a CAN frame via SocketCAN.')
    parser.add_argument('interface', type=str, help='CAN interface (e.g., can0)')
    parser.add_argument('can_id', type=lambda x: int(x, 16), help='CAN ID in hex (e.g., 0x001) or any value for setAllZero')
    parser.add_argument('frame_type', type=str, choices=['close', 'start', 'setZero', 'setAllZero'],
                        help='Type of frame to send (close, start, setZero, setAllZero)')

    args = parser.parse_args()

    # 发送CAN帧
    send_can_frame(args.interface, args.can_id, args.frame_type)
