#!/usr/bin/env python3
import rospy
import sys
import tty
import termios
import select
import threading
from mavros_msgs.msg import RCIn

class RCKeyPublisher:
    def __init__(self):
        rospy.init_node('keyboard_rc_publisher', anonymous=True)
        self.rc_pub = rospy.Publisher('/mavros/rc/in', RCIn, queue_size=10)
        self.rc_channels = [1500] * 16  # 初始中立值
        self.lock = threading.Lock()     # 保护 rc_channels
        self.running = True              # 线程控制标志
        self.publish_rate = 10           # 发布频率 (Hz)

        # 启动发布线程
        self.publish_thread = threading.Thread(target=self._publish_loop)
        self.publish_thread.start()

    def _publish_loop(self):
        """后台线程：持续发布 RC 消息"""
        rate = rospy.Rate(self.publish_rate)
        while self.running and not rospy.is_shutdown():
            with self.lock:
                self._publish_rc_message()
            rate.sleep()

    def _publish_rc_message(self):
        """实际发布消息（无打印）"""
        rc_msg = RCIn()
        rc_msg.channels = self.rc_channels.copy()  # 深拷贝避免线程冲突
        self.rc_pub.publish(rc_msg)

    def get_input(self, timeout=0.1):
        """你的非阻塞输入函数（保持不变）"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            line = ''
            sys.stdout.write("\r> ")  # 输入提示符
            sys.stdout.flush()
            while True:
                can_read, _, _ = select.select([sys.stdin], [], [], timeout)
                if not can_read:
                    if not self.running or rospy.is_shutdown():
                        break
                    continue
                ch = sys.stdin.read(1)
                if ch == '\r' or ch == '\n':
                    sys.stdout.write('\n')
                    break
                elif ord(ch) == 3:  # Ctrl+C
                    raise KeyboardInterrupt
                elif ord(ch) == 127:  # 退格键
                    if len(line) > 0:
                        line = line[:-1]
                        sys.stdout.write('\b \b')
                else:
                    line += ch
                    sys.stdout.write(ch)
                sys.stdout.flush()
            return line.strip()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def run(self):
        print("===================== 键盘控制 RC 通道 =====================")
        print("输入格式：<通道号> <值>（例如：3 1500，表示设置通道 3 的值为 1500）")
        print("输入 'q' 退出程序")
        print("==========================================================")

        try:
            while self.running and not rospy.is_shutdown():
                input_line = self.get_input()
                if not input_line:
                    continue
                
                if input_line.lower() == 'q':
                    print("\n退出程序")
                    self.running = False
                    break

                parts = input_line.split()
                if len(parts) != 2:
                    print("错误：输入需包含两个参数，例如 '3 1500'")
                    continue

                try:
                    channel = int(parts[0])
                    value = int(parts[1])
                except ValueError:
                    print("错误：请输入有效的数字")
                    continue

                if 1 <= channel <= 16:
                    index = channel - 1
                    if 1000 <= value <= 2000:
                        with self.lock:
                            self.rc_channels[index] = value
                        print(f"已设置通道 {channel} 的值为 {value}")
                    else:
                        print("错误：值范围应为 1000-2000")
                else:
                    print("错误：通道号范围应为 1-16")
        except KeyboardInterrupt:
            print("\n用户中断操作")
        finally:
            self.running = False
            self.publish_thread.join()

if __name__ == "__main__":
    try:
        rc_publisher = RCKeyPublisher()
        rc_publisher.run()
    except rospy.ROSInterruptException:
        pass