#!/usr/bin/env python3
import rospy
import math
from quadrotor_msgs.msg import PositionCommand
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header

class TrajectoryPublisher:
    def __init__(self):
        rospy.init_node('trajectory_publisher')
        
        # 创建发布者
        self.cmd_pub = rospy.Publisher('/position_cmd', PositionCommand, queue_size=10)
        
        # 设置发布频率 (20Hz)
        self.rate = rospy.Rate(20) 
        
        # 轨迹参数
        self.start_time = rospy.Time.now()
        self.trajectory_type = "circle"  # 可选项：circle, line, hover
        
        # 初始化消息
        self.cmd_msg = PositionCommand()
        self.cmd_msg.kx = [5.0, 5.0, 5.0]  # 位置增益
        self.cmd_msg.kv = [0.4, 0.4, 0.4]  # 速度增益
        self.cmd_msg.trajectory_id = 1
        self.cmd_msg.trajectory_flag = 1

    def generate_trajectory(self):
        """生成轨迹数据"""
        t = (rospy.Time.now() - self.start_time).to_sec()
        
        if self.trajectory_type == "circle":
            # 圆形轨迹 (半径2米，周期16秒)
            radius = 2.0
            omega = 2 * math.pi / 8
            
            self.cmd_msg.position.x = radius * (math.cos(omega * t) - 1)
            self.cmd_msg.position.y = radius * (math.sin(omega * t))
            self.cmd_msg.position.z = 3
            
            self.cmd_msg.velocity.x = 0
            self.cmd_msg.velocity.y = 0
            self.cmd_msg.velocity.z = 0.0
            
            self.cmd_msg.yaw = 0
            
        elif self.trajectory_type == "line":
            # 直线往复运动 (x轴方向)
            amplitude = 3.0
            period = 6.0
            
            self.cmd_msg.position.x = amplitude * math.sin(2 * math.pi / period * t)
            self.cmd_msg.position.y = 0.0
            self.cmd_msg.position.z = 3
            
            self.cmd_msg.velocity.x = amplitude * (2 * math.pi / period) * math.cos(2 * math.pi / period * t)
            self.cmd_msg.velocity.y = 0.0
            self.cmd_msg.velocity.z = 0.0
            
            self.cmd_msg.yaw = 0.0
            
        else:  # hover
            # 悬停模式
            self.cmd_msg.position.x = 0.0
            self.cmd_msg.position.y = 0.0
            self.cmd_msg.position.z = 3
            self.cmd_msg.velocity = Vector3(0,0,0)
            self.cmd_msg.yaw = 0.0

    def run(self):
        while not rospy.is_shutdown():
            # 更新header时间戳
            self.cmd_msg.header = Header(
                stamp=rospy.Time.now(),
                frame_id="world"  # 坐标系名称
            )
            
            # 生成轨迹数据
            self.generate_trajectory()
            
            # 发布消息
            self.cmd_pub.publish(self.cmd_msg)
            
            # 维持发布频率
            self.rate.sleep()

if __name__ == '__main__':
    try:
        publisher = TrajectoryPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass