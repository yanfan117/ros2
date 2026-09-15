#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""ROS turtlesim 小海龟画圆节点（闭环位姿反馈版）。"""

import math

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class CircleDrawer:
    """持续发布线速度和角速度，让小海龟精确绘制一个完整的圆。"""

    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node("turtle_circle_node")

        # 向小海龟的速度话题发布 Twist 消息
        self.pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

        # 订阅小海龟位姿，用于判断是否已经旋转一周
        self.pose = None
        rospy.Subscriber("/turtle1/pose", Pose, self.pose_callback)

        # 50 Hz 高频控制，避免 turtlesim 的速度看门狗将海龟停下
        self.rate = rospy.Rate(50)

        # 运动参数：圆半径 R = v / w = 1.0 / 0.5 = 2.0 m
        self.linear_speed = 1.0
        self.angular_speed = 0.5
        self.target_angle = 2 * math.pi

        rospy.on_shutdown(self.stop)

    def pose_callback(self, msg):
        """位姿回调：保存小海龟的最新坐标和朝向角。"""
        self.pose = msg

    def publish_velocity(self, linear=0.0, angular=0.0):
        """组装并发布速度指令。"""
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.pub.publish(twist)

    def wait_for_pose(self):
        """等待第一帧位姿数据，保证闭环反馈可用。"""
        while self.pose is None and not rospy.is_shutdown():
            rospy.loginfo_throttle(1.0, "等待 /turtle1/pose 位姿数据...")
            self.rate.sleep()

    @staticmethod
    def normalize_angle(angle):
        """将角度差归一化到 [-pi, pi]，处理跨越 ±pi 时的数值跳变。"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def stop(self):
        """发布全零速度，让小海龟安全停下。"""
        self.publish_velocity()

    def draw_circle(self):
        """根据实时朝向角累计旋转量，达到 2π 后停止。"""
        self.wait_for_pose()
        if rospy.is_shutdown():
            return

        start_x = self.pose.x
        start_y = self.pose.y
        previous_theta = self.pose.theta
        total_angle = 0.0

        # 角速度为正时逆时针画圆，为负时顺时针画圆
        direction = 1.0 if self.angular_speed >= 0.0 else -1.0
        radius = abs(self.linear_speed / self.angular_speed)

        rospy.loginfo(
            "开始画圆：半径 %.2f 米，线速度 %.2f m/s，角速度 %.2f rad/s",
            radius,
            self.linear_speed,
            self.angular_speed,
        )

        while total_angle < self.target_angle and not rospy.is_shutdown():
            # 线速度与角速度同时存在时，小海龟将沿圆弧运动
            self.publish_velocity(self.linear_speed, self.angular_speed)
            self.rate.sleep()

            current_theta = self.pose.theta
            delta = self.normalize_angle(current_theta - previous_theta)
            previous_theta = current_theta

            # 只累计期望方向上的角度变化，避免微小抖动导致倒退
            total_angle += max(0.0, direction * delta)

        self.stop()

        endpoint_error = math.hypot(
            self.pose.x - start_x,
            self.pose.y - start_y,
        )
        rospy.loginfo(
            "圆形绘制完成！累计旋转 %.3f 弧度，终点与起点距离 %.3f 米",
            total_angle,
            endpoint_error,
        )


if __name__ == "__main__":
    node = CircleDrawer()
    try:
        node.draw_circle()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点被中断，让小海龟停下来")
    finally:
        node.stop()
