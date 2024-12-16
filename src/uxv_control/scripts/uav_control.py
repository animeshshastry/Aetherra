#!/usr/bin/env python3
import rclpy
from rclpy import qos
from rclpy.node import Node
import numpy as np
import time
from math import cos, sin, sqrt, acos, atan2, pi
from pyquaternion import Quaternion

from std_srvs.srv import Empty
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path

pos_ctrl_gain = [0.5, 0.5, 1.0]
vel_ctrl_gain = [0.5, 0.5, 0.5]
yaw_gain = 1.0
yaw_rate_gain = 0.2
pos_max_err = 8.0
yaw_max_err = 0.5
wpt_accept_radius = 0.5

def clamp(n, smallest, largest): return max(smallest, min(n, largest))

# def get_heading_from_two_pos(pos1,pos2):
#     pos_e = pos1-pos2
#     return atan2(pos_e[1],pos_e[0])

def get_heading_from_quat(q):
    # yaw (z-axis rotation)
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = atan2(siny_cosp, cosy_cosp)
    return yaw

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('uav_control_node')
        
        self.waypoint_list = []

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cmd_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        # self.pose_subscriber = self.create_subscription(PoseWithCovarianceStamped,'/openvins/poseimu', uav_tools.pose_callback, 10)
        self.pose_subscriber = self.create_subscription(Odometry,'/raw_odom', self.pose_callback, 10)
        self.path_subscriber = self.create_subscription(Path,'/goal_path', self.path_callback, 10)

    def pose_callback(self, msg):
        self.pose = msg.pose.pose
        self.twist = msg.twist.twist

    def timer_callback(self):
        self.mission_step()
    
    def clear_wpt(self):
        self.waypoint_list.clear()

    def path_callback(self, msg):
        print("Recived path msg")
        print(msg)
        self.clear_wpt()
        pose_list = msg.poses
        for pose in pose_list:
            self.add_wpt(pose.pose)

    def add_wpt(self, pose):
        self.waypoint_list.append([pose.position.x, pose.position.y, pose.position.z, get_heading_from_quat(pose.orientation)])
        # print(waypoint_list)

    def mission_step(self):
        if (len(self.waypoint_list) > 0):
            wpt = self.waypoint_list[0]
            # print(wpt)
            reached = self.publish_cmd(wpt,wpt_accept_radius)
            if (reached):
                self.waypoint_list.pop(0)
    
    def publish_cmd(self, wpt, tolerance):
        target_orient = Quaternion(axis=(0.0, 0.0, 1.0), radians=wpt[3])
        target_position = np.array([wpt[0],wpt[1],wpt[2]])
        current_position = np.array([self.pose.position.x, self.pose.position.y, self.pose.position.z])
        current_velocity = np.array([self.twist.linear.x, self.twist.linear.y, self.twist.linear.z])
        position_error_in_world = target_position-current_position
        current_orient = Quaternion(self.pose.orientation.w,self.pose.orientation.x,self.pose.orientation.y,self.pose.orientation.z)
        position_error_in_body = (current_orient.inverse).rotate(position_error_in_world)
        distance_to_target = np.linalg.norm(target_position-current_position)
        if (distance_to_target > pos_max_err):
            position_error_unit_vector = position_error_in_body / distance_to_target
            position_error_in_body = pos_max_err*position_error_unit_vector
        orient_error_quat = target_orient * (current_orient.conjugate)
        orient_cmd = -yaw_gain*clamp(orient_error_quat[3],-yaw_max_err,yaw_max_err) + yaw_rate_gain*self.twist.angular.z
        if (distance_to_target<tolerance+0.1):
            return True
        vel = Twist()
        vel.linear.x = pos_ctrl_gain[0]*position_error_in_body[0] - vel_ctrl_gain[0]*current_velocity[0]
        vel.linear.y = pos_ctrl_gain[1]*position_error_in_body[1] - vel_ctrl_gain[1]*current_velocity[1]
        vel.linear.z = pos_ctrl_gain[2]*position_error_in_body[2] - vel_ctrl_gain[2]*current_velocity[2]
        vel.angular.x, vel.angular.y, vel.angular.z = 0.0, 0.0, orient_cmd
        try:
            self.cmd_publisher.publish(vel)
            # time.sleep(0.05)
            return False
        except Exception as e:
            return True

def main():
    rclpy.init()
    robot_control_node = RobotControlNode()
    rclpy.spin(robot_control_node)
    robot_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()