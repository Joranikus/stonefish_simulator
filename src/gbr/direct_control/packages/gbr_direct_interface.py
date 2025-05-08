#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Vector3, Quaternion
import math

class GBRDirectInterface:
    
    def __init__(self, node: Node):
        self._node = node
        self._publisher = node.create_publisher(
            Float64MultiArray,
            '/gbr/thrusters',
            10
        )

        self._position = Point()
        self._orientation = Quaternion()
        self._linear_velocity = Vector3()
        self._angular_velocity = Vector3()
        self._odom_subscriber = node.create_subscription(
            Odometry,
            '/gbr/odom',
            self._odom_callback,
            10
        )
    
    def _odom_callback(self, msg: Odometry):
        self._position = msg.pose.pose.position
        self._orientation = msg.pose.pose.orientation
        self._linear_velocity = msg.twist.twist.linear
        self._angular_velocity = msg.twist.twist.angular
    
    def get_pose(self):
        position = (self._position.x, self._position.y, self._position.z)
        quaternion = self._orientation
        roll = math.atan2(2*(quaternion.w*quaternion.x + quaternion.y*quaternion.z), 
                         1 - 2*(quaternion.x*quaternion.x + quaternion.y*quaternion.y))
        pitch = math.asin(2*(quaternion.w*quaternion.y - quaternion.z*quaternion.x))
        yaw = math.atan2(2*(quaternion.w*quaternion.z + quaternion.x*quaternion.y), 
                        1 - 2*(quaternion.y*quaternion.y + quaternion.z*quaternion.z))
        return position, (roll, pitch, yaw)
    
    def get_velocity(self):
        lin_vel = (self._linear_velocity.x, 
                  self._linear_velocity.y, 
                  self._linear_velocity.z)
        
        ang_vel = (self._angular_velocity.x,
                  self._angular_velocity.y,
                  self._angular_velocity.z)
        return lin_vel, ang_vel
    
    def print_state(self):
        position, orientation = self.get_pose()
        lin_vel, ang_vel = self.get_velocity()
        self._node.get_logger().info(
            f'\nPosition (x,y,z) [m]: ({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f})\n'
            f'Orientation (r,p,y) [rad]: ({orientation[0]:.2f}, {orientation[1]:.2f}, {orientation[2]:.2f})\n'
            f'Linear velocity (x,y,z) [m/s]: ({lin_vel[0]:.2f}, {lin_vel[1]:.2f}, {lin_vel[2]:.2f})\n'
            f'Angular velocity (r,p,y) [rad/s]: ({ang_vel[0]:.2f}, {ang_vel[1]:.2f}, {ang_vel[2]:.2f})'
        )
    
    def set_thrusters(self, values):
        if len(values) != 8:
            self._node.get_logger().error('Must provide 8 thruster values')
            return
        msg = Float64MultiArray()
        msg.data = values
        self._publisher.publish(msg)
    
    def stop(self):
        self.set_thrusters([0.0] * 8)