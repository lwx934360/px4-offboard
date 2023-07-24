#!/usr/bin/env python
import csv
import rclpy
import numpy as np
import math
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus


class OffboardControl(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        timer_period = 0.005  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.dt = timer_period
        # self.theta = 0.0
        # self.radius = 10.0
        # self.omega = 0.5
        self.i = 0
 
    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        print("NAV_STATUS: ", msg.nav_state)
        print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state

    def readstate(self):
        self.column_j = []   # x
        self.column_k = []   # y
        self.column_l = []   # z
        with open('map/blackbird_state_tiltedthrice_YF_6p0.csv', 'r') as file:
            reader = csv.reader(file)
            next(reader)
            for row in reader:
                value_j = float(row[9])
                value_k = float(row[10])
                value_l = float(row[11])
                # Add the values to their respective lists
                self.column_j.append(value_j)
                self.column_k.append(value_k)
                self.column_l.append(value_l)
    
    def readquat(self):
        self.column_q1 = []   # x
        self.column_q2 = []   # y
        self.column_q3 = []   # z
        self.column_q4 = []   # z
        with open('map/blackbird_state_tiltedthrice_YF_6p0.csv', 'r') as file:
            reader = csv.reader(file)
            next(reader)
            for row in reader:
                value_q1 = float(row[13])
                value_q2 = float(row[14])
                value_q3 = float(row[15])
                value_q4 = float(row[16])
                # Add the values to their respective lists
                self.column_q1.append(value_q1)
                self.column_q2.append(value_q2)
                self.column_q3.append(value_q3)
                self.column_q4.append(value_q4)
            

    def cmdloop_callback(self):
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position=True
        offboard_msg.velocity=False
        offboard_msg.acceleration=True
        
        self.publisher_offboard_mode.publish(offboard_msg)
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.position[0] = self.column_j[self.i]
            trajectory_msg.position[1] = self.column_k[self.i]
            trajectory_msg.position[2] = -5

            yaw = math.atan2(2 * ((self.column_q2[self.i] * self.column_q3[self.i]) + (self.column_q1[self.i] * self.column_q4[self.i])),
                              self.column_q1[self.i]**2 + self.column_q2[self.i]**2 - self.column_q3[self.i]**2 - self.column_q4[self.i]**2)
            trajectory_msg.yaw = yaw

            self.publisher_trajectory.publish(trajectory_msg)
            self.i+=1
            # self.theta = self.theta + self.omega * self.dt


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()
    offboard_control.readstate()
    offboard_control.readquat()
    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
