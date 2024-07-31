#!/usr/bin/env python3
import os
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from px4_msgs.msg import VehicleLocalPosition,TrajectorySetpoint, IrlockReport
from caleuche_uav_interfaces.msg import AprilTagDetection, AprilTagDetectionArray, Point # type: ignore
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
import numpy as np
import math
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class PrecisionLandingController(Node):
    
    def __init__(self):
        super().__init__('precision_landing_controller')

        self.bridge = CvBridge()
        self.px4_qos_settings = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability = DurabilityPolicy.TRANSIENT_LOCAL,
            history = HistoryPolicy.KEEP_ALL,
            depth = 1
        )

        self.apriltag_detection_subscriber = self.create_subscription(
            AprilTagDetectionArray, 'oak_1_w/data/apriltag_array', self.apriltag_detection_callback, 10, callback_group=ReentrantCallbackGroup())
        
        #self.vehicle_local_position_subscriber = self.create_subscription(
        #    VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, self.px4_qos_settings, callback_group=ReentrantCallbackGroup())
        
        #self.trajectory_setpoint_publisher = self.create_publisher(
        #    TrajectorySetpoint, '/fmu/in/trajectory_setpoint', self.px4_qos_settings)
        
        self.irlock_report_publisher = self.create_publisher(
            IrlockReport, '/fmu/in/irlock_report', self.px4_qos_settings)
        
    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg

    def apriltag_detection_callback(self, msg):
        self.publish_irlock_report(msg)

    def calculate_pose_offset(self, msg):
        image_width = 1280
        image_height = 720
        for detection in msg.detections:
            if detection.id == 1:
                delta_x = detection.centre.x - image_width/2
                delta_y = detection.centre.y - image_height/2
                offset_x = delta_x/image_width
                offset_y = delta_y/image_height
        return offset_x, offset_y

    def publish_position_setpoint_local(self, offset_x, offset_y):
        if self.vehicle_local_position is None:
            return
        msg = TrajectorySetpoint()
        msg.velocity = [offset_x, offset_y, 0.0]
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_irlock_report(self, msg):
        for detection in msg.detections:
            if detection.id == 1:
                #https://github.com/wattsinnovations/LandoPrecissian/blob/master/src/main.cpp
                msg = IrlockReport()
                #normalize to unit vector
                x = detection.pose.x
                y = detection.pose.y
                z = detection.pose.z
                print(x)
                print(y)
                print(z)
                r = math.sqrt(x*x + y*y + z*z)
                x = x/r
                y = y/r
                z = z/r
                msg.pos_x = x/z
                msg.pos_y = y/z
                print("X: ", msg.pos_x)
                print("Y: ", msg.pos_y)
                self.irlock_report_publisher.publish(msg)
    

def main():
    rclpy.init()
    node = PrecisionLandingController()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

