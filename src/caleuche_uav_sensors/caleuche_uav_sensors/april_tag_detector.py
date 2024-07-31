#!/usr/bin/env python3
import os
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image, CameraInfo
from caleuche_uav_interfaces.msg import AprilTagDetection, AprilTagDetectionArray, Point # type: ignore
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
import json
import cv2.aruco
import numpy as np
import math


class AprilTagDetector(Node):
    
    def __init__(self):
        super().__init__('april_tag_detector')
        self.load_camera_params()
        self.detector_setup()
        self.define_marker_params()
        self.bridge = CvBridge()

        self.image_subscriber = self.create_subscription(Image, "/camera", self.image_callback, 10)

        self.detection_image_publisher = self.create_publisher(Image, 'oak_1_w/rgb/image_aruco_detect', 10)
        self.detection_data_publisher = self.create_publisher(AprilTagDetectionArray, 'oak_1_w/data/apriltag_array', 10)
    
    def load_camera_params(self):
        #self.path_to_share = get_package_share_directory('caleuche_uav_sensors')
        #self.path_to_calibration = os.path.join(self.path_to_share, 'calibration', 'oak_1_w_calibration.json')

        #with open(self.path_to_calibration) as f:
        #    self.camera_params = json.load(f)

        #self.camera_matrix = np.array(self.camera_params['cameraData'][0][1]['intrinsicMatrix'], dtype="double")
        #self.distorion_coefs = np.array(self.camera_params['cameraData'][0][1]['distortionCoeff'], dtype="double")
        #self.no_distortion = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype="double")

        #True Parametes for OAK-1-W at 1280x720
        self.distorion_coefs = np.array([-0.2627985027226353, 0.02853855756301266, -0.012800864717835954, 0.0005381702806697179, 0.0], dtype="double")
        self.camera_matrix = np.array([[1132.8096975185235, 0.0, 580.5210235451025], [0.0, 1121.8161481549002, 412.9899959358472], [0.0, 0.0, 1.0]], dtype="double")

        #Simulation Parameter Load for 1280x720
        self.simulation = False
        if self.simulation:
            self.get_logger().info("Running in Simulation Mode!")
            self.path_to_share = get_package_share_directory('caleuche_uav_sensors')
            self.path_to_calibration = os.path.join(self.path_to_share, 'calibration', 'oak_1_w_simulation_calibration.json')
            with open(self.path_to_calibration) as f:
                self.camera_params = json.load(f)
            self.camera_matrix = np.array(self.camera_params['cameraData'][0][1]['intrinsicMatrix'], dtype="double")
            self.distorion_coefs = np.array(self.camera_params['cameraData'][0][1]['distortionCoeff'], dtype="double")

        print("Camera Matrix: ", self.camera_matrix)
        print("Distortion Coefficients: ", self.distorion_coefs)

    def detector_setup(self):
        # Check if the correct version of OpenCV is installed
        if cv2.__version__ != "4.6.0":
            self.get_logger().error("Please install OpenCV 4.6.0 with pip3 install opencv-contrib-python==4.6.0.66")
            return
        self.tag_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5)
        self.detector_params = cv2.aruco.DetectorParameters_create()
        self.get_logger().info("AprilTag detector initialized")

    def define_marker_params(self):
        #Long range guidance marker
        self.tag_id_long = 1
        self.tag_size_long = 0.18

        #Short range guidance marker
        self.tag_id_short = 0
        self.tag_size_short = 0.08

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV Image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect AprilTags
        (corners, ids, rejected) = cv2.aruco.detectMarkers(gray, self.tag_dict, parameters=self.detector_params)

        # Publish AprilTagArray
        if ids is not None:
            detection_array = AprilTagDetectionArray()
            detection_array.header = msg.header

            for i in range(len(ids)):
                corner_array = []
                detection = AprilTagDetection()
                detection.id = int(ids[i][0])
                detection.family = "tag16h5"
                (rvec, tvec, markerPoints) = cv2.aruco.estimatePoseSingleMarkers(corners[i], self.tag_size_long, self.camera_matrix, self.distorion_coefs)
                #print("rvec: ", rvec)
                print("tvec: ", tvec)
                detection.pose.x = float(tvec[0][0][0])
                detection.pose.y = float(tvec[0][0][1])
                detection.pose.z = float(tvec[0][0][2])
                for j in range(4):
                    corner = corners[i][0][j]
                    point = Point()
                    point.x = float(corner[0])
                    point.y = float(corner[1])
                    #print(point)
                    corner_array.append(point)

                detection.corners = corner_array
        
                cX = int((corners[i][0][0][0] + corners[i][0][2][0]) / 2)
                cY = int((corners[i][0][0][1] + corners[i][0][2][1]) / 2)

                detection.centre = Point(x=float(cX), y=float(cY))

                detection_array.detections.append(detection)

            self.detection_data_publisher.publish(detection_array)

        # Draw AprilTags
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            cv2.drawFrameAxes(cv_image, self.camera_matrix, self.distorion_coefs, rvec, tvec, 0.01)

        # Publish Image with AprilTags
        self.detection_image_publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))


def main(args=None):
    rclpy.init(args=args)
    april_tag_detector = AprilTagDetector()
    rclpy.spin(april_tag_detector)
    april_tag_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

