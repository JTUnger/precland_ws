#!/usr/bin/env python3

import cv2
import depthai as dai
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class CameraNode(Node):
    
    def __init__(self):
        # Initialize the node
        super().__init__('downward_camera_node')

        # Create self.pipeline
        self.pipeline = dai.Pipeline()

        # Create CvBridge
        self.bridge = CvBridge()

        # Define source and output
        self.camRgb = self.pipeline.create(dai.node.ColorCamera)
        self.xoutVideo = self.pipeline.create(dai.node.XLinkOut)
        self.xoutVideo.setStreamName("video")

        # Properties
        self.camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        self.camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        self.camRgb.setVideoSize(1280, 720)

        self.xoutVideo.input.setBlocking(False)
        self.xoutVideo.input.setQueueSize(1)

        # Linking
        self.camRgb.video.link(self.xoutVideo.input)

        # Create publishers
        self.video_pub = self.create_publisher(Image, 'oak_1_w/rgb/image_raw', 1)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'oak_1_w/data/camera_info', 1)

        # Specify the MxID of the device to use
        self.device_info = dai.DeviceInfo("18443010215644F500") #MxID of the OAK-1

        # Connect to device and start self.pipeline
        with dai.Device(self.pipeline, self.device_info) as device:
            #Print device info
            self.get_logger().info("Device Name: " + device.getDeviceInfo().name)
            self.get_logger().info("USB Speed: " + str(device.getUsbSpeed()))
            self.get_logger().info("MXID: " + device.getMxId())

            video = device.getOutputQueue(name="video", maxSize=1, blocking=False)

            while True:
                videoIn = video.get()

                # Get BGR frame from NV12 encoded video frame to show with opencv
                cv2_frame = videoIn.getCvFrame()

                # Publish the frame to ROS2
                msg = self.bridge.cv2_to_imgmsg(cv2_frame, encoding="bgr8")
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera'
                msg.width = videoIn.getWidth()
                msg.height = videoIn.getHeight()
                self.video_pub.publish(msg)

                # Publish camera info
                camera_info_msg = CameraInfo()
                camera_info_msg.header.stamp = self.get_clock().now().to_msg()
                camera_info_msg.header.frame_id = 'camera'
                camera_info_msg.width = videoIn.getWidth()
                camera_info_msg.height = videoIn.getHeight()
                camera_info_msg.distortion_model = 'plumb_bob'
                camera_info_msg.d = [-0.2627985027226353, 0.02853855756301266, -0.012800864717835954, 0.0005381702806697179, 0.0]
                camera_info_msg.k = [1132.8096975185235, 0.0, 580.5210235451025, 0.0, 1121.8161481549002, 412.9899959358472, 0.0, 0.0, 1.0]
                camera_info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
                camera_info_msg.p = [1016.205810546875, 0.0, 565.3316306687339, 0.0, 0.0, 1090.0118408203125, 410.6618227733852, 0.0, 0.0, 0.0, 1.0, 0.0]

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()