#!/usr/bin/env python3

import cv2
import depthai as dai
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy

class CameraNode(Node):
    
    def __init__(self):
        # Initialize the node
        super().__init__('forward_camera_node')

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
        self.camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)
        self.camRgb.setVideoSize(1280, 720)

        self.xoutVideo.input.setBlocking(False)
        self.xoutVideo.input.setQueueSize(1)

        # Linking
        self.camRgb.video.link(self.xoutVideo.input)


        #QoS Profile
        self.qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Create publisher
        self.video_pub = self.create_publisher(Image, 'oak_1/rgb/image_raw', 1)

        # Specify the MxID of the device to use
        self.device_info = dai.DeviceInfo("184430106162680F00") #MxID of the OAK-1

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
                # Visualizing the frame on slower hosts might have overhead
                cv2_frame = videoIn.getCvFrame()
                #cv2.imshow("video", cv2_frame)

                # Publish the frame to ROS2
                msg = self.bridge.cv2_to_imgmsg(cv2_frame, encoding="bgr8")
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera'
                self.video_pub.publish(msg)

                #if cv2.waitKey(1) == ord('q'):
                #    break

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()