#!/usr/bin/env python3
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from std_msgs.msg import Int32 # ArUco Zone for publisher
from cv_bridge import CvBridge, CvBridgeError # ROS2 package to convert between ROS and OpenCV Images
import cv2 # Python OpenCV library
import numpy as np

class ArucoDetection(Node):
    def __init__(self):
        super().__init__('detect_aruco')

        # Subscriber
        self.window_name = "ArUco position display"
        self.subscription = self.create_subscription(Image,'image_raw',self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.zones = {
            'center': 0,
            'upper': 1,
            'lower': 2,
        }
        self.zone = 0
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

        # Publisher
        queue_size = 10
        self.publisher_ = self.create_publisher(Int32, 'aruco_zone', queue_size)
        timer_period = 1/4  # time of callbacks: 4Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0


    def listener_callback(self, image_data):
        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(image_data, "bgr8")
        except CvBridgeError as e:
            print(e)
        # print(type(image_data), print(type(cv_image))) % sensor_msgs.msg.Image, np.array

        corners, ids, _ = cv2.aruco.detectMarkers(cv_image, self.aruco_dict)
        position_display = np.zeros((image_data.height,image_data.width,3), np.uint8)
        if(ids is not None):
            self.zone, found_corner = self.find_zone(cv_image, corners, ids)
            if(found_corner is not None):
                # Draws ArUco markers
                cv2.rectangle(position_display,(found_corner[0][0],found_corner[0][1]),(found_corner[3][0],found_corner[3][1]),(255,255,0),2)
        # Center line
        cv2.line(position_display,(0,image_data.height//2-1),(image_data.width-1,image_data.height//2-1),(255,255,0),2)
        cv2.imshow(self.window_name, position_display)
        cv2.waitKey(25)

    def verify_position(image, corners):
        position = ArucoDetection.zones['center']
        height = image.shape[0]
        width = image.shape[1]
        # print(width, height)
        image_center_y = height // 2 - 1
        image_center_x = width // 2 - 1
        if corners[0][1] > image_center_y and corners[1][1] > image_center_y: # Up left, Up right
            position = ArucoDetection.zones['lower']
        elif corners[2][1] < image_center_y and corners[3][1] < image_center_y:  # Down left, down right
            position = ArucoDetection.zones['upper']

        return position

    def find_zone(image, corners, ids):
        """Finds the zone - image placement of aruco with id 9.
        returns 0 when aruco is on the middle x axis,
                1 when aruco is above the middle x axis,
                2 when aruco is below the middle x axis
        returns corner coordinates of aruco marker, if no marker is found returns None"""
        zone = 0
        corners_id = 0
        found_corner = None
        for id in ids:
            if id == 9:
                found_corner = corners[corners_id][0]
                zone = ArucoDetection.verify_position(image, corners[corners_id][0])
                break
            else:
                corners_id += 1
                continue

        return zone, found_corner
    
    def timer_callback(self):
        publish_zone = Int32()
        # Floats
        publish_zone.data = self.zone
        self.publisher_.publish(publish_zone)
        self.get_logger().info('detect_aruco: zone "%d" published to /aruco_zone.' % publish_zone.data)
        self.i += 1 # Whatever




def main(args=None):
    rclpy.init(args=args)
    aruco_detection = ArucoDetection()
    rclpy.spin(aruco_detection)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    aruco_detection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
