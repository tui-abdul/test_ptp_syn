import rclpy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from rclpy.node import Node
from builtin_interfaces.msg import Time
import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy import qos
import message_filters
from sensor_msgs_py.point_cloud2 import read_points

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        image_color ='/basler_pole_a_left_id_103_sn_603/my_camera/pylon_ros2_camera_node/image_raw'
        image_color1 ='/basler_pole_a_right_id_104_sn_618/my_camera/pylon_ros2_camera_node/image_raw'
        #ouster = '/ouster_pole_a_1108/points'

        # Subscribe to topics
        image_sub = message_filters.Subscriber(self, Image, image_color)
        image_sub1 = message_filters.Subscriber(self, Image, image_color1)
        #ouster_sub = message_filters.Subscriber(self, PointCloud2, ouster,qos_profile= qos.qos_profile_sensor_data)

        # Synchronize the topics by time
        ats = message_filters.ApproximateTimeSynchronizer(
            [image_sub, image_sub1], queue_size=10, slop=0.20, allow_headerless=True)
        ats.registerCallback(self.callback)

        self.bridge = CvBridge()
        self.get_logger().info('Initialization complete')

    def callback(self, image_msg, image_msg1):
        self.get_logger().info('New message arrived')
        # Convert ROS Image messages to OpenCV images
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        cv_image1 = self.bridge.imgmsg_to_cv2(image_msg1, desired_encoding='bgr8')


        cv2.namedWindow("Image 1", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Image 2", cv2.WINDOW_NORMAL)
        # Process the images (example: show them)
        cv2.imshow("Image 1", cv_image)
        cv2.imshow("Image 2", cv_image1)

        cv2.waitKey(1)  # Display images for a short period of time
        #self.get_logger().info(f'PointCloud received with {len(pointcloud2_msg)} points')

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()

    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
