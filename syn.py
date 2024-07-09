import rclpy
import yaml
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from rclpy.node import Node
from builtin_interfaces.msg import Time
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2
from rclpy import  qos
import message_filters
from sensor_msgs_py.point_cloud2 import read_points


class camera_publisher(Node):

    def __init__(self):
        super().__init__('publisher')

    
        image_color = '/basler_pole_a_left_id_103_sn_603/my_camera/pylon_ros2_camera_node/image_raw'
        image_color1 = '/basler_pole_a_right_id_104_sn_618/my_camera/pylon_ros2_camera_node/image_raw'
        ouster = '/ouster_pole_a_1108/points'
                # Subscribe to topics
        image_sub = message_filters.Subscriber(self,Image,image_color)
        image_sub1 = message_filters.Subscriber(self,Image,image_color1)
        ouster = message_filters.Subscriber(self, PointCloud2,ouster,qos_profile= qos.qos_profile_sensor_data)

        # Synchronize the topics by time
        ats = message_filters.ApproximateTimeSynchronizer(
            [image_sub, image_sub1, ouster], queue_size=20, slop=0.10, allow_headerless=True)
        ats.registerCallback(self.callback)
        self.bridge = CvBridge()
        print('it works till here')
    
    
    def callback(self, image_msg,image_msg1,lidar_msg):
        print('new msg arrived')
        

def main(args=None):
    rclpy.init(args=args)
    publisher = camera_publisher()

    rclpy.spin( publisher  )       # execute simple_node 

    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()