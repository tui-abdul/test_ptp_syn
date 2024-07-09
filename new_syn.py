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
import ros2_numpy
import open3d as o3d
import subprocess

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')


        self.publisher_lidar = self.create_publisher(PointCloud2, '/lidar/points', 10)
        self.lidar_msg = PointCloud2()
        image_color ='/basler_pole_a_left_id_103_sn_603/my_camera/pylon_ros2_camera_node/image_raw'
        ouster = '/ouster_pole_a_1108/points'
        image_color1 ='/basler_pole_a_right_id_104_sn_618/my_camera/pylon_ros2_camera_node/image_raw'

        #image_color = '/my_camera/pylon_ros2_camera_node/image_raw'
        #ouster = '/ouster/points'
        # Subscribe to topics
        image_sub = message_filters.Subscriber(self, Image, image_color)
        image_sub1 = message_filters.Subscriber(self, Image, image_color1)
        ouster_sub = message_filters.Subscriber(self, PointCloud2, ouster,qos_profile= qos.qos_profile_sensor_data)

        # Synchronize the topics by time
        ats = message_filters.ApproximateTimeSynchronizer(
            [image_sub,image_sub1,ouster_sub], queue_size=10, slop=0.2)#, allow_headerless=True)
        ats.registerCallback(self.callback)

        self.bridge = CvBridge()
        self.get_logger().info('Initialization complete')

        try:
            subprocess.run(['rviz2'])
        except FileNotFoundError:
            print("rviz2 is not installed or not found in the system PATH.")
  

    def callback(self, image_msg,image_msg1 ,osuter_msg):
        self.get_logger().info('New message arrived')
        # Convert ROS Image messages to OpenCV images
        print('camera 1',image_msg.header.stamp.sec)
        print('camera 2',image_msg1.header.stamp.sec)
        print('lidar',osuter_msg.header.stamp.sec)
        self.lidar_msg = osuter_msg
        self.publisher_lidar.publish(self.lidar_msg)
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        cv_image1 = self.bridge.imgmsg_to_cv2(image_msg1, desired_encoding='bgr8')


        cv2.namedWindow("Image 1", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Image 2", cv2.WINDOW_NORMAL)
        # Process the images (example: show them)
        cv2.imshow("Image 1", cv_image)
        cv2.imshow("Image 2", cv_image1)

        #cv2.waitKey(15)  # Display images for a short period of time



        # Close all OpenCV windows

        '''
        
        vis = o3d.visualization.Visualizer()
        vis.create_window('edges')
        pc_as_numpy_array = np.array(ros2_numpy.point_cloud2.point_cloud2_to_array(osuter_msg)['xyz'])
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(pc_as_numpy_array  )
        # geometry is the point cloud used in your animaiton
        
        vis.add_geometry(point_cloud)
        #view_ctl = self.vis.get_view_control()
        #view_ctl.set_zoom(0.15)
        #coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
        opt = vis.get_render_option()
        opt.show_coordinate_frame = True
        #self.vis.add_geometry(coordinate_frame)
        
        #self.vis.update_geometry(point_cloud)
        #self.vis.poll_events()
        #self.vis.update_renderer()
        vis.run()
        vis.destroy_window()
        '''
        while True:
            key = cv2.waitKey(0) & 0xFF
            # Close the window when 'q' is pressed
            if key == ord('q'):
                break
        
        cv2.destroyAllWindows()

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
