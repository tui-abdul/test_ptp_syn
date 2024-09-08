## ROS2 Camera and LiDAR Synchronization Node

This tutorial provides an overview of the `CameraPublisher` node, which synchronizes and processes image and LiDAR data streams using ROS 2, OpenCV, Open3D, and `message_filters`. This node subscribes to camera and LiDAR topics, synchronizes them based on timestamps, and performs simple visualization.

### Features
- Subscribes to two image streams from Basler cameras and one LiDAR point cloud stream.
- Synchronizes the image and LiDAR messages using an approximate time synchronizer.
- Converts the image streams from ROS messages to OpenCV format.
- Publishes the synchronized LiDAR data and visualizes the images using OpenCV.

### Dependencies
Ensure you have the following dependencies installed before running the node:
- ROS 2 (Foxy, Galactic, Humble, etc.)
- OpenCV (`opencv-python`)
- Open3D (`open3d`)
- cv_bridge
- ros2_numpy
- message_filters
- sensor_msgs
- rviz2 (optional for visualizing in ROS)

Install the required ROS 2 Python packages:
```bash
sudo apt install ros-<ros2-distro>-cv-bridge
sudo apt install ros-<ros2-distro>-sensor-msgs
sudo apt install ros-<ros2-distro>-message-filters
```

You will also need to install `cv2`, `open3d`, and `ros2_numpy`:
```bash
pip install opencv-python open3d ros2-numpy
```

### Code Breakdown

#### 1. **Importing Required Libraries**
The code starts by importing libraries for ROS 2 message handling (`rclpy`, `sensor_msgs`), OpenCV (`cv2`), Open3D (`open3d`), and other dependencies.
```python
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
```

#### 2. **CameraPublisher Class**
The `CameraPublisher` class is derived from the `Node` class of ROS 2. It subscribes to the image and LiDAR topics, sets up the message synchronizer, and provides callback functionality.
```python
class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
```

#### 3. **Setting Up Publishers and Subscribers**
- **Publisher**: The node publishes LiDAR point cloud data to `/lidar/points`.
- **Subscribers**: Three subscribers are initialized to handle two image streams and one LiDAR point cloud stream.
```python
self.publisher_lidar = self.create_publisher(PointCloud2, '/lidar/points', 10)
image_sub = message_filters.Subscriber(self, Image, image_color)
image_sub1 = message_filters.Subscriber(self, Image, image_color1)
ouster_sub = message_filters.Subscriber(self, PointCloud2, ouster, qos_profile=qos.qos_profile_sensor_data)
```

#### 4. **Approximate Time Synchronizer**
The node uses `message_filters.ApproximateTimeSynchronizer` to synchronize the image and LiDAR topics, allowing for slight time differences between the messages. The `slop` parameter is set to 0.2 seconds.
```python
ats = message_filters.ApproximateTimeSynchronizer([image_sub, image_sub1, ouster_sub], queue_size=10, slop=0.2)
ats.registerCallback(self.callback)
```

#### 5. **Callback Function**
The callback function processes the synchronized messages:
- The image messages are converted to OpenCV images using `CvBridge`.
- LiDAR data is stored in `self.lidar_msg` and re-published.
- OpenCV is used to display the images in windows.
```python
def callback(self, image_msg, image_msg1, osuter_msg):
    self.get_logger().info('New message arrived')
    cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
    cv_image1 = self.bridge.imgmsg_to_cv2(image_msg1, desired_encoding='bgr8')

    cv2.namedWindow("Image 1", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Image 2", cv2.WINDOW_NORMAL)
    cv2.imshow("Image 1", cv_image)
    cv2.imshow("Image 2", cv_image1)

    # Close windows when 'q' is pressed
    while True:
        key = cv2.waitKey(0) & 0xFF
        if key == ord('q'):
            break
    cv2.destroyAllWindows()
```

#### 6. **Launching RViz2**
An attempt is made to launch RViz2 in the background to visualize the data in ROS 2. If RViz2 is not installed, an error is logged.
```python
try:
    subprocess.run(['rviz2'])
except FileNotFoundError:
    print("rviz2 is not installed or not found in the system PATH.")
```

#### 7. **Main Function**
The `main` function initializes the ROS 2 system, creates a `CameraPublisher` node, and spins it to keep it running.
```python
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
```

### Usage
1. Clone this repository or download the script to your ROS 2 workspace.
2. Source your ROS 2 environment:
   ```bash
   source /opt/ros/<ros2-distro>/setup.bash
   ```
3. Run the script:
   ```bash
   python3 camera_publisher.py
   ```

4. Ensure that the LiDAR and camera topics are publishing data.

### Conclusion
This node offers a simple way to synchronize image and point cloud data in ROS 2, while displaying images using OpenCV. The provided code can serve as a foundation for more complex computer vision or SLAM applications using synchronized sensor data.

