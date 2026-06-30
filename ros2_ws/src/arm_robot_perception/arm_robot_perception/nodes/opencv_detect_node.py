## It should  subscribe 
##        image/depth/camera_info
##        publish detections
##        load parameters

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, PointStamped
from rclpy.action import ActionClient
import tf2_ros
from cv_bridge import CvBridge
from arm_robot_perception.detectors.opencv_detect import OpenCVDetect
from arm_robot_interfaces.msg import DetectedObject

class OpenCVDetectNode(Node):
    def __init__(self):
        super().__init__('opencv_detect_node')

        self.latest_image = None
        self.latest_depth_image = None
        self.latest_camera_info = None
        self.bridge = CvBridge()
        
        # Subscribers
        self.image_subscriber = self.create_subscription(
            Image,
            '/overhead_camera/image',
            self.image_callback,
            10
        )
        
        self.depth_subscriber = self.create_subscription(
            Image,
            '/overhead_camera/depth_image',
            self.depth_callback,
            10
        )
        
        self.camera_info_subscriber = self.create_subscription(
            CameraInfo,
            '/overhead_camera/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Publisher
        self.detections_pose_publisher = self.create_publisher(DetectedObject, 'arm_robot/detections', 10)
        self.image_publisher = self.create_publisher(Image, '/arm_robot/debug_image', 10)
        
        # Load parameters (example)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.is_navigating = False
        self.declare_parameter('detection_threshold', 1000)
        self.detection_threshold = self.get_parameter('detection_threshold').get_parameter_value().double_value
        
    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    def depth_callback(self, msg):
        self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        
    def camera_info_callback(self, msg):
        self.latest_camera_info = msg

    def timer_callback(self):
        if self.latest_image is None:
            self.get_logger().info("No latest image received yet")
            return

        if self.latest_depth_image is None:
            self.get_logger().info("No latest depth image received yet")
            return

        if self.latest_camera_info is None:
            self.get_logger().info("No latest camera info received yet")
            return
    
        self.get_logger().info("Image received, processing for part detection")
        detections, debug_image = OpenCVDetect(detection_threshold=self.detection_threshold).detect(self.latest_image, self.latest_depth_image, self.latest_camera_info)

        debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
        debug_msg.header.stamp = self.get_clock().now().to_msg()
        debug_msg.header.frame_id = "overhead_camera/camera/rgbd_camera"
        self.image_publisher.publish(debug_msg)

        self.get_logger().info(f"Detections: {len(detections)}")
        self.send_navigation_point(detections)

    def send_navigation_point(self, detections):
        for detection in detections:
            detected_object = DetectedObject()
            detected_object.header.stamp = rclpy.time.Time().to_msg()
            detected_object.header.frame_id = "overhead_camera/camera/rgbd_camera"
            detected_object.color = detection["color"]
            detected_object.shape = detection["shape"]
            detected_object.angle = detection["angle"]
            detected_object.point.x = detection["x"]
            detected_object.point.y = detection["y"]
            detected_object.point.z = detection["z"]
            detected_object.confidence = 1.0

            self.detections_pose_publisher.publish(detected_object)  # Publish the detected point

def main(args=None):
      rclpy.init(args=args)
      node = OpenCVDetectNode()
      rclpy.spin(node)
      node.destroy_node()
      rclpy.shutdown()

if __name__ == "__main__":
    main()
