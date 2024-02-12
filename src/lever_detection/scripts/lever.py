import rospy
import torch
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import open3d as o3d
from std_msgs.msg import Header
import tf2_ros
import tf2_geometry_msgs
import sys

sys.path.append('/home/maira/catkin_ws/src/lever_detection/scripts')

# Import custom module
from coordetector import t2d2t3d as CustomDetector

class LeverDetectionSystem:
    def __init__(self):
        # Initialize ROS node and necessary variables
        rospy.init_node('lever_detection_system', anonymous=True)
        self.bridge = CvBridge()
        self.display_image = np.ones((480, 640, 3), dtype=np.uint8)
        self.image_subscriber = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.cloud_subscriber = rospy.Subscriber("/camera/depth_registered/points", PointCloud2, self.point_cloud_callback)
        self.pose_publisher = rospy.Publisher("lever_pose", PoseStamped, queue_size=10)
        self.image_publisher = rospy.Publisher("lever_image", Image, queue_size=10)

        # Load YOLOv5 model for object detection
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='src/lever_detection/model/door.pt')
        self.cv_image = None
        self.cloud_data = None
        self.detector = CustomDetector()
       
        self.tf_buffer = tf2_ros.Buffer()

    def image_callback(self, data):
        # Callback function for image subscriber
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def point_cloud_callback(self, data):
        # Callback function for point cloud subscriber
        try:
            self.cloud_data = data
        except ValueError as e:
            print(e)

    def detect_lever(self):
        # Detect lever using YOLOv5 model and publish its pose
        if self.cv_image is None or self.cloud_data is None:
            rospy.logwarn("Image or point cloud data is not yet available.")
            return

        # Perform object detection using YOLOv5 model
        results = self.model(self.cv_image, size=416)
        self.publish_image(results.render()[0])
        detections = results.pandas().xyxy[0]
        detected = False

        bboxs = []
        for index, row in detections.iterrows():
            try:
                if row['class'] == 2:
                    detected = True
                    box = [[int(row['xmin']), int(row['ymin'])], [int(row['xmax']), int(row['ymax'])]]
                    bboxs.append(box)
            except:
                rospy.logerr("Detection failed!")
        
        if not detected:
            return 'failed'

        final_pose = None
        
        # Iterate through detected objects to find the lever
        for i in range(10):
            try:
                cloud = self.cloud_data
                whole, obj_clus = self.detector.get_box_voxel(box, cloud)
                obj_pose = self.detector.get_3D_coords(obj_clus)
                real_object_camera = self.transform_3D_to_camera([obj_pose])[0]
                final_pose = real_object_camera

                self.pose_publisher.publish(final_pose)
                break
            except Exception as e:
                print(e)
                continue
        
        if final_pose is None:
            return 'failed'

    def show_point_cloud(self, point, col=[1, 1, 1]):
        # Visualize a point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector([point])
        pcd.colors = o3d.utility.Vector3dVector([np.array(col)])
        return pcd

    def publish_image(self, image):
        # Publish an image
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        image_message = self.bridge.cv2_to_imgmsg(image, encoding="passthrough")
        self.image_publisher.publish(image_message)

    def publish_detected_pose(self, pose_3d):
        # Publish detected pose
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "BASE"
        pose_msg.pose.position = Point(pose_3d[0], pose_3d[1], pose_3d[2])
        pose_msg.pose.orientation = Quaternion(0, 0, 0, 1)
        self.pose_publisher.publish(pose_msg)

    def run_detection_system(self):
        # Main function to run lever detection system
        rospy.loginfo('[Lever Detection] Attempting to detect the nearest lever')
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.detect_lever()
            rate.sleep()

    def transform_3D_to_camera(self, obj_poses):
        # Transform 3D coordinates to camera frame
        transformed = []
        for obj_pose in obj_poses:
            x, y, z = obj_pose.pose.position.x, obj_pose.pose.position.y, obj_pose.pose.position.z
            obj_pose.header.frame_id = "camera_link"
            obj_pose.pose.position.x = y
            obj_pose.pose.position.y = z
            obj_pose.pose.position.z = x
            transformed.append(obj_pose)
        return transformed

    def transform_camera_to_base(self, obj_poses):
        # Transform camera coordinates to base frame
        transformed = []
        for obj_pose in obj_poses:
            transformation = self.tf_buffer.lookup_transform('BASE', "camera_link", rospy.Time())
            transformed_pose = tf2_geometry_msgs.do_transform_pose(obj_pose, transformation)
            transformed.append(transformed_pose)
        return transformed

if __name__ == '__main__':
    detection_system = LeverDetectionSystem()
    detection_system.run_detection_system()
