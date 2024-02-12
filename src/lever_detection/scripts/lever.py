#!/usr/bin/env python3
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

class LeverDetector:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('lever_detector_node', anonymous=True)

        # Initialize CvBridge for image conversion
        
        self.bridge = CvBridge()

        # Initialize placeholder for displaying image
        self.display_image = np.ones((480,640,3), dtype=np.uint8)

         # Subscribe to image and point cloud topics
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.cloud_sub = rospy.Subscriber("/camera/depth_registered/points", PointCloud2, self.point_cloud_callback)
        
        # Publishers for lever pose and image
        self.pose_pub = rospy.Publisher("lever_pose", PoseStamped, queue_size=10)
        self.image_pub = rospy.Publisher("lever_image", Image, queue_size=10)

        # Load object detection model
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='src/lever_detection/model/door.pt')
        
        # Initialize variables for storing image and point cloud data        
        self.cv_image = None
        self.cloud_data = None

        # Initialize custom detector for 2D to 3D conversion
        self.td23D = CustomDetector()

        # Initialize tf2 buffer for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()

    def image_callback(self, data):
     # Callback function for processing incoming images

        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def point_cloud_callback(self, data):
    # Callback function for processing incoming point clouds

        try:
            self.cloud_data = data
        except ValueError as e:
            print(e)

    # Function to process object detection
    def process_detection(self):
        if self.cv_image is None or self.cloud_data is None:
            rospy.logwarn("Image or point cloud data is not yet available.")
            return

        # Perform object detection on the image
        results = self.model(self.cv_image, size=416)
        self.publish_image(results.render()[0])
        print(results.pandas().xyxy[0])
        detections = results.pandas().xyxy[0]
        detected = False

        # Iterate through detections and process poses
        bboxs = []
        for index, row in detections.iterrows():
            try:
                if row['class'] == 2:
                    detected= True
                    box = [[int(row['xmin']),int(row['ymin'])],[int(row['xmax']),int(row['ymax'])]]
                    bboxs.append(box)
            except:
                rospy.logerr("Failed to process detection!")

        if not detected:
            return 'failed'
        print("Detection DONE!")

        final_pose = None

        # Attempt to obtain 3D coordinates of detected objects
        for i in range(10):
            try:
                cloud = self.cloud_data
                whole, obj_clus = self.td23D.get_box_voxel(box, cloud)
                obj_pose = self.td23D.get_3D_cords(obj_clus)
                real_object_camera = self.transform_3D_to_camera([obj_pose])[0]
                final_pose = real_object_camera
                
                # Publish the final pose
                self.pose_pub.publish(final_pose)
                break
            except Exception as e:
                print(e)
                continue
        
        if final_pose is None:
            return 'failed'

        print(real_object_camera)

    # Function to display a point cloud
    def show_point_cloud(self, point, col=[1,1,1]):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector([point])
        pcd.colors = o3d.utility.Vector3dVector([np.array(col)])
        return pcd

    # Placeholder function for calculating 3D pose from detection
    def calculate_3d_pose(self, detection_row):
        return None

    # Function to publish images
    def publish_image(self, image):
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        image_message = self.bridge.cv2_to_imgmsg(image, encoding="passthrough")
        self.image_pub.publish(image_message)

    # Function to publish 3D poses
    def publish_pose(self, pose_3d):
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "BASE"
        pose_msg.pose.position = Point(pose_3d[0], pose_3d[1], pose_3d[2])
        pose_msg.pose.orientation = Quaternion(0, 0, 0, 1)
        self.pose_pub.publish(pose_msg)

    # Main function to run the lever detection node
    def run(self):
        rospy.loginfo('[Lever Detection] Attempting to detect nearest lever')
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.process_detection()
            rate.sleep()

    # Callback function for processing additional data
    def callback1(self, data):
        try:
            self.cloud_data = data
        except ValueError as e:
            print(e)

    # Function to transform 3D coordinates to camera frame
    def transform_3D_to_camera(self, obj_poses):
        transformed = []
        for obj_pose in obj_poses:
            x, y, z = obj_pose.pose.position.x, obj_pose.pose.position.y, obj_pose.pose.position.z
            obj_pose.header.frame_id = "camera_link"
            obj_pose.pose.position.x = y
            obj_pose.pose.position.y = z
            obj_pose.pose.position.z = x
            transformed.append(obj_pose)
        return transformed

    # Function to transform camera frame to base frame
    def transform_camera_to_base(self, obj_poses):
        transformed = []
        for obj_pose in obj_poses:
            transformation = self.tf_buffer.lookup_transform('BASE', "camera_link", rospy.Time())
            transformed_pose = tf2_geometry_msgs.do_transform_pose(obj_pose, transformation)
            transformed.append(transformed_pose)
        return transformed

if __name__ == '__main__':
    detector = LeverDetector()
    detector.run()
