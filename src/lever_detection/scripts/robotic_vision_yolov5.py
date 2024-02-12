#!/usr/bin/env python3 

import rospy
import ros_numpy
import numpy as np
import io
import cv2
import tf
import yolov5
import rospkg 
import torch

from PIL import Image as img
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


class RoboticVision():

    def __init__(self) -> None: 

        self.rospack = rospkg.RosPack()
        self.marker = Marker()
        self.uv_to_xyz = PinholeCameraModel()
        self.bridge = CvBridge()
        self.object_transformer = tf.TransformBroadcaster()

        # Yolov5
        rospy.loginfo_once('Initializing Yolov5')
        rospy.loginfo_once('GPU is being used: ' + str(torch.cuda.is_available()))
        self.model = yolov5.load('/home/maira/catkin_ws/src/lever_detection/model/door.pt')
        # self.model.cuda()
        self.model.conf = 0.15  # NMS confidence threshold
        self.model.iou = 0.25  # NMS IoU threshold
        self.model.agnostic = True  # NMS class-agnostic
        self.model.multi_label = True  # NMS multiple labels per box
        self.model.max_det = 50  # maximum number of detections per image
        rospy.loginfo_once('Complete initialization of Yolov5')

        self.target_object = None 
        self.bounding_box = None
        self.depth = None
        self.b_img = None

        self.image_received       = False
        self.camera_info_received = False
        self.need_camera_info     = True
        self.depth_image_received = False
        self.perception_pipeline  = False
        self.detected_something   = False
        
    def start_robovision(self):

        # Subscribers
        rospy.Subscriber("/camera/depth_registered/sw_registered/image_rect_raw", Image, self.depth_image_callback)
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback)
        self.pose_pub = rospy.Publisher("lever_pose", PoseStamped, queue_size=10)
        # Publishers
        # self.pub_tray_marker = rospy.Publisher("tray_marker", Marker, queue_size = 2)   
        # tray_marker = self.target_marker(frame='zed2i_left_camera_optical_frame', x=x_m, y=y_m, z=z_m)
        # self.pub_tray_marker.publish(tray_marker)  
        self.pub_yolov5_img = rospy.Publisher('/lever_image', Image, queue_size=1)

        while not rospy.is_shutdown():
            self.detect_yolov5() 

    def detect_yolov5(self):

       

        # Check if camera is on
        if self.image_received:
            self.results = self.model(self.image)
            self.df_detection = self.results.pandas().xyxy[0].to_dict(orient="records")

            # Check if detection is according to requirment
            if self.results:
                rospy.loginfo_once('Detection Successful!')
                self.detection_cycle(['lever'])
                    
    def detection_cycle(self, objs):

        for obj in objs:
            self.compute_spatial_coordinates(obj) 

        self.detected_something = True   
        self.b_img = self.image

        results = self.results.xyxy[0]
        df_detection = self.df_detection
        b_img = self.b_img
        bridge = self.bridge

        for i, box in enumerate(results): 
            xB = int(box[2])
            xA = int(box[0])
            yB = int(box[3])
            yA = int(box[1])
            b_img = cv2.rectangle(b_img, (xA, yA), (xB, yB), (0,255,94), 2)

            detected_objs = df_detection[i]
            b_img = cv2.putText(b_img, detected_objs['name'] + ': ' + str(round(detected_objs['confidence'], 2)), 
                                (xA, yA), cv2.FONT_HERSHEY_DUPLEX, 0.3, (255,255,255), 1)

        rospy.loginfo_once('Publishing detection data!')
        self.pub_yolov5_img.publish(bridge.cv2_to_imgmsg(b_img))

    def compute_spatial_coordinates(self, required_object):

        if (self.depth_image_received == True) and (self.camera_info_received == True):
            # rospy.loginfo_throttle('inside compute spatial!')
            self.perception_pipeline = True

            # Skip if obj not available
            if required_object not in [obj['name'] for obj in self.df_detection]:
                rospy.loginfo_throttle(30, required_object + ' not detected!')
                return

            for object in self.df_detection:

                if object['name'] == required_object:

                    det_obj_x_pos = (object['xmin'] + object['xmax'])/2
                    det_obj_y_pos = (object['ymin'] + object['ymax'])/2
                    # Get Cameraframe to TF and Depth position
                    x_m, y_m, z_m = self.conversion_pixel_to_pointcloud([det_obj_x_pos, det_obj_y_pos])

                    # Publish object markers and transforms for picking
                    lever_pose=self.create_pose(x=x_m,y=y_m,z=z_m,frame_id='camera_link')
                    self.pose_pub.publish(lever_pose)
                    # self.transform_from_camera_to_arm(x=x_m,y=y_m,z=z_m,camera_frame='camera_link', tf_name=required_object)                          
        else:
            rospy.loginfo_throttle(3, required_object + ' not detected!')

    def create_pose(self,x,y,z,frame_id,ox=0.0,oy=0.0,oz=0.0,ow=1.0):
        pose_obj = PoseStamped()
        pose_obj.header.frame_id = frame_id
        pose_obj.header.stamp = rospy.Time().now()
        pose_obj.pose.position.x = x
        pose_obj.pose.position.y = y
        pose_obj.pose.position.z = z
        pose_obj.pose.orientation.x = ox
        pose_obj.pose.orientation.y = oy
        pose_obj.pose.orientation.z = oz
        pose_obj.pose.orientation.w = ow
        return pose_obj
            
    def conversion_pixel_to_pointcloud(self, pixel):

        depth_value = self.depth[int(pixel[1])][int(pixel[0])]
        # depth_value_meters =  depth_value/1000
        depth_value_meters =  depth_value

        xy_coord = self.uv_to_xyz.projectPixelTo3dRay(tuple(pixel)) 
        xyz_coord = [x * (depth_value_meters/xy_coord[2]) for x in xy_coord]

        return xyz_coord[0], xyz_coord[1], xyz_coord[2]

    def view_image(self, image):

        cv2.imshow('img', image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()   

    def check_depth_point(self, depth, pt):

        img = np.copy(depth)
        img.setflags(write=1)
        img[int(pt[1])][int(pt[0])] = -2000
        self.view_image(img) 

    def transform_from_camera_to_arm(self, x, y, z, camera_frame, tf_name):
        
        self.object_transformer.sendTransform((x, y, z),
                     [0,0,0,1],
                     rospy.Time.now(), tf_name, camera_frame)
    
    def check_image_information(self, img):

        print(f"Image resolution: {img.shape}")
        print(f"Data type: {img.dtype}")
        print(f"Min value: {np.min(img)}")
        print(f"Max value: {np.max(img)}") 
    
    def target_marker(self, frame, marker=1, x=0.0, y=0.0, z=0.0, q1=0.0, q2=0.0, q3=0.0, q4=1.0, c=(255,0,0,0.5), ns=''):

        self.marker.header.frame_id = frame
        self.marker.header.stamp = rospy.Time.now()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        self.marker.type = marker
        self.marker.id = 0
        self.marker.ns = ns

        # Set the scale of the marker
        self.marker.scale.x = 0.02
        self.marker.scale.y = 0.02
        self.marker.scale.z = 0.01

        # Set the color
        self.marker.color.r = c[0]
        self.marker.color.g = c[1]
        self.marker.color.b = c[2]
        self.marker.color.a = c[3]

        # Set the pose of the marker
        self.marker.pose.position.x = x
        self.marker.pose.position.y = y
        self.marker.pose.position.z = z
        self.marker.pose.orientation.x = q1
        self.marker.pose.orientation.y = q2
        self.marker.pose.orientation.z = q3
        self.marker.pose.orientation.w = q4

        return self.marker

    def pointcloud_callback(self, data):

        point_cloud = ros_numpy.numpify(data)
        height = point_cloud.shape[0]
        width = point_cloud.shape[1]
        np_points = np.zeros((height * width, 3), dtype=np.float32)

        np_points[:, 0] = np.resize(point_cloud['x'], height * width)
        np_points[:, 1] = np.resize(point_cloud['y'], height * width)
        np_points[:, 2] = np.resize(point_cloud['z'], height * width)

        self.numpy_pointcloud = np_points

    def depth_image_callback(self, depth_image):

        rospy.loginfo_once('Received Depth Image!')        
        depth_cv  = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding='passthrough')

        if self.image_received == True:
            self.depth = cv2.resize(depth_cv, (self.image_cols, self.image_rows))
            self.depth_image_received = True

    def image_callback(self, data):

        rospy.loginfo_once('Received RGB Image!') 
        # unmodified_image  = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        self.image  = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        self.image_rows, self.image_cols, _ = self.image.shape

        # Crop the image according to training
        # w = 640
        # h = 640
        # x = self.image_cols/2 - w/2
        # y = self.image_rows/2 - h/2
        # self.image = unmodified_image[int(y):int(y+h), int(x):int(x+w)]
        self.image_received = True

    def camera_info_callback(self, data):

        if self.need_camera_info == True:
            rospy.loginfo_once('Received Camera Model Info!') 
            self.uv_to_xyz.fromCameraInfo(data)
            self.camera_info_received = True
            self.need_camera_info = False

    def run(self):

        self.start_robovision()
        
if __name__ == "__main__":

    rospy.init_node('robotic_vision_preprocessing', anonymous=True)

    robov = RoboticVision()
    robov.run()

    rospy.spin()