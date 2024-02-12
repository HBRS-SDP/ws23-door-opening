#!/usr/bin/env python3
import rospy
import torch
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String  # Simple string message for demonstration

# Load a pre-trained object detection model. Adjust path as necessary.
object_detection_model = torch.hub.load('ultralytics/yolov5', 'custom', path='src/lever_detection/model/door.pt')
bridge = CvBridge()

def detect_objects_and_draw_boxes(image):
    # Detect objects in the image using the loaded model
    result = object_detection_model(image, size=416)
    detection_results = result.pandas().xyxy[0]  # Extract detection results

    # Filter detection results for levers (assuming class '2' is for levers, adjust as necessary)
    lever_detections = detection_results[detection_results['class'] == 2]

    # Initialize an empty string to accumulate bounding box information
    detected_objects_str = ""

    # Process detected objects and prepare bounding box information
    for index, row in lever_detections.iterrows():
        xmin, ymin, xmax, ymax = int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax'])
        confidence = row['confidence']
        detected_objects_str += f"Lever detected with confidence {confidence:.2f} at [{xmin}, {ymin}, {xmax}, {ymax}]\n"

    # Return the string representation of detected objects
    return detected_objects_str

def image_callback(msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    detected_objects_str = detect_objects_and_draw_boxes(cv_image)

    # Publish the detected objects
    if detected_objects_str:
        objects_pub.publish(detected_objects_str)
    else:
        objects_pub.publish("No levers detected")

if __name__ == '__main__':
    rospy.init_node('object_detector_node')
    rospy.Subscriber("/image_raw", Image, image_callback)
    objects_pub = rospy.Publisher("/detected_objects", String, queue_size=10)

    rospy.spin()
