#!/usr/bin/env python3
###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2019 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import rospy
import time
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from kortex_driver.msg import TwistCommand
import kortex_driver
from kortex_driver.srv import *
from kortex_driver.msg import *
import numpy as np
import math


class DoorOpening:
    def __init__(self):
        try:
            rospy.init_node('door_opening')

            self.HOME_ACTION_IDENTIFIER = 2

            self.action_topic_sub = None

            self.force_estimator_sub= None

            self.all_notifs_succeeded = True

            self.all_notifs_succeeded = True

            self.door_configuration = "push"


            #Get node params
            self.robot_name = rospy.get_param('~robot_name', "my_gen3")
            self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 7)
            self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", False)
            rospy.loginfo("Using robot_name " + self.robot_name)
            self._force = {'x': [], 
                       'y': [], 
                       'z': [],
                       't_x': [],
                       't_y': [],
                       't_z': []}
            # Init the action topic subscriber
            self._force_subscriber = rospy.Subscriber("/my_gen3/base_feedback", kortex_driver.msg.BaseCyclic_Feedback, self._force_callback)
            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
            self.last_action_notif_type = None
            


            # Init the services
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
            rospy.wait_for_service(set_cartesian_reference_frame_full_name)
            self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

            send_gripper_command_full_name = '/' + self.robot_name + '/base/send_gripper_command'
            rospy.wait_for_service(send_gripper_command_full_name)
            self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)
        except:
            self.is_init_success = False
        else:
            self.is_init_success = True


    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                self.all_notifs_succeeded = False
                return False
            else:
                time.sleep(0.01)

    def example_clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True


    def example_set_cartesian_reference_frame(self):
        # Prepare the request with the frame we want to set
        req = SetCartesianReferenceFrameRequest()
        req.input.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED

        # Call the service
        try:
            self.set_cartesian_reference_frame()
            print("the frame of refernce is ",req.input.reference_frame)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SetCartesianReferenceFrame")
            return False
        else:
            rospy.loginfo("Set the cartesian reference frame successfully")
            return True

        # Wait a bit
        rospy.sleep(0.25)

    def move_to_cartesian_pose(self,pose,translation_speed,orientation_speed):
        req1 = SetCartesianReferenceFrameRequest()
        req1.input.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED
        self.cartesian_speed = CartesianSpeed()
        self.cartesian_speed.translation = translation_speed # m/s
        self.cartesian_speed.orientation = orientation_speed  # deg/s
        pose.constraint.oneof_type.speed.append(self.cartesian_speed)
        req = ExecuteActionRequest()
        req.input.oneof_action_parameters.reach_pose.append(pose)
        # self.strg=str(val)
        # req.input.name = "pose"+self.strg
        req.input.handle.action_type = ActionType.REACH_POSE
        req.input.handle.identifier = 1001

        rospy.loginfo("Sending pose ...")
        self.last_action_notif_type = None
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to send  current pose ")
            return False
        else:
            rospy.loginfo("Waiting for the current pose to finish...")
        self.wait_for_action_end_or_abort()
        return True
            

        

    def gripper_command(self, value):
         # Initialize the request
        # Close the gripper
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        rospy.loginfo("Sending the gripper command...")

        # Call the service 
        try:
            self.send_gripper_command(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False
        else:
            time.sleep(0.5)
            return True

    def example_subscribe_to_a_robot_notification(self):
        # Activate the publishing of the ActionNotification
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
            return False
        else:
            rospy.loginfo("Successfully activated the Action Notifications!")

        rospy.sleep(1.0)

        return True

    def pose_assigner(self,xyz_dict):
        self.co_ordinate=ConstrainedPose()
        self.co_ordinate.target_pose.x = xyz_dict["x"]
        self.co_ordinate.target_pose.y =xyz_dict["y"]
        self.co_ordinate.target_pose.z =xyz_dict["z"]
        self.co_ordinate.target_pose.theta_x = xyz_dict["theta_x"]
        self.co_ordinate.target_pose.theta_y = xyz_dict["theta_y"]
        self.co_ordinate.target_pose.theta_z = xyz_dict["theta_z"]
        val =self.move_to_cartesian_pose(self.co_ordinate,1.5,15)
        return val

    def base_movement(self):
        responce=input("Move the base to the required pose type(yes/no)")

        if responce=="yes":
            return True
        elif responce=="no":
            return False
        else:
            return False
        
 

    def main(self):
        # For testing purposes
        success = self.is_init_success
        try:
            rospy.delete_param("/kortex_examples_test_results/cartesian_poses_with_notifications_python")
        except:
            pass

        if success:

            #*******************************************************************************
            # Make sure to clear the robot's faults else it won't move if it's already in fault
            success &= self.example_clear_faults()
            #*******************************************************************************
            
            #*******************************************************************************
            # Set the reference frame to "Mixed"
            # success &= self.example_set_cartesian_reference_frame()

            #*******************************************************************************
            # Subscribe to ActionNotification's from the robot to know when a cartesian pose is finished
            success &= self.example_subscribe_to_a_robot_notification()
            

            #*******************************************************************************

            target_pose = {
                    "x": 0,
                    "y": 0,
                    "z": 0,
                    "theta_x": 0,
                    "theta_y": 0,
                    "theta_z": 0
                }
            #for home pose
            target_pose["x"] = 1
            target_pose["y"] = 2
            target_pose["z"] = 3
            target_pose["theta_x"] = 4
            target_pose["theta_y"] = 5
            target_pose["theta_z"] = 6


            #for towards door pose
            target_pose["x"] = 1
            target_pose["y"] = 2
            target_pose["z"] = 3
            target_pose["theta_x"] = 4
            target_pose["theta_y"] = 5
            target_pose["theta_z"] = 6

                

            success &= self.all_notifs_succeeded

            # success &= self.all_notifs_succeeded


        # For testing purposes
        rospy.set_param("/kortex_examples_test_results/cartesian_poses_with_notifications_python", success)

        if not success:
            rospy.logerr("The example encountered an error.")

if __name__ == "__main__":
    ex = DoorOpening()
    rate = rospy.Rate(10)  # 10 Hz
    
    while not rospy.is_shutdown():
        ex.main()
        rate.sleep()