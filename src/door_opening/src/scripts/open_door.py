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

from kortex_driver.msg import TwistCommand
import kortex_driver
from kortex_driver.srv import *
from kortex_driver.msg import *
import numpy as np


class DoorOpening:
    def __init__(self):
        try:
            rospy.init_node('door_opening')

            self.HOME_ACTION_IDENTIFIER = 2

            self.action_topic_sub = None

            self.force_estimator_sub= None

            self.all_notifs_succeeded = True

            self.all_notifs_succeeded = True

            # Get node params
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

    def _force_callback(self, msg):

        self._force['x'].append(msg.base.tool_external_wrench_force_x)
        self._force['y'].append(msg.base.tool_external_wrench_force_y)
        self._force['z'].append(msg.base.tool_external_wrench_force_z)
        self._force['t_x'].append(msg.base.tool_external_wrench_torque_x)
        self._force['t_y'].append(msg.base.tool_external_wrench_torque_y)
        self._force['t_z'].append(msg.base.tool_external_wrench_torque_z)
        # self.checker=self._force['z']
        # if len(self.checker) >20:
        #     val=np.average(self.checker)
        #     print("Average",val)
        #     if abs(val) >10:
        #         print("high force")
        #         self.checker=[]
        #         print(len(self.checker))
        #     else:
        #         print("normal force")
        #         self.checker=[]
        #         print(len(self.checker))    


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

    def move_to_cartesian_pose(self,pose,val,translation_speed,orientation_speed):
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
    
    def open_door_clockwise(self):
        towards_door_pose = ConstrainedPose()
        towards_door_pose.target_pose.x = 0.454
        towards_door_pose.target_pose.y = 0.776
        towards_door_pose.target_pose.z = 0.661
        towards_door_pose.target_pose.theta_x = 55.529
        towards_door_pose.target_pose.theta_y = -14.266
        towards_door_pose.target_pose.theta_z = 165.524

        self.move_to_cartesian_pose(towards_door_pose,2,0.5,15)

        # Let's close the gripper at 50%
        if self.is_gripper_present:
            success &= self.gripper_command(1.0)
            time.sleep(0.5)
        else:
            rospy.logwarn("No gripper is present on the arm.") 

        #unlatch the door
        
        unlatch_door = ConstrainedPose()
        unlatch_door.target_pose.x = 0.419
        unlatch_door.target_pose.y = 0.788
        unlatch_door.target_pose.z = 0.664
        unlatch_door.target_pose.theta_x = 58.568
        unlatch_door.target_pose.theta_y = -8.615
        unlatch_door.target_pose.theta_z = 158.7

        self.move_to_cartesian_pose(unlatch_door,2,0.5,15)
            

        open_door_pose = ConstrainedPose()
        open_door_pose.target_pose.x = 0.383
        open_door_pose.target_pose.y = 0.527
        open_door_pose.target_pose.z = 0.51
        open_door_pose.target_pose.theta_x = 36.967
        open_door_pose.target_pose.theta_y = -3.882
        open_door_pose.target_pose.theta_z = 160.227

        self.move_to_cartesian_pose(open_door_pose,2,0.5,15)

    


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

            # Example of gripper command
            # Let's fully open the gripper
            # if self.is_gripper_present:
            #     success &= self.gripper_command(0.0)
            # else:
            #     rospy.logwarn("No gripper is present on the arm.")  

            # print("gripper should have closed")
            # time.sleep(0.05)

            # # Example of gripper command
            # # Let's close the gripper at 50%
            # if self.is_gripper_present:
            #     success &= self.gripper_command(1.0)
            #     time.sleep(0.5)
            # else:
            #     rospy.logwarn("No gripper is present on the arm.")   


            my_constrained_pose = ConstrainedPose()
            my_constrained_pose.target_pose.x = 0.398
            my_constrained_pose.target_pose.y = 0.598
            my_constrained_pose.target_pose.z = 0.662
            my_constrained_pose.target_pose.theta_x = 172.677
            my_constrained_pose.target_pose.theta_y = -159.219
            my_constrained_pose.target_pose.theta_z = 78.821

            success &=self.move_to_cartesian_pose(my_constrained_pose,2,0.5,15)
            print("start")
            towards_door_pose = ConstrainedPose()
            towards_door_pose.target_pose.x = 0.454
            towards_door_pose.target_pose.y = 0.776
            towards_door_pose.target_pose.z = 0.661
            towards_door_pose.target_pose.theta_x = 55.529
            towards_door_pose.target_pose.theta_y = -14.266
            towards_door_pose.target_pose.theta_z = 165.524

            success &=self.move_to_cartesian_pose(towards_door_pose,2,0.5,15)

            # Let's close the gripper at 50%
            if self.is_gripper_present:
                success &= self.gripper_command(1.0)
                time.sleep(0.5)
            else:
                rospy.logwarn("No gripper is present on the arm.") 

            #unlatch the door
            
            unlatch_door = ConstrainedPose()
            unlatch_door.target_pose.x = 0.419
            unlatch_door.target_pose.y = 0.788
            unlatch_door.target_pose.z = 0.664
            unlatch_door.target_pose.theta_x = 58.568
            unlatch_door.target_pose.theta_y = -8.615
            unlatch_door.target_pose.theta_z = 158.7

            success &=self.move_to_cartesian_pose(unlatch_door,2,0.5,15)
                

            open_door_pose = ConstrainedPose()
            open_door_pose.target_pose.x = 0.383
            open_door_pose.target_pose.y = 0.527
            open_door_pose.target_pose.z = 0.51
            open_door_pose.target_pose.theta_x = 36.967
            open_door_pose.target_pose.theta_y = -3.882
            open_door_pose.target_pose.theta_z = 160.227

            success &=self.move_to_cartesian_pose(open_door_pose,2,0.5,15)

            success &= self.all_notifs_succeeded

            success &= self.all_notifs_succeeded

        # For testing purposes
        rospy.set_param("/kortex_examples_test_results/cartesian_poses_with_notifications_python", success)

        if not success:
            rospy.logerr("The example encountered an error.")

if __name__ == "__main__":
    ex = DoorOpening()
    ex.main()