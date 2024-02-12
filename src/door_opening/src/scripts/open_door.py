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
from rospy import ServiceException
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

            self.flag=False

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
            print("Works1")
            self.pose_subscriber = rospy.Subscriber('/lever_pose',  PoseStamped, self.pose_conversion_callback)
            self.pose = tuple()
            print("check")

            #Init publishers
            # cartesian velocity publislher
            self.cartesian_velocity_pub = rospy.Publisher('/my_gen3/in/cartesian_velocity', TwistCommand, queue_size=1)


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


    def pose_conversion_callback(self,pose_msg):
        self.flag=True
        print(pose_msg)
        self.pose = self.get_kinovapose_from_pose_stamped(pose_msg)  
        print(self.pose)

    def get_kinovapose_from_pose_stamped(self,pose: PoseStamped):
        '''
        Converts a PoseStamped message to a KinovaPose.

        input: pose (PoseStamped): The PoseStamped message.
        '''

        # convert the quaternion to euler angles
        quaternion = (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
        )
        euler = euler_from_quaternion(quaternion)

        # convert the euler angles to degrees
        theta_x_deg = math.degrees(euler[0])
        theta_y_deg = math.degrees(euler[1])
        theta_z_deg = math.degrees(euler[2])

        return tuple((pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, theta_x_deg, theta_y_deg, theta_z_deg))


    def _force_callback(self, msg):
        self.average_threshold=25
        self.data_size_limit = 10
        self._force['x'].append(abs(msg.base.tool_external_wrench_force_x))
        self._force['y'].append(abs(msg.base.tool_external_wrench_force_y))
        self._force['z'].append(abs(msg.base.tool_external_wrench_force_z))
        self._force['t_x'].append(abs(msg.base.tool_external_wrench_torque_x))
        self._force['t_y'].append(abs(msg.base.tool_external_wrench_torque_y))
        self._force['t_z'].append(abs(msg.base.tool_external_wrench_torque_z))
        # Keep only the last 10 elements in each list
        for axis in self._force:
            self._force[axis] = self._force[axis][-self.data_size_limit:]

        # Check if the average force exceeds the threshold
        for axis in self._force:
            average_force = sum(self._force[axis]) / len(self._force[axis])
            if average_force > self.average_threshold:
                rospy.logwarn(f"Warning: Average force on axis {axis} exceeds. Average force: {average_force}N")
                self.door_configuration ="pull"
                print(self.door_configuration)



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
    
    # def open_door_clockwise(self):
    #     try:

    #         towards_door_pose = ConstrainedPose()
    #         towards_door_pose.target_pose.x = 0.454
    #         towards_door_pose.target_pose.y = 0.776
    #         towards_door_pose.target_pose.z = 0.661
    #         towards_door_pose.target_pose.theta_x = 55.529
    #         towards_door_pose.target_pose.theta_y = -14.266
    #         towards_door_pose.target_pose.theta_z = 165.524

    #         self.move_to_cartesian_pose(towards_door_pose,0.5,15)

    #         # Let's close the gripper at 50%
    #         if self.is_gripper_present:
    #             self.gripper_command(1.0)
    #             time.sleep(0.5)
    #         else:
    #             rospy.logwarn("No gripper is present on the arm.") 

    #         #unlatch the door
            
    #         unlatch_door = ConstrainedPose()
    #         unlatch_door.target_pose.x = 0.419
    #         unlatch_door.target_pose.y = 0.788
    #         unlatch_door.target_pose.z = 0.664
    #         unlatch_door.target_pose.theta_x = 58.568
    #         unlatch_door.target_pose.theta_y = -8.615
    #         unlatch_door.target_pose.theta_z = 158.7

    #         self.move_to_cartesian_pose(unlatch_door,0.5,15)
                

    #         open_door_pose = ConstrainedPose()
    #         open_door_pose.target_pose.x = 0.383
    #         open_door_pose.target_pose.y = 0.527
    #         open_door_pose.target_pose.z = 0.51
    #         open_door_pose.target_pose.theta_x = 36.967
    #         open_door_pose.target_pose.theta_y = -3.882
    #         open_door_pose.target_pose.theta_z = 160.227

    #         self.move_to_cartesian_pose(open_door_pose,0.5,15)

    #     except:
    #         return False
        
    #     return True


    def move_with_velocity(self, distance, time, direction, velocity=None, ref_frame=CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_TOOL):
            """
            Move the arm with velocity in a given direction

            distance: distance to move in m
            time: time to move in s
            direction: direction to move in ('x', 'y', 'z')
            velocity: velocity to move in m/s (overrides distance and time)
            ref_frame: reference frame to move in (Tool, Base): default is Tool
            """

            if velocity is None:
                velocity = distance/time

            # create twist command 
            approach_twist = TwistCommand()
            approach_twist.reference_frame = ref_frame
            if direction == 'x':
                approach_twist.twist.linear_x = velocity
            elif direction == 'y':
                approach_twist.twist.linear_y = velocity
            elif direction == 'z':
                approach_twist.twist.linear_z = velocity+0.01
                approach_twist.twist.linear_y = -velocity

            self.cartesian_velocity_pub.publish(approach_twist)
            rospy.sleep(time)
            self.stop_arm_velocity()

            return True
    
    def stop_arm_velocity(self):
        """
        Stop arm by sending zero velocity
        """

        velocity_vector = TwistCommand()
        velocity_vector.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED 
        self.cartesian_velocity_pub.publish(velocity_vector)
        return True
        
    def move_to_home_pose(self):
        home_position = ConstrainedPose()
        home_position.target_pose.x = 0.373
        home_position.target_pose.y = 0.469
        home_position.target_pose.z = 0.64
        home_position.target_pose.theta_x = 176.746
        home_position.target_pose.theta_y = -147.887
        home_position.target_pose.theta_z = 92

        self.move_to_cartesian_pose(home_position,0.5,15)
    
    def get_co_ordinate(self):
        self.value=self.pose
        self.co_ordinate=ConstrainedPose()
        self.co_ordinate.target_pose.x = self.value[0]
        self.co_ordinate.target_pose.y = self.value[1]
        self.co_ordinate.target_pose.z = self.value[2]
        self.co_ordinate.target_pose.theta_x = 55.533
        self.co_ordinate.target_pose.theta_y = -7.979
        self.co_ordinate.target_pose.theta_z = 157.812

        return self.co_ordinate
    

    def base_movement(self):
        responce=input("Move the base to the required pose type(yes/no)")

        if responce=="yes":
            return True
        elif responce=="no":
            return False
        else:
            return False
        
    def move_to_pull_home_pose(self):
        self.co_ordinate=ConstrainedPose()
        self.co_ordinate.target_pose.x = 0
        self.co_ordinate.target_pose.y = 0
        self.co_ordinate.target_pose.z = 0
        self.co_ordinate.target_pose.theta_x = 0
        self.co_ordinate.target_pose.theta_y = 0
        self.co_ordinate.target_pose.theta_z = 0
        return self.co_ordinate
    
    def move_to_push_home_pose(self):
        self.co_ordinate=ConstrainedPose()
        self.co_ordinate.target_pose.x = 0
        self.co_ordinate.target_pose.y = 0
        self.co_ordinate.target_pose.z = 0
        self.co_ordinate.target_pose.theta_x = 0
        self.co_ordinate.target_pose.theta_y = 0
        self.co_ordinate.target_pose.theta_z = 0
        return self.co_ordinate

    def unlatch_pose(self,current_co_ordinate):
        self.unlatch_co_ordinates=ConstrainedPose()
        self.unlatch_co_ordinates.target_pose.x = current_co_ordinate.target_pose.x-0
        self.unlatch_co_ordinates.target_pose.y = current_co_ordinate.target_pose.y-0
        self.unlatch_co_ordinates.target_pose.z = current_co_ordinate.target_pose.z-0
        self.unlatch_co_ordinates.target_pose.theta_x = current_co_ordinate.target_pose.theta_x
        self.unlatch_co_ordinates.target_pose.theta_y = current_co_ordinate.target_pose.theta_y
        self.unlatch_co_ordinates.target_pose.theta_z = current_co_ordinate.target_pose.theta_z

        return self.unlatch_co_ordinates


    def is_topic_publishing(self,topic_name, timeout=1.0):
        try:
            rospy.wait_for_message(topic_name, rospy.AnyMsg, timeout=timeout)
            return True
        except ServiceException:
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

            success &=self.move_to_home_pose()
            flag=False
            while(flag==False):
                print("Waiting for pose from vision module")
                flag=self.is_topic_publishing("/lever_pose", timeout=2.0)
            open_handel_pose=self.get_co_ordinate()
            success &=self.move_to_cartesian_pose(open_handel_pose,1.5,15)
            #close the gripper
            if self.is_gripper_present:
                success &= self.example_send_gripper_command(1.0)
                time.sleep(0.5)
            else:
                rospy.logwarn("No gripper is present on the arm.")  
            open_unlatch_pose=self.unlatch_pose(open_handel_pose)
            success &=self.move_to_cartesian_pose(open_unlatch_pose,1.5,15)

            moving_time =  10
            initial_time = time.now()
            current_elapsed_time = time.now()

            self.move_with_velocity(distance = 0.5,moving_time = moving_time,direction="z")
            while(current_elapsed_time < moving_time):
                current_elapsed_time = time.now() - initial_time
                if self.door_configuration =="pull":
                    self.stop_velocity()
                    break

            self.stop_velocity()
           
        

            if self.door_configuration =="pull":
                success &=self.move_to_pull_home_pose()
                #open the gripper
                if self.is_gripper_present:
                    success &= self.example_send_gripper_command(0.0)
                else:
                    rospy.logwarn("No gripper is present on the arm.")  
                success &=self.base_movement()
                print("Initializing close motions")
                
                success &=self.move_to_pull_home_pose()

                while(flag==False):
                    print("Waiting for pose from vision module")
                    flag=self.is_topic_publishing("/lever_pose", timeout=2.0)
                close_handel_pose=self.get_co_ordinate()

                success &=self.move_to_cartesian_pose(close_handel_pose,1.5,15)
                #close the gripper
                if self.is_gripper_present:
                    success &= self.example_send_gripper_command(1.0)
                    time.sleep(0.5)
                else:
                    rospy.logwarn("No gripper is present on the arm.")  
                close_unlatch_pose=self.unlatch_pose(close_handel_pose)
                success &=self.move_to_cartesian_pose(close_unlatch_pose,1.5,15)
                success &=self.move_to_pull_home_pose()
                #open the gripper
                if self.is_gripper_present:
                    success &= self.example_send_gripper_command(0.0)
                else:
                    rospy.logwarn("No gripper is present on the arm.")  

            elif self.door_configuration =="push":
                #open the gripper
                if self.is_gripper_present:
                    success &= self.example_send_gripper_command(0.0)
                else:
                    rospy.logwarn("No gripper is present on the arm.")  
                success &=self.move_to_push_home_pose()
                success &=self.base_movement()
                print("Initializing close motions")
                # close_handel_pose=self.get_co_ordinate()
                # success &=self.move_to_cartesian_pose(close_handel_pose,1.5,15)
                # close_unlatch_pose=self.unlatch_pose(close_handel_pose)
                # success &=self.move_to_cartesian_pose(close_unlatch_pose,1.5,15)
                success &=self.move_to_push_home_pose()
                

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