import sys
import rospy
import time
from kortex_driver.msg import TwistCommand
import kortex_driver
from kortex_driver.srv import *
from kortex_driver.msg import *
import numpy as np


class DoorOpening:


    def move_to_cartesian_pose(self, pose, translation_speed=0.5, orientation_speed=15):

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


        # Set Global variable door_config= <"push|pull">

        # Goto door pose.
        # Close gripper
        # Goto unlatch door 
        # Goto push door (move 2 cm into the door)
            #check force > 30N
                #yes - Goto Pull pose open
                        # Move base
                        # Open Gripper 
                        # Goto armstraight
                        # Goto pull pose 1
                        # Goto pull pose 2
                #No - Goto push pose open
                        # Move base
                        # Open Gripper
                        # Goto armstraight
                        # Goto push pose 1



    self.door_configuration = "push"
        

    self.move_to_cartesian_pose(door_pose)
    self.gripper_command(0.0)
    self.move_to_cartesian_pose(unlatch_pose)


    moving_time =  2 
    initial_time = time.now()
    current_elapsed_time = time.now()

    self.move_with_velocity(distance = 3 ,moving_time = moving_time)

    while(current_elapsed_time < moving_time):
        current_elapsed_time = time.now() - initial_time
        if self.force_check(force_threshod=30):
            self.door_configuration = "pull"
            break
    self.stop_velocity()

    if self.door_configuration == "push":

        self.move_base()
        self.gripper_command(1.0)
        rospy.sleep(0.5)

        # move arm straight
        self.move_to_cartesian_pose(armstraight_pose)

        # push pose 1
        self.move_to_cartesian_pose(pushpose1)

    else:

        self.move_base()
        self.gripper_command(1.0)
        rospy.sleep(0.5)

        # move arm straight
        self.move_to_cartesian_pose(armstraight_pose)

        # push pose 1
        self.move_to_cartesian_pose(pullpose1)




from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

self.pose_subscriber = rospy.Subscriber('your/topic',  PoseStamped, self.pose_conversion_callback)
self.pose = tuple()

def pose_conversion_callback(self, pose_msg):
    self.pose = self.get_kinovapose_from_pose_stamped(pose_msg)

def get_kinovapose_from_pose_stamped(pose: PoseStamped):
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

    return tuple(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, theta_x_deg, theta_y_deg, theta_z_deg)