import sys
import rospy
import time
import math
import tf
from std_msgs.msg import Int16MultiArray , String
from kortex_driver.msg import TwistCommand
import kortex_driver
import numpy as np

from kortex_driver.srv import *
from kortex_driver.msg import *

class forceTansformation():
    def __init__(self):
        self._force_subscriber = rospy.Subscriber("/my_gen3/base_feedback", kortex_driver.msg.BaseCyclic_Feedback, self._force_callback)
        