#! /usr/bin/python
print('transform_publisher: initialising')

import rospy
import roslib
import numpy as np
import time
import tf # this is the transform topic
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion, Vector3, Transform,TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#Do we print out everything?
global verbose
verbose = True

rospy.init_node('transform_publisher', anonymous=True)


# broadcast our new transform
br = tf.TransformBroadcaster()

def publishTf(br):
    # copied from code
    avg_trans = (1.3, 3.4, 4.5)
    avg_rot = [0.0002744677207134834, 0.022041032710562913, -0.01244858913492393, 0.9996795237340175] 

    # target_frame = "avg_person_est2"
    # base_frame = "base"  # we want both to have a common base frame
    # copying the frames (that work) from ros_realsense_1203.py
    camera_frame_id = "base"
    target_frame_id = "avg_person_est2"
    #print('publishing transform on frame ' + target_frame)
    if verbose:
        print('the average transform is ' + str(avg_trans))
        print('the average rot is ' + str(avg_rot))
    br.sendTransform(avg_trans, avg_rot, rospy.Time.now(),target_frame_id, camera_frame_id)






if verbose:
    print('node setup complete. Initiating loop')
while not rospy.is_shutdown():
    print('\n\n*******Reading Inputs')


    publishTf(br)
    rospy.sleep(0.5) #pause for a bit