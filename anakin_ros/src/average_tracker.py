#! /usr/bin/python
print('Average_tracker: initialising')
import rospy
import roslib
import numpy as np
import time
import tf # this is the transform topic
from std_msgs.msg import String
#from sensor_msgs.msg import Image
#from sensor_msgs.msg import CompressedImage
#from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Vector3, Transform,TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#Do we print out everything?
global verbose
verbose = True

rospy.init_node('average_tracker', anonymous=True)

#subsrcibe to realsense
real_target_frame = "realsense_person_est"
real_listener = tf.TransformListener()

#subscribe to the velodyne transform
vel_target_frame = "velodyne_person_est"
vel_listener = tf.TransformListener()

# broadcast our new transform
br = tf.TransformBroadcaster()

vel_exists = False
real_exists = False #flags to indicate whether either has a transform

base_frame = "base"  # we want both to have a common base frame


def publishCombinedTf(trans1,rot1,trans2,rot2,br):

    avg_trans = np.mean(np.array([trans1, trans2]), axis=0)
    avg_rot = np.mean(np.array([rot1, rot2]), axis=0)
    avg_trans = tuple(avg_trans)
    print(avg_trans)
    print(avg_rot)
    ### convert to tuple
    ## 
    # define the frames for the output transform
    target_frame = "avg_person_est2"
    # send the transform
    print('publishing transform on frame ' + target_frame)
    if verbose:
        print('the vel trans is '+str(trans1))
        print('the real trans is '+str(trans2))
        print('the vel rot is '+str(rot1))
        print('the real rot is '+str(rot2))
        print('the average transform is ' + str(avg_trans))
        print('the average rot is ' + str(avg_rot))
    #br.sendTransform(avg_trans, avg_rot, rospy.Time.now(), "realsense_d435_forward_camera", target_frame)
    # technically we should use the time we received the transforms.
    br.sendTransform(trans1, rot1, rospy.Time.now(), "base", target_frame)


#subscribe to the realsense transform
#get the IDs of the target and base frames used to generate the transform- these are hard
# coded into "ros_realsense_1203.py"





if verbose:
    print('node setup complete. Initiating loop')

# main loop
while not rospy.is_shutdown():
    print('\n\n*******Reading Inputs')


    try:
        vel_listener.waitForTransform(base_frame, vel_target_frame, rospy.Time(0), rospy.Duration(4.0))
        (vel_trans, vel_rot) = vel_listener.lookupTransform(base_frame, vel_target_frame, rospy.Time(0))
        vel_exists = True

    except:
        vel_exists = False
        print('Cant get transform from velodyne')
        continue

    try:
        real_listener.waitForTransform(base_frame, real_target_frame, rospy.Time(0), rospy.Duration(4.0))
        (real_trans, real_rot) = real_listener.lookupTransform(base_frame, real_target_frame, rospy.Time(0))
        real_exists = True
    except:
        real_exists = False
        print('Cant get transform from realsense')
        continue



    # This section accounts for the 4 possible states of velodyne and realsense inputs; so far
    # it seems that the transform is always published irrespective of both.
    publishCombinedTf(vel_trans, vel_rot, real_trans, real_rot, br)
    print('can see both')


    """

    if real_exists:
        if vel_exists:
            publishCombinedTf(vel_trans, vel_rot, real_trans, real_rot, br)
            print('can see both')
        else:
            vel_trans = real_trans
            vel_rot = real_rot
            publishCombinedTf(vel_trans, vel_rot, real_trans, real_rot, br)
            print('can only see real')
    else:
        if vel_exists:
            real_trans = vel_trans
            real_rot = vel_rot
            publishCombinedTf(vel_trans, vel_rot, real_trans, real_rot, br)
            print('can only see vel')
        else:
            print("can't see anything")

	"""

    print('\noutput:')
#    print(avg_trans)
 #   print(avg_rot)
    rospy.sleep(0.5) #pause for a bit