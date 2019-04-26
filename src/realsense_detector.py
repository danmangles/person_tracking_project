# -*- coding: utf-8 -*-
#! /usr/bin/python
import tensorflow
import numpy as np
# import math
import weights_loader
import net # the cnn that we manually defined

# import os
import cv2
# import warnings
# import sys
# import time
import tf # this is the transform topic

import rospy
import roslib

from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

import message_filters # for syncing up RGB and Depth images

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Quaternion, Vector3, Transform,TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#warnings.filterwarnings('ignore')
from cv_bridge import CvBridge, CvBridgeError


# try:
#     from PySide import QtWidgets
# except:
#     from PyQt5 import QtWidgets


class RealsenseDetector:
    #constructor
    def __init__(self, in_simulator, verbose):
        print('initiating new Realsense Detector')

        # define variables
        self.verbose = verbose
        self.in_simulator = in_simulator # are we simulated or rosbag?

        self.set_tuning_parameters(verbose = True)
        
        # setup our ROS frames
        self.world_base_frame = "odom"
        self.camera_base_frame = "realsense_d435_front_forward_camera"
        self.target_frame = "realsense_detections_poseArray" # this is the output name of all things that are published from this module

        # setup bits of TF and CVbridge
        self.tf_sess = tensorflow.InteractiveSession() #make a tensorflow session
        tensorflow.global_variables_initializer().run() # and run it
        self.bridge = CvBridge() #start a CVBridge to move images around

        #setup subscribers and parameters
        self.setup_subscribers_and_publishers(verbose = True) # setup rgb and depth subs
        self.load_yolo_weights(True) # load the weights of the neural network

        return

    """
    CALLBACKS
    """
    def combined_callback(self, rgb_image_msg, depth_image_msg):
        '''
        :param rgb_image_msg: an image in sensor_msgs/CompressedImage format of dimension 640x480
        :param depth_image_msg: an Image format depth image, of dimension 640x480

        1. takes an RGB image and gets bounding_box vertex coordinates for each person
        2. takes a depth image, calculates a distance and angle to each person
        3. publish a poseArray containing a Pose for each person
        '''
        # use a cvbridge to convert from ros image to openCv image
        rgb_image = self.bridge.compressed_imgmsg_to_cv2(rgb_image_msg, "bgr8")  # frame is a new image in cv2 format
        # convert the depth image into cv2 format
        depth_image = self.bridge.imgmsg_to_cv2(depth_image_msg, desired_encoding="passthrough")

        if self.verbose:
            print('\n\n***********combined_callback()')
            print('RGB image has shape ' + str(rgb_image.shape[:2]))
            print("Depth image has shape " + str(depth_image.shape))

        # get the bounding box from rgb data
        bounding_boxes = self.get_bounding_boxes_from_rgb_msg(rgb_image, False)

        # print(len(bounding_boxes))

        if len(bounding_boxes) == 0:  # if we can see something, the bounding box will be defined and it will have length 4
            print('bounding box is nonexistent, returning out of method')
            return


        if self.verbose:
            # check it worked by printing out the shape (will be 'none' if not working)
            print("depth shape is " + str(depth_image.shape))
            print('get_bounding_boxes_from_rgb_msg() returned %d bounding boxes '%len(bounding_boxes))

        self.update_pose_array_from_bounding_boxes(depth_image, bounding_boxes, rgb_image_msg.header.stamp, True)

        # publish the latest array of poses
        self.poseArray_publisher.publish(self.pose_array)
        if self.verbose:
            print('poses published to '+self.target_frame)


        return

    '''
    Image processing methods
    '''
    def get_bounding_boxes_from_rgb_msg(self,rgb_image, verbose):
        '''
        :param rgb_image: an image in cv2 format
        :return: bounding box: the top left and bottom right coordinates of the box bounding a person detection ???? WHAT HAPPENS IF MULTIPLE PEOPLE????
        '''

        if verbose:
           print('update_bounding_box_from_rgb_msg()')

        # carry out the preprocesssing functions
        preprocessed_image = self.preprocessing(rgb_image, verbose = True)

        # compute the predictions on the input image
        # Forward pass of the preprocessed image into the network defined in the net.py file using tensorflow
        predictions = self.tf_sess.run(net.o9, feed_dict={net.x: preprocessed_image})

        # get bounding boxes (in 640x480 coords) and an output image with the bbs drawn onto it
        output_image, bounding_boxes = self.postprocessing(predictions, rgb_image, verbose = True)

        if verbose:
            print('bounding boxes are located at')
            print(bounding_boxes)
            print('publishing image with bounding box on self.image_pub')

        # publish the image
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(output_image, "bgr8"))

        return bounding_boxes

    def preprocessing(self, input_image, verbose):
        '''
        INPUTS: input_image: a cv2 image
        method: resize input image to YOLO requirements, normalise, convert to numpy array and add a batch dimension
        '''
        # use cv2's function to resize the image to the correct height and width for yolo net
        resized_image = cv2.resize(input_image, (self.YOLO_IMAGE_DIMENSION, self.YOLO_IMAGE_DIMENSION), interpolation = cv2.INTER_CUBIC)
        # convert to a numpy array
        image_data = np.array(resized_image, dtype = 'f')

        # normalise the image from [0,255] -> [0,1]
        image_data /= 255.
        image_array = np.expand_dims(image_data, 0) # add a batch dimension ???WHAT IS THIS???
        if verbose:
            print('image resized to 416x416, converted to numpy array and normalised to [0,1]')
        return image_array

    def postprocessing(self, predictions, input_image, verbose):
        '''
        :param predictions: a tensorflow forward pass of bounding boxes, probabilities, class names
        :param input_image: an image in cv2 format 640x480
        :return output_image: the input_image with bounding box rectangles drawn on it
        :return bounding_boxes: a list of bounding box vertices based on input image dimensions
        '''
        if verbose:
            print('postprocessing() is getting the bounding boxes for an image of shape '+str(input_image.shape))

        resized_image = cv2.resize(input_image, (self.YOLO_IMAGE_DIMENSION, self.YOLO_IMAGE_DIMENSION), interpolation = cv2.INTER_CUBIC)
        # resized_image = input_image
        if verbose:
            print('resized it to shape '+str(resized_image.shape))

        # SOME CONSTANTS FOR OUR DETECTION/BOUNDING BOXES
        n_grid_cells = 13 # what is this?
        n_b_boxes = 5 # what is this?
        n_classes = 20 # number of classes in yolo

        #Names  for each class
        classes = ["aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow","diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]

        # Pre-computed YOLOv2 shapes of the k=5 B-Boxes
        anchors = [1.08, 1.19,  3.42, 4.41,  6.63, 11.38,  9.42, 5.11, 16.62, 10.52]
        thresholded_predictions = []

        # IMPORTANT: reshape to have shape = [ 13 x 13 x (5 B-Boxes) x (4 Coords + 1 Obj score + 20 Class scores ) ]
        reshape_last_dim = 4 + 1 + n_classes #see above
        predictions = np.reshape(predictions,(n_grid_cells, n_grid_cells, n_b_boxes, reshape_last_dim))

        for row in range(n_grid_cells): # loop through the number of grid_cells vertically
            for col in range(n_grid_cells): # loop through the number of grid_cells horizontally
                for b in range(n_b_boxes): # loop through the number of b_boxes...?

                    tx, ty, tw, th, tc = predictions[row, col, b, :5] # extract the predictions
                    # IMPORTANT: (416 img size) / (13 grid cells) = 32!
                    # YOLOv2 predicts parametrized coordinates that must be converted to full size

                    #final_coordinates = parametrized_coordinates * 32.0 ( You can see other EQUIVALENT ways to do this...)
                    center_x = (float(col) + self.sigmoid(tx)) * self.YOLO_COORDINATE_SCALING_PARAMETER_X # magic number, think it needs to be proportional to input_image.width
                    center_y = (float(row) + self.sigmoid(ty)) * self.YOLO_COORDINATE_SCALING_PARAMETER_Y # see above but for height

                    roi_w = np.exp(tw) * anchors[2*b + 0] * self.YOLO_COORDINATE_SCALING_PARAMETER_X
                    roi_h = np.exp(th) * anchors[2*b + 1] * self.YOLO_COORDINATE_SCALING_PARAMETER_Y

                    final_confidence = self.sigmoid(tc)

                    # Find best class
                    class_predictions = predictions[row, col, b, 5:]
                    class_predictions = self.softmax(class_predictions)

                    class_predictions = tuple(class_predictions)
                    best_class = class_predictions.index(max(class_predictions))
                    best_class_score = class_predictions[best_class]

                    # Flip the coordinates on both axes ########################## THIS IS POSSIBLY GIVING US AN INCORRECTLY POSITIONED BB
                    left   = int(center_x - (roi_w/2.))
                    right  = int(center_x + (roi_w/2.))
                    top    = int(center_y - (roi_h/2.))
                    bottom = int(center_y + (roi_h/2.))

                    # if we're confident enough that we've found something, add to theshholded_predictions
                    if ((final_confidence * best_class_score) > self.score_threshold):
                        thresholded_predictions.append([[left, top, right, bottom], final_confidence * best_class_score, classes[best_class]])


        # Sort the B-boxes by their final score
        thresholded_predictions.sort(key=lambda tup: tup[1],reverse=True)

        #what is non maximal suppression??????
        nms_predictions = []

        #create a list of predictions for which the class is 'person' from thresholded predictions
        thresholded_predictions = list(filter(lambda a: a[-1] == 'person', thresholded_predictions))
        if (len(thresholded_predictions) > 0): #if any of our predictions survived the threshold, pass thru our NMS function
            nms_predictions = self.get_nms_predictions(thresholded_predictions)

        #draw the final bounding boxes and label on the input image
        bounding_boxes = [] # initiate an array to store predictions in
        #looping thru our predicted bounding boxes
        for i in range(len(nms_predictions)):

            color = (0.0, 0., 254.) # red- for the bounding box and text
            best_class_name = nms_predictions[i][2]

            #put a class rectangle with bounding box coordinates and a class label on the image
            if verbose:
                # print ('bounding box located at '+str((nms_predictions[i][0][0], nms_predictions[i][0][1], nms_predictions[i][0][2],nms_predictions[i][0][3])))
                print('this bb is '+str(nms_predictions[i][0][:]))

            bounding_boxes.append(nms_predictions[i][0][:])
            input_image = cv2.rectangle(input_image, (nms_predictions[i][0][0], nms_predictions[i][0][1]), (nms_predictions[i][0][2],nms_predictions[i][0][3]), color)
            # print the class name onto the image
            cv2.putText(input_image,best_class_name,(int((nms_predictions[i][0][0]+nms_predictions[i][0][2])/2),int((nms_predictions[i][0][1]+nms_predictions[i][0][3])/2)),cv2.FONT_HERSHEY_SIMPLEX,1,color,3)

        # if we've predicted any classes, return them. Else just send back the same old image.
        if (len(nms_predictions) > 0):
            #print('I can see a %s at coordinates %s'%(best_class_name, str(nms_predictions[0][0])))
            if verbose:
                if len(nms_predictions) > 1:
                    print('************i can see a total of %d people'%len(nms_predictions))
            return input_image, bounding_boxes

        else:
            print('cant see anything...')
            return input_image, []

    '''
    Coordinate extraction methods
    '''

    def update_pose_array_from_bounding_boxes(self, depth_image, bounding_boxes, time_of_detection, verbose):
        '''
        :param depth_image: a cv2 format depth image
        :param bounding_boxes: a list of lists of vertex coordinates
        :param time_of_detection:
        :param verbose:
        :return:
        '''
        self.pose_array.poses = [] # clear the array

        self.pose_array.header.stamp = time_of_detection # set the time to match that at which the image was detected

        if verbose:
            print("update_pose_array_from_bounding_boxes()")

        for bounding_box in bounding_boxes: # loop thru the bounding boxes
            # if verbose:
            #     print('inspecting bounding box '+str(bounding_box))

            try:
                # get distance, yaw and pitch to the target id'd in bounding box from depth image
                target_dist, target_yaw, target_pitch = self.get_dist_yaw_pitch(depth_image, bounding_box, verbose=True)
            except ValueError:
                print('*******Depth computation has resulted in an empty depth measurement. Waiting for next one.\n')
                # raise ValueError # throw the error

            #strike a pose
            new_pose = PoseStamped()

            local_coord = [target_dist * np.cos(target_yaw) * np.cos(target_pitch), target_dist * np.sin(target_yaw) * np.cos(target_pitch), target_dist * np.sin(target_pitch) ]

            if verbose:
                print('local coord is '+str(local_coord))

            try:

                new_pose.pose.position.x = local_coord[0]
                new_pose.pose.position.y =  local_coord[1]
                new_pose.pose.position.z =  local_coord[2]
                quat = quaternion_from_euler(0, 0, 0)
                new_pose.pose.orientation.x = quat[0]
                new_pose.pose.orientation.y = quat[1]
                new_pose.pose.orientation.z = quat[2]
                new_pose.pose.orientation.w = quat[3]

                # construct a posestamped so that we can correctly transform into odom frame
                new_pose.header.frame_id = self.camera_base_frame
                new_pose.header.stamp = time_of_detection
                new_pose = self.realsense_odom_ls.transformPose("odom", new_pose)

                if verbose:
                    print('adding pose at (%f, %f, %f) to the posearray'%(new_pose.pose.position.x,new_pose.pose.position.y,new_pose.pose.position.z))
                self.pose_array.poses.append(new_pose.pose) # add the pose to the array

            except Exception as e:
                print('*****************ERROR WITH TRANSFORM\n'+repr(e))
                # raise e

        if verbose:
            print('pose_array updated.')
        return

    def get_dist_yaw_pitch(self, depth_image, bounding_box, verbose):
        '''
        :param depth_image: a cv2 format depth image from RGBD
        :param bounding_box: x,y coords of top left, bottom right corners of the bounding box for a 640x480 px image
        :param verbose: verbose?
        :return: target_dist: distance in m to target, target_yaw, target_pitch: off centre angles to bounding box in radians
        '''
        '''
        Returns the distance, yaw and pitch to a target in depth image, located by bounding_box
        '''
        # make the bounding box not exceed the edges of the image
        for i in range(len(bounding_box)): #loop thru bb coords
            if bounding_box[i] < 0:
                print('!!!!!!setting a negative dimension to zero from bounding box')
                bounding_box[i] = 0

        # get coordinates of bounding box position in cropped image coordinate frame
        l = bounding_box[0]
        t = bounding_box[1]
        r = bounding_box[2]
        b = bounding_box[3]

        cropped_depth_image = depth_image[t:b, l:r] # select a rectangle from the depth image that is cropped to x = [t:b] and y = [l:r]

        # remove zero values (these are infinite distances) to leave a valid array of depths of the person
        values = list(filter(lambda y: y != 0, cropped_depth_image.flatten()))
        target_dist = np.nanmedian(values) # take a median of the depths in the image

        # rosbag versions have depths in mm
        if not self.in_simulator:
            target_dist = target_dist/self.DEPTH_FACTOR # convert from mm to m

        '''
        get yaw and pitch. horiz_dev_from_normal is the fractional deviation of bounding box centre from frame centre, ditto for vert_....
        '''
        # these deviations range from -0.5 to 0.5. Tested, needs to be 640 x 480
        horiz_dev_from_normal = 0.5 - (l+r)/(2.0*self.REALSENSE_IMAGE_WIDTH) # horizontal deviation from normal (kinda an angle)
        vert_dev_from_normal = 0.5 - (t+b)/(2.0*self.REALSENSE_IMAGE_HEIGHT) # horizontal deviation from normal (kinda an angle)

        # convert deviations to angles using camera calibration parameters
        target_yaw = self.realsense_half_fov_angle*horiz_dev_from_normal*np.pi/180.0 # in radians4
        target_pitch = self.realsense_half_fov_angle_vert*vert_dev_from_normal*np.pi/180.0 # in radians

        # display stuff
        if verbose:
            print("dist = %sm" % str(target_dist))
            print("yaw = %f, pitch = %f" % (target_yaw, target_pitch))
            print('angle (0.5 (left) to -0.5 (right): '+str(horiz_dev_from_normal))

        # self.display_position_cross_on_image(depth_image, horiz_dev_from_normal, vert_dev_from_normal)
        self.depth_pub.publish(self.bridge.cv2_to_imgmsg(cropped_depth_image, "16UC1"))

        return target_dist, target_yaw, target_pitch
    """ this was a terrible idea
    
    def display_position_cross_on_image(self, depth_image, horiz_dev_from_normal, vert_dev_from_normal):
        '''
        :param depth_image:
        :param horiz_dev_from_normal:
        :param vert_dev_from_normal:
        :param target_yaw:
        :param target_pitch:
        :return:
        '''
        radius = 30
        color = (0, 0, 254)
        centre = (int(horiz_dev_from_normal*self.REALSENSE_IMAGE_WIDTH), int(vert_dev_from_normal*self.REALSENSE_IMAGE_HEIGHT))
        print(centre)
        depth_image = cv2.circle(depth_image, centre, radius, color, thickness = 40) # draw a filled blue circle

        cv2.imshow('depth_image', depth_image)
        cv2.waitKey(100)
       """
    '''
    Neural Network and CV methods
    '''
    def sigmoid(self,x):
        return 1. / (1. + np.exp(-x))

    def softmax(self,x):
        e_x = np.exp(x - np.max(x))
        out = e_x / e_x.sum()
        return out

    def compute_iou(self, boxA, boxB):
        '''
        Computes the intersection over union of the two boxes provided.
        This is simply the ratio: iou = insersection_area/non_intersecting_area
        for more info see: https://www.pyimagesearch.com/2016/11/07/intersection-over-union-iou-for-object-detection/
        '''
        # Determine the coordinates of the intersection rectangle
        xA = max(boxA[0], boxB[0])
        yA = max(boxA[1], boxB[1])
        xB = min(boxA[2], boxB[2])
        yB = min(boxA[3], boxB[3])

        # Compute the area of intersection
        intersection_area = (xB - xA + 1) * (yB - yA + 1)

        # Compute the area of both rectangles
        boxA_area = (boxA[2] - boxA[0] + 1) * (boxA[3] - boxA[1] + 1)
        boxB_area = (boxB[2] - boxB[0] + 1) * (boxB[3] - boxB[1] + 1)

        # Compute the IOUPointCloud
        iou = intersection_area / float(boxA_area + boxB_area - intersection_area)

        return iou

    def get_nms_predictions(self, thresholded_predictions):
            '''
            Returns a list of our non-maximal-suppression class predictions
            I think this is grading predictions against themselves, but I'm not entirely sure....???
            '''
            nms_predictions = []
            nms_predictions.append(thresholded_predictions[0]) # add the first prediction to our list

            for thresholded_prediction in thresholded_predictions:

                    prediction_is_valid = True # don't delete this prediction (yet)

                    for nms_prediction in nms_predictions:
                        #get the IOU of our thresholded prediction and nms prediction
                        current_iou = self.compute_iou(thresholded_prediction[0], nms_prediction[0])
                        if current_iou > self.iou_threshold:
                            prediction_is_valid = False
                            break

                    if prediction_is_valid:
                            nms_predictions.append(thresholded_prediction)

            return nms_predictions

    '''
    Initialisation methods
    '''
    def load_yolo_weights(self,verbose):
        '''
        loads the yolo weights into tensorflow session
        '''
        if verbose:
            print('Loading weights.')
        # point tensorflow to the correct folder
        if self.in_simulator: # possibly wrong now
            weights_path = 'yolov2-tiny-voc.weights'
            ckpt_folder_path = '/catkin_ws/src/multi_sensor_tracker/drs-main/code/src/personal/person_tracker/scripts/ckpt/'
        else:
            #paths are relative to the package
            weights_path = 'src/yolo_weights/yolov2-tiny-voc.weights'
            ckpt_folder_path = 'yolo_ckpts/'

        # Check for an existing checkpoint and load the weights (if it exists) or do it from binary file
        saver = tensorflow.train.Saver()
        _ = weights_loader.load(self.tf_sess, weights_path, ckpt_folder_path, saver) #wtf is this???

        if verbose:
            print('weights loaded.')
        return

    def setup_subscribers_and_publishers(self, verbose):
        '''
        sets up subscribers to the rgb and depth images
        '''
        if verbose:
            print('setting up subscribers')

        # set the correct topic names based on our situation
        if self.in_simulator:
            image_topic = "/realsense_d435_front_forward/rgb/image_raw/compressed"
            self.rgb_sub = rospy.Subscriber(image_topic, CompressedImage,self.rgb_callback) # every time we get an RGB image, call self.rgb_callback
            depth_topic = "/realsense_d435_front_forward/depth/image_raw"
            self.depth_sub = rospy.Subscriber(depth_topic, Image, self.depth_callback) # same for depth images

        else:
            image_topic = "/realsense_d435_front_forward/color/image_raw/compressed"
            rgb_sub = message_filters.Subscriber(image_topic, CompressedImage) # every time we get an RGB image, call self.rgb_callback
            depth_topic = "/realsense_d435_front_forward/aligned_depth_to_color/image_raw"
            depth_sub = message_filters.Subscriber(depth_topic, Image) # same for depth images

        # setup the synchronised subscriber which combines the rgb and depth subscribers into a single callback
        self.ts = message_filters.TimeSynchronizer([rgb_sub, depth_sub],10)
        self.ts.registerCallback(self.combined_callback)

        # create the publisher to output our prediction images on
        image_pub_topic = "/camera/person_detection_bounding_boxes" # publishing to this topic
        self.image_pub = rospy.Publisher(image_pub_topic,Image)

        # create the publisher to output our cropped images on
        depth_pub_topic = "/camera/cropped_depth_image" # publishing to this topic
        self.depth_pub = rospy.Publisher(depth_pub_topic,Image)

        # create the publisher to output our bb on
        bounding_box_pub_topic = "/camera/bounding_box"  # publishing to this topic
        self.bounding_box_pub = rospy.Publisher(bounding_box_pub_topic, Image)

        # transform broadcaster for our transforms
        self.br = tf.TransformBroadcaster()
        self.realsense_odom_ls = tf.TransformListener()

        #publisher for our pose estimates
        # self.pose_pub = rospy.Publisher(self.target_frame, PoseStamped, queue_size=100)
        self.poseArray_publisher = rospy.Publisher(self.target_frame, PoseArray, queue_size=1)

        self.pose_array = PoseArray() # initiate a posearray to publish
        self.pose_array.header.frame_id = self.world_base_frame # set the base frame


    def set_tuning_parameters(self, verbose):
        '''
        :param verbose: are we verbose?
        :return: void
        sets all the numerical values for this module
        '''

        self.YOLO_IMAGE_DIMENSION = 416 # we are using a CNN which takes inputs of size 416x416

        # this parameter affects the transformation into global coordinates
        self.REALSENSE_IMAGE_WIDTH, self.REALSENSE_IMAGE_HEIGHT = 640, 480 # affect where we draw the bounding box

        # define camera fov angles to calculate direction from an image
        self.DEPTH_FACTOR = 950.0 # approximately a mm->m conversion to get distance from depth image. used in get_dist_yaw_pitch()
        # affects the conversion from bounding box coords to global coords. Bigger gives more extreme values
        self.ANGLE_SCALING_FACTOR = 1.8

        if self.in_simulator:
            self.realsense_half_fov_angle = 220/2 # found this number by experiment
            self.realsense_half_fov_angle_vert = (220/69.4)*42.5/2 # scaling other number by same amount
        else:
            self.realsense_half_fov_angle = 69.4/2*self.ANGLE_SCALING_FACTOR
            self.realsense_half_fov_angle_vert = 42.5/2*3

        # these parameters affect where the bounding box is drawn in the image
        self.YOLO_COORDINATE_SCALING_PARAMETER_X = 48.0
        self.YOLO_COORDINATE_SCALING_PARAMETER_Y = 36.0

        # specific yolo_parameters
        self.score_threshold = 0.3 # threshold for picking true positives in our image
        self.iou_threshold = 0.3 #another cnn threshold
