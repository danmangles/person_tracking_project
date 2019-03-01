#! /usr/bin/python
import tensorflow
import numpy as np
import weights_loader
import net # the cnn that we manually defined

import os
import cv2
import warnings
import sys
import time
import tf # this is the transform topic

import rospy
import roslib

from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Quaternion, Vector3, Transform,TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#warnings.filterwarnings('ignore')
from cv_bridge import CvBridge, CvBridgeError

import matplotlib.pyplot as plt
import matplotlib.image as mpimg

"""
SETUP SOME SYSTEM WIDE PARAMETERS
"""
global REALSENSE_FOV_ANGLE, YOLO_IMAGE_INPUT_HEIGHT, YOLO_IMAGE_INPUT_WIDTH, REALSENSE_IMAGE_HEIGHT, REALSENSE_IMAGE_WIDTH, verbose, skipping #in degrees
#REALSENSE_FOV_ANGLE = 69.4
#REALSENSE_FOV_ANGLE = 69.4 #https://click.intel.com/intelr-realsensetm-depth-camera-d435.html
REALSENSE_FOV_ANGLE = 30
YOLO_IMAGE_INPUT_HEIGHT, YOLO_IMAGE_INPUT_WIDTH = 416, 416
REALSENSE_IMAGE_HEIGHT, REALSENSE_IMAGE_WIDTH = 480, 640 # 0301 found these values using rostopic echo /realsense_d435_forward/color/camera_info

verbose = True
skipping = True # this parameter limits the update frequency


"""
NEURAL NETWORK METHODS
"""
def sigmoid(x):
        return 1. / (1. + np.exp(-x))

def softmax(x):
        e_x = np.exp(x - np.max(x))
        out = e_x / e_x.sum()
        return out

def inference(sess,preprocessed_image):

      # Forward pass of the preprocessed image into the network defined in the net.py file
      # using tensorflow
      predictions = sess.run(net.o9,feed_dict={net.x:preprocessed_image})

      return predictions


"""
COMPUTER VISION METHODS
"""
def preprocessing(input_image):
    '''
    INPUTS: input_image: an image in ??? format
    OUTPUTS: image_array: the resized image in np.array format
    Method: resizes input_image, converts to a np.array, and uses "expand_dims" to "add a batch dimension"
    '''
    # use cv2's function to resize the image to the correct height and width
    resized_image = cv2.resize(input_image, (YOLO_IMAGE_INPUT_HEIGHT, YOLO_IMAGE_INPUT_WIDTH), interpolation = cv2.INTER_CUBIC)
    # convert into a np.array
    image_data = np.array(resized_image, dtype = 'f')

    # normalise the image from [0,255] -> [0,1]
    image_data /= 255.

    image_array = np.expand_dims(image_data, 0) # add a batch dimension ???WHAT IS THIS???
    if verbose:
        print('preprocessing() is returning an image of shape '+str(image_array.shape))
    return image_array

def compute_iou(boxA, boxB):
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

def get_nms_predictions(thresholded_predictions, iou_threshold):
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
                        current_iou = compute_iou(thresholded_prediction[0], nms_prediction[0])
                        if current_iou > iou_threshold:
                                prediction_is_valid = False
                                break

                if prediction_is_valid:
                        nms_predictions.append(thresholded_prediction)

        return nms_predictions

def postprocessing(predictions,input_image,score_threshold,iou_threshold):
        '''
        does some useful stuff with our tensorflow predictions
        '''

        if verbose:
            print('postprocessing() is getting the bounding box for an image of shape '+str(input_image.shape))

        resized_image = cv2.resize(input_image, (YOLO_IMAGE_INPUT_HEIGHT, YOLO_IMAGE_INPUT_WIDTH), interpolation = cv2.INTER_CUBIC)

        if verbose:
            print('resized it to shape '+str(resized_image.shape))

        # SOME CONSTANTS FOR OUR DETECTION/BOUNDING BOXES
        n_classes = 20
        n_grid_cells = 13
        n_b_boxes = 5
        n_b_box_coord = 4

        #### WHY CAN'T WE JUST USE ONE CLASS????
        #Names and colors for each class
        classes = ["aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow","diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]
        colors = [(254.0, 254.0, 254), (239.88888888888889, 211.66666666666669, 127),
                          (225.77777777777777, 169.33333333333334, 0), (211.66666666666669, 127.0, 254),
                          (197.55555555555557, 84.66666666666667, 127), (183.44444444444443, 42.33333333333332, 0),
                          (169.33333333333334, 0.0, 254), (155.22222222222223, -42.33333333333335, 127),
                          (141.11111111111111, -84.66666666666664, 0), (127.0, 254.0, 254),
                          (112.88888888888889, 211.66666666666669, 127), (98.77777777777777, 169.33333333333334, 0),
                          (84.66666666666667, 127.0, 254), (254, 0, 0),
                          (56.44444444444444, 42.33333333333332, 0), (42.33333333333332, 0.0, 254),
                          (28.222222222222236, -42.33333333333335, 127), (14.111111111111118, -84.66666666666664, 0),
                          (0.0, 254.0, 254), (-14.111111111111118, 211.66666666666669, 127)]

        # Pre-computed YOLOv2 shapes of the k=5 B-Boxes.... WHERE DO THESE NUMBERS COME FROM
        anchors = [1.08,1.19,  3.42,4.41,  6.63,11.38,  9.42,5.11,  16.62,10.52]
        thresholded_predictions = []

        # IMPORTANT: reshape to have shape = [ 13 x 13 x (5 B-Boxes) x (4 Coords + 1 Obj score + 20 Class scores ) ]
        predictions = np.reshape(predictions,(13,13,5,25))

        for row in range(n_grid_cells):
                for col in range(n_grid_cells):
                        for b in range(n_b_boxes):
                                tx, ty, tw, th, tc = predictions[row, col, b, :5] # extract the predictions
                                # IMPORTANT: (416 img size) / (13 grid cells) = 32!
                                # YOLOv2 predicts parametrized coordinates that must be converted to full size
                                #final_coordinates = parametrized_coordinates * 32.0 ( You can see other EQUIVALENT ways to do this...)
                                center_x = (float(col) + sigmoid(tx)) * 48.0 # magic number, think it needs to be proportional to input_image.width
                                center_y = (float(row) + sigmoid(ty)) * 32.0 # see above but for height

                                roi_w = np.exp(tw) * anchors[2*b + 0] * 32.0
                                roi_h = np.exp(th) * anchors[2*b + 1] * 32.0

                                final_confidence = sigmoid(tc)

                                # Find best class
                                class_predictions = predictions[row, col, b, 5:]
                                class_predictions = softmax(class_predictions)

                                class_predictions = tuple(class_predictions)
                                best_class = class_predictions.index(max(class_predictions))
                                best_class_score = class_predictions[best_class]

                                # Flip the coordinates on both axes ########################## THIS IS POSSIBLY GIVING US AN INCORRECTLY POSITIONED BB
                                left   = int(center_x - (roi_w/2.))
                                right  = int(center_x + (roi_w/2.))
                                top    = int(center_y - (roi_h/2.))
                                bottom = int(center_y + (roi_h/2.))

                                # if we're confident enough that we've found something, add to theshholded_predictions
                                if ((final_confidence * best_class_score) > score_threshold):
                                        thresholded_predictions.append([[left,top,right,bottom],final_confidence * best_class_score,classes[best_class]])


        # Sort the B-boxes by their final score
        thresholded_predictions.sort(key=lambda tup: tup[1],reverse=True)
        #what is non maximal suppression??????
        nms_predictions = []
        #create a list of predictions for which the class is 'person' from thresholded predictions
        thresholded_predictions = list(filter(lambda a: a[-1] == 'person', thresholded_predictions))
        if (len(thresholded_predictions) > 0): #if any of our predictions survived the threshold, pass thru our NMS function
                nms_predictions = get_nms_predictions(thresholded_predictions, iou_threshold)

        #draw the final bounding boxes and label on the input image


        #looping thru our predicted bounding boxes
        for i in range(len(nms_predictions)):

                color = (0.0, 0., 254.)
                best_class_name = nms_predictions[i][2]

                #put a class rectangle with bounding box coordinates and a class label on the image
                if verbose:
                    print ('placing a rectangle at '+str((nms_predictions[i][0][0], nms_predictions[i][0][1], nms_predictions[i][0][2],nms_predictions[i][0][3])))
                input_image = cv2.rectangle(input_image, (nms_predictions[i][0][0], nms_predictions[i][0][1]), (nms_predictions[i][0][2],nms_predictions[i][0][3]), color)
                # print the class name onto the image
                cv2.putText(input_image,best_class_name,(int((nms_predictions[i][0][0]+nms_predictions[i][0][2])/2),int((nms_predictions[i][0][1]+nms_predictions[i][0][3])/2)),cv2.FONT_HERSHEY_SIMPLEX,1,color,3)

        # if we've predicted any classes, return them. Else just send back the same old image.
        if (len(nms_predictions) > 0):
                #print('I can see a %s at coordinates %s'%(best_class_name, str(nms_predictions[0][0])))
                if len(nms_predictions) > 1:
                        print('i can also see other stuff')
                return input_image, nms_predictions[0][0]

        else:
                print('cant see anything...')
                return input_image, []



"""
GENERAL METHODS
"""
def rgb_callback(data):
    '''
    This is the top level method that runs the pipeline for preprocessing, classifying and postprocessing images
    '''


    # #skip every 5th frame
    if skipping:
        global skip
        skip += 1
        if skip % 5 != 0: # unless we are on a 5th frame, return
             return

    if verbose:
        print('\n*****rgb_callback()')
    #use a cvbridge to convert from ros image to openCv image
    frame = bridge.compressed_imgmsg_to_cv2(data, "bgr8") #frame is a new image
    print('we have an input frame with shape'+str(frame.shape[:2]))
    preprocessed_image = preprocessing(frame) #input height and width are fixed dimensions
    print('resized it to shape '+str(preprocessed_image.shape))
    #compute the predictions on the input image
    predictions = []
    predictions = inference(sess, preprocessed_image)

    #postprocess the predictions and save the output image
    output_image, bb = postprocessing(predictions, frame, score_threshold, iou_threshold)

    #set the bounding box variable (type??!!) to global for broad usage
    global bounding_box
    bounding_box = bb #set the bounding box to the box of the top prediction

    # prepare the output image for displaying
    output_image = cv2.resize(output_image, (REALSENSE_IMAGE_WIDTH,REALSENSE_IMAGE_HEIGHT), interpolation = cv2.INTER_CUBIC)
    image_pub.publish(bridge.cv2_to_imgmsg(output_image, "bgr8"))

    # uncomment this to display the output on a separate window
#    if verbose:
#        cv2.imshow('Video Visualise', output_image)
#        cv2.waitKey(100)

def depth_callback(data):
    '''
    Called by our depth subscriber.
    INPUTS: data: an imgmsg format 8 bit depth image, of dimensions ???x???
    Takes this image, converts to cv2, crops it, calculates:
             distance 'target_dist'
             yaw 'target_yaw'
             pitch 'target_pitch'
             between the forward realsense and the target. These are then sent to the tf topic using 'sendtransform'
    '''

    if skipping:
        global skip
        if skip % 5 != 0:
            return
    if verbose:
        print ('\n*****depth_callback()')

    global bounding_box # plz stop using global variables
    if verbose:
        print('investigating bounding box which is '+str(bounding_box))
    if len(bounding_box) != 4: # e.g. the bounding box isn't defined (if it is it will have length 4)
        print('bounding box is undefined, waiting for a new one')
        return

    # convert the depth image to some usable format
#     depth = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    depth = bridge.compressed_imgmsg_to_cv2(data, desired_encoding="passthrough")
#    depth = bridge.compressed_imgmsg_to_cv2(data, "brdesired_encoding="passthrough") # convert to cv2
    if verbose:
        print("\n\n NEW DEPTH READING")
#        print("depth shape is "+str(depth.shape))
        cv2.imshow('original image',depth)
        cv2.waitKey(100)

    '''
    Take the median of pixels inside the bounding box and use this as our distance
    '''

    # create an array of zeros across the span of the bb at 416x416
    bb = np.zeros((YOLO_IMAGE_INPUT_HEIGHT, YOLO_IMAGE_INPUT_WIDTH))
    # print('bb shape is %s'%str(bb.shape))
    # draw a rectangle with the two opposite vertices at (bounding_box[0],bounding_box[1]) and (bounding_box[2],bounding_box[3])
    bb = cv2.rectangle(bb,(bounding_box[0],bounding_box[1]),(bounding_box[2],bounding_box[3]),1, thickness=-1)
    # print('rect: bb shape is %s'%str(bb.shape))
    # now resize our bounding box (stretch it) to the coordinates of the output image 640x480
    bb = cv2.resize(bb,(REALSENSE_IMAGE_WIDTH, REALSENSE_IMAGE_HEIGHT), interpolation = cv2.INTER_CUBIC)
    # cv2.imshow('bb', bb)
    # cv2.waitKey(1)
    # print('resize: bb shape is %s'%str(bb.shape))
    # print('original image shape is %s'%str(depth.shape))
    # print(bb[250:260][250:260])
    sum0 =np.sum(bb, axis=0) # sum up the values across the x axis
    sum0[sum0!=0] =1	#set all nonzero values to 1

    if verbose:
        cv2.imshow('original image',depth)
        cv2.waitKey(100)
#        cv2.imshow('Cropped image',cropped)
#        cv2.waitKey(100)
    sum1 =np.sum(bb, axis=1) # sum up the values across the y axis
    sum1[sum1!=0] =1#set all nonzero values to 1

    #
    lr = np.multiply(sum0, range(0,REALSENSE_IMAGE_WIDTH)) # element-wise multiply the x axis sum with [0:640]
    lr = list(filter(lambda a: a!=0, lr))  # remove all zero values

    l = int(np.amin(lr)) # left side of box = minimum value of left-right
    r =  int(np.amax(lr)) # right side of box = max value of left-right

    tb = np.multiply(sum1, range(0,REALSENSE_IMAGE_HEIGHT)) # element-wise multiply the y axis sum with [0:480]
    tb = list(filter(lambda a: a!=0, tb)) # remove all zero values
    t =  int(np.amin(tb)) # top of box = minimum value of tb
    b =  int(np.amax(tb)) # bottom of box = max value of tb

    print("Left: %d\n Right: %d\n Top: %d\n bottom: %d"%(l,r,t,b))

    cropped = depth[t:b, l:r]    # select a rectangle from the depth image that is cropped to x = [t:b] and y = [l:r]


    #print(cropped[0:20])
    # remove zero values to leave a valid array of depths of the person
    values = list(filter(lambda y: y!=0, cropped.flatten()))

    # print(values[:20])PointCloud
    #print('Values contains some nans: '+str(any(np.isnan(values))))
    target_dist = np.nanmean(values)
    print("dist = %s"%str(target_dist))

    #this is the bit i need to change- we need to use transform coordinates....

    horiz_dev_from_normal = 0.5 - (l+r)/(2.0*REALSENSE_IMAGE_WIDTH) # horizontal deviation from normal (kinda an angle)
    vert_dev_from_normal = 0.5 - (t+b)/(2.0*REALSENSE_IMAGE_HEIGHT) # horizontal deviation from normal (kinda an angle)

    #print('angle (0.5 (left) to -0.5 (right): '+str(horiz_dev_from_normal))
#        print('angle (0.5 (top) to -0.5 (bottom): '+str(vert_dev_from_normal))
#        realsense_half_fov_angle = 69.4/2
#        realsense_half_fov_angle = REALSENSE_FOV_ANGLE/2

    #?????????????????????????????????????????????????????????????????????????????
    #realsense_half_fov_angle_vert = 42.5/2 #????????????????????????????????????????
    realsense_half_fov_angle_vert = 220*42.5/69.4/2 #????????????????????????????????????????
    #?????????????????????????????????????????????????????????????????????????????

    target_yaw = REALSENSE_FOV_ANGLE/2*horiz_dev_from_normal*np.pi/180.0 # in radians


    target_pitch = realsense_half_fov_angle_vert*vert_dev_from_normal*np.pi/180.0 # in radians

    print("yaw = %f, pitch = %f"%(target_yaw, target_pitch))
    target_x = target_dist*np.cos(target_yaw)*np.cos(target_pitch)
    target_y = target_dist*np.sin(target_yaw)*np.cos(target_pitch)
    target_z = target_dist*np.sin(target_pitch)
    #target_quat = Quaternion(target_yaw, 0.0, 0.0) # roll, pitch are both 0.0
    target_quat = quaternion_from_euler(0.0, target_pitch, target_yaw)
    target_vector = (target_x, target_y, target_z)
    #print(target_vector)

    #target_transform = Transform(target_quat, target_vector)
    camera_frame_id = "realsense_d435_forward_camera"
    target_frame_id = "realsense_person_est"

    br.sendTransform(target_vector, target_quat, rospy.Time.now(), target_frame_id, camera_frame_id)

    ###################################PLACE A MARKER
    marker = Marker()
    marker.header.frame_id = "base"

    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.pose.orientation.w = 1.0
    if verbose:
        print("Left: %d\n Right: %d\n Top: %d\n bottom: %d"%(l,r,t,b))
        # cv2.imshow('original image',depth)
        # cv2.waitKey(100)
        # cv2.imshow('Cropped image',cropped)
        # cv2.waitKey(100)
        #print('angle (0.5 (left) to -0.5 (right): '+str(horiz_dev_from_normal))
        print('angle (0.5 (top) to -0.5 (bottom): '+str(vert_dev_from_normal))


warnings.filterwarnings('ignore')
#start tensorflow
sess = tensorflow.InteractiveSession()
tensorflow.global_variables_initializer().run()
#start a CVBridge to move images around
bridge = CvBridge()


# point tensorflow to the NN weights
weights_path = 'multi_sensor_tracker/src/yolo_weights/yolov2-tiny-voc.weights'
output_image_path = './output.jpg'
ckpt_folder_path = '../drs-main/code/src/personal/person_tracker/scripts/ckpt/' # change this to a more reasonable folder
#ckpt_folder_path = '..' # not sure what this does

score_threshold = 0.3 # WHAT IS THIS????
iou_threshold = 0.3

scale_w, scale_h = REALSENSE_IMAGE_WIDTH*1.0/YOLO_IMAGE_INPUT_WIDTH, REALSENSE_IMAGE_HEIGHT*1.0/YOLO_IMAGE_INPUT_HEIGHT
skip = 0

#don't know what this does
roslib.load_manifest('person_tracker')

# Check for an existing checkpoint and load the weights (if it exists) or do it from binary file
saver = tensorflow.train.Saver()
_ = weights_loader.load(sess,weights_path,ckpt_folder_path,saver) #wtf is this???

if verbose:
        print('weights loaded. initiating CvBridge.')
bridge = CvBridge()

rospy.init_node('realsense_person_detector', anonymous=True)
if verbose:
        print('initialising node realsense_person_detector.')
bounding_box = []

# publish the person coordinates on an image
br = tf.TransformBroadcaster()

# publish the person centre coordinates on the image
image_pub = rospy.Publisher("/camera/person_detection_bounding_boxes",Image)

#subscribe to the realsense rgb input stream

image_topic = "realsense_d435_forward/color/image_raw/compressed"
sim_image_topic = "/realsense_d435_forward/rgb/image_raw/compressed"
image_sub = rospy.Subscriber(sim_image_topic, CompressedImage,rgb_callback)

#also subscribe to the depth stream

depth_topic = "/realsense_d435_forward/aligned_depth_to_color/image_raw/compressedDepth"
sim_depth_topic = "/realsense_d435_forward/depth/image_raw"
depth_sub = rospy.Subscriber(sim_depth_topic, Image, depth_callback)

if verbose:
   print('subscribers and publishers setup. Spinning ros.')

rospy.spin()
