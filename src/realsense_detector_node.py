#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import realsense_detector

def main():

    r = rospy.init_node('realsense_person_detector', anonymous=True)
    rd = realsense_detector.RealsenseDetector(in_simulator = False, verbose = True) # create a new realsense detector

    # rate = rospy.Rate(2) # set rate to 2 Hz
    rospy.spin()

main()
