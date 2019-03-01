#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import realsense_detector

def main():

    rospy.init_node('realsense_person_detector', anonymous=True)
    rd = realsense_detector.RealsenseDetector(in_simulator = False, verbose = True) # create a new realsense detector

    rospy.spin()

main()
