#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import realsense_detector

def main():

    rospy.init_node('realsense_person_detector', anonymous=True)
    rd = realsense_detector.RealsenseDetector(True, True) # create a new realsense detector

    rospy.spin()

main()
