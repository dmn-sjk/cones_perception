#!/usr/bin/env python3

from __future__ import print_function

from slam.srv import ClassifyColorSrv, ClassifyColorSrvResponse
import rospy

kab = 0

def handle_classify_color(req):
    global kab 
    kab += 1
    if kab >= 200:
        kab = 0

    # TODO: Color classification

    print("Returning: %s"%(kab))
    return ClassifyColorSrvResponse(kab)

def classify_color_server():
    rospy.init_node('classify_color_server')
    s = rospy.Service('/color_classifier', ClassifyColorSrv, handle_classify_color)
    print("Ready to classify color.")
    rospy.spin()

if __name__ == "__main__":
    classify_color_server()