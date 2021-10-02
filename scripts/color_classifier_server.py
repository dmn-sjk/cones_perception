#!/usr/bin/env python3

from __future__ import print_function

from slam.srv import ClassifyColorSrv, ClassifyColorSrvResponse
import pcl
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import rospy


class ColorClassifier:
    def __init__(self):
        self.srv = rospy.Service('/color_classifier', ClassifyColorSrv, self.handle_classify_color)
        self.pub = rospy.Publisher('/color_cones', PointCloud2, queue_size=10)

        self.kab = 0

    def handle_classify_color(self, req):
        self.kab += 1
        if self.kab >= 200:
            self.kab = 0

        # ------------------------------------------------
        # TODO: Color classification
        self.pub.publish(req.single_cone_cloud)
        # ------------------------------------------------

        rospy.loginfo("Returning: %s"%(self.kab))
        return ClassifyColorSrvResponse(self.kab)

    def run_server(self):
        rospy.init_node('classify_color_server')
        rospy.loginfo("Ready to classify color.")
        rospy.spin()

if __name__ == "__main__":
    color_classifier = ColorClassifier()
    color_classifier.run_server()
