#!/usr/bin/env python3

from __future__ import print_function

from cones_perception.srv import ClassifyColorSrv, ClassifyColorSrvResponse
import pcl
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs import point_cloud2
import rospy
import pandas as pd


class ColorClassifier:
    def __init__(self):
        self.srv = rospy.Service('/color_classifier', ClassifyColorSrv, self.handle_classify_color)
        self.pub = rospy.Publisher('/color_cones', PointCloud2, queue_size=10)
        
        self.df = pd.DataFrame({'x':[], 'y':[], 'z':[], 'intensity':[],'color':[]})

        self.kab = 0


    def handle_classify_color(self, req):
        self.kab += 1
        if self.kab >= 200:
            self.kab = 0

        # ------------------------------------------------
        # TODO: Color classification


        self.pub.publish(req.single_cone_cloud)

        # -- DATA COLLECTION -------------------------
        # cones_cloud = list(pc2.read_points(req.single_cone_cloud))

        # row = {'x':[], 'y':[], 'z':[], 'intensity':[],'color':0}
        # for p in cones_cloud:
        #     row['x'].append(p[0])
        #     row['y'].append(p[1])
        #     row['z'].append(p[2])
        #     row['intensity'].append(p[3])

        # color = input("Choose color (0 - not sure, 1 - yellow, 2 - blue, 3 - orange):\n")

        # if not color == '0':
        #     row['color'] = color

        #     self.df = self.df.append(row, ignore_index=True)

        #     # compression_opts = dict(method='zip', archive_name='out.csv')  

        #     self.df.to_pickle('src/cones_perception/cones_clouds/cones.pkl')

        # -- END DATA COLLECTION -------------------------


        # ------------------------------------------------

        # rospy.loginfo("Returning: %s"%(self.kab))
        # rospy.loginfo("Choose color: %s"%(self.kab))

        return ClassifyColorSrvResponse(self.kab)

    def run_server(self):
        rospy.init_node('classify_color_server')
        rospy.loginfo("Ready to classify color.")
        rospy.spin()

if __name__ == "__main__":
    color_classifier = ColorClassifier()
    color_classifier.run_server()
