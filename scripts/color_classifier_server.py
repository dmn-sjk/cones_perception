#!/usr/bin/env python3

from __future__ import print_function

from cones_perception.srv import ClassifyColorSrv, ClassifyColorSrvResponse
# import pcl
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs import point_cloud2
import rospy
import pandas as pd
import tensorflow as tf
import numpy as np
from scipy.interpolate import interp1d


# specified FoV in cone detection (h angles IMPORTANT to match the one in cone_detection.cpp (h_angles = 1/2 * angle threshold in cone detection))
MAX_H_ANGLE = 80
MIN_H_ANGLE = -80
H_RESOLUTION = 0.1  # (0.1 - 0.4)
NUM_OF_POINTS_H = (MAX_H_ANGLE - MIN_H_ANGLE) / H_RESOLUTION

# from lidar specs
MAX_V_ANGLE = 15
MIN_V_ANGLE = -15
V_RESOLUTION = 2
NUM_OF_LASER_LEVELS_V = int((MAX_V_ANGLE - MIN_V_ANGLE) / V_RESOLUTION)

IMG_ROWS = NUM_OF_LASER_LEVELS_V    # 15
IMG_COLS = 12

slope_vert = IMG_ROWS / (MIN_V_ANGLE - MAX_V_ANGLE)

intensity_interp = interp1d([0, 100], [0, 255])


data_collection = False


class ColorClassifier:
    def __init__(self):
        self.srv = rospy.Service('/color_classifier', ClassifyColorSrv, self.handle_classify_color)

        self.df = pd.DataFrame({'x':[], 'y':[], 'z':[], 'intensity':[],'color':[]})

        self.model = tf.keras.models.load_model('/home/damian/.workspaces_ros/PUTM_DV_Perception_2020/perception_catkin/src/cones_perception/models/dam_net')

        self.colors = [None, 'yellow', 'blue', 'orange']

    def handle_classify_color(self, req):

        cones_cloud = list(pc2.read_points(req.single_cone_cloud))

        if len(cones_cloud) == 0:
            return ClassifyColorSrvResponse(self.colors[0])

        row = {'x':[], 'y':[], 'z':[], 'intensity':[],'color':0}
        for p in cones_cloud:
            row['x'].append(p[0])
            row['y'].append(p[1])
            row['z'].append(p[2])
            row['intensity'].append(p[3])
        
        # -- DATA COLLECTION -------------------------

        if data_collection:

            color = int(input("Choose color (0 - not sure, 1 - yellow, 2 - blue, 3 - orange):\n"))

            if not color == 0:
                row['color'] = color

                self.df = self.df.append(row, ignore_index=True)

                self.df.to_pickle('src/cones_perception/cones_clouds/cones.pkl')

        # -- END DATA COLLECTION -------------------------

        else:            
            image = self.to_image(row)

            pred = self.model.predict(image[np.newaxis, ...])

            if np.max(pred) >= 0.8:
                color = np.argmax(pred) + 1
            else:
                # unknown
                color = 0

        rospy.loginfo("Returning: %s"%(self.colors[color]))

        return ClassifyColorSrvResponse(color)

    def run_server(self):
        rospy.init_node('classify_color_server')
        rospy.loginfo("Ready to classify color.")
        rospy.spin()

    @staticmethod
    def to_image(cone_cloud_dict):
        X = np.array(cone_cloud_dict['x'])
        Y = np.array(cone_cloud_dict['y'])
        Z = np.array(cone_cloud_dict['z'])
        intensities = np.array(cone_cloud_dict['intensity'])

        # -- vertical pos --
        vert_angles = np.degrees(np.arctan2(Z, np.sqrt(pow(X, 2.0) + pow(Y, 2.0))))

        img_poses_v = (np.round(slope_vert * (vert_angles - MIN_V_ANGLE))).astype(int)

        # -- horizontal  pos --
        horiz_angles = np.degrees(np.arctan2(Y, X))
        min_horiz_angle = np.min(horiz_angles)
        max_horiz_angle = np.max(horiz_angles)
        # horizontal slope is dependent on range of current angle, cuz horizontal resolution of lidar in not regular,
        # so when it is calculated like vertical one, there are blank pixels in between points (horizontally), which
        # I think could be a problem while teaching neural net.
        slope_horiz = (IMG_COLS - 1) / (max_horiz_angle - min_horiz_angle)
        img_poses_h = (np.round(slope_horiz * (horiz_angles - min_horiz_angle))).astype(int)

        image = np.zeros((IMG_ROWS, IMG_COLS, 1), np.uint8)

        image[img_poses_v, img_poses_h, 0] = intensity_interp(intensities)

        return image

if __name__ == "__main__":
    color_classifier = ColorClassifier()
    color_classifier.run_server()
