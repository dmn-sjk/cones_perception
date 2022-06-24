#!/usr/bin/env python3

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
import pathlib
import os


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

# intensity_interp = interp1d([0, 100], [0, 255])
intensity_interp = interp1d([0, 255], [0, 255])


data_collection = False


class ColorClassifier:
    def __init__(self):
        rospy.init_node('color_classifier_server')
        model_path = rospy.get_param("~model_path", None)
        is_model_tf_lite = rospy.get_param("~is_model_tf_lite", True)

        if model_path is None:
            raise ValueError("Specify path to the model!")

        if not is_model_tf_lite:
            # convert to tf lite
            model = tf.keras.models.load_model(model_path)
            converter = tf.lite.TFLiteConverter.from_keras_model(model)
            tflite_model = converter.convert()

            tf_lite_model_path = '/tmp/color_classifier_tflite_model/'
            tf_lite_model_name = 'color_classifier_model.tflite'

            tflite_models_dir = pathlib.Path(tf_lite_model_path)
            tflite_models_dir.mkdir(exist_ok=True, parents=True)

            tflite_model_file = tflite_models_dir/tf_lite_model_name
            tflite_model_file.write_bytes(tflite_model)

            model_path =  os.path.join(tf_lite_model_path, tf_lite_model_name)

        self.interpreter = tf.lite.Interpreter(model_path)
        self.interpreter.allocate_tensors()

        self.model_input_index = self.interpreter.get_input_details()[0]["index"]
        self.model_output_index = self.interpreter.get_output_details()[0]["index"]

        self.df = pd.DataFrame({'x':[], 'y':[], 'z':[], 'intensity':[],'color':[]})
        self.colors = [None, 'yellow', 'blue', 'orange']

        self.srv = rospy.Service('color_classifier', ClassifyColorSrv, self.handle_classify_color)

    def handle_classify_color(self, req):
        colors = []
        for cone in req.cones_clouds:
            cone_cloud = list(pc2.read_points(cone))

            if len(cone_cloud) == 0:
                continue

            row = {'x':[], 'y':[], 'z':[], 'intensity':[],'color':0}
            for p in cone_cloud:
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

                image = tf.constant(image[np.newaxis, ...], dtype=tf.float32)
                self.interpreter.set_tensor(self.model_input_index, image)
                self.interpreter.invoke()
                pred = self.interpreter.get_tensor(self.model_output_index)

                if np.max(pred) >= 0.8:
                    colors.append(np.argmax(pred) + 1)
                else:
                    # unknown
                    colors.append(0)

                rospy.loginfo("Classified: %s"%(self.colors[colors[-1]]))

        return ClassifyColorSrvResponse(colors)

    def run_server(self):
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
        slope_horiz = (IMG_COLS - 1) / (max_horiz_angle - min_horiz_angle + 1e-16)
        img_poses_h = (np.round(slope_horiz * (horiz_angles - min_horiz_angle))).astype(int)

        image = np.zeros((IMG_ROWS, IMG_COLS, 1), np.uint8)

        image[img_poses_v, img_poses_h, 0] = intensity_interp(intensities)

        return image

if __name__ == "__main__":
    color_classifier = ColorClassifier()
    color_classifier.run_server()
