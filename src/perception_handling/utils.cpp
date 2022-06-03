#include <Eigen/Dense>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <perception_handling/utils.hpp>

namespace perception_handling {

    geometry_msgs::TransformStamped matrix_to_trans(const ros::Time &stamp, const Eigen::Matrix4f &matrix,
                                                        const std::string &frame_id, const std::string &child_frame_id) {
        Eigen::Quaternionf quat(matrix.block<3, 3>(0, 0));
        quat.normalize();
        geometry_msgs::Quaternion rotation;
        rotation.x = quat.x();
        rotation.y = quat.y();
        rotation.z = quat.z();
        rotation.w = quat.w();

        geometry_msgs::TransformStamped transform;
        transform.header.stamp = stamp;
        transform.header.frame_id = frame_id;
        transform.child_frame_id = child_frame_id;
        transform.transform.translation.x = matrix(0, 3);
        transform.transform.translation.y = matrix(1, 3);
        transform.transform.translation.z = matrix(2, 3);
        transform.transform.rotation = rotation;

        return transform;
    }

    float euclidan_dist(float x1, float y1, float z1, float x2, float y2, float z2) {
        return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2));
    }

    bool intensity_in_cloud(sensor_msgs::PointCloud2ConstPtr cloud_msg) {
		for (auto field : cloud_msg->fields) {
			if (field.name == "intensity") {
				return true;
			}
		}
        return false;
    }

}