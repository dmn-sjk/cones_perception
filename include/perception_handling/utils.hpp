#ifndef UTILS_HPP
#define UTILS_HPP

#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include "sensor_msgs/PointCloud2.h"

namespace perception_handling {

    geometry_msgs::TransformStamped matrix_to_trans(const ros::Time &stamp, const Eigen::Matrix4f& matrix, const std::string& frame_id, const std::string& child_frame_id);
    
    float euclidan_dist(float x1, float y1, float z1, float x2, float y2, float z2);

    bool intensity_in_cloud(sensor_msgs::PointCloud2ConstPtr cloud_msg);

} // namespace perception_handling

#endif  // UTILS_HPP
