#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <math.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <slam/utils.hpp>


class Slam{
private:
    // ros 
    ros::NodeHandle nh;
    ros::Subscriber cloud_sub;
    ros::Publisher cloud_pub;
    ros::Publisher aligned_cloud_pub;
    // ros::Publisher filtered_cloud_pub;
    ros::Publisher static_cloud_pub;
    tf::TransformBroadcaster tf_br;
    tf::TransformListener tf_ls;

    // frames
    std::string odom_frame_id = "lidar";
    std::string world_frame_id = "world";
    std::string cloud_frame_id = "cloud";
    std::string map_frame_id = "map";
    
    // odometry
    pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr registration;
    ros::Time prev_time;
    Eigen::Matrix4f prev_trans;
    Eigen::Matrix4f keyframe_pose;
    ros::Time keyframe_stamp;
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr keyframe;
    pcl::PointCloud<pcl::PointXYZI>::Ptr prev_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud <pcl::PointXYZI>);

    pcl::Filter<pcl::PointXYZI>::Ptr outlier_removal_filter;

    // The minimum tranlational distance and rotation angle between keyframes.
    // If this value is zero, frames are always compared with the previous frame
    double keyframe_delta_trans = 0.25;
    double keyframe_delta_angle = 0.15;
    double keyframe_delta_time = 1.0;
    double distance_near_thresh = 1.0;
    double distance_far_thresh = 100.0;


public:

	Slam(): nh("~"){
        //whole cloud
        tf_ls.waitForTransform("map", world_frame_id, ros::Time::now(), ros::Duration(3.0));
        tf_ls.waitForTransform("map", cloud_frame_id, ros::Time::now(), ros::Duration(3.0));
        tf_ls.waitForTransform("map", odom_frame_id, ros::Time::now(), ros::Duration(3.0));

        cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/cloud", 2, &Slam::cloud_handler, this);

        static_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/static_cloud", 1);
        // filtered_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_cloud", 1);
		aligned_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/aligned_cloud", 1);
        
        registration = choose_registration("icp");

        pcl::StatisticalOutlierRemoval<pcl::PointXYZI>::Ptr sor(new pcl::StatisticalOutlierRemoval<pcl::PointXYZI>());
        sor->setMeanK(20);
        sor->setStddevMulThresh(1.0);
        outlier_removal_filter = sor;
	}

    boost::shared_ptr<pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>> choose_registration(std::string reg){
        if (reg == "icp"){
            boost::shared_ptr<pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>> icp(new pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
            icp->setTransformationEpsilon(0.001); //0.01
            icp->setMaximumIterations(256); //64
            icp->setMaxCorrespondenceDistance(2.5);
            icp->setUseReciprocalCorrespondences(false);
            return icp;
        }
        else if (reg == "gicp"){
            boost::shared_ptr<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>> gicp(new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
            gicp->setTransformationEpsilon(0.01);
            gicp->setMaximumIterations(64);
            gicp->setUseReciprocalCorrespondences(false);
            gicp->setMaxCorrespondenceDistance(2.5);
            gicp->setCorrespondenceRandomness(20);
            gicp->setMaximumOptimizerIterations(20);
            return gicp;
        }
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr downsample(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud){
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud <pcl::PointXYZI>);
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.04f, 0.04f, 0.05f);
        vg.filter(*cloud_filtered);

        return cloud_filtered;
    }

    Eigen::Matrix4f match(const ros::Time &stamp, const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud){
        
        if(!keyframe) {
            prev_time = ros::Time();
            prev_trans.setIdentity();
            keyframe_pose.setIdentity();
            keyframe_stamp = stamp;
            keyframe = downsample(cloud);
            registration->setInputTarget(keyframe);
            return Eigen::Matrix4f::Identity();
        }
        
        
        registration->setInputSource(cloud);
        registration->setInputTarget(keyframe);
        
        Eigen::Isometry3f msf_delta = Eigen::Isometry3f::Identity();
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(new pcl::PointCloud <pcl::PointXYZI>());
        registration->align(*aligned, prev_trans * msf_delta.matrix());

        if(!registration->hasConverged()){
            ROS_INFO("Not converged!!!");
            return keyframe_pose * prev_trans;
        }

        Eigen::Matrix4f transform = registration->getFinalTransformation();
        Eigen::Matrix4f odom = keyframe_pose * transform;

        prev_time = stamp;
        prev_trans = transform;

        auto keyframe_trans = slam::matrix_to_trans(stamp, keyframe_pose, world_frame_id, odom_frame_id);
        //tf_br.sendTransform(keyframe_trans);

        double delta_trans = transform.block<3, 1>(0, 3).norm();
        double delta_angle = std::acos(Eigen::Quaternionf(transform.block<3, 3>(0, 0)).w());
        double delta_time = (stamp - keyframe_stamp).toSec();
        if(delta_trans > keyframe_delta_trans || delta_angle > keyframe_delta_angle || delta_time > keyframe_delta_time) {
            keyframe = cloud;
            registration->setInputTarget(keyframe);

            keyframe_pose = odom;
            keyframe_stamp = stamp;
            prev_time = stamp;
            prev_trans.setIdentity();
        }

        // pcl::transformPointCloud (*cloud, *aligned, odom);
        // aligned->header.frame_id=odom_frame_id;
        // aligned_cloud_pub.publish(aligned);

        return odom;
    }

    pcl::PointCloud<pcl::PointXYZI>::ConstPtr distance_filter(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud) const {
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
        filtered->reserve(cloud->size());

        std::copy_if(cloud->begin(), cloud->end(), std::back_inserter(filtered->points), [&](const pcl::PointXYZI& p) {
        double d = p.getVector3fMap().norm();
        return d > distance_near_thresh && d < distance_far_thresh;
        });

        filtered->width = filtered->size();
        filtered->height = 1;
        filtered->is_dense = false;

        filtered->header = cloud->header;

        return filtered;
    }

    pcl::PointCloud<pcl::PointXYZI>::ConstPtr outlier_removal(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud) const {

        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
        outlier_removal_filter->setInputCloud(cloud);
        outlier_removal_filter->filter(*filtered);
        filtered->header = cloud->header;

        return filtered;
    }

    void prefilter(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud){
        
        tf::StampedTransform transform;
        tf_ls.waitForTransform(odom_frame_id, cloud->header.frame_id, ros::Time(0), ros::Duration(2.0));
        tf_ls.lookupTransform(odom_frame_id, cloud->header.frame_id, ros::Time(0), transform);

        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>());
        pcl_ros::transformPointCloud(*cloud, *transformed, transform);
        transformed->header.frame_id = odom_frame_id;
        transformed->header.stamp = cloud->header.stamp;
        cloud = transformed;

        pcl::PointCloud<pcl::PointXYZI>::ConstPtr filtered = distance_filter(cloud);
        filtered = downsample(filtered);
        filtered = outlier_removal(filtered);

        //filtered_cloud_pub.publish(cloud);
    }

	void cloud_handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg){
	    // Converting ROS message to point cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud <pcl::PointXYZI>);

        pcl::fromROSMsg(*cloud_msg, *input_cloud);

        //prefilter(input_cloud);

        auto cloud_filtered = downsample(input_cloud);

        //auto cloud_filtered = input_cloud;

        // if (prev_cloud->points.size() == 0){
        //     prev_cloud = cloud_filtered;
        // }

        //pcl::PointCloud<pcl::PointXYZI>::Ptr static_cloud(new pcl::PointCloud <pcl::PointXYZI>);

        Eigen::Matrix4f pose = match(cloud_msg->header.stamp, cloud_filtered);
        geometry_msgs::TransformStamped odom_trans = slam::matrix_to_trans(cloud_msg->header.stamp, pose, world_frame_id, odom_frame_id);
        tf_br.sendTransform(odom_trans);

        //pcl::transformPointCloud (*input_cloud, *static_cloud, transform);

        //sensor_msgs::PointCloud2 cloud_out;

        // ROS_INFO("distance: [%f]", this->euclidan_dist(this->cone_height, 2.0, this->cone_width));
        
        // Convert created point cloud to message
        //pcl::toROSMsg(*static_cloud, cloud_out);
        
        //cloud_out.header.frame_id = "cloud";

        // Publish message 
        //cloud_pub.publish(cloud_out);

        //prev_cloud = input_cloud;
	}

	float euclidan_dist(float x, float y, float z){
		return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
	}
};

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "slam");
  	Slam slam;
    ros::spin();

    return 0;
}
