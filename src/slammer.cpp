#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include <math.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <perception_handling/utils.hpp>


class Slam{
private:
    // ros 
    ros::NodeHandle nh;
    ros::Subscriber cloud_sub;
    ros::Publisher cloud_pub;
    ros::Publisher aligned_cloud_pub;
    tf::TransformBroadcaster tf_br;
    tf::TransformListener tf_ls;

    // frames
    std::string odom_frame_id = "lidar";
    std::string world_frame_id = "map";
    std::string cloud_frame_id = "cloud";
    
    // odometry
    pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr registration;
    ros::Time prev_time;
    Eigen::Matrix4f prev_trans;
    Eigen::Matrix4f keyframe_pose;
    ros::Time keyframe_stamp;
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr keyframe;
    pcl::PointCloud<pcl::PointXYZI>::Ptr prev_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud <pcl::PointXYZI>);

    // The minimum tranlational distance and rotation angle between keyframes.
    // If this value is zero, frames are always compared with the previous frame
    double keyframe_delta_trans = 0.25;
    double keyframe_delta_angle = 0.15;
    double keyframe_delta_time = 1.0;


public:

	Slam(): nh("~"){
        //whole cloud
        try{
            tf_ls.waitForTransform("/map", "/cloud", ros::Time::now(), ros::Duration(3.0));
        }
        catch(tf::TransformException ex){
            ROS_ERROR("/map -> cloud transform not found");
        }

        cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/cloud", 2, &Slam::cloud_handler, this);

		aligned_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/aligned_cloud", 1);
        
        registration = choose_registration("icp");
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

        //auto keyframe_trans = slam::matrix_to_trans(stamp, keyframe_pose, world_frame_id, odom_frame_id);
        geometry_msgs::TransformStamped a;
        auto keyframe_trans = a;
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

        pcl::transformPointCloud (*cloud, *aligned, odom);
        aligned->header.frame_id=odom_frame_id;
        aligned_cloud_pub.publish(aligned);

        return odom;
    }

	void cloud_handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg){
	    // Converting ROS message to point cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud <pcl::PointXYZI>);

        pcl::fromROSMsg(*cloud_msg, *input_cloud);

        auto cloud_filtered = downsample(input_cloud);

        // if (prev_cloud->points.size() == 0){
        //     prev_cloud = cloud_filtered;
        // }

        //pcl::PointCloud<pcl::PointXYZI>::Ptr static_cloud(new pcl::PointCloud <pcl::PointXYZI>);

        Eigen::Matrix4f pose = match(cloud_msg->header.stamp, cloud_filtered);
        geometry_msgs::TransformStamped odom_trans = perception_handling::matrix_to_trans(cloud_msg->header.stamp, pose, "map", cloud_msg->header.frame_id);
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
};

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "slam");
  	Slam slam;
    ros::spin();

    return 0;
}
