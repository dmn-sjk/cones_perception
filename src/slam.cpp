#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include <math.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


class Slam{
private:
    // ros 
    ros::NodeHandle nh;
    ros::Subscriber cloud_sub;
    ros::Publisher cloud_pub;

    // frames
    std::string odom_frame_id = "lidar";
    std::string world_frame_id = "map";
    std::string cloud_frame_id = "cloud";
    
    // odometry
    ros::Time prev_time;
    Eigen::Matrix4f prev_trans;
    Eigen::Matrix4f keyframe_pose;
    ros::Time keyframe_stamp;
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr keyframe;
    pcl::PointCloud<pcl::PointXYZI>::Ptr prev_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud <pcl::PointXYZI>);



public:

	Slam(): nh("~"){
        //whole cloud
		cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/cloud", 2, &Slam::cloud_handler, this);

		cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/static_cloud", 1);
        
	}
    

    void update_pose(const Eigen::Matrix4f &trans_matrix){
        static tf::TransformBroadcaster br;
        static tf::TransformListener listener;

        tf::StampedTransform transform;
        tf::Quaternion tf_prev_quat;
        tf::Vector3 tf_prev_trans;

        Eigen::Quaterniond eigen_quat(trans_matrix.block<3,3>(0,0).cast<double>());
        Eigen::Vector3d eigen_trans(trans_matrix.block<3,1>(0,3).cast<double>());

        tf::Quaternion tf_quat(eigen_quat.x(), eigen_quat.y(), eigen_quat.z(), eigen_quat.w());
        tf::Vector3 tf_trans(eigen_trans(0), eigen_trans(1), eigen_trans(2));

        try{
            listener.lookupTransform("cloud", "lidar", ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s", ex.what());
        }

        tf_prev_trans = transform.getOrigin();
        tf_prev_quat = transform.getRotation();

        tf::Vector3 tf_fin_trans(tf_prev_trans.x() + tf_trans.x(), tf_prev_trans.y() + tf_trans.y(), tf_prev_trans.z() + tf_trans.z());
        tf::Quaternion tf_fin_quat(tf_prev_quat.x() + tf_quat.x(), tf_prev_quat.y() + tf_quat.y(), tf_prev_quat.z() + tf_quat.z(), tf_prev_quat.w() + tf_quat.w());
        
        std::cout<<"\n"<<tf_fin_quat.x()<<", "<<tf_fin_quat.y()<<", "<<tf_fin_quat.z()<<", "<<tf_fin_quat.w();

        transform.setOrigin(tf_fin_trans);
        transform.setRotation(tf_fin_quat);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "cloud", "lidar"));
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

        pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
        icp.setInputSource(cloud);
        icp.setInputTarget(keyframe);
        
        Eigen::Isometry3f msf_delta = Eigen::Isometry3f::Identity();
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(new pcl::PointCloud <pcl::PointXYZI>());
        icp.align(*aligned, prev_trans * msf_delta.matrix());

        if(!icp.hasConverged()){
            ROS_INFO("Not converged!!!");
            return keyframe_pose * prev_trans;
        }

        Eigen::Matrix4f transform = icp.getFinalTransformation();
        Eigen::Matrix4f odom = keyframe_pose * transform;

        prev_time = stamp;
        prev_trans = transform;


    }

	void cloud_handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg){
	    // Converting ROS message to point cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud <pcl::PointXYZI>);

        pcl::fromROSMsg(*cloud_msg, *input_cloud);

        auto cloud_filtered = downsample(input_cloud);

        if (prev_cloud->points.size() == 0){
            prev_cloud = cloud_filtered;
        }

        //pcl::PointCloud<pcl::PointXYZI>::Ptr static_cloud(new pcl::PointCloud <pcl::PointXYZI>);

        auto transform = match(cloud_msg->header.stamp, cloud_filtered);

        update_pose(transform);
        //pcl::transformPointCloud (*input_cloud, *static_cloud, transform);

        //sensor_msgs::PointCloud2 cloud_out;

        // ROS_INFO("distance: [%f]", this->euclidan_dist(this->cone_height, 2.0, this->cone_width));
        
        // Convert created point cloud to message
        //pcl::toROSMsg(*static_cloud, cloud_out);
        
        //cloud_out.header.frame_id = "cloud";

        // Publish message 
        //cloud_pub.publish(cloud_out);

        prev_cloud = input_cloud;
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
