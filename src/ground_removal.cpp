#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <math.h>
#include <pcl/search/kdtree.h>
#include <perception_handling/utils.hpp>
#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>



class GroundRemover{
private:
	// float lidar_aperture_angle = 275;
	int num_of_sectors = 16;
	float sector_angle_rad = (360 / num_of_sectors) * M_PI / 180;

	ros::NodeHandle nh;
    ros::Subscriber cloud_sub;
    ros::Publisher groundless_cloud_pub;

	std::string input_cloud_topic;
	std::string groundless_cloud_topic;

public:
	GroundRemover(): nh("~"){
		nh.getParam("input_cloud_topic", input_cloud_topic);
		nh.getParam("groundless_cloud_topic", groundless_cloud_topic);

		cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(input_cloud_topic, 2, &GroundRemover::cloud_handler, this);
		groundless_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(groundless_cloud_topic, 1);
	}

	void cloud_handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg){

        pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud <pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_copy(new pcl::PointCloud <pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud <pcl::PointXYZI>);

        pcl::fromROSMsg(*cloud_msg, *input_cloud);

		int cloud_size = input_cloud->points.size();
		
		pcl::copyPointCloud(*input_cloud, *input_cloud_copy);


		std::vector<float> sectors_lowest_points(num_of_sectors, -0.1);
		for (std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI>>::const_iterator it = input_cloud->points.begin(); 
					it != input_cloud->points.end(); it++) {
			float atan_angle = atan2(it->y, it->x);
			float angle = (atan_angle < 0) ? atan_angle += 2 * M_PI : atan_angle;

			int sector = floor(angle / sector_angle_rad);
			if (sectors_lowest_points[sector] > it->z) {
				sectors_lowest_points[sector] = it->z;
			}
		}

		input_cloud_copy->points.erase(std::remove_if(input_cloud_copy->points.begin(), input_cloud_copy->points.end(),
										   [&](pcl::PointXYZI p) {
												float atan_angle = atan2(p.y, p.x);
												float angle = (atan_angle < 0) ? atan_angle += 2 * M_PI : atan_angle;
												int sector = floor(angle / sector_angle_rad);
												// if (sector == 0) {
												// 	return true;
												// } else {
												// 	return false;
												// }
											   return p.z < sectors_lowest_points[sector] + 0.1;
										   }),
							input_cloud_copy->points.end());




		// input_cloud_copy->points.erase(std::remove_if(input_cloud_copy->points.begin(), input_cloud_copy->points.end(),
		// 								   [&](pcl::PointXYZI p) {
		// 									   return -20 * M_PI / 180 >= atan2(p.y, p.x);
		// 								   }),
		// 					input_cloud_copy->points.end());


		input_cloud_copy->resize(cloud_size);

        sensor_msgs::PointCloud2 cones_cloud;

		cones_cloud.header = cloud_msg->header;
		cones_cloud.fields = cloud_msg->fields;

		pcl::toROSMsg(*input_cloud_copy, cones_cloud);

		groundless_cloud_pub.publish(cones_cloud);
	}
};


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "ground_remover");
  	GroundRemover ground_remover;
	ROS_INFO("Ready to remove ground.");
    ros::spin();

    return 0;
}