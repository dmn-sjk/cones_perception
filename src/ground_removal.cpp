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
#include <pcl/io/pcd_io.h>



class GroundRemover{
private:
	int num_of_sectors = 16;
	float default_lowest_point = -0.1;
	float sector_angle_rad = (360 / num_of_sectors) * M_PI / 180;

	ros::NodeHandle nh;
    ros::Subscriber cloud_sub;
    ros::Publisher groundless_cloud_pub;

	std::string input_cloud_topic = "/cloud";
	std::string groundless_cloud_topic = "groundless_cloud";

public:
	GroundRemover(): nh() {
		if (!ros::param::get("~output_cloud_topic", groundless_cloud_topic)) {
			ROS_INFO("output_cloud_topic param not found, setting to default: %s", groundless_cloud_topic.c_str());
		}
		if (!ros::param::get("~num_of_sectors", num_of_sectors)) {
			ROS_INFO("num_of_sectors param not found, setting to default: %d", num_of_sectors);
		}
		if (!ros::param::get("~default_lowest_point", default_lowest_point)) {
			ROS_INFO("default_lowest_point param not found, setting to default: %f", default_lowest_point);
		}

		cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(input_cloud_topic, 2, &GroundRemover::cloud_handler, this);
		groundless_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(groundless_cloud_topic, 1);
	}

	void run() {
		ROS_INFO("Ready to remove ground.");
		ros::spin();
	}

	void cloud_handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg){
		PointCloud::Ptr input_cloud(new PointCloud);
        PointCloud::Ptr cloud_filtered(new PointCloud);

        pcl::fromROSMsg(*cloud_msg, *input_cloud);

		int cloud_size = input_cloud->points.size();
		
		std::vector<float> sectors_lowest_points(num_of_sectors, default_lowest_point);
		for (std::vector<Point, Eigen::aligned_allocator<Point>>::const_iterator it = input_cloud->points.begin(); 
					it != input_cloud->points.end(); it++) {
			float atan_angle = atan2(it->y, it->x);
			float angle = (atan_angle < 0) ? atan_angle += 2 * M_PI : atan_angle;

			int sector = floor(angle / sector_angle_rad);
			if (sectors_lowest_points[sector] > it->z) {
				sectors_lowest_points[sector] = it->z;
			}
		}

		input_cloud->points.erase(std::remove_if(input_cloud->points.begin(), input_cloud->points.end(),
										   [&](Point p) {
												float atan_angle = atan2(p.y, p.x);
												float angle = (atan_angle < 0) ? atan_angle += 2 * M_PI : atan_angle;
												int sector = floor(angle / sector_angle_rad);
											   return p.z < sectors_lowest_points[sector] + 0.1;
										   }),
							input_cloud->points.end());

		input_cloud->resize(cloud_size);

        sensor_msgs::PointCloud2 cones_cloud;

		cones_cloud.header = cloud_msg->header;
		cones_cloud.fields = cloud_msg->fields;

		pcl::toROSMsg(*input_cloud, cones_cloud);

		groundless_cloud_pub.publish(cones_cloud);
	}
};


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "ground_remover");
  	GroundRemover ground_remover;
	ground_remover.run();

    return 0;
}