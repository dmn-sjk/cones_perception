#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <math.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <perception_handling/utils.hpp>
#include <perception_handling/color_classifier.hpp>
#include "slam/ClassifyColorSrv.h"


class ConeDetector{
private:
	float cone_width = 0.228;
	float cone_height = 0.325;

	float lidar_hor_res = 0.25; //degrees
	float lidar_ver_res = 7.5;

public:

	ConeDetector(): nh("~"){
		cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/cloud", 2, &ConeDetector::cloud_handler, this);
		cones_pub = nh.advertise<sensor_msgs::PointCloud2>("/cones_cloud", 1);
		color_srv_client = nh.serviceClient<slam::ClassifyColorSrv>("/color_classifier");
	}

	ros::NodeHandle nh;
    ros::Subscriber cloud_sub;
    ros::Publisher cones_pub;
	ros::ServiceClient color_srv_client;
	slam::ClassifyColorSrv color_srv;

	void cloud_handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg){

        pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud <pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud <pcl::PointXYZI>);

        pcl::fromROSMsg(*cloud_msg, *input_cloud);

		int cloud_size = input_cloud->points.size();

		// Deleting points too far away, too close and limiting horizontal FoV
		filter_points_position(input_cloud);
		
		//input_cloud->resize(cloud_size);

		cloud_filtered = downsample(input_cloud);

  		std::vector<pcl::PointIndices> cluster_indices;
		cluster_indices = euclidan_cluster(cloud_filtered);

	  	pcl::PointCloud<pcl::PointXYZI>::Ptr centroid_cloud (new pcl::PointCloud<pcl::PointXYZI>);
		
		centroid_cloud = get_centroid_cloud(cloud_filtered, cluster_indices);

        sensor_msgs::PointCloud2 cones_cloud;
        
        pcl::toROSMsg(*centroid_cloud, cones_cloud);
		//pcl::toROSMsg(*input_cloud, cones_cloud);
        
		cones_cloud.header = cloud_msg->header;
		cones_cloud.fields = cloud_msg->fields;

        cones_pub.publish(cones_cloud);
	}

	void filter_points_position(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud){
        cloud->points.erase(std::remove_if(cloud->points.begin(), cloud->points.end(),
            [&](pcl::PointXYZI p){return p.z < -0.5 or perception_handling::euclidan_dist(p.x, p.y, p.z) > 12 or
			perception_handling::euclidan_dist(p.x, p.y, p.z) < 0.7 or -10 * M_PI / 18 >= atan2(p.y, p.x) or
			atan2(p.y, p.x) >= 10 * M_PI / 18;}), cloud->points.end());
     }

	std::vector<pcl::PointIndices> euclidan_cluster(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud){
		pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZI>);
  		kdtree->setInputCloud (cloud);

        std::vector<pcl::PointIndices> cluster_indices;
	  	pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
	  	ec.setClusterTolerance (sqrt(pow(this->cone_height, 2) + pow(this->cone_width / 2, 2))); 
	  	ec.setMinClusterSize (3);
	  	ec.setMaxClusterSize (50);
	  	ec.setSearchMethod (kdtree);
	  	ec.setInputCloud (cloud);
	  	ec.extract (cluster_indices);

        return cluster_indices;
    }

	pcl::PointCloud<pcl::PointXYZI>::Ptr downsample(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud){
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud <pcl::PointXYZI>);
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.04f, 0.04f, 0.05f);
        vg.filter(*cloud_filtered);

        return cloud_filtered;
    }

	pcl::PointCloud<pcl::PointXYZI>::Ptr get_centroid_cloud(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud, std::vector<pcl::PointIndices> cluster_indices) {
		pcl::PointCloud<pcl::PointXYZI>::Ptr centroid_cloud (new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr single_cone_cloud (new pcl::PointCloud<pcl::PointXYZI>);

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
		{
			pcl::PointXYZI p;
			int j = 0;
			float x, y = 0.0;
			for (const auto& idx : it->indices){
				single_cone_cloud->push_back((*cloud)[idx]);
				x += (*cloud)[idx].x;
				y += (*cloud)[idx].y;
				j++;
			}

			get_color(single_cone_cloud);
			single_cone_cloud->clear();

			p.x = x / j;
			p.y = y / j;
			p.z = 0.0;
			p.intensity = 1.0;

			centroid_cloud->push_back(p);

			j = 0;
			x = 0.0;
			y = 0.0;
		}

		return centroid_cloud;
    }

	perception_handling::Color get_color(pcl::PointCloud<pcl::PointXYZI>::Ptr &single_cone_cloud) {
		perception_handling::Color color;
		sensor_msgs::PointCloud2 single_cone_msg;

        pcl::toROSMsg(*single_cone_cloud, single_cone_msg);

		color_srv.request.single_cone_cloud = single_cone_msg;
		
		if (color_srv_client.call(color_srv)) {
			color = static_cast<perception_handling::Color>(color_srv.response.color);
			ROS_INFO("Color: %i", (int)color);
		} else {
			ROS_ERROR("Failed to call service");
		}

		return color;
	}
};


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "cone_detector");
  	ConeDetector detector;
	ROS_INFO("Ready to detect cones.");
    ros::spin();

    return 0;
}