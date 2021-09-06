#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <math.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

class GroundRemover{
private:
	float cone_width = 0.228;
	float cone_height = 0.325;

	float lidar_hor_res = 0.25; //degrees
	float lidar_ver_res = 7.5;

public:

	GroundRemover(): nh("~"){
		//cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/hdl_graph_slam/map_points", 2, &GroundRemover::cloud_handler, this);
		cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/cloud", 2, &GroundRemover::cloud_handler, this);

		cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/groundless_cloud", 1);

	}

	ros::NodeHandle nh;
    ros::Subscriber cloud_sub;
    ros::Publisher cloud_pub;

	void cloud_handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg){

        pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud <pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr groundless_cloud(new pcl::PointCloud <pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud <pcl::PointXYZI>);

        pcl::fromROSMsg(*cloud_msg, *input_cloud);

		int cloud_size = input_cloud->points.size();

		// Deleting points too far away, too close and limiting horizontal FoV
        // input_cloud->points.erase(std::remove_if(input_cloud->points.begin(), input_cloud->points.end(),
        //     [&](pcl::PointXYZI p){return p.z < -0.5 or this->euclidan_dist(p.x, p.y, p.z) > 4 or
		// 	this->euclidan_dist(p.x, p.y, p.z) < 0.7 or -10 * M_PI / 18 >= atan2(p.y, p.x) or
		// 	atan2(p.y, p.x) >= 10 * M_PI / 18 or atan2(p.z, sqrt(pow(p.x, 2.0) + pow(p.y, 2.0))) * 180.0 / M_PI < 0;}), input_cloud->points.end());

		// input_cloud->points.erase(std::remove_if(input_cloud->points.begin(), input_cloud->points.end(),
        //     [&](pcl::PointXYZI p){return atan2(p.z, sqrt(pow(p.x, 2.0) + pow(p.y, 2.0))) * 180.0 / M_PI < -2.0;}), input_cloud->points.end());

		pcl::PointXYZI bottomest_point;
		bottomest_point.z = 0;
		for (pcl::PointCloud<pcl::PointXYZI>::const_iterator pit = input_cloud->begin(); pit != input_cloud->end(); ++pit){
			if (pit->z < bottomest_point.z){
				bottomest_point = *pit;
			}
		}
		float dist_to_car = euclidan_dist(bottomest_point.x, bottomest_point.y, bottomest_point.z);
		for (pcl::PointCloud<pcl::PointXYZI>::const_iterator pit = input_cloud->begin(); pit != input_cloud->end(); ++pit){
			if (atan2(pit->z, sqrt(pow(pit->x, 2.0) + pow(pit->y, 2.0))) * 180.0 / M_PI < -2.0 and abs(euclidan_dist(pit->x, pit->y, pit->z) - dist_to_car) < 8){
				//groundless_cloud->points.push_back(*pit); 
			} else {
				groundless_cloud->points.push_back(*pit); 
			}
		}

		//input_cloud->resize(cloud_size);
        
		sensor_msgs::PointCloud2 groundless_cloud_msg;

        pcl::toROSMsg(*groundless_cloud, groundless_cloud_msg);
        
		groundless_cloud_msg.header = cloud_msg->header;
		groundless_cloud_msg.fields = cloud_msg->fields;

        cloud_pub.publish(groundless_cloud_msg);
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

	float euclidan_dist(float x, float y, float z){
		return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
	}
};

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "cone_detector");
  	GroundRemover remover;
    ros::spin();

    return 0;
}