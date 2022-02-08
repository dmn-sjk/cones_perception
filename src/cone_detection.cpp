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
#include "cones_perception/ClassifyColorSrv.h"


class ConeDetector{
private:

	float cone_width = 0.228;
	float cone_height = 0.325;
	float lidar_hor_res = 0.25; //degrees
	float lidar_ver_res = 7.5;

	double distance_treshold_max;
	double distance_treshold_min;
	double level_threshold;
	double angle_threshold;

	int min_cluster_size;
	int max_cluster_size;

	bool classify_colors;
	double cones_matching_dist_theshold;

	ros::NodeHandle nh;
    ros::Subscriber cloud_sub;
    ros::Publisher cones_pub;
	ros::ServiceClient color_srv_client;

	cones_perception::ClassifyColorSrv color_srv;
	pcl::PointCloud<pcl::PointXYZ>::Ptr prev_centroid_cloud;

	std::string cones_frame_id;
	std::string input_cloud_topic;
	std::string cones_topic;
	std::string color_classifier_srv_name;

public:
	ConeDetector(): nh("~"){
		nh.getParam("cones_frame_id", cones_frame_id);
		nh.getParam("input_cloud_topic", input_cloud_topic);
		nh.getParam("cones_topic", cones_topic);
		nh.getParam("classify_colors", classify_colors);
		nh.getParam("color_classifier_srv_name", color_classifier_srv_name);
		nh.getParam("distance_treshold_max", distance_treshold_max);
		nh.getParam("distance_treshold_min", distance_treshold_min);
		nh.getParam("level_threshold", level_threshold);
		nh.getParam("angle_threshold", angle_threshold);
		nh.getParam("min_cluster_size", min_cluster_size);
		nh.getParam("max_cluster_size", max_cluster_size);
		nh.getParam("cones_matching_dist_theshold", cones_matching_dist_theshold);

		cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(input_cloud_topic, 2, &ConeDetector::cloud_handler, this);
		cones_pub = nh.advertise<sensor_msgs::PointCloud2>(cones_topic, 1);
		color_srv_client = nh.serviceClient<cones_perception::ClassifyColorSrv>(color_classifier_srv_name);
	}

	void cloud_handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg){

        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud <pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_copy(new pcl::PointCloud <pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud <pcl::PointXYZ>);

        pcl::fromROSMsg(*cloud_msg, *input_cloud);

		int cloud_size = input_cloud->points.size();
		
		pcl::copyPointCloud(*input_cloud, *input_cloud_copy);

		// TODO: make filter function return filtered cloud, so there will be no need to copy input
		// Deleting points too far away, too close and limiting horizontal FoV
		filter_points_position(input_cloud);
		
		//input_cloud->resize(cloud_size);

		cloud_filtered = downsample(input_cloud);

  		std::vector<pcl::PointIndices> cluster_indices;
		cluster_indices = euclidan_cluster(cloud_filtered);

	  	pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_cloud (new pcl::PointCloud<pcl::PointXYZ>);
		
		centroid_cloud = get_centroid_cloud(input_cloud_copy, cloud_filtered, cluster_indices);

        sensor_msgs::PointCloud2 cones_cloud;
        
        pcl::toROSMsg(*centroid_cloud, cones_cloud);
		//pcl::toROSMsg(*input_cloud, cones_cloud);
        
		cones_cloud.header = cloud_msg->header;
		cones_cloud.fields = cloud_msg->fields;

        cones_pub.publish(cones_cloud);
	}

	void wait_till_color_classifier_ready() {
		if (classify_colors) {
			color_srv_client.waitForExistence();
		}
	}

	void filter_points_position(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
	{
		cloud->points.erase(std::remove_if(cloud->points.begin(), cloud->points.end(),
										   [&](pcl::PointXYZ p)
										   {
											   // level
											   return p.z < level_threshold or
													  // dist
													  perception_handling::euclidan_dist(p.x, p.y, p.z, 0, 0, 0) > distance_treshold_max or
													  perception_handling::euclidan_dist(p.x, p.y, p.z, 0, 0, 0) < distance_treshold_min or
													  // angle
													  -angle_threshold * M_PI / 180 >= atan2(p.y, p.x) or
													  atan2(p.y, p.x) >= angle_threshold * M_PI / 180;
										   }),
							cloud->points.end());
	}

	std::vector<pcl::PointIndices> euclidan_cluster(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud){
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
  		kdtree->setInputCloud (cloud);

        std::vector<pcl::PointIndices> cluster_indices;
	  	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setClusterTolerance(sqrt(pow(this->cone_height, 2) + pow(this->cone_width, 2)));
		ec.setMinClusterSize(min_cluster_size);
		ec.setMaxClusterSize(max_cluster_size);
		ec.setSearchMethod(kdtree);
		ec.setInputCloud(cloud);
		ec.extract(cluster_indices);

		return cluster_indices;
    }

	pcl::PointCloud<pcl::PointXYZ>::Ptr get_reconstructed_cone(const pcl::PointXYZ cone_center, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud){
        pcl::PointCloud<pcl::PointXYZ>::Ptr reconstructed_cone(new pcl::PointCloud <pcl::PointXYZ>);
		pcl::PointXYZ p;

		for (std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>>::const_iterator it = cloud->points.begin(); it != cloud->points.end(); it++) {
			if ((cone_center.x + (cone_width / 1.5) >= it->x && cone_center.x - (cone_width / 1.5) <= it->x) &&
				(cone_center.y + (cone_width / 1.5) >= it->y && cone_center.y - (cone_width / 1.5) <= it->y)) {
					p.x = it->x;
					p.y = it->y;
					p.z = it->z;
					// p.intensity = it->intensity;
					reconstructed_cone->push_back(p);
			}
		}

        return reconstructed_cone;
    }

	pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud <pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.04f, 075.04f, 0.05f);
        vg.filter(*cloud_filtered);

        return cloud_filtered;
    }

	pcl::PointCloud<pcl::PointXYZ>::Ptr get_centroid_cloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &whole_cloud,
															const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &filtered_cloud,
															std::vector<pcl::PointIndices> cluster_indices) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_cloud (new pcl::PointCloud<pcl::PointXYZ>);
		//pcl::PointCloud<pcl::PointXYZ>::Ptr single_cone_cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr single_cone_cloud_reconstruct (new pcl::PointCloud<pcl::PointXYZ>);

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
		{
			pcl::PointXYZ p;
			int j = 0;
			float x, y = 0.0;
			for (const auto& idx : it->indices){
				//single_cone_cloud_filtered->push_back((*filtered_cloud)[idx]);
				x += (*filtered_cloud)[idx].x;
				y += (*filtered_cloud)[idx].y;
				j++;
			}

			p.x = x / j;
			p.y = y / j;
			p.z = 0.0;

			// if (classify_colors) {
			// 	bool need_color = true;

			// 	if (prev_centroid_cloud != NULL) {
			// 		for (std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI>>::const_iterator it = prev_centroid_cloud->points.begin(); it != prev_centroid_cloud->points.end(); it++) {
			// 			if ((perception_handling::euclidan_dist(p.x, p.y, p.z, it->x, it->y, it->z) < cones_matching_dist_theshold)
			// 						&& (it->intensity != perception_handling::colors_to_intensities[perception_handling::kUnknownColor])) {
			// 				p.intensity = it->intensity;
			// 				need_color = false;
			// 			}
			// 		}
			// 	}

			// 	if (need_color) {
			// 		/* color classification */
			// 		single_cone_cloud_reconstruct = get_reconstructed_cone(p, whole_cloud);
			// 		p.intensity = perception_handling::colors_to_intensities[get_color(single_cone_cloud_reconstruct)];
			// 	}
			// }

			centroid_cloud->push_back(p);

			j = 0;
			x = 0.0;
			y = 0.0;
		}

		prev_centroid_cloud = centroid_cloud;

		return centroid_cloud;
	}

	perception_handling::Color get_color(pcl::PointCloud<pcl::PointXYZ>::Ptr &single_cone_cloud) {
		perception_handling::Color color;
		sensor_msgs::PointCloud2 single_cone_msg;

        pcl::toROSMsg(*single_cone_cloud, single_cone_msg);

		single_cone_msg.header.frame_id = cones_frame_id;

		color_srv.request.single_cone_cloud = single_cone_msg;
		
		if (color_srv_client.call(color_srv)) {
			color = static_cast<perception_handling::Color>(color_srv.response.color);
		} else {
			// unknown
			color = static_cast<perception_handling::Color>(0);
			ROS_ERROR("Failed to call service");
		}

		return color;
	}
};


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "cone_detector");
  	ConeDetector detector;
	detector.wait_till_color_classifier_ready();
	ROS_INFO("Ready to detect cones.");
    ros::spin();

    return 0;
}