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

	const float CONE_WIDTH = 0.228;
	const float CONE_HEIGHT = 0.325;
	const float LIDAR_HOR_RES = 0.25; //degrees
	const float LIDAR_VER_RES = 7.5;

	double distance_treshold_max = 7.0;
	double distance_treshold_min = 0.7;
	double level_threshold = -0.5;
	double angle_threshold = 90.0;

	int min_cluster_size = 3;
	int max_cluster_size = 50;

	bool classify_colors = true;
	bool use_points_buffer = false;
	bool intensity_in_cloud_checked = false;
	bool intensity_in_cloud = true;
	double cones_matching_dist_theshold = 0.5;
	double cone_position_extension_length = 0.05;
	double voxel_filter_leaf_size_x = 0.04;
	double voxel_filter_leaf_size_y = 0.04;
	double voxel_filter_leaf_size_z = 0.04;

	std::string cones_frame_id = "cloud";
	std::string input_cloud_topic = "/cloud";
	std::string cones_topics[perception_handling::kNumberOfColors] = {"cones_cloud_unknowns",
																	  "cones_cloud_yellows",
																	  "cones_cloud_blues",
																	  "cones_cloud_oranges"};
	std::string color_classifier_srv_name = "color_classifier";

	ros::NodeHandle nh;
    ros::Subscriber cloud_sub;
    ros::Publisher cones_pubs[perception_handling::kNumberOfColors];
	ros::ServiceClient color_srv_client;

	cones_perception::ClassifyColorSrv color_srv;
	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, 
			    Eigen::aligned_allocator<pcl::PointCloud <pcl::PointXYZI>::Ptr>> prev_centroid_clouds = 
			    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, 
				Eigen::aligned_allocator<pcl::PointCloud <pcl::PointXYZI>::Ptr>>(perception_handling::kNumberOfColors);

	pcl::PointCloud<pcl::PointXYZI>::Ptr prev_detected_cones;

public:
	ConeDetector(): nh() {
		if (!ros::param::get("~cones_frame_id", cones_frame_id)) {
			ROS_INFO("cones_frame_id param not found, setting to default: %s", cones_frame_id.c_str());
		}
		if (!ros::param::get("~classify_colors", classify_colors)) {
			ROS_INFO("classify_colors param not found, setting to default: %s", classify_colors ? "true" : "false");
		}
		if (!ros::param::get("~use_points_buffer", use_points_buffer)) {
			ROS_INFO("use_points_buffer param not found, setting to default: %s", use_points_buffer ? "true" : "false");
		}
		if (!ros::param::get("~color_classifier_srv_name", color_classifier_srv_name)) {
			ROS_INFO("color_classifier_srv_name param not found, setting to default: %s", color_classifier_srv_name.c_str());
		}
		if (!ros::param::get("~distance_treshold_max", distance_treshold_max)) {
			ROS_INFO("distance_treshold_max param not found, setting to default: %f", distance_treshold_max);
		}
		if (!ros::param::get("~distance_treshold_min", distance_treshold_min)) {
			ROS_INFO("distance_treshold_min param not found, setting to default: %f", distance_treshold_min);
		}
		if (!ros::param::get("~level_threshold", level_threshold)) {
			ROS_INFO("level_threshold param not found, setting to default: %f", level_threshold);
		}
		if (!ros::param::get("~angle_threshold", angle_threshold)) {
			ROS_INFO("angle_threshold param not found, setting to default: %f", angle_threshold);
		}
		if (!ros::param::get("~min_cluster_size", min_cluster_size)) {
			ROS_INFO("min_cluster_size param not found, setting to default: %d", min_cluster_size);
		}
		if (!ros::param::get("~max_cluster_size", max_cluster_size)) {
			ROS_INFO("max_cluster_size param not found, setting to default: %d", max_cluster_size);
		}
		if (!ros::param::get("~cones_matching_dist_theshold", cones_matching_dist_theshold)) {
			ROS_INFO("cones_matching_dist_theshold param not found, setting to default: %f", cones_matching_dist_theshold);
		}
		if (!ros::param::get("~cone_position_extension_length", cone_position_extension_length)) {
			ROS_INFO("cone_position_extension_length param not found, setting to default: %f", cone_position_extension_length);
		}
		if (!ros::param::get("~voxel_filter_leaf_size_x", voxel_filter_leaf_size_x)) {
			ROS_INFO("voxel_filter_leaf_size_x param not found, setting to default: %f", voxel_filter_leaf_size_x);
		}
		if (!ros::param::get("~voxel_filter_leaf_size_y", voxel_filter_leaf_size_y)) {
			ROS_INFO("voxel_filter_leaf_size_y param not found, setting to default: %f", voxel_filter_leaf_size_y);
		}
		if (!ros::param::get("~voxel_filter_leaf_size_z", voxel_filter_leaf_size_z)) {
			ROS_INFO("voxel_filter_leaf_size_z param not found, setting to default: %f", voxel_filter_leaf_size_z);
		}

		cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(input_cloud_topic, 2, &ConeDetector::cloud_handler, this);
		for (int i = 0; i < perception_handling::kNumberOfColors; i++) {
			cones_pubs[i] = nh.advertise<sensor_msgs::PointCloud2>(cones_topics[i], 1);
		}

		if (classify_colors) {
			color_srv_client = nh.serviceClient<cones_perception::ClassifyColorSrv>(color_classifier_srv_name);
		}
	}

	void run() {
		if (classify_colors) {
			color_srv_client.waitForExistence();
		}
		ROS_INFO("Ready to detect cones.");
		ros::spin();
	}

	void cloud_handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg){
		if (!intensity_in_cloud_checked) {
			if (!perception_handling::intensity_in_cloud(cloud_msg)) {
				intensity_in_cloud = false;
			}
			intensity_in_cloud_checked = true;
		}

        pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud <pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_copy(new pcl::PointCloud <pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud <pcl::PointXYZI>);

		if (!intensity_in_cloud) {
			/* to silence the warnings */
			sensor_msgs::PointCloud2 cloud_msg_copy;
			cloud_msg_copy = *cloud_msg;
			sensor_msgs::PointField f;
			f.name = "intensity";
			f.count = 1;
			f.datatype = sensor_msgs::PointField::FLOAT32;
			cloud_msg_copy.fields.push_back(f);
			pcl::fromROSMsg(cloud_msg_copy, *input_cloud);
		} else {
			pcl::fromROSMsg(*cloud_msg, *input_cloud);
		}

		int cloud_size = input_cloud->points.size();
		
		pcl::copyPointCloud(*input_cloud, *input_cloud_copy);

		filter_points_position(input_cloud);
		
		//input_cloud->resize(cloud_size);

		cloud_filtered = downsample(input_cloud);

  		std::vector<pcl::PointIndices> cluster_indices;
		cluster_indices = euclidan_cluster(cloud_filtered);

		std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZI>::Ptr>> centroid_clouds;
		for (int i = 0; i < perception_handling::kNumberOfColors; i++) {
			pcl::PointCloud<pcl::PointXYZI>::Ptr single_cloud(new pcl::PointCloud <pcl::PointXYZI>);
			centroid_clouds.push_back(single_cloud);
		}
		
		get_centroid_clouds(input_cloud_copy, cloud_filtered, cluster_indices, centroid_clouds);

        sensor_msgs::PointCloud2 cones_cloud;

		for (int i = 0; i < perception_handling::kNumberOfColors; i++) {
			pcl::toROSMsg(*centroid_clouds[i], cones_cloud);
			
			cones_cloud.header = cloud_msg->header;
			cones_cloud.fields = cloud_msg->fields;

			cones_pubs[i].publish(cones_cloud);
		}
	}

	void filter_points_position(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
	{
		cloud->points.erase(std::remove_if(cloud->points.begin(), cloud->points.end(),
										   [&](pcl::PointXYZI p)
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

	std::vector<pcl::PointIndices> euclidan_cluster(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud){
		pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZI>);
  		kdtree->setInputCloud (cloud);

        std::vector<pcl::PointIndices> cluster_indices;
	  	pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
		ec.setClusterTolerance(sqrt(pow(CONE_HEIGHT, 2) + pow(CONE_WIDTH, 2)));
		ec.setMinClusterSize(min_cluster_size);
		ec.setMaxClusterSize(max_cluster_size);
		ec.setSearchMethod(kdtree);
		ec.setInputCloud(cloud);
		ec.extract(cluster_indices);

		return cluster_indices;
    }

	pcl::PointCloud<pcl::PointXYZI>::Ptr get_reconstructed_cone(const pcl::PointXYZI cone_center, const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud){
        pcl::PointCloud<pcl::PointXYZI>::Ptr reconstructed_cone(new pcl::PointCloud <pcl::PointXYZI>);
		pcl::PointXYZI p;

		for (std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI>>::const_iterator it = cloud->points.begin(); it != cloud->points.end(); it++) {
			if ((cone_center.x + (CONE_WIDTH / 1.5) >= it->x && cone_center.x - (CONE_WIDTH / 1.5) <= it->x) &&
				(cone_center.y + (CONE_WIDTH / 1.5) >= it->y && cone_center.y - (CONE_WIDTH / 1.5) <= it->y)) {
					p.x = it->x;
					p.y = it->y;
					p.z = it->z;
					p.intensity = it->intensity;
					reconstructed_cone->push_back(p);
			}
		}

        return reconstructed_cone;
    }

	pcl::PointCloud<pcl::PointXYZI>::Ptr downsample(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud){
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud <pcl::PointXYZI>);
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(cloud);
        // vg.setLeafSize(0.04f, 0.04f, 0.05f);
		vg.setLeafSize(voxel_filter_leaf_size_x, voxel_filter_leaf_size_y, voxel_filter_leaf_size_z);
        vg.filter(*cloud_filtered);

        return cloud_filtered;
    }

	void get_centroid_clouds(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &whole_cloud,
															const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &filtered_cloud,
															std::vector<pcl::PointIndices> cluster_indices,
															std::vector<pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>, Eigen::aligned_allocator<pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>>> centroid_clouds) {
		pcl::PointCloud<pcl::PointXYZI>::Ptr single_cone_cloud_reconstruct (new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr currently_detected_cones (new pcl::PointCloud<pcl::PointXYZI>);

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
		{
			pcl::PointXYZI p;
			int j = 0;
			float x, y = 0.0;
			for (const auto& idx : it->indices){
				x += (*filtered_cloud)[idx].x;
				y += (*filtered_cloud)[idx].y;
				j++;
			}

			p.x = x / j;
			p.y = y / j;
			p.z = 0.0;

			// move centroid further back, closer to the middle of cone
			float vector_len = perception_handling::euclidan_dist(p.x, p.y, p.z, 0, 0, 0);
			p.x = p.x + p.x / vector_len * cone_position_extension_length;
			p.y = p.y + p.y / vector_len * cone_position_extension_length;

			currently_detected_cones->push_back(p);

			if (prev_detected_cones != NULL) {
				for (std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI>>::const_iterator it_prev_cones = prev_detected_cones->points.begin(); 
					it_prev_cones != prev_detected_cones->points.end(); it_prev_cones++) {
					// if points buffer is not used or cone was detected in previous loop (then publish)
					if (!use_points_buffer || perception_handling::euclidan_dist(p.x, p.y, p.z, it_prev_cones->x, it_prev_cones->y, it_prev_cones->z) < cones_matching_dist_theshold) {
						if (classify_colors) {
							bool need_color = true;

							// unknown color always need color classification
							for (int i = perception_handling::kUnknownColor + 1; i < perception_handling::kNumberOfColors; i++) {
								if (prev_centroid_clouds[i] != NULL) {
									// check if detected cone was detected in previous loop and color was classified
									for (std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI>>::const_iterator it_colored_cones = prev_centroid_clouds[i]->points.begin(); 
										it_colored_cones != prev_centroid_clouds[i]->points.end(); it_colored_cones++) {
										if ((perception_handling::euclidan_dist(p.x, p.y, p.z, it_colored_cones->x, it_colored_cones->y, it_colored_cones->z) < cones_matching_dist_theshold)) {
											// color already classified earlier
											need_color = false;
											centroid_clouds[i]->push_back(p);
											break;
										}
									}
									if (!need_color) {
										break;
									}
								}
							}
							
							if (need_color) {
								/* color classification */
								single_cone_cloud_reconstruct = get_reconstructed_cone(p, whole_cloud);
								perception_handling::Color color = get_color(single_cone_cloud_reconstruct);
								centroid_clouds[color]->push_back(p);
							}
						} else {
							centroid_clouds[perception_handling::kUnknownColor]->push_back(p);
						}
						break;
					}
				}
			}
			j = 0;
			x = 0.0;
			y = 0.0;
		}

		for (int i = 0; i < perception_handling::kNumberOfColors; i++) {
			prev_centroid_clouds[i] = centroid_clouds[i];
		}

		prev_detected_cones = currently_detected_cones;
	}

	perception_handling::Color get_color(pcl::PointCloud<pcl::PointXYZI>::Ptr &single_cone_cloud) {
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
	detector.run();
    return 0;
}