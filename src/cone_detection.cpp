#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <math.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>


class ConeDetector{
private:
	float cone_width = 0.228;
	float cone_height = 0.325;

	float lidar_hor_res = 0.25; //degrees
	float lidar_ver_res = 7.5;

	double distance_near_thresh = 1.0;
    double distance_far_thresh = 100.0;

	std::string odom_frame_id = "lidar";
    std::string world_frame_id = "world";
    std::string cloud_frame_id = "cloud";
    std::string map_frame_id = "map";

	ros::Publisher filtered_cloud_pub;
	pcl::Filter<pcl::PointXYZI>::Ptr outlier_removal_filter;
	tf::TransformListener tf_ls;

public:

	ConeDetector(): nh("~"){
		cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/cloud", 2, &ConeDetector::cloud_handler, this);

		cones_pub = nh.advertise<sensor_msgs::PointCloud2>("/cones_cloud", 1);
		
		filtered_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_cloud", 1);
		pcl::StatisticalOutlierRemoval<pcl::PointXYZI>::Ptr sor(new pcl::StatisticalOutlierRemoval<pcl::PointXYZI>());
        sor->setMeanK(20);
        sor->setStddevMulThresh(1.0);
        outlier_removal_filter = sor;
	}

	ros::NodeHandle nh;
    ros::Subscriber cloud_sub;
    ros::Publisher cones_pub;

	void cloud_handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg){

		// Converting ROS message to point cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud <pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud <pcl::PointXYZI>);

        pcl::fromROSMsg(*cloud_msg, *input_cloud);

		int cloud_size = input_cloud->points.size();

		// filtered_cloud------------
		if (tf_ls.canTransform(odom_frame_id, input_cloud->header.frame_id, ros::Time(0))){
			prefilter(input_cloud);
		}
		// --------------------------

        ////
        //arr_size = *(&arr + 1) - arr;

        input_cloud->points.erase(std::remove_if(input_cloud->points.begin(), input_cloud->points.end(),
            [&](pcl::PointXYZI p){return p.z < -0.5 or this->euclidan_dist(p.x, p.y, p.z) > 4 or
			this->euclidan_dist(p.x, p.y, p.z) < 0.7 or -10 * M_PI / 18 >= atan2(p.y, p.x) or
			atan2(p.y, p.x) >= 10 * M_PI / 18;}), input_cloud->points.end());

        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(input_cloud);
        vg.setLeafSize(0.04f, 0.04f, 0.05f);
        vg.filter(*cloud_filtered);


  		pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZI>);
  		kdtree->setInputCloud (cloud_filtered);

  		std::vector<pcl::PointIndices> cluster_indices;
	  	pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
	  	ec.setClusterTolerance (sqrt(pow(this->cone_height, 2) + pow(this->cone_width / 2, 2))); // 2cm
	  	ec.setMinClusterSize (3);
	  	ec.setMaxClusterSize (50);
	  	ec.setSearchMethod (kdtree);
	  	ec.setInputCloud (cloud_filtered);
	  	ec.extract (cluster_indices);

	  	pcl::PointCloud<pcl::PointXYZI>::Ptr centroid_cloud (new pcl::PointCloud<pcl::PointXYZI>);
	  	int j = 0;
		float x, y = 0.0;

		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		{
		    pcl::PointXYZI p;
		    for (const auto& idx : it->indices){
		    	//cloud_cluster->push_back ((*cloud_filtered)[idx]); //*
		    	x = (*cloud_filtered)[idx].x + x;
		    	y = (*cloud_filtered)[idx].y + y;
		    	j++;
		    }
		    p.x = x / j;
		    p.y = y / j;
		    p.z = 0.0;
		    p.intensity = 1.0;

		    centroid_cloud->push_back(p);
		    j = 0;
			x = 0.0;
			y = 0.0;
		    //cloud_cluster->width = cloud_cluster->size ();
		    //cloud_cluster->height = 1;
		    //cloud_cluster->is_dense = true;
		}
        ////

        //input_cloud->points.resize(cloud_size);

        sensor_msgs::PointCloud2 cones_cloud;

        // ROS_INFO("distance: [%f]", this->euclidan_dist(this->cone_height, 2.0, this->cone_width));
        
        // Convert created point cloud to message
        pcl::toROSMsg(*centroid_cloud, cones_cloud);

		cones_cloud.header = cloud_msg->header;
		cones_cloud.fields = cloud_msg->fields;
		cones_cloud.header.frame_id = "cloud";

        // Publish message 
        cones_pub.publish(cones_cloud);
	}

	float euclidan_dist(float x, float y, float z){
		return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
	}

	pcl::PointCloud<pcl::PointXYZI>::Ptr downsample(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud){
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud <pcl::PointXYZI>);
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.04f, 0.04f, 0.05f);
        vg.filter(*cloud_filtered);

        return cloud_filtered;
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
		transform.setIdentity();
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>());
        pcl_ros::transformPointCloud(*cloud, *transformed, transform);
        transformed->header.frame_id = odom_frame_id;
        transformed->header.stamp = cloud->header.stamp;
        cloud = transformed;

        pcl::PointCloud<pcl::PointXYZI>::ConstPtr filtered = distance_filter(cloud);
        filtered = downsample(filtered);
        filtered = outlier_removal(filtered);

        filtered_cloud_pub.publish(cloud);
    }

};

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "cone_detector");
  	ConeDetector detector;
    ros::spin();

    return 0;
}