#include "ros/ros.h"
#include <perception_handling/color_classifier.hpp>
#include "cones_perception/ClassifyColorSrv.h"

bool ClassifyColor(cones_perception::ClassifyColorSrv::Request &req,
                   cones_perception::ClassifyColorSrv::Response &res) {

    perception_handling::Color color;

    // TODO: color classification

    // color = perception_handling::Color::kYellow;

    // res.colors = std::vector<perception_handling::Color>(color);

    // ROS_INFO("Classified color: %i", res.colors);

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "color_classifier_server");
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("color_classifier", ClassifyColor);
    ROS_INFO("Ready to classify color.");
    ros::spin();

    return 0;
}
