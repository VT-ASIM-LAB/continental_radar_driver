#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "ars_40X/ClusterList.h"
#include "ars_40X/ObjectList.h"
#include "ars_40X/ros/ars_40X_rviz.hpp"

namespace ars_40X
{
    ContinentalRadarRViz::ContinentalRadarRViz()
    {
        ros::NodeHandle nh;

        // Define publishers.
        clusters_pub_ = nh.advertise<visualization_msgs::MarkerArray>("visualize_clusters", 10);
        objects_pub_ = nh.advertise<visualization_msgs::MarkerArray>("visualize_objects", 10);
        text_pub_ = nh.advertise<visualization_msgs::MarkerArray>("visualize_text", 10);

        // Define subscribers.
        clusters_sub_ =
            nh.subscribe("clusters", 10, &ContinentalRadarRViz::clusters_callback, this);
        objects_sub_ =
            nh.subscribe("objects", 10, &ContinentalRadarRViz::objects_callback, this);
    }

    ContinentalRadarRViz::~ContinentalRadarRViz() {}

    void ContinentalRadarRViz::clusters_callback(ars_40X::ClusterList cluster_list)
    {    
        visualization_msgs::MarkerArray marker_array;

        for (auto cluster : cluster_list.clusters)
        {    
            visualization_msgs::Marker marker;

            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;

            marker.header.frame_id = cluster_list.header.frame_id;
            marker.header.stamp = cluster_list.header.stamp;
            marker.id = cluster.id;

            marker.pose = cluster.position.pose;

            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.4;

            switch (cluster.prob_of_exist) {
                case INVALID: {
                    marker.color.a = 0.1;
                    marker.ns = "Invalid";
                    break;
                }

                case PERCENT_25: {
                    marker.color.a = 0.25;
                    marker.ns = "25%";
                    break;
                }

                case PERCENT_50: {
                    marker.color.a = 0.5;
                    marker.ns = "50%";
                    break;
                }

                case PERCENT_75: {
                    marker.color.a = 0.75;
                    marker.ns = "75%";
                    break;
                }

                case PERCENT_90: {
                    marker.color.a = 0.9;
                    marker.ns = "90%";
                    break;
                }

                case PERCENT_99: {
                    marker.color.a = 0.99;
                    marker.ns = "99%";
                    break;
                }

                case PERCENT_99_9: {
                    marker.color.a = 0.999;
                    marker.ns = "99.9%";
                    break;
                }

                case PERCENT_100: {
                    marker.color.a = 1.0;
                    marker.ns = "100%";
                    break;
                }
            }

            marker.scale.x = 0.8;
            marker.scale.y = 0.8;
            marker.scale.z = 0.8;

            marker.lifetime.fromSec(1 / 12.9);

            marker_array.markers.push_back(marker);
        }

        clusters_pub_.publish(marker_array);
    }

    void ContinentalRadarRViz::objects_callback(ars_40X::ObjectList object_list)
    {
        visualization_msgs::MarkerArray marker_array, text_array;

        for (auto object : object_list.objects)
        {
            visualization_msgs::Marker marker;

            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;

            marker.header.frame_id = object_list.header.frame_id;
            marker.header.stamp = object_list.header.stamp;
            marker.id = object.id;

            marker.pose = object.position.pose;

            switch (object.class_type) {
                case POINT: {
                    marker.color.r = 0.6;
                    marker.color.g = 0.6;
                    marker.color.b = 0.6;
                    break;
                }

                case CAR: {
                    marker.color.r = 0.2;
                    marker.color.g = 0.2;
                    marker.color.b = 0.8;
                    break;
                }

                case TRUCK: {
                    marker.color.r = 0.2;
                    marker.color.g = 0.8;
                    marker.color.b = 0.2;
                    break;
                }

                case PEDESTRIAN: {
                    marker.color.r = 0.2;
                    marker.color.g = 0.8;
                    marker.color.b = 0.8;
                    break;
                }

                case MOTORCYCLE: {
                    marker.color.r = 0.8;
                    marker.color.g = 0.2;
                    marker.color.b = 0.2;
                    break;
                }

                case BICYCLE: {
                    marker.color.r = 0.8;
                    marker.color.g = 0.2;
                    marker.color.b = 0.8;
                    break;
                }

                case WIDE: {
                    marker.color.r = 0.8;
                    marker.color.g = 0.8;
                    marker.color.b = 0.2;
                    break;
                }

                case RESERVED: {
                    marker.color.r = 0.8;
                    marker.color.g = 0.8;
                    marker.color.b = 0.8;
                    break;
                }
            }

            switch (object.prob_of_exist) {
                case INVALID: {
                    marker.color.a = 0.1;
                    marker.ns = "Invalid";
                    break;
                }

                case PERCENT_25: {
                    marker.color.a = 0.25;
                    marker.ns = "25%";
                    break;
                }

                case PERCENT_50: {
                    marker.color.a = 0.5;
                    marker.ns = "50%";
                    break;
                }

                case PERCENT_75: {
                    marker.color.a = 0.75;
                    marker.ns = "75%";
                    break;
                }

                case PERCENT_90: {
                    marker.color.a = 0.9;
                    marker.ns = "90%";
                    break;
                }

                case PERCENT_99: {
                    marker.color.a = 0.99;
                    marker.ns = "99%";
                    break;
                }

                case PERCENT_99_9: {
                    marker.color.a = 0.999;
                    marker.ns = "99.9%";
                    break;
                }

                case PERCENT_100: {
                    marker.color.a = 1.0;
                    marker.ns = "100%";
                    break;
                }
            }

            marker.scale.x = object.length;
            marker.scale.y = object.width;
            marker.scale.z = 1.0;

            marker.lifetime.fromSec(1 / 12.9);

            marker_array.markers.push_back(marker);

            visualization_msgs::Marker text;

            text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::Marker::ADD;

            text.header.frame_id = object_list.header.frame_id;
            text.header.stamp = object_list.header.stamp;
            text.id = object.id;

            int precisionVal = 1;

            text.text = "Object " + std::to_string(object.id) + "\n"
                + "LongD:" + std::to_string(object.position.pose.position.x).substr(0,
                    std::to_string(object.position.pose.position.x).find(".") + precisionVal + 1)
                    + "m "
                + "LatD:" + std::to_string(object.position.pose.position.y).substr(0,
                    std::to_string(object.position.pose.position.y).find(".") + precisionVal + 1)
                    + "m\n"
                + "RLongV:" + std::to_string(object.relative_velocity.twist.linear.x).substr(0,
                    std::to_string(object.relative_velocity.twist.linear.x).find(".")
                    + precisionVal + 1) + "m/s " 
                + "RLatV:" + std::to_string(object.relative_velocity.twist.linear.y).substr(0,
                    std::to_string(object.relative_velocity.twist.linear.y).find(".")
                    + precisionVal + 1) + "m/s\n"
                + dynamic_property[object.dynamic_property] + " " + class_type[object.class_type];

            text.pose = object.position.pose;
            text.pose.position.z = 2.0;

            text.scale.z = 0.6;

            text.color.r = 1.0;
            text.color.g = 1.0;
            text.color.b = 1.0;
            text.color.a = 1.0;

            text.lifetime.fromSec(1 / 12.9);

            text_array.markers.push_back(text);
        }

        objects_pub_.publish(marker_array);
        text_pub_.publish(text_array);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ars_40X_rviz");
    ars_40X::ContinentalRadarRViz ars_40X_rviz;
    ros::spin();
}