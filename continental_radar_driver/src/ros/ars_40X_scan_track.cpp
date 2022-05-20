#include <ros/ros.h>
#include <math.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "ars_40X/ros/ars_40X_scan_track.hpp"

namespace ars_40X
{
    ScanTrack::ScanTrack() {
        ros::NodeHandle nh;

        cluster_list_sub_ = nh.subscribe("clusters", 10, &ScanTrack::cluster_list_callback, this);
        object_list_sub_ = nh.subscribe("objects", 10, &ScanTrack::object_list_callback, this);

        scan_pub_ = nh.advertise<radar_msgs::RadarScan>("scan", 10);
        track_pub_ = nh.advertise<radar_msgs::RadarTracks>("tracks", 10);
        discovery_pub_ = nh.advertise<cav_msgs::DriverStatus>("discovery", 10);

        discovery_msg_.name = "/hardware_interface/radar";
        discovery_msg_.radar = true;

        last_discovery_pub_ = ros::Time::now();
    }

    ScanTrack::~ScanTrack() {}

    void ScanTrack::cluster_list_callback(ars_40X::ClusterList cluster_list) {
        radar_msgs::RadarScan scan;

        scan.header.frame_id = cluster_list.header.frame_id;
        scan.header.stamp = ros::Time::now();

        for (auto cluster : cluster_list.clusters) {
            radar_msgs::RadarReturn radar_return;

            radar_return.range = hypot(cluster.position.pose.position.x,
                cluster.position.pose.position.y);
            radar_return.azimuth = atan(cluster.position.pose.position.y
                / cluster.position.pose.position.x);
            radar_return.elevation = 0;

            radar_return.doppler_velocity = hypot(cluster.relative_velocity.twist.linear.x,
                cluster.relative_velocity.twist.linear.y);
            radar_return.amplitude = cluster.rcs;

            scan.returns.push_back(radar_return);
        }

        scan_pub_.publish(scan);

        discovery_msg_.status = cav_msgs::DriverStatus::OPERATIONAL;

        if (last_discovery_pub_ == ros::Time(0) || (ros::Time::now() - last_discovery_pub_).toSec() > 0.8) {
            discovery_pub_.publish(discovery_msg_);
        last_discovery_pub_ = ros::Time::now();
        }
    }

    void ScanTrack::object_list_callback(ars_40X::ObjectList object_list) {
        radar_msgs::RadarTracks tracks;

        tracks.header.frame_id = object_list.header.frame_id;
        tracks.header.stamp = ros::Time::now();

        for (auto object : object_list.objects) {
            radar_msgs::RadarTrack track;
            
            track.uuid.uuid[0] = object.id;
            
            track.position = object.position.pose.position;
            track.velocity = object.relative_velocity.twist.linear;
            track.acceleration = object.relative_acceleration.accel.linear;

            track.size.x = object.length;
            track.size.y = object.width;

            track.classification = 32000 + object.class_type;

            track.position_covariance[0] = object.position.covariance[0];
            track.position_covariance[3] = object.position.covariance[7];
            track.velocity_covariance[0] = object.relative_velocity.covariance[0];
            track.velocity_covariance[3] = object.relative_velocity.covariance[7];
            track.acceleration_covariance[0] = object.relative_acceleration.covariance[0];
            track.acceleration_covariance[3] = object.relative_acceleration.covariance[7];

            tracks.tracks.push_back(track);
        }

        track_pub_.publish(tracks);

        discovery_msg_.status = cav_msgs::DriverStatus::OPERATIONAL;

        if (last_discovery_pub_ == ros::Time(0) || (ros::Time::now() - last_discovery_pub_).toSec() > 0.8) {
            discovery_pub_.publish(discovery_msg_);
        last_discovery_pub_ = ros::Time::now();
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "scan_track");
    
    ars_40X::ScanTrack scan_track;
    
    ros::spin();
}