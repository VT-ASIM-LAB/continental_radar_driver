#ifndef ARS_40X_SCAN_TRACK_HPP
#define ARS_40X_SCAN_TRACK_HPP

#include <ros/ros.h>

#include "ars_40X/ClusterList.h"
#include "ars_40X/ObjectList.h"

#include <radar_msgs/RadarScan.h>
#include <radar_msgs/RadarTracks.h>
#include <cav_msgs/DriverStatus.h>

namespace ars_40X
{
    class ScanTrack
    {
        public:
            ScanTrack();

            ~ScanTrack();

        private:
            void cluster_list_callback(ars_40X::ClusterList cluster_list);

            void object_list_callback(ars_40X::ObjectList object_list);

            ros::Publisher scan_pub_;

            ros::Publisher track_pub_;

            ros::Publisher discovery_pub_;

            ros::Subscriber cluster_list_sub_;

            ros::Subscriber object_list_sub_;

            cav_msgs::DriverStatus discovery_msg_;
            
            ros::Time last_discovery_pub_;
    };
}

#endif //ARS_40X_SCAN_TRACK_HPP
