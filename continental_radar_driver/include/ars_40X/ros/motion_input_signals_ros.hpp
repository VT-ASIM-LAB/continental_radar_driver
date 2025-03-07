#ifndef ARS_40X_MOTION_INPUT_SIGNALS_ROS_HPP
#define ARS_40X_MOTION_INPUT_SIGNALS_ROS_HPP

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

#include <cstdint>

#include "ars_40X/ars_40X_can.hpp"

namespace ars_40X
{
    class MotionInputSignalsROS
    {
        public:
            MotionInputSignalsROS(ros::NodeHandle &nh, ARS_40X_CAN *ars_40X_can);

            ~MotionInputSignalsROS();

        private:
            void odom_callback(geometry_msgs::TwistStamped msg);

            ARS_40X_CAN *ars_40X_can_;

            motion_input_signals::SpeedInformation *speed_information_;

            motion_input_signals::YawRateInformation *yaw_rate_information_;

            ros::Subscriber odom_sub_;
    };
}

#endif //ARS_40X_MOTION_INPUT_SIGNALS_ROS_HPP
