Continental Radar Driver for CARMA
==================================

This is a fork of the [ars_40X](https://github.com/Project-MANAS/ars_40X) package that is used for connecting to, receiving data from, and configuring Continental [ARS 404-21](https://conti-engineering.com/components/ars-404/) and [ARS 408-21](https://conti-engineering.com/components/ars-408/) radars. This fork has been modified to allow for building a Docker image that can serve as a radar driver for the [CARMA Platform](https://github.com/usdot-fhwa-stol/carma-platform).

Ubuntu 20.04 Installation
-------------------------
Assuming the CARMA Platform is installed at `~/carma_ws/src`,
```
cd ~/carma_ws/src
git clone https://github.com/VT-ASIM-LAB/continental_radar_driver.git
cd continental_radar_driver/docker
sudo ./build-image.sh -d
```
After the Docker image is successfully built, connect the Continental ARS 404-21 or ARS 408-21 radar to your device and run `sudo ip link set can0 up type can bitrate 500000` in the terminal. Add the following lines to the appropriate `docker-compose.yml` file in the `carma-config` directory:
```
continental-radar-driver:
  image: usdotfhwastoldev/carma-continental-radar-driver:develop
  container_name: continental-radar-driver
  network_mode: host
  privileged: true
  devices:
    - /dev/bus/usb:/dev/bus/usb
  volumes_from:
    - container:carma-config:ro
  environment:
    - ROS_IP=127.0.0.1
  volumes:
    - /opt/carma/logs:/opt/carma/logs
    - /opt/carma/.ros:/home/carma/.ros
    - /opt/carma/vehicle/calibration:/opt/carma/vehicle/calibration
  command: bash -c '. ./devel/setup.bash && export ROS_NAMESPACE=$${CARMA_INTR_NS} && wait-for-it.sh localhost:11311 -- roslaunch /opt/carma/vehicle/config/drivers.launch drivers:=continental_radar'
```
Finally, add the following lines to the `drivers.launch` file in the same directory as `docker-compose.yml`.
```
<include if="$(arg continental_radar)" file="$(find ars_40X)/launch/ars_40X.launch">
</include>
```

ROS API
-------

### ars_40X

#### Nodes
* `ars_40X_ros`
* `ars_40X_rviz`
* `ars_40X_scan_track`

#### Published Topics
Publication frequencies are provided for a [Continental ARS 408-21](https://conti-engineering.com/components/ars-408/) radar.
* `ars_40X_ros/clusters [ars_40X/ClusterList]`: publishes a list of detected clusters (13 Hz).
* `ars_40X_ros/objects [ars_40X/ObjectList]`: publishes a list of detected objects (13 Hz).
* `ars_40X_ros/radar_status [ars_40X/RadarStatus]`: publishes the radar status, including information on non-volatile memory (NVM) read/write state; maximum detection distance; persistent, temperature, temporary, or voltage errors; interference; sensor ID; sorting method; transmitted radar power; type of output (cluster or object); [radar cross-section (RCS)](https://en.wikipedia.org/wiki/Radar_cross-section) threshold (standard or high sensitivity); whether quality and extended information is received; and whether vehicle motion signals are available. (1 Hz).
* `ars_40X_rviz/visualize_clusters [visualization_msgs/MarkerArray]`: publishes array of markers representing detected clusters (13 Hz).
* `ars_40X_rviz/visualize_objects [visualization_msgs/MarkerArray]`: publishes array of markers representing detected objects (13 Hz).
* `ars_40X_rviz/visualize_texts [visualization_msgs/MarkerArray]`: publishes array of texts that correspond to detected objects and provide information such as relative velocity and acceleration, [RCS](https://en.wikipedia.org/wiki/Radar_cross-section), probability of detection, and object class (13 Hz).
* `ars_40X_scan_track/scan [radar_msgs/RadarScan]`: publishes a scan, which is a list of detected returns, i.e. clusters (13 Hz).
* `ars_40X_scan_track/tracks [radar_msgs/RadarTracks]`: publishes a list of detected tracks, i.e. objects (13 Hz).
* `discovery [cav_msgs/DriverStatus]`: publishes the CARMA [DriverStatus](https://github.com/usdot-fhwa-stol/carma-msgs/blob/develop/cav_msgs/msg/DriverStatus.msg) message (1.25 Hz).

#### Subscribed Topics
* `odom [geometry_msgs/TwistStamped]`: `ars_40X_ros` subscribes to this topic to receive the vehicle twist (linear and angular velocity) that is used for calculation of object attributes.
* `ars_40X_ros/clusters [ars_40X/ClusterList]`: `ars_40X_rviz` and `ars_40X_scan_track` both subscribe to this topic to receive a list of detected clusters.
* `ars_40X_ros/objects [ars_40X/ObjectList]`: `ars_40X_rviz` and `ars_40X_scan_track` both subscribe to this topic to receive a list of detected objects.

#### Services
* `set_ctrl_relay_cfg [std_srvs/SetBool]`: sends relay control message to activate collision detection.
* `set_max_distance [ars_40X/MaxDistance]`: configures the maximum distance of far scan, which also changes the range resolution proportionally (near scan maximum distance is set proportionally to half of the far scan maximum distance). For ARS 404-21 the allowed maximum distance range is 150 m - 190 m for the Standard Range firmware and 90 m - 1000 m for the Extended Range firmware. For ARS 408-21 the allowed maximum distance range is 196 m - 260 m for the Standard Range firmware and 196 m - 1200 m for the Extended Range firmware.
* `set_output_type [ars_40X/OutputType]`: configures whether to output clusters, objects, or even to just stand by.
* `set_radar_power [ars_40X/RadarPower]`: configures the transmitted radar power (Tx attenuation). The output [RCS](https://en.wikipedia.org/wiki/Radar_cross-section) of clusters and objects will be compensated for this attenuation. Reducing the output power can improve detection in case of close range scenarios or inside rooms.
* `set_rcs_threshold [ars_40X/RCSThreshold]`: sets the sensitivity of cluster detection to standard or high sensitivity.
* `set_send_ext_info [std_srvs/SetBool]`: configures whether extended object information is sent.
* `set_send_quality [std_srvs/SetBool]`: configures whether object and cluster quality information is sent.
* `set_sensor_id [ars_40X/SensorID]`: sets the sensor ID to a value from 0 to 7.
* `set_sort_index [ars_40X/SortIndex]`: selects the sorting index for object list (range or [RCS](https://en.wikipedia.org/wiki/Radar_cross-section)).
* `set_store_in_nvm [std_srvs/SetBool]`: stores the current configuration to NVM to be read and set at sensor startup.

#### Parameters
N/A

Examples
--------

See the `ars_40X.launch` file in the `continental_radar_driver/launch` directory that is used to launch the ARS 404-21 and ARS 408-21.

Original ARS_40X Documentation
==============================

 ARS_40X package contains a driver for the Continental radar ARS_404 / ARS_408.
 The package also contains a ROS Wrapper for the driver.

#### Requirements

- [socket_can](https://github.com/Project-MANAS/socket_can)

#### Launching with arguments

```bash
roslaunch ars_40X ars_40X.launch visualize:=true obstacle_array:=true
```

#### Arguments available

- **visualize** *(default:"true")* : Launches RViz to display the clusters/obstacles as markers.
- **obstacle_array** *(default:"false")* : Launches ars_40X_obstacle_array node which publishes obstacles as geometry_msgs/Polygon

#### Publications

|Message|Type|Description|Message Box|
|---|---|---|---|
|/radar_status|ars_40X/RadarStatus|Describe the radar configuration|0x201|
|/ars_40X/clusters|ars_40X/ClusterList|Raw clusters data from radar|0x600, 0x701|
|/ars_40X/objects|ars_40X/ObjectList|Raw objects data from radar|0x60A, 0x60B, 0x60C, 0x60D|
|/visualize_clusters|visualization_msgs/MarkerArray|Clusters markers for RViz visualization| - |
|/visualize_objects|visualization_msgs/MarkerArray|Object markers for RViz visualization| - |

#### Subscription

|Message|Type|Description|Message Box|
|---|---|---|---|
|/odom|nav_msgs/Odometry|Velocity and accleration information|0x300, 0x301|


#### Services
The following services are available for configuring the radar options available in 0x200

|Services|
|---|
|/set_ctrl_relay_cfg|
|/set_max_distance|
|/set_output_type|
|/set_radar_power|
|/set_rcs_threshold|
|/set_send_ext_info|
|/set_send_quality|
|/set_sensor_id|
|/set_sort_index|
|/set_store_in_nvm|

