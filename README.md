Dataspeed Universal Lat/Lon Controller (ULC) Driver for CARMA
=======================================================

This is a fork of the [dataspeed_ulc_ros](https://bitbucket.org/DataspeedInc/dataspeed_ulc_ros/src/master/) package that is used to communicate with the Universal Lat/Lon Controller (ULC) feature of the [Dataspeed Drive-by-Wire Kit](https://www.dataspeedinc.com/adas-by-wire-system/) for Lincoln MKZ / Ford Fusion vehicles. It may also work for other vehicles compatible with the [Dataspeed Drive-by-Wire Kit](https://www.dataspeedinc.com/adas-by-wire-system/), though this is not guaranteed. This fork has been modified to allow for building a Docker image that can serve as a controller driver for the [CARMA Platform](https://github.com/usdot-fhwa-stol/carma-platform).

Ubuntu 20.04 Installation
-------------------------
Assuming the CARMA Platform is installed at `~/carma_ws/src`,
```
cd ~/carma_ws/src
git clone https://github.com/GoodarzMehr/dataspeed_controller_driver.git
cd dataspeed_controller_driver/docker
sudo ./build-image.sh -d
```
After the Docker image is successfully built, connect the Drive-by-Wire Kit USB cable to your device and run `lsusb` in the terminal to determine which bus and device number it has been assigned to. Assuming here that it is Device 007 on Bus 001, add the following lines to the appropriate `docker-compose.yml` file in the `carma-config` directory, and make sure that the current user (and not `root`) is the owner of `/dev/bus/usb/001/007`.
```
dataspeed-controller-driver:
  image: usdotfhwastoldev/carma-dataspeed-controller-driver:develop
  container_name: dataspeed-controller-driver
  network_mode: host
  privileged: true
  devices:
    - /dev/bus/usb/001/007:/dev/bus/usb/001/007
  volumes_from:
    - container:carma-config:ro
  environment:
    - ROS_IP=127.0.0.1
  volumes:
    - /opt/carma/logs:/opt/carma/logs
    - /opt/carma/.ros:/home/carma/.ros
    - /opt/carma/vehicle/calibration:/opt/carma/vehicle/calibration
  command: bash -c '. ./devel/setup.bash && export ROS_NAMESPACE=$${CARMA_INTR_NS} && wait-for-it.sh localhost:11311 -- roslaunch /opt/carma/vehicle/config/drivers.launch drivers:=dataspeed_controller'
```
Finally, add the following lines to the `drivers.launch` file in the same directory as `docker-compose.yml`.
```
<include if="$(arg dataspeed_controller)" file="$(find dataspeed_ulc_can)/launch/ulc.launch">
</include>
```
Note that for this driver to function it has to be launched concurrently with [dataspeed_can_driver](https://github.com/VT-ASIM-LAB/dataspeed_can_driver) to communicate with the [CAN bus](https://en.wikipedia.org/wiki/CAN_bus).

ROS API
-------

### dataspeed_ulc_can

#### Nodes
* `ulc_node`

#### Published Topics
* `can_tx [can_msgs/Frame]`: publishes commands intended for the vehicle [CAN bus](https://en.wikipedia.org/wiki/CAN_bus).
* `robot_status [cav_msgs/RobotEnabled]`: publishes the current status of automated vehicle control (10 Hz).
* `ulc_report [dataspeed_ulc_msgs/UlcReport]`: publishes feedback data from the ULC (5 Hz).
* `discovery [cav_msgs/DriverStatus]`: publishes the CARMA [DriverStatus](https://github.com/usdot-fhwa-stol/carma-msgs/blob/develop/cav_msgs/msg/DriverStatus.msg) message (1.25 Hz).

#### Subscribed Topics
* `can_rx [can_msgs/Frame]`: receives CAN messages read from the vehicle [CAN bus](https://en.wikipedia.org/wiki/CAN_bus).
* `ulc_cmd [dataspeed_ulc_msgs/UlcCmd]`: receives input commands intended for the ULC. In addition to the speed and steering command inputs, this topic also configures the behavior of the ULC. It allows the user to turn the speed and steering components of the ULC on and off, switch shifting and steering modes, and configure longitudinal and lateral acceleration limits.
* `cmd_vel (geometry_msgs/Twist)`: receives simplified commands intended for the ULC. When this topic is used to command the ULC, the speed component of the ULC is activated and tracks the `linear.x` field of the `geometry_msgs/Twist` message, assuming the units are `m/s`; the steering component of the ULC is activated in yaw rate mode and tracks the `angular.z` field of the `geometry_msgs/Twist` message, assuming the units are `rad/s`; and all longitudinal and lateral acceleration limits use the default settings outlined in the [ULC User's Guide](https://bitbucket.org/DataspeedInc/dataspeed_ulc_ros/downloads/ULC_UserGuide-RevA04.pdf).
* `cmd_vel_stamped (geometry_msgs/TwistStamped)`: receives simplified commands intended for the ULC and ignores the header of the incoming message, acting similar to the `cmd_vel` topic.
* `dbw_enabled [std_msgs/Bool]`: receives the status (enabled/disabled) of the Drive-by-Wire system.
* `vehicle_cmd [autoware_msgs/VehicleCmd]`: receives Autoware commands intended for the ULC. This can either be a control command (desired speed and steering wheel angle) or a twist command (desired speed and yaw rate).

#### Services
N/A

#### Parameters
N/A

Examples
--------

See the `ulc.launch` file in the `dataspeed_ulc_can/launch` directory that is used to launch the ULC.

Original Dataspeed Universal Lat/Lon Controller Interface Documentation
=======================================================================

This repository contains a ROS interface for the Universal Lat/Lon Controller (ULC) feature in Dataspeed ADAS Kit firmware.

The User's Guide for the ULC can be found on the [downloads page](https://bitbucket.org/DataspeedInc/dataspeed_ulc_ros/downloads)

Documentation of the ROS driver node for the ULC can be found in the [dataspeed_ulc_can](dataspeed_ulc_can/README.md) README.
