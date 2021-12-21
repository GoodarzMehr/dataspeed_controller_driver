Dataspeed Universal Lat/Lon Controller Driver for CARMA
=======================================================

This is a fork of the [dataspeed_ulc_ros](https://bitbucket.org/DataspeedInc/dataspeed_ulc_ros/src/master/) package that is used to communicate with the Universal Lat/Lon Controller (ULC) feature of the [Dataspeed Drive-by-Wire Kit](https://www.dataspeedinc.com/adas-by-wire-system/) for Lincoln MKZ / Ford Fusion vehicles. This fork has been modified to allow for building a Docker image that can serve as a controller driver for the [CARMA Platform](https://github.com/usdot-fhwa-stol/carma-platform).

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

ROS API (stable)
----------------

### dataspeed_ulc_can

#### Nodes
* `ulc_node`

#### Topics
* ``: .
* ``: .
* `/discovery`: publishes the CARMA [DriverStatus](https://github.com/usdot-fhwa-stol/carma-msgs/blob/develop/cav_msgs/msg/DriverStatus.msg) message.

#### Services
* ``

#### Parameters
* ``: .
* `parameter`: .

Examples
--------

See the `ulc.launch` file in the `dataspeed_ulc_can/launch` directory that is used to launch the Drive-by-Wire Kit.

Original Dataspeed Universal Lat/Lon Controller Interface Documentation
=======================================================================

This repository contains a ROS interface for the Universal Lat/Lon Controller (ULC) feature in Dataspeed ADAS Kit firmware.

The User's Guide for the ULC can be found on the [downloads page](https://bitbucket.org/DataspeedInc/dataspeed_ulc_ros/downloads)

Documentation of the ROS driver node for the ULC can be found in the [dataspeed_ulc_can](dataspeed_ulc_can/README.md) README.
