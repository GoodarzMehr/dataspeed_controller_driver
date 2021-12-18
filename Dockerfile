ARG ROS_DISTRO=noetic

FROM ros:${ROS_DISTRO}-ros-core AS build-env
ENV DEBIAN_FRONTEND=noninteractive \
    BUILD_HOME=/var/lib/build \
    DSC_SDK_PATH=/opt/dataspeed_controller

RUN set -xue \
# Kinetic and melodic have python3 packages but they seem to conflict
&& [ $ROS_DISTRO = "noetic" ] && PY=python3 || PY=python \
# Turn off installing extra packages globally to slim down rosdep install
&& echo 'APT::Install-Recommends "0";' > /etc/apt/apt.conf.d/01norecommend \
&& apt-get update \
&& apt-get install -y \
 build-essential cmake \
 fakeroot dpkg-dev debhelper git \
 $PY-rosdep $PY-rospkg $PY-bloom

# Set up non-root build user
ARG BUILD_UID=1000
ARG BUILD_GID=${BUILD_UID}

RUN set -xe \
&& groupadd -o -g ${BUILD_GID} build \
&& useradd -o -u ${BUILD_UID} -d ${BUILD_HOME} -rm -s /bin/bash -g build build

# Install build dependencies using rosdep
COPY --chown=build:build dataspeed_ulc/package.xml ${DSC_SDK_PATH}/dataspeed_ulc/package.xml
COPY --chown=build:build dataspeed_ulc_msgs/package.xml ${DSC_SDK_PATH}/dataspeed_ulc_msgs/package.xml
COPY --chown=build:build autoware_msgs/package.xml ${DSC_SDK_PATH}/autoware_msgs/package.xml
COPY --chown=build:build cav_msgs/package.xml ${DSC_SDK_PATH}/cav_msgs/package.xml

RUN set -xe \
&& apt-get update \
&& rosdep init \
&& rosdep update --rosdistro=${ROS_DISTRO} \
&& rosdep install -y --from-paths ${DSC_SDK_PATH}

COPY --chown=build:build dataspeed_ulc_can/package.xml ${DSC_SDK_PATH}/dataspeed_ulc_can/package.xml

RUN sudo git clone --depth 1 https://github.com/vishnubob/wait-for-it.git ~/.base-image/wait-for-it && \
    sudo mv ~/.base-image/wait-for-it/wait-for-it.sh /usr/bin

# Set up build environment
COPY --chown=build:build dataspeed_ulc ${DSC_SDK_PATH}/dataspeed_ulc
COPY --chown=build:build dataspeed_ulc_can ${DSC_SDK_PATH}/dataspeed_ulc_can
COPY --chown=build:build dataspeed_ulc_msgs ${DSC_SDK_PATH}/dataspeed_ulc_msgs
COPY --chown=build:build autoware_msgs ${DSC_SDK_PATH}/autoware_msgs
COPY --chown=build:build cav_msgs ${DSC_SDK_PATH}/cav_msgs


USER build:build
WORKDIR ${BUILD_HOME}

RUN set -xe \
&& mkdir src \
&& ln -s ${DSC_SDK_PATH} ./src

FROM build-env

RUN /opt/ros/${ROS_DISTRO}/env.sh catkin_make -DCMAKE_BUILD_TYPE=Release

# Entrypoint for running Dataspeed ULC ROS:
CMD ["bash", "-c", "set -e \
&& . ./devel/setup.bash \
&& roslaunch dataspeed_ulc_can ulc.launch \"$@\" \
", "ros-entrypoint"]
