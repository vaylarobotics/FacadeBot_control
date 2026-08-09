#!/usr/bin/env bash
#
# setup_camera.sh — rebuild the Camera Module 3 vision stack on a fresh RPi.
#
# Reconstructed from the working install on `harthikpi` (built 2026-05-08).
# Target: Raspberry Pi OS (arm64) + ROS 2 Jazzy, Pi Camera Module 3 (imx708).
#
# Why libcamera is built from SOURCE and not just apt-installed:
#   Camera Module 3 (imx708 sensor) support lives in the Raspberry Pi FORK of
#   libcamera, which is ahead of the apt package. camera_ros is compiled
#   against that in-tree build. The apt libcamera-dev is still installed (some
#   build deps pull it in), but the workspace build is what actually gets used.
#
# Exact versions this was verified against (see also camera_ws.repos):
#   raspberrypi/libcamera      @ 26bfadc6
#   christianrauch/camera_ros  @ 03c9e03
#
# This script is idempotent-ish: re-running re-clones only if missing and
# rebuilds the workspace. It does NOT pin commits itself — use
# `vcs import src < camera_ws.repos` (see below) if you need the exact pins.

set -euo pipefail

ROS_DISTRO_NAME="jazzy"
CAMERA_WS="${HOME}/camera_ws"

echo "==> 1/5  System + build dependencies (apt)"
sudo apt update
# libcamera build toolchain + its dependencies. libboost-{log,system,thread}-dev
# were added after an initial build failure — camera_ros needs them at link time.
sudo apt install -y \
  git build-essential pkg-config meson ninja-build cmake \
  python3-venv python3-dev python3-pip pybind11-dev \
  libboost-dev libboost-program-options-dev \
  libboost-log-dev libboost-system-dev libboost-thread-dev \
  libgnutls28-dev libssl-dev openssl \
  libtiff-dev libjpeg-dev libpng-dev \
  libglib2.0-dev libgstreamer-plugins-base1.0-dev \
  python3-ply python3-yaml \
  libexif-dev libavcodec-dev libdrm-dev libepoxy-dev libopencv-dev \
  i2c-tools v4l-utils

echo "==> 2/5  ROS 2 ${ROS_DISTRO_NAME} + colcon/rosdep tooling (apt)"
# Assumes the ROS 2 apt repo is already configured. If not, follow the ROS 2
# Jazzy install docs for the repo/key setup first.
sudo apt install -y \
  ros-${ROS_DISTRO_NAME}-ros-base \
  ros-${ROS_DISTRO_NAME}-image-transport-plugins \
  ros-dev-tools \
  python3-colcon-common-extensions python3-colcon-meson python3-rosdep

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

echo "==> 3/5  Fetch sources into ${CAMERA_WS}/src"
mkdir -p "${CAMERA_WS}/src"
cd "${CAMERA_WS}/src"
# Raspberry Pi fork of libcamera (imx708 / Camera Module 3 support), NOT upstream.
[ -d libcamera ]  || git clone https://github.com/raspberrypi/libcamera.git
[ -d camera_ros ] || git clone https://github.com/christianrauch/camera_ros.git

echo "==> 4/5  rosdep (skip libcamera — we build it from source above)"
cd "${CAMERA_WS}"
source /opt/ros/${ROS_DISTRO_NAME}/setup.bash
rosdep install -y --from-paths src --ignore-src \
  --rosdistro "${ROS_DISTRO_NAME}" --skip-keys=libcamera

echo "==> 5/5  Build the workspace (libcamera then camera_ros)"
colcon build --event-handlers=console_direct+

echo
echo "Done. To use it in a new terminal:"
echo "  source /opt/ros/${ROS_DISTRO_NAME}/setup.bash"
echo "  source ${CAMERA_WS}/install/setup.bash"
echo "  ros2 run camera_ros camera_node --ros-args -p orientation:=180 -p width:=800 -p height:=600"
