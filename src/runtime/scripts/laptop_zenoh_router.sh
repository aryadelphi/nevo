#!/usr/bin/env bash
# Laptop side of the NEVO Zenoh link: run a router that connects to nevo's router.
# Run inside the ubuntu24 distrobox. Every ROS terminal on the laptop then needs
#   export RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DOMAIN_ID=7 && ros2 daemon stop
# Usage: laptop_zenoh_router.sh [lan|ts|<ip>]   (default lan = 192.168.12.195,
#        ts = Tailscale 100.89.187.91)
# Do NOT run a sim on domain 7 while this router is linked: its /scan, /tf and
# /cmd_vel would mix with the real robot. For the sim use a plain local router
# (ros2 run rmw_zenoh_cpp rmw_zenohd) or another ROS_DOMAIN_ID.

case "${1:-lan}" in
  lan) HOST=192.168.12.195 ;;
  ts)  HOST=100.89.187.91 ;;
  *)   HOST="$1" ;;
esac

source /opt/ros/jazzy/setup.bash
echo "zenoh router -> tcp/${HOST}:7447"
ZENOH_CONFIG_OVERRIDE="connect/endpoints=[\"tcp/${HOST}:7447\"]" \
  exec ros2 run rmw_zenoh_cpp rmw_zenohd
