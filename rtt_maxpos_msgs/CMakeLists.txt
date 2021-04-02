cmake_minimum_required(VERSION 2.8.3)
project(rtt_maxpos_msgs)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm)

ros_generate_rtt_typekit(maxpos_msgs)
ros_generate_rtt_service_proxies(maxpos_msgs)

orocos_generate_package(
  DEPENDS maxpos_msgs
  DEPENDS_TARGETS rtt_roscomm  rtt_std_msgs

)
