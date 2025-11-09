# This is a ROS2 version of the following work

# VDB-EDT: An Efficient Euclidean Distance Transform Algorithm Based on VDB Data Structure

# The video demonstration for ICRA-2021 paper can be found here https://youtu.be/Bojh6ylYUOo

VDB-EDT is an efficient and robust library for large-scale occupancy grid mapping and Euclidean distance transform.  Compared with the state-of-the-art method, VDB-EDT can provide high-quality and more complete mapping results, while consuming less memory and processing time. 



## Installation

1.  This package is migrated for ROS2, tested on humble version
2.  With proper dependencies including openVDB and PCL, it can be installed with
   `colcon build --packages-select vdb_edt`

### Run VDB-EDT

1. Make sure the current path is at VDB-EDT folder, i.e., in the catkin workspace

   `source install/setup.bash`
   `ros2 launch vdb_edt vdb_edt.launch.xml`

### Acknowledgement

The VDB Ros Wrapper inside this package is developed by Rohit Garg from Air Lab CMU based on the work of ETHZ ASL. We give our special acknowledgement to him.
