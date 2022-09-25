# COMP0132 MSc Robotics and Computation Project
## 

## Integrating Road Marking Extraction From LIDAR Intensity into HDL Graph SLAM Framework

[**Road Marking Extraction Algorithm**](https://github.com/abdulbaasitt/road_markings) is a project focused on **Road Marking Extraction from LIDAR Intensity Data using a Intensity Thresholding Method**  



### Installation

To run this package, please 



* Install the hdl\_graph\_slam package utilised for building the map and upon which the road marking extraction is integrated. Please follow for detail on how to install this package [hdl_graph_slam](https://github.com/koide3/hdl_graph_slam)

* Install all dependencies - Libraries and ROS packages

    - Libraries -  OpenMP, PCL, g2o, suiteparse, geodesy, nmea_msgs. Please follow [Libraries] for instruction (https://github.com/koide3/hdl_graph_slam/blob/master/README.md).
    - Packages  - geodesy, nmea_msgs, pcl_ros, ndt_omp, fast_gicp, glog_catkin Please follow [packages] for instruction (https://github.com/koide3/hdl_graph_slam/blob/master/README.md).

* Install the CloudCompare software, to visualise the map built using this package and also compare with the groundtruth map please follow: [CloudCompare](https://www.danielgm.net/cc/).





Install this package by clone the repository into the src folder of your workspace:  

```
cd ../your_ws
```

```
git clone https://github.com/abdulbaasitt/road_markings.git
```

Build and source the workspace 
```
catkin build
```

```
source devel/setup.bash

```


In another Terminal, launch the package using this command:

```
roslaunch road_markings hdl_gs_new.launch

```


Launch RVIZ and play the bag files contained in the dataset on [OneDrive]:

```
roscd hdl_graph_slam/rviz
rviz -d hdl_graph_slam.rviz
rosbag play --clock PSA_APM_8380_AIDE_TB2_TB3L1_WRK_X6_2021-09-03-11-07-08_0.bag

```
To build the a full map of the Area in the dataset. Please Run all the bags files in [OneDrive] and save the map built on RVIZ using:

```
 rosrun pcl_ros pointcloud_to_pcd input:=/hdl_graph_slam/map_points

```





#### Datasets
Find the dataset(ROSbag files) used for testing this package [OneDrive](https://1drv.ms/u/s!AlyJLAe_KcLYhYZ-hE4GSYI0GPUMVA?e=hZYrjl)

Note: to access this dataset. Kindly contact the author. The dataset is the IP of AIDrivers Ltd where this project was carried and requires explicit authorisation for use from AIDrivers LTD. 

### Licence
This project is authored by Abdulbaasit Sanusi and is licensed under the [MIT License](https://github.com/abdulbaasitt/road_markings/blob/main/LICENCE).