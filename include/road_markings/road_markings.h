#ifndef ROAD_MARKINGS_H_
#define ROAD_MARKINGS_H_

#define _USE_MATH_DEFINES

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "std_msgs/String.h"


// NEW additions

#include<iostream>
#include<cmath>
using namespace std;

// Type defs
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;


/** \brief Road Markings Script
  *
  * \author Abdulbaasit Sanusi
  */

class RM
{
  public:

    RM(ros::NodeHandle& nh);

    
    unsigned int OTSU(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

    void Filter (PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr, unsigned int THRESHOLD);

    void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg1);


    ros::NodeHandle nh_; 
    ros::Publisher pub_;
    std_msgs::Header _velodyne_header;
    ros::Subscriber sub_;
    PointCPtr g_cloud_ptr;
    PointCPtr g_cloud_filtered;


  protected:
    bool debug_;
};
#endif
