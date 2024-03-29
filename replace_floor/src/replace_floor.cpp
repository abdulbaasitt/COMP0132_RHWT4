#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace message_filters;

std_msgs::Header _velodyne_header;
typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;

void Filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr)
{
  out_cloud_ptr->points.clear();
  
  for ( pcl::PointCloud<pcl::PointXYZI>::iterator it = in_cloud_ptr->begin(); it != in_cloud_ptr->end(); it++)
  {
    if ( ( it->z >= 0.2) && ( abs(it->x) >= 1.5) && ( abs(it->y) >= 1.5))

    {
       out_cloud_ptr->points.push_back(*it);
    } 
  } 
}

void Add(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2)
{
  for ( pcl::PointCloud<pcl::PointXYZI>::iterator it = cloud1->begin(); it != cloud1->end(); it++)
  {
    cloud2->points.push_back(*it);
  } 
  
}

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    pub_ = n_.advertise<sensor_msgs::PointCloud2> ("/road_markings_floor", 10);
  }
 void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg1, const sensor_msgs::PointCloud2ConstPtr& cloud_msg2 )
{// declare type
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1f(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_all(new pcl::PointCloud<pcl::PointXYZI>);
pcl::fromROSMsg(*cloud_msg1, *cloud1f); // filtered points
pcl::fromROSMsg(*cloud_msg2, *cloud2); // road markings points
// add both clouds
Filter(cloud1f, cloud1); //removed floor
Add(cloud1, cloud2);// add rm floor
Add(cloud2, cloud_all);
// write head and publish output
_velodyne_header = cloud_msg1->header;
  sensor_msgs::PointCloud2 output;
   
  pcl::toROSMsg(*cloud_all, output);
  output.header = _velodyne_header;
  pub_.publish (output);
}

 private:
  ros::NodeHandle n_; 
  ros::Publisher pub_; 

};
////////////////////////////////////////////////////////
int main (int argc, char** argv)
{
  ros::init (argc, argv, "replace_floor");
  ros::NodeHandle n;
    
  message_filters::Subscriber<sensor_msgs::PointCloud2> c1(n, "/filtered_points", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> c2(n, "/road_markings_points", 1);
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), c1, c2);
 
  SubscribeAndPublish SAPObject;
  
  sync.registerCallback(boost::bind(&SubscribeAndPublish::callback, &SAPObject, _1, _2));
   
  ros::spin ();
return 0;
} 