#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "std_msgs/String.h"
#include <cmath>
#include <stdlib.h>

ros::Publisher pub;
std_msgs::Header _velodyne_header;

/* The returned threshold of intensity is obtained by OTSU. */
unsigned int OTSU(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)

{
	
	unsigned int thrIntensity = 1;
 
	/* 1 Intensity histogram */
	unsigned int histogramIntensity[65536] = { 0 };
	 unsigned int maxIntensity = 0, minIntensity = 666666;//Maximum and minimum intensity values
	
for ( pcl::PointCloud<pcl::PointXYZI>::iterator it = cloud->begin(); it != cloud->end(); it++)
	{
		// correct intensity to find corrected threshold
		
		unsigned int vIntensity = it->intensity;
		if (vIntensity > maxIntensity)
		{
			maxIntensity = vIntensity;
		}
		if (vIntensity < minIntensity)
		{
			minIntensity = vIntensity;
		}
		++histogramIntensity[vIntensity];
	}
 
	 /* 2 total mass moment + = strength * points */
	double sumIntensity = 0.0;
	for (int k = minIntensity; k <= maxIntensity; k++)
	{
		sumIntensity += (double)k * (double)histogramIntensity[k];
	}
 
	 /* 3 traversal calculation */
	double otsu = -1.0;
	 int w0 = 0;//The number of points less than or equal to the current threshold (number of previous scenic spots)
	 double sumFore = 0.0;//Foreground quality moment

	unsigned int pcCount = cloud->size();
	for (int k = minIntensity; k <= maxIntensity; k++)
	{
		w0 += histogramIntensity[k];
                 
		 int w1 = pcCount-w0;//(the number of post-sites)
		if (w0 == 0)
			continue;
		if (w1 == 0)
			break;
 
		sumFore += (double)k * histogramIntensity[k];
 
		 double u0 = sumFore / w0; //The average gray level of the foreground
		 double u1 = (sumIntensity-sumFore) / w1; //The average gray level of the background
		 double g = (double)w0 * (double)w1 * (u0-u1) * (u0-u1); //variance between classes
 
		if (g > otsu)
		{
			otsu = g;
			thrIntensity = k;
		}
	}
 //ROS_INFO("thIntensity: %d", THRESHOLD);
	return thrIntensity;
}
///////////////////////////////////////////////////////////////////////////////////////////////
// Will correct intensity 

void Correct_Intensity(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr)
{   double modified_intensity; 
		        
  out_cloud_ptr->points.clear();
  
  for ( pcl::PointCloud<pcl::PointXYZI>::iterator it = in_cloud_ptr->begin(); it != in_cloud_ptr->end(); it++)
  {
    modified_intensity = (it->intensity) * ( (it->x)*(it->x) + (it->y)*(it->y) ) *sqrt(4.0 + (it->x)*(it->x))/ (100.0 * abs(it->x) );
    
    if(1) 
    {
    	 it->intensity = modified_intensity;
       out_cloud_ptr->points.push_back(*it);
    } 
  } 
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Will return points having intensity higher than a threshold


void Filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr, unsigned int THRESHOLD)
{
  out_cloud_ptr->points.clear();
  
  for ( pcl::PointCloud<pcl::PointXYZI>::iterator it = in_cloud_ptr->begin(); it != in_cloud_ptr->end(); it++)
  {
    
    if ( it->intensity > THRESHOLD ) 
    {
    	 out_cloud_ptr->points.push_back(*it);
    } 
  } 
}
///////////////////////////////

void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_corrected(new pcl::PointCloud<pcl::PointXYZI>);

  pcl::fromROSMsg(*cloud_msg, *cloud);
  //Correct_Intensity(cloud, cloud_corrected);

  unsigned int THRESHOLD;
  //THRESHOLD = OTSU(cloud_corrected);
  //ROS_INFO("Threshold: %d", THRESHOLD);

  Filter(cloud, cloud_filtered, 200);
_velodyne_header = cloud_msg->header;
    
   // Convert to ROS data type
   sensor_msgs::PointCloud2 output;
   
   pcl::toROSMsg(*cloud_filtered, output);
   output.header = _velodyne_header;

    // Publish the data
    pub.publish (output);
}
//////////////////////////////////////////////
int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "road_markings");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/points_input", 1, callback);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/road_markings_points", 1);

  // Spin
  ros::spin ();
}