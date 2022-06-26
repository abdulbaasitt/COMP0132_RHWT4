#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "std_msgs/String.h"
#include <cmath>
#include <stdlib.h>

#include <sensor_msgs/NavSatFix.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <std_msgs/Time.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <pcl_ros/point_cloud.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <typeinfo>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

std_msgs::Header _velodyne_header;

double sepMeasure;

std::vector<std::string> all_intensity_list; // list to save all intensity value in each iteration

unsigned int OTSU(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
	unsigned int thrIntensity = 1;

	/* 1 Intensity histogram */
	unsigned int histogramIntensity[256] = {0};
	unsigned int maxIntensity = 0, minIntensity = 666666; // Maximum and minimum intensity values

	int M = cloud->size();

	for (pcl::PointCloud<pcl::PointXYZI>::iterator it = cloud->begin(); it != cloud->end(); it++)
	{
		// correct intensity to find corrected threshold

		double modified_intensity = (it->intensity) * ((it->x) * (it->x) + (it->y) * (it->y)) * sqrt(4.0 + (it->x) * (it->x)) / (100.0 * abs(it->x));

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

	unsigned int pcCount = cloud->size();

	double cumSum = 0.0;
	double cumSumArray[maxIntensity] = {0};

	for (int k = 0; k <= maxIntensity; k++)
	{
		cumSum += (double)histogramIntensity[k]; // 	/pcCount;
		cumSumArray[k] = double(cumSum);
	}

	double cumMean = 0.0;
	double cumMeanArray[maxIntensity] = {0};
	for (int k = 0; k <= maxIntensity; k++)
	{
		cumMean += (double)k * ((double)histogramIntensity[k]); // 	/pcCount);
		cumMeanArray[k] = double(cumMean);
	}

	double globalMean = 0.0;
	double globalMeanArray[256] = {0};

	for (int k = 0; k <= 255; k++)
	{
		globalMean += (double)k * ((double)histogramIntensity[k]); // /pcCount);
		globalMeanArray[k] = double(globalMean);
	}

	double globalVar = 0.0;
	double globalVarArray[256] = {0};

	for (int k = 0; k <= 255; k++)
	{
		globalVar += (((double)k - globalMean) * ((double)k - globalMean) * ((double)histogramIntensity[k])); // /pcCount));
		globalVarArray[k] = double(globalVar);
	}

	double localVariance = 0.0;
	double localVarianceArray[maxIntensity] = {0};

	for (int k = 0; k <= maxIntensity; k++)
	{

		int divisor = (histogramIntensity[k] * (1 - histogramIntensity[k]));
		// int divisor = (  cumSumArray[k]  * ( 1 - cumSumArray[k])  );

		// ROS_INFO_STREAM("Divisor" << divisor);

		if (divisor == 0)
		{

			localVariance = (((globalMean * histogramIntensity[k]) - cumMeanArray[k]) / 0.00001); // / 0.0000001 );
																								  // localVariance = (  ( (globalMean * cumSumArray[k]) - cumMeanArray[k] ) /0.0001);		// / 0.0000001 );
		}
		else
		{

			// localVariance = (  ( (globalMean * cumSumArray[k]) - cumMeanArray[k] )  /  (  cumSumArray[k] * ( 1 - cumSumArray[k] )  ) );

			localVariance = (((globalMean * histogramIntensity[k]) - cumMeanArray[k]) / (histogramIntensity[k] * (1 - histogramIntensity[k])));
		}

		// localVariance = (  ( (globalMean * histogramIntensity[k]) - cumMeanArray[k] )  /  (  histogramIntensity[k]  * ( 1 - histogramIntensity[k] )  ) );

		localVarianceArray[k] = localVariance;
	}

	thrIntensity = std::distance(localVarianceArray, std::max_element(localVarianceArray, localVarianceArray + maxIntensity));

	return thrIntensity;
}

/////////////////////
void Correct_Intensity(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr)
{
	double modified_intensity;

	out_cloud_ptr->points.clear();

	for (pcl::PointCloud<pcl::PointXYZI>::iterator it = in_cloud_ptr->begin(); it != in_cloud_ptr->end(); it++)
	{
		ROS_INFO_STREAM("Intensity Before: " << it->intensity << std::endl);

		if ((it->x) == 0)
		{

			continue;
		}

		modified_intensity = (it->intensity) * ((it->x) * (it->x) + (it->y) * (it->y)) * sqrt(4.0 + (it->x) * (it->x)) / (100.0 * abs(it->x));

		ROS_INFO_STREAM("Intensity After: " << modified_intensity << std::endl);

		if (1)
		{
			(it->intensity) = modified_intensity;
			out_cloud_ptr->points.push_back(*it);
		}
	}
}
///////////////////////
void Filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr, unsigned int THRESHOLD)
{
	out_cloud_ptr->points.clear();

	for (pcl::PointCloud<pcl::PointXYZI>::iterator it = in_cloud_ptr->begin(); it != in_cloud_ptr->end(); it++)
	{

		if ((it->intensity > THRESHOLD) && (it->x > 0))
		// if ((it->intensity < THRESHOLD) && (it->x > 0))
		{
			out_cloud_ptr->points.push_back(*it);
		}
	}
}
class SubscribeAndPublish
{
public:
	SubscribeAndPublish()
	{
		// Create a ROS subscriber for the input point cloud
		sub = nh.subscribe("/points_input", 1, &SubscribeAndPublish::callback, this);

		// Create a ROS publisher for the output point cloud
		pub = nh.advertise<sensor_msgs::PointCloud2>("/road_markings_points", 1);
	}
	///////////////////
	void callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_corrected(new pcl::PointCloud<pcl::PointXYZI>);

		pcl::fromROSMsg(*cloud_msg, *cloud);
		// Correct_Intensity(cloud, cloud_corrected);

		unsigned int THRESHOLD;
		THRESHOLD = OTSU(cloud);
		ROS_INFO("Threshold: %d", THRESHOLD);

		Filter(cloud, cloud_filtered, THRESHOLD);
		_velodyne_header = cloud_msg->header;

		// Convert to ROS data type
		sensor_msgs::PointCloud2 output;

		pcl::toROSMsg(*cloud_filtered, output);
		output.header = _velodyne_header;

		// Publish the data
		pub.publish(output);
	}

private:
	ros::NodeHandle nh;
	ros::Publisher pub;
	ros::Subscriber sub;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "road_markings");
	SubscribeAndPublish SAPObject;
	ros::spin();
	return 0;
}