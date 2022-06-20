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



std_msgs::Header _velodyne_header;

// int arg_max(std::vector const& vec) {
//   return (int)(std::distance(vec.begin(), max_element(vec.begin(), vec.end())));
// }


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


	double cumSum = 0.0;
	double cumSumArray[maxIntensity] = {0};

	for (int k = 0; k <= maxIntensity; k++)
	{
		cumSum += (double)histogramIntensity[k];
		cumSumArray[k] = double(cumSum);
	}

	double cumMean = 0.0;
	double cumMeanArray[maxIntensity] = {0};
	for (int k = 0; k <= maxIntensity; k++)
	{
		cumMean += (double)k * (double)histogramIntensity[k];
		cumMeanArray[k] = double(cumMean);
	}

	double globalMean = 0.0;
	double globalMeanArray[256]= {0};

	for (int k = 0; k <= 255; k++)
	{
		globalMean += (double)k * (double)histogramIntensity[k];
		globalMeanArray[k]= double(globalMean);
	}

	double globalVar = 0.0;
	double globalVarArray [256] = {0};

	for (int k = 0; k <= 255; k++)
	{
		globalVar += (((double)k - globalMean) * ((double)k - globalMean) * (double)histogramIntensity[k]);
		globalVarArray[k]= double(globalVar);
	}

	double localVariance = 0.0;
	double localVarianceArray[maxIntensity] = {0};

	for (int k = 0; k <= maxIntensity; k++)
	{
		int divisor = (  histogramIntensity[k]  * ( 1 - histogramIntensity[k] )  );

		if (  divisor == 0 ){

			localVariance = (  ( (globalMean * histogramIntensity[k]) - cumMeanArray[k] )  / 0.0000001 );

		}
		else {
			
			localVariance = (  ( (globalMean * histogramIntensity[k]) - cumMeanArray[k] )  /  (  histogramIntensity[k]  * ( 1 - histogramIntensity[k] )  ) );

		}

		// localVariance = (  ( (globalMean * histogramIntensity[k]) - cumMeanArray[k] )  /  (  histogramIntensity[k]  * ( 1 - histogramIntensity[k] )  ) );

		localVarianceArray[k] = localVariance;

	}
	

	 // // to find the argmax of the array

	// double min = *std::max_element(localVarianceArray,localVarianceArray+maxIntensity);

	// std::cout << "LOCAL_VARIANCE " << min<< '\n';

	// std::cout << "Index of max element: "<< std::distance(localVarianceArray, std::max_element(localVarianceArray,localVarianceArray+maxIntensity))<< std::endl;

	thrIntensity =std::distance(localVarianceArray, std::max_element(localVarianceArray,localVarianceArray+maxIntensity));
	
	return thrIntensity;

	}



// 	/* 2 total mass moment + = strength * points */
// 	double sumIntensity = 0.0;
// 	for (int k = minIntensity; k <= maxIntensity; k++)
// 	{
// 		sumIntensity += (double)k * (double)histogramIntensity[k];
// 	}

// 	/* 3 traversal calculation */
// 	double otsu = -1.0;
// 	int w0 = 0;			  // The number of points less than or equal to the current threshold (number of previous scenic spots)
// 	double sumFore = 0.0; // Foreground quality moment

// 	unsigned int pcCount = cloud->size();
// 	for (int k = minIntensity; k <= maxIntensity; k++)
// 	{
// 		w0 += histogramIntensity[k];

// 		int w1 = pcCount - w0; //(the number of post-sites)
// 		if (w0 == 0)
// 			continue;
// 		if (w1 == 0)
// 			break;

// 		sumFore += (double)k * histogramIntensity[k];

// 		double u0 = sumFore / w0;									// The average gray level of the foreground
// 		double u1 = (sumIntensity - sumFore) / w1;					// The average gray level of the background
// 		double g = (double)w0 * (double)w1 * (u0 - u1) * (u0 - u1); // variance between classes

// 		if (g > otsu)
// 		{
// 			otsu = g;
// 			thrIntensity = k;
// 		}
// 	}

// 	return thrIntensity;
// }


/////////////////////
void Correct_Intensity(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr)
{
	double modified_intensity;

	out_cloud_ptr->points.clear();

	for (pcl::PointCloud<pcl::PointXYZI>::iterator it = in_cloud_ptr->begin(); it != in_cloud_ptr->end(); it++)
	{
		ROS_INFO_STREAM("Intensity Before: " << it->intensity << std::endl);

		if ((it->x) == 0){

			continue;
		}

		modified_intensity = (it->intensity) * ((it->x) * (it->x) + (it->y) * (it->y)) * sqrt(4.0 + (it->x) * (it->x)) / (100.0 * abs(it->x));

		ROS_INFO_STREAM("Intensity After: " <<  modified_intensity << std::endl);

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

		Filter(cloud, cloud_filtered, 2 * THRESHOLD);
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