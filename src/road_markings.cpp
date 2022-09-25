/* Software License Agreement (MIT License)
 *
 *  Copyright (c) 2022-, Abdulbaasit Sanusi
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */


//include all necessary libraries

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
#include <pcl/surface/concave_hull.h> 
#include <algorithm>
#include <typeinfo>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

using namespace std;


std_msgs::Header _velodyne_header;

// double sepMeasure;


// vector to save all intensity value in each iteration
std::vector<std::string> all_intensity_list; 


/* OTSU segmentation Algorithm is implemented in this function.
	Intensity correction and Normalization is initially carried 
	before segmentation via OTSU.
 */

///////////////////////////////////////////////////////////////////////////////////////////////////
unsigned int OTSU(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
	unsigned int thrIntensity = 1;

	/* Intializing an Intensity histogram to store all intensity values for each LIDAR scan  */
	unsigned int histogramIntensity[256] = {0};

	vector<double> histCorrectedIntensity (256, 0);

	// unsigned double histCorrectedIntensity[256] = {0};

	// Initializing Maximum and minimum intensity values
	unsigned int maxIntensity = 0, minIntensity = 666666; 

	// Initiliazing Max and Min Corrected Intensity Value
	double maxcorIntensity = 0.0, mincorIntensity = 10000000000.0; 

	int M = cloud->size();

	double corrected_intensity = 0; 

	// loop through all intensity values to save in the histogram
	for (pcl::PointCloud<pcl::PointXYZI>::iterator it = cloud->begin(); it != cloud->end(); it++)
	{

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
		
		
		// compute the corrected intensity

		double corrected_intensity = 0; 

		double R  = ((it->x) * (it->x) + (it->y) * (it->y));

		double alpha = atan2(R, (it -> z));

		double alpha_rad = alpha  * 3.14159/180;
		
		if (alpha < 60.0){
			double R_ref = 4;
			double corrected_intensity = ((it->intensity) * R / (R_ref * cos(alpha_rad)));
		}

		else if (alpha >  60.0 && alpha < 75.0){
			double R_ref = 8;
			double corrected_intensity = ((it->intensity) * R / (R_ref * cos(alpha_rad)));
		}

		else if (alpha >  75.0 && alpha < 90.0){
			double R_ref = 8;
			double corrected_intensity = ((it->intensity) * R / (R_ref * cos(alpha_rad)));
		}


		if (corrected_intensity > maxcorIntensity)
		{
			maxcorIntensity = corrected_intensity;
		}
		if (corrected_intensity < mincorIntensity)
		{
			mincorIntensity = corrected_intensity;
		}
		++histCorrectedIntensity[corrected_intensity];
	}


	//OTSU Implementation

	// Variable to store the size of the cloud
	unsigned int pcCount = cloud->size();

	//Computing the cumulative sum
	double cumSum = 0.0;
	double cumSumArray[int(maxcorIntensity)] = {0};

	for (int k = 0; k <= int(maxcorIntensity); k++)
	{
		// Normalizing the corrected Intensity
		cumSum += (double)histCorrectedIntensity[k]/pcCount;
		cumSumArray[k] = double(cumSum);
	}


	// computing the cumulative mean till intensity value k
	double cumMean = 0.0;
	double cumMeanArray[int(maxcorIntensity)] = {0};
	for (int k = 0; k <= int(maxcorIntensity); k++)
	{
		cumMean += (double)k * ((double)histCorrectedIntensity[k])/pcCount;
		cumMeanArray[k] = double(cumMean);
	}


	// computing the total mean for all intensity values
	double globalMean = 0.0;
	double globalMeanArray[256] = {0};

	for (int k = 0; k <= 255; k++)
	{
		globalMean += (double)k * ((double)histCorrectedIntensity[k])/pcCount;
		globalMeanArray[k] = double(globalMean);
	}


	// computing the total variance for all points in the array
	double globalVar = 0.0;
	double globalVarArray[256] = {0};

	for (int k = 0; k <= 255; k++)
	{
		globalVar += (((double)k - globalMean) * ((double)k - globalMean) * ((double)histCorrectedIntensity[k]))/pcCount;
		globalVarArray[k] = double(globalVar);
	}

	// computing the local variance for the roadmarking point class
	double localVariance = 0.0;
	double localVarianceArray[int(maxcorIntensity)] = {0};

	for (int k = 0; k <= int(maxcorIntensity); k++)
	{

		int divisor = (histCorrectedIntensity[k] * (1 - histCorrectedIntensity[k]));

		if (divisor == 0)
		{

			localVariance = (((globalMean * histCorrectedIntensity[k]) - cumMeanArray[k]) / 0.00001);
																								
		}
		else
		{

			localVariance = (((globalMean * histCorrectedIntensity[k]) - cumMeanArray[k]) / (histCorrectedIntensity[k] * (1 - histCorrectedIntensity[k])));
		}


		localVarianceArray[k] = localVariance;
	}

	// selecting the threshold value
	thrIntensity = std::distance(localVarianceArray, std::max_element(localVarianceArray, localVarianceArray + int(maxcorIntensity)));


	// checking the separbility criterion
	if (localVarianceArray[thrIntensity]/globalVar < 0.85){

		thrIntensity = 2 * thrIntensity;
		
		return thrIntensity;
	}

	else if (localVarianceArray[thrIntensity]/globalVar > 0.85){
		
		return thrIntensity;

	}
	
}

//////////
///////////////////////////////////////////////////////////////////////////////////////////
unsigned int OTSU_Filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud){

	int threshold = 0;
	int cloudnumber = cloud->size();

	/* 1 Intensity histogram */
	vector<int> histogramIntensity (256, 0);
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

	//OTSU Method

	//Define Histogram
	int N = 256;


	float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
	for (int k = 0; k < N; k++)
	{
		w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;

		for (int t = 0; t < N; t++)
		{
			if (t <= k)
			{
				w0 += histogramIntensity[t];
				u0tmp += t * histogramIntensity[t];
			}
			else     
			{
				w1 += histogramIntensity[t];
				u1tmp += t * histogramIntensity[t];
			}
		}
		u0 = u0tmp / w0;       
		u1 = u1tmp / w1;     
		u = u0tmp + u1tmp;    

		deltaTmp = w0 * (u0 - u)*(u0 - u) + w1 * (u1 - u)*(u1 - u);

		if (deltaTmp > deltaMax)
		{
			deltaMax = deltaTmp;
			threshold = k;
		}
	}


	return threshold;
}
/////////////////////
unsigned int OTSU_default(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
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

	/* 2 total mass moment + = strength * points */
	double sumIntensity = 0.0;
	for (int k = minIntensity; k <= maxIntensity; k++)
	{
		sumIntensity += (double)k * (double)histogramIntensity[k];
	}

	/* 3 traversal calculation */
	double otsu = -1.0;
	int w0 = 0;			  // The number of points less than or equal to the current threshold (number of previous scenic spots)
	double sumFore = 0.0; // Foreground quality moment

	unsigned int pcCount = cloud->size();
	for (int k = minIntensity; k <= maxIntensity; k++)
	{
		w0 += histogramIntensity[k];

		int w1 = pcCount - w0; //(the number of post-sites)
		if (w0 == 0)
			continue;
		if (w1 == 0)
			break;

		sumFore += (double)k * histogramIntensity[k];

		double u0 = sumFore / w0;									// The average gray level of the foreground
		double u1 = (sumIntensity - sumFore) / w1;					// The average gray level of the background
		double g = (double)w0 * (double)w1 * (u0 - u1) * (u0 - u1); // variance between classes

		if (g > otsu)
		{
			otsu = g;
			thrIntensity = k;
		}
	}

	return thrIntensity;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr, unsigned int THRESHOLD)
{
	out_cloud_ptr->points.clear();

	for (pcl::PointCloud<pcl::PointXYZI>::iterator it = in_cloud_ptr->begin(); it != in_cloud_ptr->end(); it++)
	{

		if ((it->intensity > THRESHOLD) && (( (it->x)*(it->x) + (it->y)*(it->y) ) < 800))
		// if ((it->intensity < THRESHOLD) && (it->x > 0))
		// if (it->intensity < THRESHOLD)
		{
			out_cloud_ptr->points.push_back(*it);
		}
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
		// pcl::PointCloud<pcl::PointXYZI>::Ptr boundaryclouds(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_corrected(new pcl::PointCloud<pcl::PointXYZI>);
		// pcl::PointCloud<pcl::PointXYZI> cloud_real(new pcl::PointCloud<pcl::PointXYZI>);
		
		pcl::fromROSMsg(*cloud_msg, *cloud);

		unsigned int THRESHOLD;
		THRESHOLD = OTSU(cloud);
		// THRESHOLD = OTSU_default(cloud);
		ROS_INFO("Threshold: %d", THRESHOLD);

		// Filter(cloud, cloud_filtered, 2 * THRESHOLD);
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
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
	ros::init(argc, argv, "road_markings");
	SubscribeAndPublish SAPObject;
	ros::spin();
	return 0;
}