#include <road_markings/road_markings.h>

#include <fstream>
#include <iterator>
#include <string>
#include <vector>
// using namespace std;

// Type defs
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

std::vector<std::string> max_intensity_list; // list to save the max intensity value in each iteration
std::vector<std::string> min_intensity_list; // list to save the min intensity value in each iteration

RM::RM(ros::NodeHandle &nh) : g_cloud_ptr(new PointC),		// input point cloud
							  g_cloud_filtered(new PointC), // filtered point cloud
							  debug_(false)
{
	nh_ = nh;

	// Create a ROS publisher for the output point cloud
	pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/road_markings_points", 1, true);

	// Create a ROS subscriber for the input point cloud
	sub_ = nh_.subscribe("/filtered_points", 1, &RM::callback, this);
}

void RM::callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg1)
{
	PointCPtr cloud(new PointC);
	PointCPtr cloud_filtered(new PointC);

	pcl::fromROSMsg(*cloud_msg1, *cloud);

	unsigned int THRESHOLD;
	// THRESHOLD = OTSU(cloud_msg1);
	THRESHOLD = 0;

	ROS_INFO("Threshold: %d", THRESHOLD);

	Filter(cloud, cloud_filtered, THRESHOLD);
	_velodyne_header = cloud_msg1->header;

	sensor_msgs::PointCloud2 output;

	pcl::toROSMsg(*cloud_filtered, output);
	output.header = _velodyne_header;
	pub_.publish(output);

}

/* The returned threshold of intensity is obtained by OTSU. */
unsigned int
RM::OTSU(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::fromROSMsg(*cloud_msg, *cloud);
	unsigned int thrIntensity = 1;

	/* 1 Intensity histogram */
	unsigned int histogramIntensity[65536] = {0};
	// unsigned int maxIntensity = -999999, minIntensity = 999999; // Maximum and minimum intensity values
	unsigned int maxIntensity = 0, minIntensity = 999999;

	double corrected_Intensity = 0.0;

	// for loop to do measurement about max intensity
	for (pcl::PointCloud<pcl::PointXYZI>::iterator it = cloud->begin(); it != cloud->end(); it++)
	{
		double vIntensity = double(it->intensity);

		if (vIntensity != 0)
		{

			// double current_radius = (it->x * it->x) + (it->y * it->y);

			// double angle = atan2(it->z, sqrt(current_radius));

			// double ref_intensity = 25;

			// double rad_ref = current_radius / ref_intensity;

			// corrected_Intensity = (vIntensity * rad_ref * ((1.0 / (cos(angle) + 0.0001))));

			// if (corrected_Intensity > maxIntensity)
			// {

			// 	maxIntensity = corrected_Intensity;

			// 	std::cout << "max Intensity: " << maxIntensity << std::endl;

			// 	max_intensity_list.push_back(to_string(maxIntensity));
			// }
			
			// if (corrected_Intensity < minIntensity)
			// {

			// 	minIntensity = corrected_Intensity;
			// }


			if ( vIntensity> maxIntensity)
			{

				maxIntensity = vIntensity;

				// std::cout << "max Intensity: " << maxIntensity << std::endl;

				// max_intensity_list.push_back(to_string(maxIntensity));
			}

			if (vIntensity < minIntensity)
			{

				minIntensity = vIntensity;
				// std::cout << "min Intensity: " << minIntensity << std::endl;
				// min_intensity_list.push_back(to_string(minIntensity));
			}

			++histogramIntensity[int(vIntensity)];
		}

	}
	
	//conversion of max intensity values to txt file
	// std::ofstream output_file("/home/abdulbaasit/Desktop/example.txt");
	// std::ostream_iterator<std::string> output_iterator(output_file, "\n");
	// std::copy(max_intensity_list.begin(), max_intensity_list.end(), output_iterator);


	//conversion of min intensity values to txt file
	// std::ofstream output_file("/home/abdulbaasit/Desktop/min_intensity.txt");
	// std::ostream_iterator<std::string> output_iterator(output_file, "\n");
	// std::copy(min_intensity_list.begin(), min_intensity_list.end(), output_iterator);

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

void RM::Filter(PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr, unsigned int THRESHOLD)
{

	for (PointC::iterator it = in_cloud_ptr->begin(); it != in_cloud_ptr->end(); it++)
	{
		// out_cloud_ptr->points.push_back(*it);
		if (it->intensity >= THRESHOLD)

		{
			out_cloud_ptr->points.push_back(*it);
		}
	}
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "road_markings");

	ros::NodeHandle nh("~");

	RM SCLObject(nh);

	ros::spin();

	return (0);
}