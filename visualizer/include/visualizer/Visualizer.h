#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <pcl_typedefs/pcl_typedefs.h>
#include <ros/ros.h>
//#include <pcl/point_types.h>

//typedef const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> > PointCloudConstPtr;

class VisualizerImpl;
class Visualizer
{
public:
	Visualizer();
	~Visualizer();

	void addPointCloudColor(PointCloudConstPtr &input_cloud);
	void addPointCloud(PointCloudConstPtr &input_cloud);
	void spinOnce();
	void removeAllClouds();


private:
	VisualizerImpl *Impl;
};

#endif // VISUALIZER_H
