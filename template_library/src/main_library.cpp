/*
 * main_library.cpp
 *
 *  Created on: Sep 28, 2012
 *      Author: Karol Hausman
 */

#include <ros/ros.h>
#include "template_library/template_library.h"
#include <ros/package.h>
#include <boost/filesystem.hpp>


void create_directories(std::string name)
{
	boost::system::error_code error;
	boost::filesystem::create_directories(name, error);
	if (error) 
	{
		ROS_ERROR_STREAM(
			"Error " << error
			<< ": Could not create directory " << name
		);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "library");

	TemplateLibrary library;

	std::string package_path            = ros::package::getPath("template_library");
	std::string source_directory        = package_path + "/source/";
	std::string data_directory          = package_path + "/data/";
	std::string training_directory      = package_path + "/training/";
	std::string training_data_directory = package_path + "/training_data/";

	// create directories if necessary
	create_directories(source_directory);
	create_directories(data_directory);
	create_directories(data_directory);
	create_directories(training_directory);
	create_directories(training_data_directory);

//    library.generateTemplateData(source_directory, data_directory);

	library.generateTemplateData(training_directory, training_data_directory);

//    library.loadTemplates();
//    pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr cloud_for_dense(
//                new pcl::PointCloud<pcl::PointXYZLRegionF>);
//    pcl::io::loadPCDFile("/home/karol/Desktop/RESULT_plane.pcd", *cloud_for_dense);
//    for (uint i=0;i<cloud_for_dense->size();i++)
//    {
//        if (cloud_for_dense->points[i].reg==4)
//        {
//            cloud_for_dense->points[i].x=0;
//            cloud_for_dense->points[i].y=0;
//            cloud_for_dense->points[i].z=0;


//        }
//    }
//    pcl::io::savePCDFile("/home/karol/Desktop/final.pcd",*cloud_for_dense);
}
