/*
 * cloud_saver.cpp
 *
 *  Created on: jan 21, 2013
 *      Author: Karol Hausman
 */

#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>

#include <pcl_typedefs/pcl_typedefs.h>
#include <cloud_saver/cloud_saver.h>

CloudSaver::CloudSaver():
	nh_ ("~/cloud_saver"),
	visualizer_(),
	reconfig_srv_(nh_),
	input_cloud_ptr_(new PointCloud),
	cloud_number_(0)
{
	reconfig_callback_ = boost::bind(&CloudSaver::reconfigCallback, this, _1, _2);
	reconfig_srv_.setCallback(reconfig_callback_);

	cloud_subscriber_ = nh_.subscribe("/camera/depth_registered/points", 1, &CloudSaver::cloudCallback, this);
}

void CloudSaver::cloudCallback(const sensor_msgs::PointCloud2Ptr& cloud_msg)
{
	// convert sensor_msgs::PointCloud2Ptr to a pcl::PointCloud<pcl::pointXYZRGB>
	pcl::PCLPointCloud2 pcl_pc;
	pcl_conversions::toPCL(*cloud_msg, pcl_pc);

	input_cloud_ptr_.reset(new PointCloud); // necessary?
	pcl::fromPCLPointCloud2(pcl_pc, *input_cloud_ptr_);

	PointCloudConstPtr cloud_const_ptr(input_cloud_ptr_);

	visualizer_.removeAllClouds();
	visualizer_.addPointCloudColor(cloud_const_ptr);
	visualizer_.spinOnce();
}


void CloudSaver::reconfigCallback (cloud_saver::SaverConfig &config, uint32_t level)
{
	cloud_name_ = config.cloud_name;
	cloud_number_++;

	if (config.save_cloud)
	{
		std::stringstream ststream;
		ststream << cloud_name_;
		ststream << "/" << cloud_number_ << ".pcd";

		pcl::io::savePCDFile(ststream.str(), *input_cloud_ptr_);
		config.save_cloud=false;
	}
}


void CloudSaver::spinVisualizer()
{
	visualizer_.spinOnce();
}


//void CloudSaver::saveTemplates()
//{
//    for(uint i=0;i<templates_.size();i++)
//    {
//        Template temp=templates_[i];

//        std::stringstream ss_cloud_rgb;
//        std::stringstream ss_cloud_inliers;
//        std::stringstream ss_image;
//        std::stringstream ss_no_plane_image;

//        generateNames(i,ss_image,ss_no_plane_image,ss_cloud_rgb,ss_cloud_inliers);

//        ROS_INFO_STREAM(ss_cloud_rgb.str());
//        pcl::io::savePCDFile(ss_cloud_inliers.str(),*temp.cloud_with_inliers_ptr_);
//        pcl::io::savePCDFile(ss_cloud_rgb.str(),*temp.cloud_ptr_);
//        cv::imwrite( ss_image.str(), temp.image_);
//        cv::imwrite( ss_no_plane_image.str(), temp.no_plane_image_);

//    }

//}

//void TemplateLibrary::generateNames(const int &i,std::stringstream &ss_image,std::stringstream &ss_no_plane_image,std::stringstream &ss_cloud_rgb,std::stringstream &ss_cloud_inliers)
//{

//    ss_cloud_rgb<<"/home/karol/ros_workspace/interactive_object_recognition/template_library/data/template" <<i<<"cloud_rgb.pcd";
//    ss_cloud_inliers<<"/home/karol/ros_workspace/interactive_object_recognition/template_library/data/template" <<i<<"cloud_inliers.pcd";
//    ss_image<<"/home/karol/ros_workspace/interactive_object_recognition/template_library/data/template" <<i<<"image.jpg";
//    ss_no_plane_image<<"/home/karol/ros_workspace/interactive_object_recognition/template_library/data/template" <<i<<"image_no_plane.jpg";

//}
