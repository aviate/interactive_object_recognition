#include <ros/ros.h>
#include <boost/filesystem.hpp>

#include "template_matching/objects_database.h"
#include "template_matching/probabilisticmatcher.h"

const char* filename = "MODEL_SIFT_STANDARD.txt";

int main (int argc, char** argv)
{
	ros::init(argc, argv, "template_matcher");

	if (!boost::filesystem::exists(filename)) {
		ROS_ERROR_STREAM("File \"" << filename << "\" could not be found.");
		ROS_ERROR_STREAM("Models could not be loaded.");
		return 1;
	}

	ros::NodeHandle nh("~");
	ObjectsDatabase database;

	database.loadModels(filename);
	ROS_INFO_STREAM("Finished loading models from " << filename);
	database.printDatabases();

	ProbabilisticMatcher matcher(nh, &database);

	ros::Rate loop_rate(30);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}
