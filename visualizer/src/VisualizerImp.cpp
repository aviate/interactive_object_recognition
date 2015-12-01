#include <visualizer/VisualizerImpl.h>
#include <visualizer/Visualizer.h>

Visualizer::Visualizer() {
	Impl = new VisualizerImpl();
}

Visualizer::~Visualizer() {
	delete Impl;
}

void Visualizer::addPointCloud(PointCloudConstPtr& input_cloud) {
	Impl->addPointCloud(input_cloud);
}

void Visualizer::addPointCloudColor(PointCloudConstPtr& input_cloud) {
	Impl->addPointCloudColor(input_cloud);
}

void Visualizer::spinOnce() {
	Impl->spinOnce();
}

void Visualizer::removeAllClouds() {
	Impl->removeAllClouds();
}
