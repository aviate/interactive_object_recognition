/*
 * DenseReconstruction.h
 *
 *  Created on: Aug 17, 2012
 *      Author: Karol Hausman
 */

#ifndef DENSERECONSTRUCTION_H_
#define DENSERECONSTRUCTION_H_

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/pcl_search.h>
#include <pcl/features/boundary.h>

#include "dense_reconstruction/point_type.h"

template<typename PointType>
class DenseReconstruction {
public:
	typedef pcl::PointCloud<PointType> PointCloud;
	typedef typename PointCloud::Ptr PointCloudPtr;
	typedef typename PointCloud::ConstPtr PointCloudConstPtr;

	DenseReconstruction(typename pcl::PointCloud<PointType>::Ptr saved_cloud) {
		debug_ = false;
		plane_segmentation_ = true;
		plane_segmentation_dist_thresh_ = 0.02; // 0.006;
		plane_segmentation_max_iter_ = 1000;
		normals_radius_search_ = 0.03;
		boundary_radius_ = 0.02;
		euclidean_tolerance_ = 0.01; // 1cm
		min_cluster_size_ = 0; // 10;
		max_cluster_size_ = 50000;

		cloud_.reset(new pcl::PointCloud<PointType>(*saved_cloud));
		cloud_operational_.reset(new pcl::PointCloud<PointType>);

		pcl::copyPointCloud(*cloud_, *cloud_operational_);

		if (debug_)
			pcl::io::savePCDFile("After_plane.pcd", *cloud_operational_);
	}

	void planeSegmentation(
		const PointCloudPtr &cloud,
		pcl::ModelCoefficients &coefficients,
		pcl::PointIndices &inliers
	) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(
			new pcl::PointCloud<pcl::PointXYZ>
		);
		pcl::copyPointCloud(*cloud, *cloud_temp);
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		seg.setOptimizeCoefficients(false);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
	//	seg.setMaxIterations(plane_segmentation_max_iter_);
		seg.setDistanceThreshold(plane_segmentation_dist_thresh_);
		seg.setInputCloud(cloud_temp);
		seg.segment(inliers, coefficients);
	}

	void planeExtraction(
		const PointCloudPtr &cloud_input,
		pcl::PointIndices::Ptr &inliers,
		PointCloud &cloud_output
	) {
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_input_temp(
			new pcl::PointCloud<pcl::PointXYZI>
		);
		pcl::PointCloud<pcl::PointXYZI> cloud_output_temp;

		pcl::copyPointCloud(*cloud_input, *cloud_input_temp);

		pcl::ExtractIndices<pcl::PointXYZI> extract;
		extract.setInputCloud(cloud_input_temp);
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(cloud_output_temp);
		pcl::copyPointCloud(cloud_output_temp, cloud_output);

	//part for extracting the features. PointTypa is gotta be of type pcl::PointCloud<pcl::PointXYZLRegionF>
	//  for (uint i = 0; i < cloud_input->points.size(); i++) {
	//		if (cloud_input->points[i].f != 0) {

	//			pcl::PointXYZI searchPointTemp;
	//			searchPointTemp.x = cloud_input->points[i].x;
	//			searchPointTemp.y = cloud_input->points[i].y;
	//			searchPointTemp.z = cloud_input->points[i].z;
	//			searchPointTemp.intensity = cloud_input->points[i].f;

	//			pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;

	//			kdtree.setInputCloud(cloud_output_temp.makeShared());

	//			std::vector<int> pointIdxRadiusSearch;
	//			std::vector<float> pointRadiusSquaredDistance;

	//			if (kdtree.nearestKSearch(searchPointTemp, 1, pointIdxRadiusSearch,
	//					pointRadiusSquaredDistance) > 0) {

	//				cloud_output.points[pointIdxRadiusSearch[0]].f =
	//						cloud_input->points[i].f;

	//			}

	//		}
	//	}
	}

	void normalsEstimation(
		const PointCloudPtr &cloud,
		pcl::PointCloud<pcl::Normal>::Ptr &normals
	) {
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp_cloud(
			new pcl::PointCloud<pcl::PointXYZRGBA>
		);
		pcl::copyPointCloud(*cloud, *temp_cloud);

		// Create a KD-Tree
		tree_.reset(new pcl::search::KdTree<pcl::PointXYZRGBA>);

		pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
		ne.setInputCloud(temp_cloud);
		ne.setSearchMethod(tree_);
		ne.setRadiusSearch(normals_radius_search_);
		ne.compute(*normals);
	}

	void boundaryEstimation(
		const PointCloudPtr &cloud_input,
		const pcl::PointCloud<pcl::Normal>::Ptr &normals,
		pcl::PointCloud<pcl::Boundary> &boundaries
	) {
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_input_temp(
			new pcl::PointCloud<pcl::PointXYZRGBA>
		);
		pcl::copyPointCloud(*cloud_input, *cloud_input_temp);
		pcl::BoundaryEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::Boundary> est;
		est.setInputCloud(cloud_input_temp);
		est.setInputNormals(normals);
		est.setRadiusSearch(boundary_radius_);
		est.setSearchMethod(tree_);
		est.compute(boundaries);
	}

	void activeSegmentation(
		const PointCloud &cloud_input,
		float search_radius,
		double eps_angle,
		int fp_index,
		pcl::PointIndices &indices_out
	) {
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		pcl::PointCloud<pcl::Boundary> boundary;

		normalsEstimation(cloud_input.makeShared(), normals);

		boundaryEstimation(cloud_input.makeShared(), normals, boundary);

		//BIG HACK :)
	//        fp_index=magic_index_;

		std::cerr
			<< "Index of seed point is: " << fp_index << std::endl
			<< "Curvature of seed point: "
			<< normals->points[fp_index].curvature << std::endl;

		pcl::PointCloud<pcl::PointXYZRGBA> cloud_in;
		pcl::copyPointCloud(cloud_input, cloud_in);

		pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(
			new pcl::search::KdTree<pcl::PointXYZRGBA>()
		);
		tree->setInputCloud(cloud_in.makeShared());

		if (fp_index > static_cast<int>(cloud_in.points.size()) || fp_index < 0) {
			PCL_ERROR(
				"[pcl::activeSegmentation] Fixation point given is invalid\n"
			);
			return;
		}

		if (boundary.points.size() != cloud_in.points.size()) {
			PCL_ERROR(
				"[pcl::activeSegmentation] Boundary map given was built for a different dataset (%zu) than the input cloud (%zu)!\n",
				boundary.points.size(),
				cloud_in.points.size()
			);
			return;
		}

		if (tree->getInputCloud()->points.size() != cloud_in.points.size()) {
			PCL_ERROR(
				"[pcl::activeSegmentation] Tree built for a different point cloud dataset (%zu) than the input cloud (%zu)!\n",
				tree->getInputCloud()->points.size(),
				cloud_in.points.size()
			);
			return;
		}

		if (normals->points.size() != cloud_in.points.size()) {
			PCL_ERROR(
				"[pcl::activeSegmentation] Input Normals are for a different point cloud dataset (%zu) than the input cloud (%zu)!\n",
				normals->points.size(),
				cloud_in.points.size()
			);
			return;
		}

		std::vector<int> seed_queue;
		seed_queue.push_back(fp_index);

		indices_out.indices.push_back(fp_index);

		std::vector<bool> processed(cloud_in.size(), false);
		processed[fp_index] = true;

		std::vector<int> nn_indices;
		std::vector<float> nn_distances;

		//process while there are valid seed points
		for (size_t seed_idx = 0; seed_idx < seed_queue.size(); ++seed_idx) {
			int curr_seed = seed_queue[seed_idx];

			// Search for seeds
			if (!tree->radiusSearch(curr_seed, search_radius, nn_indices, nn_distances))
				continue;

			//process all points found in the neighborhood
			bool stop_growing = false;
			size_t indices_old_size = indices_out.indices.size();
			for(std::vector<int>::iterator nn = boost::next(nn_indices.begin()); nn != nn_indices.end(); ++nn) {
				if (processed[*nn])
					continue;
				if (boundary.points[*nn].boundary_point != 0) {
					stop_growing = true;
					indices_out.indices.push_back(*nn);
					processed[*nn] = true;
					break;
				}

				pcl::PointXYZRGBA p_in = cloud_in.points[fp_index];
				pcl::PointXYZRGBA p_nn = cloud_in.points[*nn];

				pcl::PointXYZRGBA temp;
				temp.x = p_in.x - p_nn.x;
				temp.y = p_in.y - p_nn.y;
				temp.z = p_in.z - p_nn.z;

				double dot_p =
					normals->points[*nn].normal[0] * temp.x
					+ normals->points[*nn].normal[1] * temp.y
					+ normals->points[*nn].normal[2] * temp.z;

				dot_p = dot_p > 1 ? 1 : dot_p;
				dot_p = dot_p < -1 ? -1 : dot_p;

				if (acos(dot_p) > eps_angle * M_PI / 180) {
					// is convex
					indices_out.indices.push_back(*nn);
					processed[*nn] = true;
				} else {
					break;
				}
			} //new neighbor

			if (!stop_growing && (indices_old_size != indices_out.indices.size())) {
				for (size_t j = indices_old_size - 1; j < indices_out.indices.size(); ++j)
					seed_queue.push_back(indices_out.indices.at(j));
			}
		} //new seed point
	}

	void reconstructDenseModel(
		const int feature_number,
		float search_radius = 0.01f,
		double eps_angle = 89.0
	) {
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr euclidian_cloud(
			new pcl::PointCloud<pcl::PointXYZRGB>
		);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr segments(
			new pcl::PointCloud<pcl::PointXYZRGB>
		);

		std::vector<int> feature_indices;
		for (int i = 0; i < cloud_operational_->size(); ++i) {
			if (cloud_operational_->points[i].f == feature_number)
				feature_indices.push_back(i);
		}

		pcl::copyPointCloud(*cloud_operational_, feature_indices, *euclidian_cloud);

		//euclidean clustering
		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kd_tree(
			new pcl::search::KdTree<pcl::PointXYZRGB>
		);

		ec.setClusterTolerance(euclidean_tolerance_); // 1cm
		ec.setMinClusterSize(min_cluster_size_);
		ec.setMaxClusterSize(max_cluster_size_);
		ec.setSearchMethod(kd_tree);
		ec.setInputCloud(euclidian_cloud);
		ec.extract(cluster_indices);

		std::cerr
			<< "Size of the cluster_indices: " << cluster_indices.size()
			<< std::endl;

		for (
			int cluster_nu = 0;
			cluster_nu < cluster_indices.size();
			cluster_nu++
		) {
			if (debug_) {
				std::cerr
					<< "Size of one cluster from cluster indices: "
					<< cluster_indices[cluster_nu].indices.size()
					<< std::endl;
			}
			int fp_index = 0;

			Eigen::Vector4f c;

			pcl::PointIndices index;
			for (std::vector<int>::const_iterator pit =
					cluster_indices[cluster_nu].indices.begin();
				pit != cluster_indices[cluster_nu].indices.end();
				pit++
			) {
				index.indices.push_back(feature_indices[*pit]);
			}

			pcl::compute3DCentroid<pcl::PointXYZLRegionF>(
				*cloud_operational_,
				index,
				c
			);
			pcl::copyPointCloud(*cloud_operational_, *segments);

			pcl::PointXYZRGB searchPointTemp;
			searchPointTemp.x = c(0);
			searchPointTemp.y = c(1);
			searchPointTemp.z = c(2);

			pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;

			kdtree.setInputCloud(segments);

			std::vector<int> pointIdxRadiusSearch;
			std::vector<float> pointRadiusSquaredDistance;

			if (kdtree.nearestKSearch(
					searchPointTemp, 1, pointIdxRadiusSearch,
					pointRadiusSquaredDistance) > 0
			) {
				fp_index = pointIdxRadiusSearch[0];
			}			

			pcl::PointCloud<pcl::Normal>::Ptr normals(
				new pcl::PointCloud<pcl::Normal>
			);

			if (debug_) {
				std::cerr
					<< "Number of indices with the feature number "		
					<< feature_number << ": " << feature_indices.size()
					<< std::endl;
			}

			normalsEstimation(cloud_operational_, normals);

			pcl::PointIndices indices_out;
			activeSegmentation(
				*cloud_operational_, search_radius, eps_angle,
				fp_index, indices_out
			);

			//save segment to pcd file if it contains more points the a certain threshold

			PointCloudPtr cloud_segment(new pcl::PointCloud<PointType>);
			for (
				std::vector<int>::const_iterator pit = indices_out.indices.begin();
				pit != indices_out.indices.end();
				pit++
			) {
				cloud_segment->points.push_back(cloud_operational_->points[*pit]);
				cloud_operational_->points[*pit].reg = feature_number + 1;
			}

			cloud_segment->width = static_cast<uint32_t>(cloud_segment->points.size());
			cloud_segment->height = 1;
			cloud_segment->is_dense = true;

			if (debug_) {
				std::cout
					<< "PointCloud representing the segment has "
					<< cloud_segment->points.size() << " data points."
					<< std::endl;
			}

			pcl::io::savePCDFile("segment_cloud.pcd", *cloud_segment);
		}

		for (
			std::vector<int>::const_iterator pit = cluster_indices[0].indices.begin();
			pit != cluster_indices[0].indices.end();
			pit++
		) {
			cloud_operational_->points[feature_indices[*pit]].label = feature_number;
		}

		std::cout << "Resulting cloud saved in RESULT.pcd." << std::endl;
		pcl::io::savePCDFile("RESULT.pcd", *cloud_operational_);
	}

protected:
	PointCloudPtr cloud_operational_;
	PointCloudPtr cloud_;
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree_;
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_;
	bool debug_;
	bool plane_segmentation_;
	int plane_segmentation_max_iter_;
	double plane_segmentation_dist_thresh_;
	double normals_radius_search_;
	double boundary_radius_;
	double euclidean_tolerance_;
	int min_cluster_size_;
	int max_cluster_size_;
	int magic_index_;
};

#endif /* DENSERECONSTRUCTION_H_ */
