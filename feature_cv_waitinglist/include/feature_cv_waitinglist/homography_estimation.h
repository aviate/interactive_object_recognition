/*
 * HomographyEstimation.h
 *
 *  Created on: Nov 18, 2012
 *      Author: kidson
 */

 //TODO CANNOT BE COMPILED

#ifndef HOMOGRAPHYESTIMATION_H_
#define HOMOGRAPHYESTIMATION_H_

#include "feature_matching.h"
#include <ros/console.h>
#include <Eigen/Core>

class HomographyEstimation
{
public:
	HomographyEstimation();
	virtual ~HomographyEstimation();

	void calculateHomography(
		const std::vector<FeatureMatching>& input_matches,
		Eigen::Vector3f& homography_solution
	);
private:
	void normalizePoints(
		const std::vector<FeatureMatching>& input_matches,
		std::vector<FeatureMatching>& output_matches,
		Eigen::Vector3f & transform
	);
};

#endif /* HOMOGRAPHYESTIMATION_H_ */
