## *********************************************************
## 
## File autogenerated for the feature_cv_waitinglist package 
## by the dynamic_reconfigure package.
## Please do not edit.
## 
## ********************************************************/

##**********************************************************
## Software License Agreement (BSD License)
##
##  Copyright (c) 2008, Willow Garage, Inc.
##  All rights reserved.
##
##  Redistribution and use in source and binary forms, with or without
##  modification, are permitted provided that the following conditions
##  are met:
##
##   * Redistributions of source code must retain the above copyright
##     notice, this list of conditions and the following disclaimer.
##   * Redistributions in binary form must reproduce the above
##     copyright notice, this list of conditions and the following
##     disclaimer in the documentation and/or other materials provided
##     with the distribution.
##   * Neither the name of the Willow Garage nor the names of its
##     contributors may be used to endorse or promote products derived
##     from this software without specific prior written permission.
##
##  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
##  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
##  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
##  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
##  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
##  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
##  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
##  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
##  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
##  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
##  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
##  POSSIBILITY OF SUCH DAMAGE.
##**********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'upper': 'DEFAULT', 'lower': 'groups', 'srcline': 233, 'name': 'Default', 'parent': 0, 'srcfile': '/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT', 'field': 'default', 'state': True, 'parentclass': '', 'groups': [], 'parameters': [{'srcline': 259, 'description': 'Feature Detector', 'max': 6, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'feature_detector', 'edit_method': "{'enum_description': 'Feature Detectors', 'enum': [{'srcline': 10, 'description': 'SIFT', 'srcfile': '/home/eric/catkin_ws/src/feature_cv_waitinglist/cfg/FeatureConfig.cfg', 'cconsttype': 'const int', 'value': 0, 'ctype': 'int', 'type': 'int', 'name': 'SIFT'}, {'srcline': 11, 'description': 'SIFTGPU', 'srcfile': '/home/eric/catkin_ws/src/feature_cv_waitinglist/cfg/FeatureConfig.cfg', 'cconsttype': 'const int', 'value': 1, 'ctype': 'int', 'type': 'int', 'name': 'SIFTGPU'}, {'srcline': 12, 'description': 'SURF', 'srcfile': '/home/eric/catkin_ws/src/feature_cv_waitinglist/cfg/FeatureConfig.cfg', 'cconsttype': 'const int', 'value': 2, 'ctype': 'int', 'type': 'int', 'name': 'SURF'}, {'srcline': 13, 'description': 'FAST', 'srcfile': '/home/eric/catkin_ws/src/feature_cv_waitinglist/cfg/FeatureConfig.cfg', 'cconsttype': 'const int', 'value': 3, 'ctype': 'int', 'type': 'int', 'name': 'FAST'}, {'srcline': 14, 'description': 'FAST', 'srcfile': '/home/eric/catkin_ws/src/feature_cv_waitinglist/cfg/FeatureConfig.cfg', 'cconsttype': 'const int', 'value': 4, 'ctype': 'int', 'type': 'int', 'name': 'FAST_grid'}, {'srcline': 15, 'description': 'MSER', 'srcfile': '/home/eric/catkin_ws/src/feature_cv_waitinglist/cfg/FeatureConfig.cfg', 'cconsttype': 'const int', 'value': 5, 'ctype': 'int', 'type': 'int', 'name': 'MSER'}, {'srcline': 16, 'description': 'ORB', 'srcfile': '/home/eric/catkin_ws/src/feature_cv_waitinglist/cfg/FeatureConfig.cfg', 'cconsttype': 'const int', 'value': 6, 'ctype': 'int', 'type': 'int', 'name': 'ORB'}, {'srcline': 17, 'description': 'STAR', 'srcfile': '/home/eric/catkin_ws/src/feature_cv_waitinglist/cfg/FeatureConfig.cfg', 'cconsttype': 'const int', 'value': 7, 'ctype': 'int', 'type': 'int', 'name': 'STAR'}]}", 'default': 0, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'Feature Extractor', 'max': 5, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'feature_extractor', 'edit_method': "{'enum_description': 'Feature Extractors', 'enum': [{'srcline': 21, 'description': 'SIFT', 'srcfile': '/home/eric/catkin_ws/src/feature_cv_waitinglist/cfg/FeatureConfig.cfg', 'cconsttype': 'const int', 'value': 0, 'ctype': 'int', 'type': 'int', 'name': 'SIFT_'}, {'srcline': 22, 'description': 'SIFTGPU', 'srcfile': '/home/eric/catkin_ws/src/feature_cv_waitinglist/cfg/FeatureConfig.cfg', 'cconsttype': 'const int', 'value': 1, 'ctype': 'int', 'type': 'int', 'name': 'SIFTGPU_'}, {'srcline': 23, 'description': 'SURF', 'srcfile': '/home/eric/catkin_ws/src/feature_cv_waitinglist/cfg/FeatureConfig.cfg', 'cconsttype': 'const int', 'value': 2, 'ctype': 'int', 'type': 'int', 'name': 'SURF_'}, {'srcline': 24, 'description': 'ORB', 'srcfile': '/home/eric/catkin_ws/src/feature_cv_waitinglist/cfg/FeatureConfig.cfg', 'cconsttype': 'const int', 'value': 3, 'ctype': 'int', 'type': 'int', 'name': 'ORB_'}, {'srcline': 25, 'description': 'FREAK', 'srcfile': '/home/eric/catkin_ws/src/feature_cv_waitinglist/cfg/FeatureConfig.cfg', 'cconsttype': 'const int', 'value': 4, 'ctype': 'int', 'type': 'int', 'name': 'FREAK_'}, {'srcline': 26, 'description': 'BRIEF', 'srcfile': '/home/eric/catkin_ws/src/feature_cv_waitinglist/cfg/FeatureConfig.cfg', 'cconsttype': 'const int', 'value': 5, 'ctype': 'int', 'type': 'int', 'name': 'BRIEF_'}]}", 'default': 0, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'Descriptor matcher', 'max': 4, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'descriptor_matcher', 'edit_method': "{'enum_description': 'Extractor Matchers', 'enum': [{'srcline': 30, 'description': 'FLANN', 'srcfile': '/home/eric/catkin_ws/src/feature_cv_waitinglist/cfg/FeatureConfig.cfg', 'cconsttype': 'const int', 'value': 0, 'ctype': 'int', 'type': 'int', 'name': 'FLANN'}, {'srcline': 31, 'description': 'Bruteforce_Hamming_1', 'srcfile': '/home/eric/catkin_ws/src/feature_cv_waitinglist/cfg/FeatureConfig.cfg', 'cconsttype': 'const int', 'value': 1, 'ctype': 'int', 'type': 'int', 'name': 'Bruteforce_Hamming_1'}, {'srcline': 32, 'description': 'Bruteforce_Hamming_2', 'srcfile': '/home/eric/catkin_ws/src/feature_cv_waitinglist/cfg/FeatureConfig.cfg', 'cconsttype': 'const int', 'value': 2, 'ctype': 'int', 'type': 'int', 'name': 'Bruteforce_Hamming_2'}, {'srcline': 33, 'description': 'Bruteforce_L1', 'srcfile': '/home/eric/catkin_ws/src/feature_cv_waitinglist/cfg/FeatureConfig.cfg', 'cconsttype': 'const int', 'value': 3, 'ctype': 'int', 'type': 'int', 'name': 'Bruteforce_L1'}, {'srcline': 34, 'description': 'Bruteforce_L2', 'srcfile': '/home/eric/catkin_ws/src/feature_cv_waitinglist/cfg/FeatureConfig.cfg', 'cconsttype': 'const int', 'value': 4, 'ctype': 'int', 'type': 'int', 'name': 'Bruteforce_L2'}]}", 'default': 0, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'Enter the name of a algorithm parameter to change', 'max': '', 'cconsttype': 'const char * const', 'ctype': 'std::string', 'srcfile': '/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'feature_detector_parameter_name', 'edit_method': '', 'default': 'nFeatures', 'level': 0, 'min': '', 'type': 'str'}, {'srcline': 259, 'description': 'Change the parameter here.  It will automatically be converted to the correct type', 'max': '', 'cconsttype': 'const char * const', 'ctype': 'std::string', 'srcfile': '/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'feature_detector_parameter_value', 'edit_method': '', 'default': '0', 'level': 0, 'min': '', 'type': 'str'}, {'srcline': 259, 'description': 'Enter the name of a algorithm parameter to change', 'max': '', 'cconsttype': 'const char * const', 'ctype': 'std::string', 'srcfile': '/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'feature_extractor_parameter_name', 'edit_method': '', 'default': 'nFeatures', 'level': 0, 'min': '', 'type': 'str'}, {'srcline': 259, 'description': 'Change the parameter here.  It will automatically be converted to the correct type', 'max': '', 'cconsttype': 'const char * const', 'ctype': 'std::string', 'srcfile': '/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'feature_extractor_parameter_value', 'edit_method': '', 'default': '0', 'level': 0, 'min': '', 'type': 'str'}, {'srcline': 259, 'description': 'only searches in the same place of the image for a match', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'reduce_search_area', 'edit_method': '', 'default': True, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 259, 'description': 'number of pixels away from keypoint to search for a match', 'max': 150, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'search_distance', 'edit_method': '', 'default': 1, 'level': 0, 'min': 1, 'type': 'int'}, {'srcline': 259, 'description': 'enables the following sliders.  Does radius matching (in descriptor distance space) and rejects close or bad matches', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'distinct_matches', 'edit_method': '', 'default': True, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 259, 'description': 'radius search threshold', 'max': 2000.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'max_radius_search_dist', 'edit_method': '', 'default': 0.5, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'matching_distance_ratio_threshold', 'max': 1.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'matching_distance_ratio_threshold', 'edit_method': '', 'default': 0.5, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'Horizontal matches', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'horizontal_matches', 'edit_method': '', 'default': False, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 259, 'description': 'Horizontal matches threshold', 'max': 100, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'horizontal_threshold', 'edit_method': '', 'default': 1, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'Tracking matches', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'tracking_matches', 'edit_method': '', 'default': True, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 259, 'description': 'Draws two images - one is the template image.', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'drawing_template', 'edit_method': '', 'default': False, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 259, 'description': "keep all matches that didn't match from prev frame or discard them", 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'retain_all_prev_matches', 'edit_method': '', 'default': True, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 259, 'description': 'when matches between current and prev. frame, check both left and right get correct matches based on correspondences in current and prev frames', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'double_check_tracking_matches', 'edit_method': '', 'default': True, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 259, 'description': 'Minimum matches to track', 'max': 100, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'min_tracking_matches', 'edit_method': '', 'default': 1, 'level': 0, 'min': 1, 'type': 'int'}, {'srcline': 259, 'description': 'Tracking matches threshold', 'max': 200.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'tracking_threshold', 'edit_method': '', 'default': 0.1, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'Remove outliers based on distance to the average match movement distance', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'outlier_removal', 'edit_method': '', 'default': True, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 259, 'description': 'Remove circular matches that are a factor of the average distance between matches', 'max': 3.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'distance_factor', 'edit_method': '', 'default': 1.0, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'use this get rosinfos for detection, extracting, matching', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'timing_debug', 'edit_method': '', 'default': False, 'level': 0, 'min': False, 'type': 'bool'}], 'type': '', 'id': 0}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])    
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']

Feature_SIFT = 0
Feature_SIFTGPU = 1
Feature_SURF = 2
Feature_FAST = 3
Feature_FAST_grid = 4
Feature_MSER = 5
Feature_ORB = 6
Feature_STAR = 7
Feature_SIFT_ = 0
Feature_SIFTGPU_ = 1
Feature_SURF_ = 2
Feature_ORB_ = 3
Feature_FREAK_ = 4
Feature_BRIEF_ = 5
Feature_FLANN = 0
Feature_Bruteforce_Hamming_1 = 1
Feature_Bruteforce_Hamming_2 = 2
Feature_Bruteforce_L1 = 3
Feature_Bruteforce_L2 = 4