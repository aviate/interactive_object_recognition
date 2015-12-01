cmake_minimum_required(VERSION 2.8.3)
project(feature_cv_waitinglist)

##################
## SIFT Options ##
##################
set(USE_SIFT_GPU        0)
set(ENV{SIFT_GPU_MODE}  2) #CUDA = 1, GLSL = 2
set(ENV{SIFT_GPU_CUDA_PATH} /usr/local/cuda)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})

############
## OpenCV ##
############
find_package(OpenCV REQUIRED)
set(LIBS_LINK ${LIBS_LINK} ${OpenCV_LIBS})
MESSAGE("OpenCV libs"${OpenCV_LIBS})

###########
## Eigen ##
###########
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.packagey. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/FeatureConfig.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES feature_cv_waitinglist
  CATKIN_DEPENDS  roscpp dynamic_reconfigure
#  DEPENDS system_lib
)

#############
## SiftGPU ##
#############
IF (${USE_SIFT_GPU})
  SET(sift_gpu_build_path ${CMAKE_SOURCE_DIR}/external/siftgpu/linux)

  include_directories(${CMAKE_CURRENT_BINARY_DIR} external/siftgpu/src)

  MESSAGE("\n------------------------------------------------------------------\n")
  MESSAGE("Compiling SiftGPU...")
  EXECUTE_PROCESS(
    COMMAND make siftgpu
    WORKING_DIRECTORY ${sift_gpu_build_path}
    RESULT_VARIABLE sift_gpu_return
  )
  MESSAGE("\n------------------------------------------------------------------\n")

  #on error
  IF (NOT ${sift_gpu_return} EQUAL 0)
    MESSAGE(FATAL_ERROR "SiftGPU cannot be compiled. Returned: ${sift_gpu_return}")
  ENDIF (NOT ${sift_gpu_return} EQUAL 0)

  FILE(COPY external/siftgpu/linux/bin/libsiftgpu.so DESTINATION ../lib)

  add_definitions(-DUSE_SIFT_GPU)
  add_definitions(-DSIFT_GPU_MODE=$ENV{SIFT_GPU_MODE})


  SET(LIBS_LINK ${LIBS_LINK} siftgpu)

ENDIF (${USE_SIFT_GPU})

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(feature_cv_waitinglist
#   src/${PROJECT_NAME}/feature_cv_waitinglist.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(feature_cv_waitinglist ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_library(feature_cv_waitinglist src/feature_matching.cpp src/sift_gpu_wrapper.cpp)
#add_library(homography src/homography_estimation.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(feature_cv_waitinglist ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(main_reconstruction ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(feature_cv_waitinglist ${LIBS_LINK})
#target_link_libraries(homography ${LIBS_LINK})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS feature_cv_waitinglist feature_cv_waitinglist_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_feature_cv_waitinglist.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
