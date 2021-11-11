cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pointcloud_preprocess)

add_compile_options(-std=c++11)


set(PCL_DIR "/usr/local/pcl-1.9/share/pcl-1.9")
find_package(PCL 1.9 REQUIRED COMPONENTS)
include_directories(
  include ${catkin_INCLUDE_DIRS}${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES surface_reconstruction
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(
  include ${catkin_INCLUDE_DIRS}
# include
# ${catkin_INCLUDE_DIRS}
)




add_executable(pre_processing
  src/pre_processing.cpp
)
add_dependencies(pre_processing ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pre_processing
  ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)

add_executable(disp_pcd_file
  src/disp_pcd_file.cpp
)
add_dependencies(disp_pcd_file ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(disp_pcd_file
  ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)

add_executable(pointcloud_topic_use_sample
  src/pointcloud_topic_use_sample.cpp
)
add_dependencies(pointcloud_topic_use_sample ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pointcloud_topic_use_sample
  ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)

add_executable(load_pcd_and_segmentation
  src/load_pcd_and_segmentation.cpp
)
add_dependencies(load_pcd_and_segmentation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(load_pcd_and_segmentation
  ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)

add_executable(cluster_extraction
  src/cluster_extraction.cpp
)
add_dependencies(cluster_extraction ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cluster_extraction
  ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)

add_executable(make_obb_and_extraction
  src/make_obb_and_extraction.cpp
)
add_dependencies(make_obb_and_extraction ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(make_obb_and_extraction
  ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)