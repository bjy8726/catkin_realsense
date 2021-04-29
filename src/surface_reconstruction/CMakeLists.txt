cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(surface_reconstruction)

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


add_executable(greedy_projection_triangulation src/greedy_projection_triangulation.cpp)
add_dependencies(greedy_projection_triangulation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#后面的${PCL_LIBRARIES}一定要加,否则会出现函数未定义的错误,容易忘记.
target_link_libraries(greedy_projection_triangulation ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(bspline_fitting src/bspline_fitting.cpp)
add_dependencies(bspline_fitting ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#每次添加一个cpp文件，都要添加PCL的动态库，否则编译会报错，“函数......未定义的引用”
target_link_libraries(bspline_fitting ${catkin_LIBRARIES} ${PCL_LIBRARIES})





add_executable(test2
  src/modify2.cpp
  include/surface_reconstruction/modify2.h
  src/test2.cpp
)
add_dependencies(test2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test2 ${catkin_LIBRARIES} ${PCL_LIBRARIES})