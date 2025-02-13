cmake_minimum_required(VERSION 2.8.3)

project(world_modelling_cdt)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  eigen_conversions
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf
  cdt_msgs
  grid_map_ros
  grid_map_core
  grid_map_cv
  grid_map_msgs
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS  roscpp
                  eigen_conversions
                  geometry_msgs
                  std_msgs
                  tf2
                  tf2_ros
                  tf
                  cdt_msgs
                  grid_map_ros
                  grid_map_core
                  grid_map_cv
                  grid_map_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Create library
add_library(${PROJECT_NAME}
            src/world_modelling_cdt/world_modelling.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBS})

# Create node
add_executable(${PROJECT_NAME}_node  src/world_modelling_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})


#############
## Install ##
#############

install(TARGETS ${TARGETS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
