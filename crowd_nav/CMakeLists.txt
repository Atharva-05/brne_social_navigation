cmake_minimum_required(VERSION 2.8.3)
project(crowd_nav)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  tf2
  tf2_ros
  geometry_msgs
  visualization_msgs
  nav_msgs
  sensor_msgs
  image_geometry
  crowd_nav_interfaces
  brnelib
  actionlib
  mbf_msgs
)

find_package(Armadillo REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${ARMADILLO_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    std_srvs
    tf2
    tf2_ros
    geometry_msgs
    visualization_msgs
    nav_msgs
    sensor_msgs
    image_geometry
    crowd_nav_interfaces
    brnelib
    actionlib
    mbf_msgs
)

add_executable(crowd_nav src/brne_nav.cpp)
add_dependencies(crowd_nav ${catkin_EXPORTED_TARGETS} crowd_nav_interfaces)
target_link_libraries(crowd_nav ${catkin_LIBRARIES} brnelib ${ARMADILLO_LIBRARIES})

add_executable(controller src/controller.cpp)
target_link_libraries(controller
  ${catkin_LIBRARIES}
)

add_executable(mbf_exe_path_test src/mbf_exe_path_test.cpp)
target_link_libraries(mbf_exe_path_test ${catkin_LIBRARIES})

# add_executable(arena_brne_nav src/arena_brne_nav.cpp)
# target_link_libraries(arena_brne_nav ${catkin_LIBRARIES})

# add_executable(pub_goal src/pub_goal.cpp)
# target_link_libraries(pub_goal
#   ${catkin_LIBRARIES}
# )

install(TARGETS crowd_nav controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)