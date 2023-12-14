cmake_minimum_required(VERSION 3.8)
project(stereo_vo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mpopcnt")
list( APPEND CMAKE_MODULE_PATH /home/nine-fish/envs/g2o/cmake_modules ) # g2o安装位置

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV 4.8.0 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Sophus REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(FMT REQUIRED)
find_package(G2O REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIRS}
)

set(dependencies 
  rclcpp
  rclcpp_components
  OpenCV
  image_transport
  sensor_msgs
  Eigen3
  Sophus
  Ceres
  cv_bridge
  FMT
  G2O
  tf2
  tf2_ros
  geometry_msgs
)

set(sources
  src/stereo_vo/stereo_vo.cpp
  src/stereo_vo/structs.cpp
  src/stereo_vo/map.cpp
  src/stereo_vo/camera_vo.cpp
  src/stereo_vo/frontend.cpp
  src/stereo_vo/backend.cpp
  src/stereo_vo_node.cpp
)

add_library(${PROJECT_NAME} SHARED
  ${sources} 
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}  
)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE "COMPOSITION_BUILDING_DLL"
)

rclcpp_components_register_nodes(${PROJECT_NAME}
  PLUGIN "stereo_vo::StereoVONode"
  EXECUTABLE ${PROJECT_NAME}_node
)

add_executable(${PROJECT_NAME}_node
  ${sources} 
)

ament_target_dependencies(${PROJECT_NAME}_node
  ${dependencies}
)

target_link_libraries(${PROJECT_NAME}_node
  ${OpenCV_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${G2O_LIBRARIES}
  g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension
)

install(TARGETS 
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS 
  ${PROJECT_NAME}_node
  # testing 
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
