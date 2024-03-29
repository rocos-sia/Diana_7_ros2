cmake_minimum_required(VERSION 3.8)
project(diana7_bonebilling)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

include_directories(DianaApi/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/DianaApi/bin)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/DianaApi/lib)
# find_library(DianaApi_LIBRARY NAMES DianaApi BasicSdk GenericAlgorithm ToolSdk xml2 HINTS ${CMAKE_CURRENT_SOURCE_DIR}/DianaApi/lib  ${CMAKE_CURRENT_SOURCE_DIR}/DianaApi/bin)
find_package(Threads)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(orocos_kdl REQUIRED)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable(joint_state_publisher src/sensor.cpp)
#target_link_libraries(talker  ${catkin_LIBRARIES} rocos::hardware rocos::kinematics rocos::drive rocos::protocol rocos::robot  Eigen3::Eigen gflags::gflags )
target_link_libraries(joint_state_publisher  ${catkin_LIBRARIES} DianaApi BasicSdk GenericAlgorithm ToolSdk xml2 Boost::system Boost::thread Threads::Threads orocos-kdl  yaml-cpp)
#ament_target_dependencies(talker rclcpp std_msgs sensor_msgs)
ament_target_dependencies(joint_state_publisher rclcpp std_msgs sensor_msgs)

install(TARGETS
joint_state_publisher
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include DianaApi
  DESTINATION share/${PROJECT_NAME})





if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
