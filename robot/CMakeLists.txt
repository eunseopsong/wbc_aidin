cmake_minimum_required(VERSION 3.8)
project(robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies # ROS1 -> ROS2: ament_cmake 빌드 시스템 사용
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED) ## added
find_package(LCM REQUIRED)    ## added

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

########## Start

# Include directories
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})
include_directories("include/")
include_directories("./")
include_directories("../common/include/")
include_directories("../third-party")
include_directories("../third-party/ParamHandler")
include_directories("../third-party/vectornav/include")
include_directories("../third-party/vectornav/include/vn")
include_directories("../third-party/lord_imu/Include")

include_directories("../lcm-types/cpp")
include_directories("/usr/local/include/lcm/")   # lcm includes
include_directories("../third-party/SOEM/osal")
include_directories("../third-party/SOEM/osal/linux")
include_directories("../third-party/SOEM/oshw")
include_directories("../third-party/SOEM/oshw/linux")

# Source files
file(GLOB sources "src/*.cpp" "src/rt/*.c" "src/rt/*.cpp")

# Library
add_library(robot SHARED ${sources})

# ROS1 -> ROS2: ament_target_dependencies 사용
ament_target_dependencies(robot rclcpp std_msgs Eigen3 LCM)

target_link_libraries(robot biomimetics pthread ${LCM_LIBRARIES} inih dynacore_param_handler lord_imu soem)

if(CMAKE_SYSTEM_NAME MATCHES Linux)
  target_link_libraries(robot libvnc rt)
endif()

# Ament package
ament_export_include_directories(include)  # ROS1 -> ROS2: ament_export_include_directories 사용
ament_export_libraries(robot)  # ROS1 -> ROS2: ament_export_libraries 사용
ament_package()  # ROS1 -> ROS2: ament_package 사용

########## End
