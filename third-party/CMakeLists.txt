cmake_minimum_required(VERSION 3.8)
project(third-party)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

#### Start

# 서브 디렉토리 추가
add_subdirectory(Goldfarb_Optimizer)
add_subdirectory(ParamHandler)
add_subdirectory(inih)
add_subdirectory(osqp)
add_subdirectory(JCQP)
add_subdirectory(qpOASES)
add_subdirectory(lord_imu)
add_subdirectory(SOEM)

if(CMAKE_SYSTEM_NAME MATCHES Linux)
  add_subdirectory(vectornav)
endif()

#### End

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
