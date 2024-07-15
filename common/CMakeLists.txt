cmake_minimum_required(VERSION 3.8)
project(common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)  # ROS1 -> ROS2: Eigen3 의존성 추가
find_package(LCM REQUIRED)     # ROS1 -> ROS2: LCM 의존성 추가


######## Start

# Source/Include files
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})  # ROS1 -> ROS2: Eigen3 include 디렉터리 추가
include_directories(${LCM_INCLUDE_DIR})  # ROS1 -> ROS2: LCM include 디렉터리 추가
include_directories("../lcm-types/cpp")
include_directories("../third-party/inih")
include_directories("../third-party/osqp/include")
include_directories("../third-party/ParamHandler")
include_directories(FootstepPlanner)

file(GLOB_RECURSE sources "src/*.cpp")

# Library
add_library(biomimetics SHARED ${sources})
ament_target_dependencies(biomimetics rclcpp std_msgs)  # ROS1 -> ROS2: ament_target_dependencies 사용

target_link_libraries(biomimetics inih dynacore_param_handler JCQP osqp ${LCM_LIBRARIES})


# Google Test integration for Linux
if(CMAKE_SYSTEM_NAME MATCHES Linux)
    # Pull in Google Test
    include(CTest)
    if (CMAKE_VERSION VERSION_LESS 3.2)
        set(UPDATE_DISCONNECTED_IF_AVAILABLE "")
    else()
        set(UPDATE_DISCONNECTED_IF_AVAILABLE "UPDATE_DISCONNECTED 1")
    endif()
    include(${CMAKE_SOURCE_DIR}/common/DownloadProject.cmake)
    download_project(PROJ                googletest
                     GIT_REPOSITORY      https://github.com/google/googletest.git
                     GIT_TAG             master
                     ${UPDATE_DISCONNECTED_IF_AVAILABLE}
                     QUIET
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR})
    if (CMAKE_VERSION VERSION_LESS 2.8.11)
        include_directories("${gtest_SOURCE_DIR}/include"
                            "${gmock_SOURCE_DIR}/include")
    endif()

    # Test
    file(GLOB_RECURSE test_sources "test/test_*.cpp")
    add_executable(test-common ${test_sources})
    ament_target_dependencies(test-common rclcpp std_msgs)  # ROS1 -> ROS2: ament_target_dependencies 사용
    target_link_libraries(test-common gtest gmock_main ${LCM_LIBRARIES} rt inih osqp dynacore_param_handler pthread biomimetics)
    target_link_libraries(test-common Goldfarb_Optimizer)
    target_link_libraries(test-common JCQP)

    add_test(NAME example_test COMMAND test-common)
endif()

# Our libraries
add_subdirectory(FootstepPlanner)

####### End


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

# Ament package
ament_export_include_directories(include)  # ROS1 -> ROS2: ament_export_include_directories 사용
ament_export_libraries(biomimetics)  # ROS1 -> ROS2: ament_export_libraries 사용
ament_package()  # ROS1 -> ROS2: ament_package 사용
