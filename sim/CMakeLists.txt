cmake_minimum_required(VERSION 3.8)
project(sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# ROS1 -> ROS2: Qt5 및 OpenGL 관련 의존성 설정
find_package(Qt5Core CONFIG REQUIRED)
find_package(Qt5Widgets CONFIG REQUIRED)
find_package(Qt5Gui CONFIG REQUIRED)
find_package(Qt5Gamepad CONFIG REQUIRED)
find_package(OpenGL REQUIRED)

############ Start

# Include directories
include_directories(include)
include_directories(../common/include)
include_directories(../lcm-types/cpp)
include_directories(/usr/local/include/lcm)   # lcm includes
include_directories(../third-party/ParamHandler)
include_directories(${CMAKE_BINARY_DIR})

file(GLOB sources "src/*.cpp")

set(QOBJECT_HEADERS
    include/Graphics3D.h
    include/GameController.h
    include/SimControlPanel.h
    include/JoystickTest.h)

# ROS1 -> ROS2: QT5_WRAP_CPP 대신 qt5_wrap_cpp 사용
qt5_wrap_cpp(tst_hdr_moc ${QOBJECT_HEADERS})

set(OpenGL_GL_PREFERENCE LEGACY)

include_directories(${OPENGL_INCLUDE_DIR})

# ROS1 -> ROS2: add_executable 및 ament_target_dependencies 사용
add_executable(sim ${sources} ${tst_hdr_moc})
ament_target_dependencies(sim rclcpp std_msgs)

target_link_libraries(sim biomimetics pthread lcm inih dynacore_param_handler 
    Qt5::Widgets Qt5::Core Qt5::Gui Qt5::Gamepad ${OPENGL_LIBRARIES})

if (CMAKE_SYSTEM_NAME MATCHES Linux)
  target_link_libraries(sim rt ${GLUT_glut_LIBRARIES})
  include(CheckIncludeFileCXX)
endif()

if (APPLE)
  target_link_libraries(sim "-framework GLUT")
endif()

############ End

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
ament_export_include_directories(include)
ament_export_libraries(sim)
ament_package()
