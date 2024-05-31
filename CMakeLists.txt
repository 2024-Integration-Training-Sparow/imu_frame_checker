cmake_minimum_required(VERSION 3.14)
project(imu_frame_checker)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()


find_package(ament_cmake_auto REQUIRED)
find_package(sensor_msgs REQUIRED)  # sensor_msgsパッケージの追加。多分これないと動かんけど理由わからん。
ament_auto_find_build_dependencies()


ament_auto_add_executable(imu_frame_checker src/imu_frame_checker.cpp)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
