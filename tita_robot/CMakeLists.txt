cmake_minimum_required(VERSION 3.8)
project(tita_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -O3")

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
include_directories(include)
set(SRC_FILES
    src/tita_robot.cpp
    src/can_receiver.cpp
    src/can_sender.cpp
)
ament_auto_add_library(${PROJECT_NAME} SHARED ${SRC_FILES} )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  add_executable(test_executable test/test.cpp)
  target_link_libraries(test_executable tita_robot)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
