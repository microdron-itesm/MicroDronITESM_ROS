cmake_minimum_required(VERSION 2.8.3)
project(drone_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp)
find_package(SFML REQUIRED network audio graphics window system)

catkin_package(
   CATKIN_DEPENDS roscpp
)

link_libraries(${catkin_LIBRARIES} sfml-graphics sfml-window sfml-system)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE drone_control_src
        "src/*.h"
        "src/*.cpp"
        "src/**/*.cpp"
        "src/**/*.hpp"
        "src/**/*.h"
        "src/*.hpp"
        "src/*.c"
        "src/**/*.c"
)



add_executable(${PROJECT_NAME}_node ${drone_control_src})