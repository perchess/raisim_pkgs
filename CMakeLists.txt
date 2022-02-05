cmake_minimum_required(VERSION 3.5)
project(raisim_examples LANGUAGES CXX)

#==
# Dependencies
#==
set(CMAKE_BUILD_TYPE "Release")
# Tacke env cmake_prefix and append  path to raisim libs
string(REPLACE ":" ";" CMAKE_PREFIX_PATH "$ENV{CMAKE_PREFIX_PATH}")
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../raisim_ros/raisim/linux")
message ("CMAKE_PREFIX : " ${CMAKE_PREFIX_PATH})

#find_package(raisim CONFIG REQUIRED)
find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})
find_package(Boost REQUIRED)
find_package(catkin REQUIRED
  roscpp
  trajectory_msgs
  raisim
  quadruped_msgs
  quadruped_ctrl
  roslib
  tf
  geometry_msgs
#  dynamic_reconfigure
  COMPONENTS)
#generate_dynamic_reconfigure_options(
#  cfg/general.cfg
#)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES
   CATKIN_DEPENDS
   roscpp
   raisim
   roslib
   quadruped_msgs
   message_runtime
   quadruped_ctrl
   DEPENDS)


#if(WIN32)
#    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/../raisim/win32/mt_release/bin")
#    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_SOURCE_DIR}/../raisim/win32/mt_debug/bin")
#    file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/../rsc" DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE})
#    file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/../rsc" DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG})
#else()
#    file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/../rsc" DESTINATION ${CMAKE_BINARY_DIR}/examples)
#endif()
#message("LIBS TEST " ${catkin_LIBRARIES})
include_directories (
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS})

#==
# Build Targets
#==

function(create_executable app_name file_name)
    add_executable(${app_name} ${file_name})
#    set_target_properties(${app_name} PROPERTIES MACOSX_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../raisim/mac/lib")
#    if(WIN32)
#        target_link_libraries(${app_name} PUBLIC raisim::raisim Ws2_32 Winmm)
#        target_compile_options(${app_name} PRIVATE "/MP")
#    else()
        target_link_libraries(${app_name} PUBLIC pthread ${catkin_LIBRARIES} )
#    endif()
    target_include_directories(${app_name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
endfunction()

# benchmark examples
create_executable(speed_test src/benchmark/speed_test.cpp)

# server examples
create_executable(primitives src/server/primitives.cpp)
create_executable(meshes src/server/meshes.cpp)
create_executable(laikago src/server/laikago.cpp)
create_executable(balls src/server/balls.cpp)
create_executable(atlas src/server/atlas.cpp)
create_executable(heightmap src/server/heightmap.cpp)
create_executable(robots src/server/aliengo.cpp)
set (minicheetah_src
    src/server/minicheetah/ros_controller.cpp
    src/server/minicheetah/minicheetah.cpp
    include/ros_controller.h )
add_executable(minicheetah ${minicheetah_src})
target_link_libraries(minicheetah PUBLIC pthread ${catkin_LIBRARIES} Eigen3::Eigen ${Boost_LIBRARIES})
create_executable(materials src/server/material.cpp)
create_executable(materialStaticFriction src/server/materialStaticFriction.cpp)
create_executable(heightMapUsingPng src/server/heightMapUsingPng.cpp)
create_executable(wheeledRobot src/server/wheeledRobot.cpp)
create_executable(newtonsCradle src/server/newtonsCradle.cpp)
create_executable(visualObjects src/server/visualObjects.cpp)
create_executable(rayDemo src/server/rayDemo.cpp)
create_executable(rayDemo2 src/server/rayDemo2.cpp)
create_executable(anymals src/server/anymals.cpp)
create_executable(anymal_stress_test src/server/anymals_stress_test.cpp)
create_executable(compound src/server/compound.cpp)
create_executable(kinova src/server/kinova.cpp)
create_executable(cartPole src/server/cartPole.cpp)
create_executable(springTest src/server/springTest.cpp)
create_executable(trackedRobotAndTemplatedURDF src/server/trackedRobotAndTemplatedURDF.cpp)

# xml reader
create_executable(xmlRader src/xml/xmlReader.cpp)

# mjcf files
create_executable(mjcf_humanoid src/mjcf/humanoid.cpp)
create_executable(mjcf_ant src/mjcf/ant.cpp)
create_executable(mjcf_cassie src/mjcf/cassie.cpp)


add_dependencies(minicheetah quadruped_msgs_gencfg)
