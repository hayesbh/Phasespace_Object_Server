cmake_minimum_required(VERSION 2.8.3)
project(Phasespace_Object_Server)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  visualization_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)



#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(server src/server.cpp src/save.cpp src/PhaseSpace/Object.cpp src/PhaseSpace/ObjectType.cpp src/PhaseSpace/ManualObject.cpp src/PhaseSpace/PSObject.cpp src/PhaseSpace/DefaultType.cpp src/PhaseSpace/GloveType.cpp src/PhaseSpace/Point.cpp src/PhaseSpace/quaternion.cpp)
target_link_libraries(server owlsock)

add_executable(add_object src/add_object.cpp)
target_link_libraries(add_object)

add_executable(add_points src/add_points.cpp)
target_link_libraries(add_points)

add_executable(delete_object src/delete_object.cpp)
target_link_libraries(delete_object)

add_executable(add_manual src/add_manual.cpp)
target_link_libraries(add_manual)

add_executable(collides src/collides.cpp)
target_link_libraries(collides)

add_executable(box_filled src/box_filled.cpp)
target_link_libraries(box_filled)

add_executable(save_object src/save_object.cpp)
target_link_libraries(save_object)

add_executable(save_env src/save_env.cpp)
target_link_libraries(save_env)

add_executable(load_object src/load_object.cpp)
target_link_libraries(load_object)

add_executable(load_env src/load_env.cpp)
target_link_libraries(load_env)

set(OBJECTFILES src/PhaseSpace/Point.cpp src/PhaseSpace/quaternion.cpp src/PhaseSpace/GloveType.cpp src/PhaseSpace/DefaultType.cpp src/PhaseSpace/ManualObject.cpp src/PhaseSpace/ObjectType.cpp src/PhaseSpace/Object.cpp src/PhaseSpace/PSObject.cpp)

add_gtest(test/ServerTest src/gtests/ServerTest.cpp src/save.cpp ${OBJECTFILES})

add_gtest(test/PSObjectTest src/gtests/PSObjectTest.cpp ${OBJECTFILES})

add_gtest(test/DefaultObjectTest src/gtests/DefaultObjectTest.cpp ${OBJECTFILES})

add_gtest(test/ObjectTest src/gtests/ObjectTest.cpp src/PhaseSpace/Point.cpp src/PhaseSpace/quaternion.cpp src/PhaseSpace/GloveType.cpp src/PhaseSpace/DefaultType.cpp src/PhaseSpace/ManualObject.cpp src/PhaseSpace/ObjectType.cpp src/PhaseSpace/Object.cpp)
add_gtest(test/GloveTest src/gtests/GloveTest.cpp src/PhaseSpace/Point.cpp src/PhaseSpace/GloveType.cpp src/PhaseSpace/ObjectType.cpp src/PhaseSpace/quaternion.cpp)
add_gtest(test/QuatTest src/gtests/QuatTest.cpp src/PhaseSpace/Point.cpp src/PhaseSpace/quaternion.cpp)
add_gtest(test/PointTest src/gtests/PointTest.cpp src/PhaseSpace/Point.cpp)
