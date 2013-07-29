cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()


include_directories(${PROJECT_SOURCE_DIR}/ext/rapidjson/include)
#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(server src/server.cpp src/PhaseSpace/Object.cpp src/PhaseSpace/ObjectType.cpp src/PhaseSpace/ManualObject.cpp src/PhaseSpace/PSObject.cpp src/PhaseSpace/DefaultType.cpp src/PhaseSpace/GloveType.cpp src/PhaseSpace/Point.cpp src/PhaseSpace/quaternion.cpp)
target_link_libraries(server owlsock)

rosbuild_add_executable(add_object src/add_object.cpp)
target_link_libraries(add_object)

rosbuild_add_executable(add_points src/add_points.cpp)
target_link_libraries(add_points)

rosbuild_add_executable(delete_object src/delete_object.cpp)
target_link_libraries(delete_object)

rosbuild_add_executable(add_manual src/add_manual.cpp)
target_link_libraries(add_manual)

set(OBJECTFILES src/PhaseSpace/Point.cpp src/PhaseSpace/quaternion.cpp src/PhaseSpace/GloveType.cpp src/PhaseSpace/DefaultType.cpp src/PhaseSpace/ManualObject.cpp src/PhaseSpace/ObjectType.cpp src/PhaseSpace/Object.cpp src/PhaseSpace/PSObject.cpp)

rosbuild_add_gtest(test/ServerTest src/gtests/ServerTest.cpp ${OBJECTFILES})

rosbuild_add_gtest(test/PSObjectTest src/gtests/PSObjectTest.cpp ${OBJECTFILES})

rosbuild_add_gtest(test/DefaultObjectTest src/gtests/DefaultObjectTest.cpp ${OBJECTFILES})

rosbuild_add_gtest(test/ObjectTest src/gtests/ObjectTest.cpp src/PhaseSpace/Point.cpp src/PhaseSpace/quaternion.cpp src/PhaseSpace/GloveType.cpp src/PhaseSpace/DefaultType.cpp src/PhaseSpace/ManualObject.cpp src/PhaseSpace/ObjectType.cpp src/PhaseSpace/Object.cpp)
rosbuild_add_gtest(test/GloveTest src/gtests/GloveTest.cpp src/PhaseSpace/Point.cpp src/PhaseSpace/GloveType.cpp src/PhaseSpace/ObjectType.cpp src/PhaseSpace/quaternion.cpp)
rosbuild_add_gtest(test/QuatTest src/gtests/QuatTest.cpp src/PhaseSpace/Point.cpp src/PhaseSpace/quaternion.cpp)
rosbuild_add_gtest(test/PointTest src/gtests/PointTest.cpp src/PhaseSpace/Point.cpp)
