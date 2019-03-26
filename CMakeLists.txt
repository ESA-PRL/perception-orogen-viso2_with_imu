PROJECT(viso2_with_imu)
cmake_minimum_required(VERSION 2.6)

add_definitions(-DWall)
add_definitions(-DBASE_LOG_NAMESPACE=${PROJECT_NAME})
add_definitions(-DBASE_LOG_INFO)

SET (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/.orogen/config")
INCLUDE(viso2_with_imuBase)
