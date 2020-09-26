PROJECT(motion_planning)
cmake_minimum_required(VERSION 2.6)

include_directories(/usr/include/python3.6m)
link_directories(/usr/lib/python3.6/config-3.6m-x86_64-linux-gnu)

SET (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/.orogen/config")
INCLUDE(motion_planningBase)

# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)
