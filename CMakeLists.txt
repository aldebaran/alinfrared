
project(ALInfrared)
cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)

include("${CMAKE_CURRENT_SOURCE_DIR}/bootstrap.cmake")
use(NAOQI-PLUGINS-TOOLS)

create_module(infrared DEFAULT_REMOTE_OFF)

add_subdirectory(src)
