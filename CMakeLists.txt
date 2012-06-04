##
## CMakeLists.txt
## Login : <ctaf@localhost.localdomain>
##
## Author(s):
##  - Cedric GESTES <gestes@aldebaran-robotics.com>
##
## Copyright (C) 2009, 2011 Aldebaran Robotics
##

project(ALInfrared)
cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)

if(NOT I_AM_A_ROBOT)
    return()
endif()

include("${CMAKE_CURRENT_SOURCE_DIR}/bootstrap.cmake")

set(INFRARED_SRCS
  src/alinfrared.h
  src/alinfrared.cpp
  src/infraredmain.cpp
)

qi_create_lib(infrared SHARED SUBFOLDER naoqi ${INFRARED_SRCS})
qi_use_lib(infrared ALCOMMON ALMATH LIRCCLIENT)

