## Copyright (c) 2012 Aldebaran Robotics. All rights reserved
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file

cmake_minimum_required(VERSION 2.8)
project(metapodfromurdf)
find_package(qibuild)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

if (NOT CMAKE_CROSSCOMPILING)
  include_directories(include)
  qi_create_bin(metapodfromurdf
    src/metapodfromurdf.cpp)
  qi_use_lib(metapodfromurdf boost_program_options tinyxml urdf roscpp metapod metapod_robotbuilder eigen3)
  qi_stage_bin(metapodfromurdf)
endif()
