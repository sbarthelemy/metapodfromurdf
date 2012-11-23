## Copyright (c) 2012 Aldebaran Robotics. All rights reserved
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file

cmake_minimum_required(VERSION 2.8)
project(metapodfromurdf)
find_package(qibuild)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

if (NOT CMAKE_CROSSCOMPILING)
  include_directories(include)
  qi_create_lib(metapodfromurdf
    src/metapodfromurdf.cpp
    include/metapodfromurdf/metapodfromurdf.hpp)

  qi_use_lib(metapodfromurdf tinyxml urdf roscpp metapod eigen3)
  qi_stage_lib(metapodfromurdf)

  qi_create_bin(metapodfromurdf-bin
    src/metapodfromurdf-bin)
  qi_use_lib(metapodfromurdf-bin metapodfromurdf boost_program_options)
  qi_stage_bin(metapodfromurdf-bin)
endif()
