## Copyright (c) 2012 Aldebaran Robotics. All rights reserved
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file

set(ROS_ROOT "/opt/ros/fuerte")
set(ROS_ROBOT_MODEL_ROOT "${ROS_ROOT}/stacks/robot_model")
clean(URDF)
fpath(URDF "urdf/model.h" HINTS "${ROS_ROBOT_MODEL_ROOT}/urdf/include")
fpath(URDF "urdf_interface/link.h" HINTS "${ROS_ROBOT_MODEL_ROOT}/urdf_interface/include")
fpath(URDF "urdf_parser/urdf_parser.h" HINTS "${ROS_ROBOT_MODEL_ROOT}/urdf_parser/include")
fpath(URDF "collada_parser/collada_parser.h" HINTS "${ROS_ROBOT_MODEL_ROOT}/collada_parser/include")
flib(URDF "urdf" HINTS "${ROS_ROBOT_MODEL_ROOT}/urdf/lib")
flib(URDF "minizip" HINTS "${ROS_ROBOT_MODEL_ROOT}/colladadom/lib")
export_lib(URDF)
