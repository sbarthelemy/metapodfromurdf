## Copyright (c) 2012 Aldebaran Robotics. All rights reserved
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file
set(ROS_ROOT "/opt/ros/fuerte")
clean(ROSCPP)
fpath(ROSCPP "ros/console.h" HINTS "${ROS_ROOT}/include")
flib(ROSCPP "roscpp" HINTS "${ROS_ROOT}/lib")
flib(ROSCPP "rosconsole" HINTS "${ROS_ROOT}/lib")
export_lib(ROSCPP)
