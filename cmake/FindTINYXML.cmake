## Copyright (c) 2012 Aldebaran Robotics. All rights reserved
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file

clean(TINYXML)
fpath(TINYXML "tinyxml.h")
flib(TINYXML "libtinyxml.so")
flib(TINYXML "libtinyxml.a")
export_lib(TINYXML)
