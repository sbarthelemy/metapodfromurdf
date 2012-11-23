metapodfromurdf
---------------

Library and CLI tool to generate the C++ source code from a robot model
from its description in the URDF format.

The tools has glitches (grep for TODO) but might prove useful though.

The library API mimicks the one from kdl_parser, in the robot_model ROS stack.

Run ``metapodfromurdf-bin --help`` for some scarse help.

Be aware that

* metapodfromurdf does *not* add a floating (aka free) joint at the root of
  the robot tree. If you want such a joint, add it to your URDF file.

* the URDF parser mixes up the joint ordering found in the URDF file.
  Joint ordering in metapod is important since it defines the meaning of
  the lines and columins in the linearized model. By default, joints are
  ordered by walking the tree by depth first, with siblings sorted
  alphabetically. metapodfromurdf-bin lets you re-order siblings with the
  --joint argument.

Dependencies
------------

* metapod (for some utilities)
* eigen3
* urdf and tinyxml
* roscpp
* boost program options

URDF disambiguation
-------------------

URDF denotes several distinct but related things:

* an XML-based file format to describe robots
* a C++ API defined in urdf_interface headers
* a library which parses XML files (both URDF-XML and COLLADA-XML) and
  returns C++ objects implementing the urdf C++ API

liburdf loads xml files and returns C++ urdf::Model objects. It does so by
calling either urdf_parser or collada_parser according to the file type.

Headers from all these libs are needed.

Module in cmake/ find them all, looking for them at the standard non-standard
location (the one used by the official ROS ubuntu packages).

Todo/wishlist
-------------

- add an option to choose between REVOLUTE_AXIS_{X,ANY}

- add a little test (sigh)

- deal with the floating joint, which we require but is deprecated in URDF

- instead of generating files directly, let the user pass ostreams and fill
  them

- filter models from a python script

- move back to rosbuild?

- depend on less ROS-related stuff?

- move a subset of the tool directly in metapod?

- cleanup the API

- build a statically linked binary
