metapodfromurdf
---------------

CLI tool to generate the C++ source code from a robot model from its
description in the URDF format.

The tools has glitches (grep for TODO) but might prove useful though.

Run ``metapodfromurdf --help`` for some scarse help.

Be aware that

* metapodfromurdf does *not* add a floating (aka free) joint at the root of
  the robot tree. If you want such a joint, add it to your URDF file.

* the URDF parser mixes up the joint ordering found in the URDF file.
  Joint ordering in metapod is important since it defines the meaning of
  the lines and columns in the linearized model. By default, joints are
  ordered by walking the tree by depth first, with siblings sorted
  alphabetically. metapodfromurdf lets you re-order siblings with the
  --joint argument.

Dependencies
------------

* metapod_robotbuilder
* eigen3
* urdf and tinyxml
* roscpp
* boost program_options

URDF disambiguation
-------------------

URDF denotes several distinct but related things:

* an XML-based file format to describe robots
* a C++ API defined in urdf_interface headers
* a library which parses XML files (both URDF-XML and COLLADA-XML) and
  returns C++ objects implementing the urdf C++ API

So, liburdf loads xml files and returns C++ urdf::Model objects. It does so by
calling either urdf_parser or collada_parser according to the file type.

Headers from all these libs are needed.

Modules in cmake/ find them all, looking for them at the standard non-standard
location (the one used by the official ROS ubuntu packages).

How to test
-----------

Metapod comes with two exaple models: simple_arm and simple_humanoid.

One can use the data/simple_arm.urdf example to generate an equivalent version
of this metapod model, and compare it to the original::

  metapodfromurdf
    --config-file=data/simple_arm.config \
    data/simple_arm.urdf
  metapodfromurdf
    --config-file=data/simple_humanoid.config \
    data/simple_humanoid.urdf

By default, the models are writen to /tmp/simple-{arm,humanoid}.

You might also want to write them directly in your metapod source tree,
and build it to ensure it passes the automated tests. Just provide the
proper --directory option. For instance::

  --directory ${METAPOD_SRC}/include/metapod/models/simple-humanoid

Todo/wishlist
-------------

- add a little test (sigh)

- deal with the floating joint, which we require but is deprecated in URDF

- filter models from a python script

- move back to rosbuild?

- reduce dependencies (especially ROS-specific ones)

- build a statically linked binary
