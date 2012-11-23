// Copyright (c) 2012 Aldebaran Robotics. All rights reserved
// Use of this source code is governed by a BSD-style license that can be
// found in the COPYING file
#include <iostream>
#include <urdf/model.h>
#include <metapodfromurdf/metapodfromurdf.hpp>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

int main(int argc, char** argv)
{
  // Declare a group of options that will be
  // allowed only on command line
  po::options_description generic("");
  generic.add_options()
    ("help", "produce help message")
    ;

  // Declare a group of options that will be
  // allowed both on command line and in
  // config file
  po::options_description config("");
  config.add_options()
      ("name", po::value<std::string>(),
       "the robot name")
      ("directory", po::value<std::string>(),
       "directory where the files will be generated")
      ("namespace", po::value<std::string>(),
       "namespace the generated code will lie in (possibly composed)")
      ("inclusion-guard-prefix", po::value<std::string>(),
       "prefix for the reinclusion guards. Usually ends with '_'")
      ("joint", po::value<std::vector<std::string> >(),
       "joint name, pass several of them to specify joints ordering");

  // Hidden options, will be allowed both on command line and
  // in config file, but will not be shown to the user.
  po::options_description hidden("Hidden options");
  hidden.add_options()
      ("input-file", po::value<std::string>()->required(),
       "input file in urdf format");

  po::options_description cmdline_options;
  cmdline_options.add(generic).add(config).add(hidden);

  po::positional_options_description pos;
  pos.add("input-file", -1);
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).
                  options(cmdline_options).positional(pos).run(), vm);
  po::options_description visible("Usage:\n metapodfromurdf-bin [options] input-file\n\nOptions");
  visible.add(generic).add(config);
  if (vm.count("help"))
  {
    std::cout << visible << "\n";
    return 0;
  }
  // we do not parse config files for now
  //po::options_description config_file_options;
  //config_file_options.add(config).add(hidden);
  try {
    po::notify(vm);
  }
  catch(boost::program_options::required_option)
  {
    std::cout << visible << "\n";
    return 0;
  }
  urdf::Model robot_model;
  if (!robot_model.initFile(vm["input-file"].as<std::string>()))
  {
    std::cerr << "Could not generate robot model" << std::endl;
    return 1;
  }
  metapodfromurdf::RobotBuilder mmodel;
  if (vm.count("name"))
  {
    mmodel.set_name(vm["name"].as<std::string>());
  }
  if (vm.count("namespace"))
  {
    mmodel.set_namespace(vm["namespace"].as<std::string>());
  }
  if (vm.count("directory"))
  {
    mmodel.set_directory(vm["directory"].as<std::string>());
  }
  if (vm.count("inclusion-guard-prefix"))
  {
    mmodel.set_reinclusion_guard_prefix(vm["inclusion-guard-prefix"].as<std::string>());
  }
  if (vm.count("joint"))
  {
    mmodel.set_joint_ordering(vm["joint"].as<std::vector<std::string> >());
  }
  if (mmodel.init())
  {
    return 1;
  }

  metapodfromurdf::treeFromUrdfModel(robot_model, mmodel);
  return 0;
}
