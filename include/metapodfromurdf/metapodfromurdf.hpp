// Copyright (c) 2012 Aldebaran Robotics. All rights reserved
// Use of this source code is governed by a BSD-style license that can be
// found in the COPYING file

#ifndef METAPODFROMURDF_METAPODFROMURDF_H
#define METAPODFROMURDF_METAPODFROMURDF_H

#include <string>
#include <urdf_interface/model.h>
#include <fstream>
#include <sstream>

namespace metapodfromurdf {

enum Status
{
    STATUS_SUCCESS = 0,
    STATUS_FAILURE = 1
};

// for use with std::sort.
class LinkComparer
{
public:
  LinkComparer();
  Status init(const std::vector<std::string>& joint_names);
  // Returns true when link0 sorts before link1
  bool operator()(
      boost::shared_ptr<urdf::Link> link0,
      boost::shared_ptr<urdf::Link> link1);
private:
  std::vector<std::string> joint_ordering_;
};

class RobotBuilder
{
public:
  RobotBuilder(const char*);
  RobotBuilder();
  ~RobotBuilder();
  Status set_name(const std::string& name);
  Status set_directory(const std::string& directory);
  Status set_namespace(const std::string& combined_namespace);
  Status set_reinclusion_guard_prefix(const std::string& text);
  Status set_joint_ordering(const std::vector<std::string>& joint_names);
  Status init();
  Status addSubTree(
    boost::shared_ptr<const urdf::Link> root,
    const std::string& parent_body_name,
    bool has_parent);
private:
  Status finalizeOptions();
  void openInclusionGuard(std::ostream& stream, const char* name);
  void closeInclusionGuard(std::ostream& stream, const char* name);
  RobotBuilder(const RobotBuilder&); // forbid copy-constuction
  unsigned int nb_dof_;
  unsigned int nb_bodies_;
  unsigned int depth_;
  bool is_initialized_;
  LinkComparer link_comparer_;
  std::string name_;
  std::string directory_;
  std::string reinclusion_guard_prefix_;
  std::string namespace_;
  std::vector<std::string> namespaces_;
  std::string namespaces_opening_;
  std::string namespaces_closing_;
  std::ofstream body_hh_;
  std::ofstream init_cc_;
  std::ofstream init_hh_;
  std::ofstream joint_hh_;
  std::ofstream robot_hh_;
  std::ostringstream tree_;
};

/** Generate a metapod model source code from a file, given the file name
 * \param file The filename from where to read the xml
 * \param builder The generator class
 * returns true on success, false on failure
 */
bool treeFromFile(const std::string& file, RobotBuilder& builder);

/** Generate a metapod model source code from the parameter server,
 * given the parameter name
 * \param param the name of the parameter on the parameter server
 * \param builder The generator class
 * returns true on success, false on failure
 */
bool treeFromParam(const std::string& param, RobotBuilder& builder);

/** Generate a metapod model source code from a string containing xml
 * \param xml A string containting the xml description of the robot
 * \param builder The generator class
 * returns true on success, false on failure
 */
bool treeFromString(const std::string& xml, RobotBuilder& builder);

/** Generate a metapod model source code from a TiXmlDocument
 * \param xml_doc The TiXmlDocument containting the xml description of the robot
 * \param builder The generator class
 * returns true on success, false on failure
 */
bool treeFromXml(TiXmlDocument *xml_doc, RobotBuilder& builder);

/** Generate a metapod model source code from a URDF robot model
 * \param robot_model The URDF robot model
 * \param builder The generator class
 * returns true on success, false on failure
 */
bool treeFromUrdfModel(const urdf::ModelInterface& robot_model, RobotBuilder& builder);
}

#endif
