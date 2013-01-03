// Copyright (c) 2012 Aldebaran Robotics. All rights reserved
// Use of this source code is governed by a BSD-style license that can be
// found in the COPYING file

#ifndef METAPODFROMURDF_METAPODFROMURDF_H
#define METAPODFROMURDF_METAPODFROMURDF_H


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

class RobotBuilder : public metapod::RobotBuilder
{
public:
  RobotBuilder(const char*);
  RobotBuilder();
  ~RobotBuilder();
  Status set_joint_ordering(const std::vector<std::string>& joint_names);
  Status init();
  Status addSubTree(
    boost::shared_ptr<const urdf::Link> root,
    const std::string& parent_body_name,
    bool has_parent);
private:
  RobotBuilder(const RobotBuilder&); // forbid copy-constuction
  bool is_initialized_;
  LinkComparer link_comparer_;
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
