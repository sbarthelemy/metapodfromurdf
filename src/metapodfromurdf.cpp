// Copyright (c) 2012 Aldebaran Robotics. All rights reserved
// Use of this source code is governed by a BSD-style license that can be
// found in the COPYING file
#include <string>
#include <vector>
#include <fstream>
#include <algorithm>
#include <cctype>
#include <Eigen/Dense>
#include <boost/tokenizer.hpp>
#include <ros/console.h>
#include <urdf/model.h>
#include <metapodfromurdf/metapodfromurdf.hpp>
#include <metapod/tools/spatial.hh>
#include <metapod/tools/buildrobot.hh>

using namespace std;

namespace metapodfromurdf {

// Utility functions

// construct vector
metapod::Vector3d toMetapod(urdf::Vector3 v)
{
  return metapod::Vector3d(v.x, v.y, v.z);
}

// construct pose
metapod::Spatial::Transform toSpatialTransform(urdf::Pose p)
{
  Eigen::Quaterniond q(p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z);
  metapod::Matrix3d R;
  R = q.cast<metapod::FloatType>();
  metapod::Vector3d r = toMetapod(p.position);
  return metapod::Spatial::Transform(R, -R.transpose()*r);
}

metapod::Spatial::Transform toSpatialTransform(Eigen::Quaterniond q)
{
  metapod::Matrix3d R;
  R = q;
  return metapod::Spatial::Transform(R, metapod::Vector3d::Zero());
}

LinkComparer::LinkComparer() {}

Status LinkComparer::init(const std::vector<std::string>& joint_names)
{
  // TODO: check joint_names has no dupe
  joint_ordering_ = joint_names;
  return STATUS_SUCCESS;
}

bool LinkComparer::operator()(
    boost::shared_ptr<urdf::Link> link0,
    boost::shared_ptr<urdf::Link> link1)
{
  const std::string& joint0 = link0->parent_joint->name;
  const std::string& joint1 = link1->parent_joint->name;
  std::vector<std::string>::const_iterator it0, it1;
  it0 = std::find(joint_ordering_.begin(), joint_ordering_.end(), joint0);
  it1 = std::find(joint_ordering_.begin(), joint_ordering_.end(), joint1);
  if (it0 == joint_ordering_.end())
  {
    if (it1 == joint_ordering_.end())
    {
      // none are in the list, use normal ordering
      return (joint0 < joint1);
    }
    else
    {
      // joint1 is in the list, it comes first
      return false;
    }
  }
  else
  {
    if (it1 == joint_ordering_.end())
    {
      // joint0 is in the list, it comes first
      return true;
    }
    else
    {
      // both are in the list, use list ordering
      return (it0 < it1);
    }
  }
}

RobotBuilder::RobotBuilder()
  : nb_dof_(0),
    nb_bodies_(0),
    depth_(0),
    is_initialized_(false)
{}

RobotBuilder::~RobotBuilder()
{
  body_hh_ << namespaces_closing_;
  closeInclusionGuard(body_hh_, "BODY_HH");

  init_cc_ << namespaces_closing_;

  joint_hh_ << namespaces_closing_;
  closeInclusionGuard(joint_hh_, "JOINT_HH");

  const std::string tab("  ");
  robot_hh_
      << "class METAPOD_DLLEXPORT Robot {\n" // TODO: handle the DLLEXPORT as an user-defined argument
      << "public:\n"
      << tab << "// Global constants or variable of the robot\n"
      << tab << "enum { NBDOF = " << nb_dof_ << " };\n"
      << tab << "enum { NBBODIES = " << nb_bodies_ << " };\n"
      << tab << "static Eigen::Matrix< FloatType, NBDOF, NBDOF > H;\n"
      << tab << "typedef Eigen::Matrix< FloatType, NBDOF, 1 > confVector;\n\n"
      << tab << "// Definition of the multibody tree as a type.\n"
      << tab << "typedef \n"
      << tree_.str() << " Tree;\n"
      << "};\n";
  robot_hh_ << namespaces_closing_;
  closeInclusionGuard(robot_hh_, "ROBOT_HH");
  ROS_INFO("FINISHING in directory '%s'", directory_.c_str());
}

Status RobotBuilder::set_directory(const std::string & directory)
{
  if (is_initialized_)
  {
    ROS_ERROR("one cannot call set_directory() after having called init()");
    return STATUS_FAILURE;
  }
  directory_ = directory;
  return STATUS_SUCCESS;
}

Status RobotBuilder::set_name(const std::string & name)
{
  if (is_initialized_)
  {
    ROS_ERROR("one cannot call set_name() after having called init()");
    return STATUS_FAILURE;
  }
  name_ = name;
  return STATUS_SUCCESS;
}

Status RobotBuilder::set_namespace(const std::string & combined_namespace)
{
  if (is_initialized_)
  {
    ROS_ERROR("one cannot call set_namespace() after having called init()");
    return STATUS_FAILURE;
  }
  namespace_ = combined_namespace;// TODO: we should check namespace is valid
  typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
  boost::char_separator<char> sep(":"); // TODO: we should split at ::, not :
  tokenizer tokens(combined_namespace, sep);
  namespaces_.clear();
  std::stringstream opening;
  std::stringstream closing;
  for (tokenizer::iterator tok_iter = tokens.begin();
       tok_iter != tokens.end();
       ++tok_iter)
  {
    namespaces_.push_back(*tok_iter);
    opening << "namespace " << *tok_iter << " {\n";
    closing << "} // closing namespace " << *tok_iter << "\n";
  }
  namespaces_opening_ = opening.str();
  namespaces_closing_ = closing.str();

  return STATUS_SUCCESS;
}

Status RobotBuilder::set_reinclusion_guard_prefix(const std::string& text)
{
  if (is_initialized_)
  {
    ROS_ERROR("one cannot call set_reinclusion_guard_prefix() after having called init()");
    return STATUS_FAILURE;
  }
  reinclusion_guard_prefix_ = text;
  return STATUS_SUCCESS;
}

Status RobotBuilder::set_joint_ordering(
   const std::vector<std::string>& joint_names)
{
  if (is_initialized_)
  {
    ROS_ERROR("one cannot call set_joint_ordering() after having called init()");
    return STATUS_FAILURE;
  }
  return link_comparer_.init(joint_names);
}

Status RobotBuilder::finalizeOptions()
{
  return STATUS_SUCCESS;
}

Status RobotBuilder::init()
{
  if (is_initialized_)
  {
    ROS_ERROR("one can only call init() once");
    return STATUS_FAILURE;
  }
  if (finalizeOptions())
  {
    return STATUS_FAILURE;
  }

  body_hh_.open(std::string(directory_ + "/body.hh").c_str(),
           std::ofstream::out);
  init_cc_.open(std::string(directory_ + "/" + name_ + ".cc").c_str(),
           std::ofstream::out);
  init_hh_.open(std::string(directory_ + "/" + name_ + ".hh").c_str(),
           std::ofstream::out);
  joint_hh_.open(std::string(directory_ + "/joint.hh").c_str(),
           std::ofstream::out);
  robot_hh_.open(std::string(directory_ + "/robot.hh").c_str(),
           std::ofstream::out);
  openInclusionGuard(body_hh_, "BODY_HH");
  body_hh_ << "# include \"metapod/tools/bodymacros.hh\"\n\n" // TODO: use "" or <> ?
           << namespaces_opening_;

  init_cc_
    << "# include \"" << name_ << ".hh\"\n\n"
    << namespaces_opening_;

  init_cc_
    << "\n"
    << "// Initialization of the robot global constants\n"
    << "Eigen::Matrix< FloatType, Robot::NBDOF, Robot::NBDOF > Robot::H;\n";

  openInclusionGuard(init_hh_, "INIT_HH");
  init_hh_
    << "# include \"robot.hh\"\n\n";
  closeInclusionGuard(init_hh_, "INIT_HH");


  openInclusionGuard(joint_hh_, "JOINT_HH");
  joint_hh_
    << "# include \"metapod/tools/jointmacros.hh\"\n"
    << namespaces_opening_;

  openInclusionGuard(robot_hh_, "ROBOT_HH");
  robot_hh_
    << "\n"
    << "# include \"metapod/tools/common.hh\"\n"
    << "# include \"joint.hh\"\n"
    << "# include \"body.hh\"\n\n"
    << namespaces_opening_;

  ROS_INFO("INIT in directory '%s'", directory_.c_str());
  is_initialized_ = true;
  return STATUS_SUCCESS;
}

void RobotBuilder::openInclusionGuard(
    std::ostream& stream,
    const char* name)
{
  stream << "#ifndef " << reinclusion_guard_prefix_ << name << "\n"
         << "# define " << reinclusion_guard_prefix_ << name << "\n\n";
}

void RobotBuilder::closeInclusionGuard(
  std::ostream& stream,
  const char* name)
{
  stream << "#endif // " << reinclusion_guard_prefix_ << name << "\n";
}


// recursive function to walk through the tree
Status RobotBuilder::addSubTree(
    boost::shared_ptr<const urdf::Link> root,
    const std::string& parent_body_name,
    bool has_parent)
{
  const std::string tab("\t");

  if (not is_initialized_)
  {
    ROS_ERROR("one must call init() once before calling addSubTree()");
    return STATUS_FAILURE;
  }
  std::vector<boost::shared_ptr<urdf::Link> > children = root->child_links;
  if (children.size() > 5)
  {
    ROS_ERROR(
        "metapod supports at most 5 children per node, but link %s has %i "
        "children.",
        root->name.c_str(),
        static_cast<int>(children.size()));
    return STATUS_FAILURE;
  }


  // add root itself
  // convert the joint
  boost::shared_ptr<urdf::Joint> jnt = root->parent_joint;
  unsigned int metapod_joint_type;
  unsigned int joint_nb_dof;
  switch (jnt->type) {
  case urdf::Joint::REVOLUTE:
  case urdf::Joint::CONTINUOUS:
  {
    ROS_INFO("Adding joint '%s' as a REVOLUTE_AXIS_ANY joint", jnt->name.c_str());
    metapod_joint_type = metapod::REVOLUTE_AXIS_ANY;
    joint_nb_dof = 1;
    break;
  }
  case urdf::Joint::FLOATING:
  {
    metapod_joint_type = metapod::FREE_FLYER;
    joint_nb_dof = 6;
    ROS_INFO("Adding joint '%s' as a FREE_FLYER joint", jnt->name.c_str());
    break;
  }
  default:
  {
    ROS_ERROR("Joint '%s' is of unknown type", jnt->name.c_str());
    return STATUS_FAILURE;
    break;
  }
  }
  metapod::Spatial::Transform static_transform =
      toSpatialTransform(jnt->parent_to_joint_origin_transform).inverse();
  metapod::Vector3d axis = toMetapod(jnt->axis);

  // convert the link (a.k.a. body)
  // TODO: check root->name is a valid class name

  // constructs the optional inertia
  metapod::Vector3d center_of_mass = metapod::Vector3d::Zero();
  metapod::Matrix3d rotational_inertia = metapod::Matrix3d::Identity();
  double mass = 1.;
  if (root->inertial)
  {
    mass = root->inertial->mass; // TODO: check mass >0
    urdf::Pose& p = root->inertial->origin;
    center_of_mass = toMetapod(p.position);
    // metapod expects the rotational inertia in a frame aligned with the frame
    // of the link, but with origin at center of mass
    // urdf specifies it in a frame whose origin is the link center of mass,
    // and whose basis is arbitrary
    // So, let's rotate it! (Yeah!)
    Eigen::Quaterniond q(p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z);
    metapod::Matrix3d R;
    R = q.cast<metapod::FloatType>();
    Eigen::Matrix3d tmp;
    tmp << root->inertial->ixx, root->inertial->ixy, root->inertial->ixz,
           root->inertial->ixy, root->inertial->iyy, root->inertial->iyz,
           root->inertial->ixz, root->inertial->iyz, root->inertial->izz;
    rotational_inertia = R * tmp;
  }

  int joint_label = nb_bodies_;
  int body_label = nb_bodies_; // TODO: understand why body labels start at 0 in simple-humanoid
  int joint_position_in_conf = nb_dof_;
  metapod::createJoint(
      joint_hh_,
      init_cc_,
      metapod_joint_type,
      jnt->name, // TODO: check joint->name is a valid class name
      joint_label,
      joint_position_in_conf,
      static_transform.E(),
      static_transform.r(),
      tab,
      axis[0],
      axis[1],
      axis[2]);
  metapod::createBody(
      body_hh_,
      init_cc_,
      root->name,
      has_parent ? parent_body_name : std::string("NP"),
      jnt->name,
      body_label,
      mass,
      center_of_mass,
      rotational_inertia,
      tab);
  ++nb_bodies_;
  nb_dof_ += joint_nb_dof;
  const unsigned int tab_size = std::string("Node< ").length(); // TODO: account for initial indent
  const std::string indent(tab_size * depth_, ' ');
  // start a new Node
  tree_ << indent << "Node< " << root->name << ",\n";
  // update the indentation accordingly
  ++depth_;
  // continue the Node with the joint name
  tree_ << indent << std::string(tab_size, ' ') << jnt->name;
  // ...and the children (sort them first)
  std::sort(children.begin(), children.end(), link_comparer_);
  for (size_t i=0; i<children.size(); ++i)
  {
    const bool has_parent = true;
    tree_ << ",\n";
    if (addSubTree(children[i], root->name, has_parent) == STATUS_FAILURE)
      return STATUS_FAILURE;
  }
  // restore indent
  --depth_;
  tree_ << "\n" << indent << "    >";
  return STATUS_SUCCESS;
}

bool treeFromUrdfModel(const urdf::ModelInterface& robot_model, RobotBuilder& mmodel)
{
  //  add all children
  bool is_success = false;
  const bool has_parent = false;
  for (size_t i=0; i<robot_model.getRoot()->child_links.size(); ++i)
  {
    // TODO: what happens when there are two robots?
    Status status = mmodel.addSubTree(
        robot_model.getRoot()->child_links[i],
        std::string("GROUND"),
        has_parent);
    if (status == STATUS_FAILURE)
      return is_success;
  }
  is_success = true;
  return is_success;
}

bool treeFromParam(const string& param, RobotBuilder& world)
{
  urdf::Model robot_model;
  if (!robot_model.initParam(param)){
    ROS_ERROR("Could not generate robot model");
    return false;
  }
  return treeFromUrdfModel(robot_model, world);
}

bool treeFromXml(TiXmlDocument *xml_doc, RobotBuilder& world)
{
  urdf::Model robot_model;
  if (!robot_model.initXml(xml_doc)){
    ROS_ERROR("Could not generate robot model");
    return false;
  }
  return treeFromUrdfModel(robot_model, world);
}

bool treeFromFile(const string& file, RobotBuilder& world)
{
  TiXmlDocument urdf_xml;
  urdf_xml.LoadFile(file);
  return treeFromXml(&urdf_xml, world);
}

bool treeFromString(const string& xml, RobotBuilder& world)
{
  TiXmlDocument urdf_xml;
  urdf_xml.Parse(xml.c_str());
  return treeFromXml(&urdf_xml, world);
}
}

