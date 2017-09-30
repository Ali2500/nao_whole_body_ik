/*
*  nao_whole_body_kinematics.cpp
*
*  Created on: 8 Apr 2016
*      Author: Ali Athar
*/

#include <nao_whole_body_ik/nao_whole_body_kinematics.h>

namespace nao_whole_body_ik
{

NaoWholeBodyKinematics::NaoWholeBodyKinematics(const robot_state::RobotStatePtr& robot_state, ARMS_INCLUSION arm_inclusion) :
  arm_inclusion_(arm_inclusion)
{
  robot_state_ = robot_state;

  //from r_sole up to torso:
  KDL::Chain right_leg_chain;
  right_leg_chain.addSegment(KDL::Segment("r_sole", KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Vector(0.0, 0.0, 0.04519))));
  right_leg_chain.addSegment(KDL::Segment("RAnkleRoll_link", KDL::Joint(KDL::Joint::RotX), KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));
  right_leg_chain.addSegment(KDL::Segment("RAnklePitch_link", KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0.0, 0.0, 0.1029))));
  right_leg_chain.addSegment(KDL::Segment("RKneePitch_link", KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0.0, 0.0, 0.1))));
  right_leg_chain.addSegment(KDL::Segment("RHipPitch_link", KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));
  right_leg_chain.addSegment(KDL::Segment("RHipRoll_link", KDL::Joint(KDL::Joint::RotX), KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));
  right_leg_chain.addSegment(KDL::Segment("RHipYawPitch_link", KDL::Joint(KDL::Vector(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.7071, 0.7071), KDL::Joint::RotAxis), KDL::Frame(KDL::Vector(0.0, 0.05, 0.085))));
  right_leg_chain.addSegment(KDL::Segment("torso", KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));

  //from torso to l_sole:
  KDL::Chain left_leg_chain;
  left_leg_chain.addSegment(KDL::Segment("torso_to_LHip_link", KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Vector(0.0, 0.05, -0.085))));
  left_leg_chain.addSegment(KDL::Segment("LHipYawPitch_link", KDL::Joint(KDL::Vector(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.7071, -0.7071), KDL::Joint::RotAxis), KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));
  left_leg_chain.addSegment(KDL::Segment("LHipRoll_link", KDL::Joint(KDL::Joint::RotX), KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));
  left_leg_chain.addSegment(KDL::Segment("LHipPitch_link", KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0.0, 0.0, -0.1))));
  left_leg_chain.addSegment(KDL::Segment("LKneePitch_link", KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0.0, 0.0, -0.1029))));
  left_leg_chain.addSegment(KDL::Segment("LAnklePitch_link", KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));
  left_leg_chain.addSegment(KDL::Segment("LAnkleRoll_link", KDL::Joint(KDL::Joint::RotX), KDL::Frame(KDL::Vector(0.0, 0.0, -0.04519))));
  left_leg_chain.addSegment(KDL::Segment("l_sole", KDL::Joint(KDL::Joint::None),KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));

  //from torso to r_gripper:
  KDL::Chain right_arm_chain;
  right_arm_chain.addSegment(KDL::Segment("torso_to_RShoulder_link", KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Vector(0.0, -0.098, 0.1))));
  right_arm_chain.addSegment(KDL::Segment("RShoulderPitch_link", KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));
  right_arm_chain.addSegment(KDL::Segment("RShoulderRoll_link", KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(0.105, -0.015, 0.0))));
  right_arm_chain.addSegment(KDL::Segment("RElbowYaw_link", KDL::Joint(KDL::Joint::RotX), KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));
  right_arm_chain.addSegment(KDL::Segment("RElbowRoll_link", KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(0.05595, 0.0, 0.0))));
  right_arm_chain.addSegment(KDL::Segment("RWristYaw_link", KDL::Joint(KDL::Joint::RotX), KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));
  right_arm_chain.addSegment(KDL::Segment("r_gripper", KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Vector(0.05775, 0.0, -0.01231))));

  //from torso to l_gripper:
  KDL::Chain left_arm_chain;
  left_arm_chain.addSegment(KDL::Segment("torso_to_LShoulder_link", KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Vector(0.0, 0.098, 0.1))));
  left_arm_chain.addSegment(KDL::Segment("LShoulderPitch_link", KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));
  left_arm_chain.addSegment(KDL::Segment("LShoulderRoll_link", KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(0.105, 0.015, 0.0))));
  left_arm_chain.addSegment(KDL::Segment("LElbowYaw_link", KDL::Joint(KDL::Joint::RotX), KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));
  left_arm_chain.addSegment(KDL::Segment("LElbowRoll_link", KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(0.05595, 0.0, 0.0))));
  left_arm_chain.addSegment(KDL::Segment("LWristYaw_link", KDL::Joint(KDL::Joint::RotX), KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));
  left_arm_chain.addSegment(KDL::Segment("l_gripper", KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Vector(0.05775, 0.0, -0.01231))));

  nao_kinematic_tree_.addChain(right_leg_chain, "root");
  nao_kinematic_tree_.addChain(left_leg_chain, "torso");
  end_effector_names_.push_back("l_sole");

  if (arm_inclusion_ == RIGHT_ARM_ONLY || arm_inclusion_ == BOTH_ARMS)
  {
    nao_kinematic_tree_.addChain(right_arm_chain, "torso");
    end_effector_names_.push_back("r_gripper");
  }
  if (arm_inclusion_ == LEFT_ARM_ONLY || arm_inclusion_ == BOTH_ARMS)
  {
    nao_kinematic_tree_.addChain(left_arm_chain, "torso");
    end_effector_names_.push_back("l_gripper");
  }

  for(std::size_t i = 0; i < end_effector_names_.size(); i++)
  {
    tree_ik_vars_.frames.insert(KDL::Frames::value_type(end_effector_names_[i], KDL::Frame::Identity()));
    tree_ik_vars_.delta_twists.insert(KDL::Twists::value_type(end_effector_names_[i], KDL::Twist::Zero()));
  }
  tree_ik_vars_.maxiter = 150;
  tree_ik_vars_.eps = 0.005;

  const robot_state::JointModelGroup* right_leg = robot_state_->getJointModelGroup("right_leg_6dof");
  const moveit::core::JointBoundsVector& r_leg_lims = right_leg->getActiveJointModelsBounds();

  const robot_state::JointModelGroup* left_leg = robot_state_->getJointModelGroup("left_leg_6dof");
  const moveit::core::JointBoundsVector& l_leg_lims = left_leg->getActiveJointModelsBounds();

  const robot_state::JointModelGroup* right_arm = robot_state_->getJointModelGroup("right_arm");
  const moveit::core::JointBoundsVector& r_arm_lims = right_arm->getActiveJointModelsBounds();

  const robot_state::JointModelGroup* left_arm = robot_state_->getJointModelGroup("left_arm");
  const moveit::core::JointBoundsVector& l_arm_lims = left_arm->getActiveJointModelsBounds();

  int loop_limit;
  if (arm_inclusion_ == RIGHT_ARM_ONLY || arm_inclusion_ == LEFT_ARM_ONLY)
  {
    min_joint_limits_.resize(17);
    max_joint_limits_.resize(17);
    tree_ik_vars_.delta_q.resize(17);
  }
  else
  {
    min_joint_limits_.resize(22);
    max_joint_limits_.resize(22);
    tree_ik_vars_.delta_q.resize(22);
  }

  for(int i = 0; i < 6; i++)
  {
    min_joint_limits_(i) = r_leg_lims[i]->at(0).min_position_;
    max_joint_limits_(i) = r_leg_lims[i]->at(0).max_position_;

    min_joint_limits_(i+6) = l_leg_lims[i]->at(0).min_position_;
    max_joint_limits_(i+6) = l_leg_lims[i]->at(0).max_position_;

    if (arm_inclusion_ == RIGHT_ARM_ONLY && i < 5)
    {
      min_joint_limits_(i+12) = r_arm_lims[i]->at(0).min_position_;
      max_joint_limits_(i+12) = r_arm_lims[i]->at(0).max_position_;
    }
    else if (arm_inclusion_ == LEFT_ARM_ONLY && i < 5)
    {
      min_joint_limits_(i+12) = l_arm_lims[i]->at(0).min_position_;
      max_joint_limits_(i+12) = l_arm_lims[i]->at(0).max_position_;
    }
    else if (arm_inclusion_ == BOTH_ARMS && i < 5)
    {
      min_joint_limits_(i+12) = r_arm_lims[i]->at(0).min_position_;
      max_joint_limits_(i+12) = r_arm_lims[i]->at(0).max_position_;

      min_joint_limits_(i+17) = l_arm_lims[i]->at(0).min_position_;
      max_joint_limits_(i+17) = l_arm_lims[i]->at(0).max_position_;
    }
  }

  nao_tree_fk_.reset(new KDL::TreeFkSolverPos_recursive(nao_kinematic_tree_));
  nao_tree_vik_.reset(new KDL::TreeIkSolverVel_wdls(nao_kinematic_tree_, end_effector_names_));
  nao_tree_vik_->setLambda(0.01);

}

NaoWholeBodyKinematics::~NaoWholeBodyKinematics()
{

}

void NaoWholeBodyKinematics::setRightArmTSWeight(const std::vector<double>& ts_weight)
{
  assert(ts_weight.size() == 6);

  if (arm_inclusion_ == LEFT_ARM_ONLY)
  {
    ROS_ERROR_STREAM("Right arm was not included in kinematic structure. Cannot set its TS weight");
    throw std::runtime_error("Right arm was not included in kinematic structure. Cannot set its TS weight");
  }
  else if (arm_inclusion_ == RIGHT_ARM_ONLY)
  {
    Eigen::MatrixXd ts_weight_matrix = Eigen::MatrixXd::Identity(12, 12);
    const Eigen::MatrixXd& existing_ts_weight = nao_tree_vik_->getWeightTS();

    for(int i = 0; i < 6; i++)
    {
      ts_weight_matrix(i, i) = existing_ts_weight(i, i);
      ts_weight_matrix(i+6, i+6) = ts_weight[i];
    }
    nao_tree_vik_->setWeightTS(ts_weight_matrix);
  }
  else if (arm_inclusion_ == BOTH_ARMS)
  {
    Eigen::MatrixXd ts_weight_matrix = Eigen::MatrixXd::Identity(18, 18);
    const Eigen::MatrixXd& existing_ts_weight = nao_tree_vik_->getWeightTS();

    for(int i = 0; i < 6; i++)
    {
      ts_weight_matrix(i, i) = existing_ts_weight(i, i);
      ts_weight_matrix(i+6, i+6) = existing_ts_weight(i+6, i+6);
      ts_weight_matrix(i+12, i+12) = ts_weight[i];
    }
    nao_tree_vik_->setWeightTS(ts_weight_matrix);
  }

  // const Eigen::MatrixXd test_weight = nao_tree_vik_->getWeightTS();
  // std::cout << "TS weight: ";
  // for(int i = 0; i < 12; i++)
  // {
  // 	std::cout << test_weight(i,i) << ", ";
  // }
  // std::cout << std::endl;
}

void NaoWholeBodyKinematics::setLeftArmTSWeight(const std::vector<double>& ts_weight)
{
  assert(ts_weight.size() == 6);

  if (arm_inclusion_ == RIGHT_ARM_ONLY)
  {
    ROS_ERROR_STREAM("Left arm was not included in kinematic structure. Cannot set its TS weight");
    throw std::runtime_error("Left arm was not included in kinematic structure. Cannot set its TS weight");
  }
  else if (arm_inclusion_ == LEFT_ARM_ONLY)
  {
    Eigen::MatrixXd ts_weight_matrix = Eigen::MatrixXd::Identity(12, 12);
    const Eigen::MatrixXd& existing_ts_weight = nao_tree_vik_->getWeightTS();

    for(int i = 0; i < 6; i++)
    {
      ts_weight_matrix(i, i) = ts_weight[i];
      ts_weight_matrix(i+6, i+6) = existing_ts_weight(i+6, i+6);
    }
    nao_tree_vik_->setWeightTS(ts_weight_matrix);
  }
  else if (arm_inclusion_ == BOTH_ARMS)
  {
    Eigen::MatrixXd ts_weight_matrix = Eigen::MatrixXd::Identity(18, 18);
    const Eigen::MatrixXd& existing_ts_weight = nao_tree_vik_->getWeightTS();

    for(int i = 0; i < 6; i++)
    {
      ts_weight_matrix(i, i) = ts_weight[i];
      ts_weight_matrix(i+6, i+6) = existing_ts_weight(i+6, i+6);
      ts_weight_matrix(i+12, i+12) = existing_ts_weight(i+12, i+12);
    }
    nao_tree_vik_->setWeightTS(ts_weight_matrix);
  }
}

void NaoWholeBodyKinematics::setLeftFootTSWeight(const std::vector<double>& ts_weight)
{
  assert(ts_weight.size() == 6);

  if (arm_inclusion_ == RIGHT_ARM_ONLY)
  {
    Eigen::MatrixXd ts_weight_matrix = Eigen::MatrixXd::Identity(12, 12);
    const Eigen::MatrixXd& existing_ts_weight = nao_tree_vik_->getWeightTS();

    for(int i = 0; i < 6; i++)
    {
      ts_weight_matrix(i, i) = ts_weight[i];
      ts_weight_matrix(i+6, i+6) = existing_ts_weight(i+6, i+6);
    }
    nao_tree_vik_->setWeightTS(ts_weight_matrix);
  }
  else if (arm_inclusion_ == LEFT_ARM_ONLY)
  {
    Eigen::MatrixXd ts_weight_matrix = Eigen::MatrixXd::Identity(12, 12);
    const Eigen::MatrixXd& existing_ts_weight = nao_tree_vik_->getWeightTS();

    for(int i = 0; i < 6; i++)
    {
      ts_weight_matrix(i, i) = existing_ts_weight(i, i);
      ts_weight_matrix(i+6, i+6) = ts_weight[i];
    }
    nao_tree_vik_->setWeightTS(ts_weight_matrix);
  }
  else if (arm_inclusion_ == BOTH_ARMS)
  {
    Eigen::MatrixXd ts_weight_matrix = Eigen::MatrixXd::Identity(18, 18);
    const Eigen::MatrixXd& existing_ts_weight = nao_tree_vik_->getWeightTS();

    for(int i = 0; i < 6; i++)
    {
      ts_weight_matrix(i, i) = existing_ts_weight(i, i);
      ts_weight_matrix(i+6, i+6) = ts_weight[i];
      ts_weight_matrix(i+12, i+12) = existing_ts_weight(i+12, i+12);
    }
    nao_tree_vik_->setWeightTS(ts_weight_matrix);
  }

  const Eigen::MatrixXd test_weight = nao_tree_vik_->getWeightTS();
  std::cout << "TS weight: ";
  for(int i = 0; i < 12; i++)
  {
    std::cout << test_weight(i,i) << ", ";
  }
  std::cout << std::endl;
}

void NaoWholeBodyKinematics::getWholeBodyFKSolution(const std::vector<double>& joint_values,
                                                    const std::string& eef_name, KDL::Frame& f_out)
{
  assert(eef_name == "l_sole" || eef_name == "l_gripper" || eef_name == "r_gripper");
  if (arm_inclusion_ == RIGHT_ARM_ONLY || arm_inclusion_ == LEFT_ARM_ONLY)
  {
    assert((int)joint_values.size() == 17);
  }
  else
  {
    assert((int)joint_values.size() == 22);
  }

  KDL::JntArray q_in((int)joint_values.size());
  for(int i = 0; i < (int)joint_values.size(); i++)
  {
    q_in(i) = joint_values[i];
  }
  nao_tree_fk_->JntToCart(q_in, f_out, eef_name);
}

bool NaoWholeBodyKinematics::getWholeBodyIKSolution(const std::map<std::string, Eigen::Affine3d>& target_pose_map,
                                                    const std::vector<double>& seed_state, std::vector<double>& joint_values)
{
  if (arm_inclusion_ == RIGHT_ARM_ONLY || arm_inclusion_ == LEFT_ARM_ONLY)
  {
    assert((int)seed_state.size() == 17);
  }
  else
  {
    assert((int)seed_state.size() == 22);
  }

  KDL::Frames p_in;
  for(std::map<std::string, Eigen::Affine3d>::const_iterator iter = target_pose_map.begin(); iter != target_pose_map.end(); iter++)
  {
    KDL::Frame f_in;
    CommonFunctions::EigenAffineToKDLFrame(iter->second, f_in);
    p_in.insert(KDL::Frames::value_type(iter->first, f_in));
  }

  KDL::JntArray q_init, ik_solution;
  CommonFunctions::STLVectorToKDLJntArray(seed_state, q_init);

  double ik_success = calculateWholeBodyIKSolutionIteratively(q_init, p_in, ik_solution);
  if (ik_success >= 0)
  {
    CommonFunctions::KDLJntArrayToSTLVector(ik_solution, joint_values);
    return true;
  }
  else
  {
    return false;
  }
}

double NaoWholeBodyKinematics::calculateWholeBodyIKSolutionIteratively(const KDL::JntArray& q_init, const KDL::Frames& p_in,
                                                                       KDL::JntArray& q_out)
{
  q_out = q_init;

  //First check if all elements in p_in are available:
  for(KDL::Frames::const_iterator f_des_it = p_in.begin(); f_des_it != p_in.end(); ++f_des_it)
  {
    //std::cout << f_des_it->first << ", ";
    if (tree_ik_vars_.frames.find(f_des_it->first) == tree_ik_vars_.frames.end())
    {
      return -2;
    }
  }
  //std::cout << std::endl;

  unsigned int k = 0;
  while (++k <= tree_ik_vars_.maxiter)
  {
    for (KDL::Frames::const_iterator f_des_it = p_in.begin(); f_des_it != p_in.end(); ++f_des_it)
    {
      //Get all iterators for this endpoint
      KDL::Frames::iterator f_it = tree_ik_vars_.frames.find(f_des_it->first);
      KDL::Twists::iterator delta_twist = tree_ik_vars_.delta_twists.find(f_des_it->first);

      nao_tree_fk_->JntToCart(q_out, f_it->second, f_it->first);
      delta_twist->second = diff(f_it->second, f_des_it->second);

    }

    double res = nao_tree_vik_->CartToJnt(q_out, tree_ik_vars_.delta_twists, tree_ik_vars_.delta_q);

    if (res < tree_ik_vars_.eps)
    {
      return res;
    }

    Add(q_out, tree_ik_vars_.delta_q, q_out);

    for (unsigned int j = 0; j < q_out.rows(); j++)
    {
      if (q_out(j) < min_joint_limits_(j))
      {
        q_out(j) = min_joint_limits_(j);
      }

      else if (q_out(j) > max_joint_limits_(j))
      {
        q_out(j) = max_joint_limits_(j);
      }
    }

    //enforce equal mimic joint values:
    q_out(5) = -1.0 * q_out(6);
  }

  if (k <= tree_ik_vars_.maxiter)
  {
    return 0;
  }
  else
  {

    return -3;
  }
}

void NaoWholeBodyKinematics::lockRightArmJoints()
{
  assert (arm_inclusion_ != LEFT_ARM_ONLY);

  if (arm_inclusion_ == RIGHT_ARM_ONLY)
  {
    Eigen::MatrixXd js_weight = Eigen::MatrixXd::Identity(17, 17);
    for(int i = 12; i < 17; i++)
    {
      js_weight(i, i) = 0.0;
    }
    nao_tree_vik_->setWeightJS(js_weight);
  }
  else
  {
    Eigen::MatrixXd js_weight = Eigen::MatrixXd::Identity(22, 22);
    for(int i = 12; i < 17; i++)
    {
      js_weight(i, i) = 0.0;
    }
    nao_tree_vik_->setWeightJS(js_weight);
  }
}

void NaoWholeBodyKinematics::lockLeftArmJoints()
{
  assert (arm_inclusion_ != RIGHT_ARM_ONLY);

  if (arm_inclusion_ == LEFT_ARM_ONLY)
  {
    Eigen::MatrixXd js_weight = Eigen::MatrixXd::Identity(17, 17);
    for(int i = 12; i < 17; i++)
    {
      js_weight(i, i) = 0.0;
    }
    nao_tree_vik_->setWeightJS(js_weight);
  }
  else
  {
    Eigen::MatrixXd js_weight = Eigen::MatrixXd::Identity(22, 22);
    for(int i = 12; i < 17; i++)
    {
      js_weight(i, i) = 0.0;
    }
    nao_tree_vik_->setWeightJS(js_weight);
  }
}

} //end namespace nao_whole_body_ik
