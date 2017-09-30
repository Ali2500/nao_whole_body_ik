/*
*  nao_leg_kinematics.cpp
*
*  Created on: 9 Apr 2016
*      Author: Ali Athar
*/

#include <nao_whole_body_ik/nao_leg_kinematics.h>

namespace nao_whole_body_ik
{

NaoLegKinematics::NaoLegKinematics(const robot_state::RobotStatePtr& robot_state) : min_joint_limits_(12), max_joint_limits_(12)
{
  const robot_state::JointModelGroup* right_leg_group = robot_state->getJointModelGroup("right_leg_6dof");
  const moveit::core::JointBoundsVector& r_leg_lims = right_leg_group->getActiveJointModelsBounds();
  const robot_state::JointModelGroup* left_leg_group = robot_state->getJointModelGroup("left_leg_6dof");
  const moveit::core::JointBoundsVector& l_leg_lims = left_leg_group->getActiveJointModelsBounds();

  for(int i = 0; i < 6; i++)
  {
    min_joint_limits_(i) = r_leg_lims[i]->at(0).min_position_;
    max_joint_limits_(i) = r_leg_lims[i]->at(0).max_position_;
    min_joint_limits_(i+6) = l_leg_lims[i]->at(0).min_position_;
    max_joint_limits_(i+6) = l_leg_lims[i]->at(0).max_position_;
  }

  //from r_sole up to torso:
  KDL::Chain leg_chain;
  leg_chain.addSegment(KDL::Segment("r_sole", KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Vector(0.0, 0.0, 0.04519))));
  leg_chain.addSegment(KDL::Segment("RAnkleRoll_link", KDL::Joint(KDL::Joint::RotX), KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));
  leg_chain.addSegment(KDL::Segment("RAnklePitch_link", KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0.0, 0.0, 0.1029))));
  leg_chain.addSegment(KDL::Segment("RKneePitch_link", KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0.0, 0.0, 0.1))));
  leg_chain.addSegment(KDL::Segment("RHipPitch_link", KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));
  leg_chain.addSegment(KDL::Segment("RHipRoll_link", KDL::Joint(KDL::Joint::RotX), KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));
  leg_chain.addSegment(KDL::Segment("RHipYawPitch_link", KDL::Joint(KDL::Vector(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.7071, 0.7071), KDL::Joint::RotAxis), KDL::Frame(KDL::Vector(0.0, 0.05, 0.085))));
  leg_chain.addSegment(KDL::Segment("torso", KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));

  //from torso to l_sole:
  KDL::Chain left_leg_chain;
  leg_chain.addSegment(KDL::Segment("torso_to_LHip_link", KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Vector(0.0, 0.05, -0.085))));
  leg_chain.addSegment(KDL::Segment("LHipYawPitch_link", KDL::Joint(KDL::Vector(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.7071, -0.7071), KDL::Joint::RotAxis), KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));
  leg_chain.addSegment(KDL::Segment("LHipRoll_link", KDL::Joint(KDL::Joint::RotX), KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));
  leg_chain.addSegment(KDL::Segment("LHipPitch_link", KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0.0, 0.0, -0.1))));
  leg_chain.addSegment(KDL::Segment("LKneePitch_link", KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0.0, 0.0, -0.1029))));
  leg_chain.addSegment(KDL::Segment("LAnklePitch_link", KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));
  leg_chain.addSegment(KDL::Segment("LAnkleRoll_link", KDL::Joint(KDL::Joint::RotX), KDL::Frame(KDL::Vector(0.0, 0.0, -0.04519))));
  leg_chain.addSegment(KDL::Segment("l_sole", KDL::Joint(KDL::Joint::None),KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));

  KDL::Tree nao_leg_tree;
  nao_leg_tree.addChain(leg_chain, "root");
  leg_chain_fk_.reset(new KDL::TreeFkSolverPos_recursive(nao_leg_tree));
  std::vector<std::string> end_effector_name(1, "l_sole");
  leg_chain_vik_.reset(new KDL::TreeIkSolverVel_wdls(nao_leg_tree, end_effector_name));
  leg_chain_vik_->setLambda(0.01);

  chain_ik_vars_.frames.insert(KDL::Frames::value_type("l_sole", KDL::Frame::Identity()));
  chain_ik_vars_.delta_twists.insert(KDL::Twists::value_type("l_sole", KDL::Twist::Zero()));
  chain_ik_vars_.delta_q.resize(12);
  chain_ik_vars_.maxiter = 100;
  chain_ik_vars_.eps = 0.005;
}

NaoLegKinematics::~NaoLegKinematics()
{

}

double NaoLegKinematics::calculateLegIKSolutionIteratively(const KDL::JntArray& q_init,
                                                           const KDL::Frames& p_in, KDL::JntArray& q_out)
{
  q_out = q_init;

  //First check if all elements in p_in are available:
  for(KDL::Frames::const_iterator f_des_it = p_in.begin(); f_des_it != p_in.end(); ++f_des_it)
  {
    if (chain_ik_vars_.frames.find(f_des_it->first) == chain_ik_vars_.frames.end())
    {
      return -2;
    }
  }

  unsigned int k = 0;
  while (++k <= chain_ik_vars_.maxiter)
  {
    for (KDL::Frames::const_iterator f_des_it = p_in.begin(); f_des_it != p_in.end(); ++f_des_it)
    {
      //Get all iterators for this endpoint
      KDL::Frames::iterator f_it = chain_ik_vars_.frames.find(f_des_it->first);
      KDL::Twists::iterator delta_twist = chain_ik_vars_.delta_twists.find(f_des_it->first);

      leg_chain_fk_->JntToCart(q_out, f_it->second, f_it->first);
      delta_twist->second = diff(f_it->second, f_des_it->second);

    }

    double res = leg_chain_vik_->CartToJnt(q_out, chain_ik_vars_.delta_twists, chain_ik_vars_.delta_q);

    if (res < chain_ik_vars_.eps)
    {
      return res;
    }

    Add(q_out, chain_ik_vars_.delta_q, q_out);

    for (unsigned int j = 0; j < q_out.rows(); j++)
    {
      if (q_out(j) < min_joint_limits_(j))
      {
        q_out(j) = max_joint_limits_(j);
      }

      else if (q_out(j) > max_joint_limits_(j))
      {
        q_out(j) = max_joint_limits_(j);
      }
    }

    //enforce equal mimic joint values:
    q_out(5) = -1.0 * q_out(6);
  }

  if (k <= chain_ik_vars_.maxiter)
  {
    return 0;
  }
  else
  {

    return -3;
  }
}

bool NaoLegKinematics::adjustLegJointValues(const Eigen::Affine3d& target_pose, const std::vector<double>& seed_state,
                                            std::vector<double>& ik_solution)
{
  assert ((int)seed_state.size() == 12);

  KDL::Frame target_frame;
  nao_whole_body_ik::CommonFunctions::EigenAffineToKDLFrame(target_pose, target_frame);
  KDL::Frames p_in;
  p_in.insert(KDL::Frames::value_type("l_sole", target_frame));

  KDL::JntArray q_in, q_out;
  nao_whole_body_ik::CommonFunctions::STLVectorToKDLJntArray(seed_state, q_in);
  double success = calculateLegIKSolutionIteratively(q_in, p_in, q_out);
  if (success >= 0)
  {
    nao_whole_body_ik::CommonFunctions::KDLJntArrayToSTLVector(q_out, ik_solution);
    return true;
  }
  else
  {
    return false;
  }

}

bool NaoLegKinematics::adjustLegJointValues(const std::vector<double>& original_values, const std::vector<int>& fixed_joint_indices,
                                            std::vector<double>& leg_joint_values)
{
  assert ((int)leg_joint_values.size() == 12);
  assert ((int)original_values.size() == 12);

  KDL::Frame target_frame;
  KDL::JntArray original_values_kdl;
  nao_whole_body_ik::CommonFunctions::STLVectorToKDLJntArray(original_values, original_values_kdl);
  leg_chain_fk_->JntToCart(original_values_kdl, target_frame, "l_sole");
  //std::cout << "original l_sole pose:" << std::endl << target_frame << std::endl;

  KDL::Frames p_in;
  p_in.insert(KDL::Frames::value_type("l_sole", target_frame));
  Eigen::MatrixXd js_weight = Eigen::MatrixXd::Identity(12, 12);
  for(int i = 0; i < (int)fixed_joint_indices.size(); i++)
  {
    assert (fixed_joint_indices[i] < 5);
    js_weight(fixed_joint_indices[i], fixed_joint_indices[i]) = 0.0;
  }
  leg_chain_vik_->setWeightJS(js_weight);

  KDL::JntArray q_in, q_out;
  nao_whole_body_ik::CommonFunctions::STLVectorToKDLJntArray(leg_joint_values, q_in);
  double success = calculateLegIKSolutionIteratively(q_in, p_in, q_out);
  if (success >= 0)
  {
    nao_whole_body_ik::CommonFunctions::KDLJntArrayToSTLVector(q_out, leg_joint_values);
    return true;
  }
  else
  {
    return false;
  }
}

}
