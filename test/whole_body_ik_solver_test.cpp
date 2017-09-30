#include "whole_body_ik_solver_test.h"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>


WholeBodyIKSolverTest::WholeBodyIKSolverTest()
{
}

WholeBodyIKSolverTest::~WholeBodyIKSolverTest()
{
}

void WholeBodyIKSolverTest::SetUp(const nao_whole_body_ik::DatabaseReader2Ptr& database_reader,
                                  nao_whole_body_ik::NaoWholeBodyKinematics::ARMS_INCLUSION arms)
{
  robot_model_loader::RobotModelLoader model_loader("robot_description");
  robot_state_.reset(new robot_state::RobotState(model_loader.getModel()));

  whole_body_ik_.reset(new nao_whole_body_ik::WholeBodyWrapperIK(robot_state_, arms, false));
  whole_body_ik_->setDatabaseReader(database_reader);
}

bool WholeBodyIKSolverTest::rightArmIKSeedTest()
{
  const robot_state::JointModelGroup* joints = robot_state_->getJointModelGroup("legs_with_right_hand");
  robot_state_->setToRandomPositions(joints);

  std::map<std::string, Eigen::Affine3d> target_poses;
  target_poses["l_sole"] = robot_state_->getFrameTransform("l_sole");
  target_poses["r_gripper"] = robot_state_->getFrameTransform("r_gripper");

  std::vector<double> seed_state;
  robot_state_->copyJointGroupPositions(joints, seed_state);
  for(std::size_t i = 0; i < seed_state.size(); ++i)
    seed_state[i] = getGaussianRandomNumber(seed_state[i], 0.15);

  robot_state_->setJointGroupPositions("legs_with_right_hand", seed_state);
  robot_state_->enforceBounds();
  robot_state_->setVariablePosition("LHipYawPitch", -robot_state_->getVariablePosition("RHipYawPitch"));
  robot_state_->copyJointGroupPositions("legs_with_right_hand", seed_state);

  std::vector<double> ik_solution;
  return whole_body_ik_->getWholeBodyIKSolution(target_poses, seed_state, ik_solution);
}

bool WholeBodyIKSolverTest::rightArmIKFromDatabaseTest()
{
  const robot_state::JointModelGroup* right_arm = robot_state_->getJointModelGroup("right_arm");
  robot_state_->setToRandomPositions(right_arm);

  const robot_state::JointModelGroup* right_leg = robot_state_->getJointModelGroup("right_leg");
  robot_state_->setToRandomPositions(right_leg);

  std::map<std::string, Eigen::Affine3d> target_poses;
  target_poses["l_sole"] = robot_state_->getFrameTransform("l_sole");
  target_poses["r_gripper"] = robot_state_->getFrameTransform("r_gripper");

  std::vector<double> ik_solution;
  return whole_body_ik_->getWholeBodyIKSolution(target_poses, ik_solution);
}

bool WholeBodyIKSolverTest::leftArmIKSeedTest()
{
  const robot_state::JointModelGroup* joints = robot_state_->getJointModelGroup("legs_with_left_hand");
  robot_state_->setToRandomPositions(joints);

  std::map<std::string, Eigen::Affine3d> target_poses;
  target_poses["l_sole"] = robot_state_->getFrameTransform("l_sole");
  target_poses["l_gripper"] = robot_state_->getFrameTransform("l_gripper");

  std::vector<double> seed_state;
  robot_state_->copyJointGroupPositions(joints, seed_state);
  for(std::size_t i = 0; i < seed_state.size(); ++i)
    seed_state[i] = getGaussianRandomNumber(seed_state[i], 0.15);

  robot_state_->setJointGroupPositions("legs_with_left_hand", seed_state);
  robot_state_->enforceBounds();
  robot_state_->setVariablePosition("LHipYawPitch", -robot_state_->getVariablePosition("RHipYawPitch"));
  robot_state_->copyJointGroupPositions("legs_with_left_hand", seed_state);

  std::vector<double> ik_solution;
  return whole_body_ik_->getWholeBodyIKSolution(target_poses, seed_state, ik_solution);
}

bool WholeBodyIKSolverTest::leftArmIKFromDatabaseTest()
{
  const robot_state::JointModelGroup* left_arm = robot_state_->getJointModelGroup("left_arm");
  robot_state_->setToRandomPositions(left_arm);

  const robot_state::JointModelGroup* right_leg = robot_state_->getJointModelGroup("right_leg");
  robot_state_->setToRandomPositions(right_leg);

  std::map<std::string, Eigen::Affine3d> target_poses;
  target_poses["l_sole"] = robot_state_->getFrameTransform("l_sole");
  target_poses["l_gripper"] = robot_state_->getFrameTransform("l_gripper");

  std::vector<double> ik_solution;
  return whole_body_ik_->getWholeBodyIKSolution(target_poses, ik_solution);
}

bool WholeBodyIKSolverTest::bothArmsIKSeedTest()
{
  const robot_state::JointModelGroup* joints = robot_state_->getJointModelGroup("whole_body_no_head");
  robot_state_->setToRandomPositions(joints);

  std::map<std::string, Eigen::Affine3d> target_poses;
  target_poses["l_sole"] = robot_state_->getFrameTransform("l_sole");
  target_poses["l_gripper"] = robot_state_->getFrameTransform("l_gripper");
  target_poses["r_gripper"] = robot_state_->getFrameTransform("r_gripper");

  std::vector<double> seed_state;
  robot_state_->copyJointGroupPositions(joints, seed_state);
  for(std::size_t i = 0; i < seed_state.size(); ++i)
    seed_state[i] = getGaussianRandomNumber(seed_state[i], 0.05);

  robot_state_->setJointGroupPositions("whole_body_no_head", seed_state);
  robot_state_->enforceBounds();
  robot_state_->setVariablePosition("LHipYawPitch", -robot_state_->getVariablePosition("RHipYawPitch"));
  robot_state_->copyJointGroupPositions("whole_body_no_head", seed_state);

  std::vector<double> ik_solution;
  return whole_body_ik_->getWholeBodyIKSolution(target_poses, seed_state, ik_solution);
}

bool WholeBodyIKSolverTest::bothArmsIKFromDatabaseTest()
{
  const robot_state::JointModelGroup* right_arm = robot_state_->getJointModelGroup("right_arm");
  robot_state_->setToRandomPositions(right_arm);

  const robot_state::JointModelGroup* left_arm = robot_state_->getJointModelGroup("left_arm");
  robot_state_->setToRandomPositions(left_arm);

  const robot_state::JointModelGroup* right_leg = robot_state_->getJointModelGroup("right_leg");
  robot_state_->setToRandomPositions(right_leg);

  std::map<std::string, Eigen::Affine3d> target_poses;
  target_poses["l_sole"] = robot_state_->getFrameTransform("l_sole");
  target_poses["l_gripper"] = robot_state_->getFrameTransform("l_gripper");
  target_poses["r_gripper"] = robot_state_->getFrameTransform("r_gripper");

  std::vector<double> ik_solution;
  return whole_body_ik_->getWholeBodyIKSolution(target_poses, ik_solution);
}

double WholeBodyIKSolverTest::getGaussianRandomNumber(double mean, double std)
{
  boost::normal_distribution<> nd(mean, std);
  boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_gen(rng_, nd);
  return var_gen();
}
