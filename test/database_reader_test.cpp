#include "database_reader_test.h"

#include <algorithm>

#include <nao_whole_body_ik/common_functions.h>

#define NUM_NEIGHBORS 20

DatabaseReaderTest::DatabaseReaderTest()
{
}

DatabaseReaderTest::~DatabaseReaderTest()
{
}

void DatabaseReaderTest::SetUp()
{
  if(!database_reader)
  {
    database_reader.reset(new nao_whole_body_ik::DatabaseReader());
  }

  robot_model_loader::RobotModelLoader model_loader("robot_description");
  robot_state_.reset(new robot_state::RobotState(model_loader.getModel()));
}

bool DatabaseReaderTest::databaseLoadTest()
{
  try
  {
    database_reader->loadMultiFootDatasets(true, true);
  }
  catch(...)
  {
    return false;
  }

  return database_reader->isDatabaseLoaded();
}

bool DatabaseReaderTest::knnDifferenceDefaultPositionRightArm()
{
  if(!database_reader->isDatabaseLoaded())
    throw std::runtime_error("Databases aren't loaded");

  robot_state_->setToDefaultValues();
  const Eigen::Affine3d right_eef_pose = robot_state_->getFrameTransform("r_gripper");

  std::vector<double> right_arm_diffs = getKnnDifferences(right_eef_pose, nao_whole_body_ik::DatabaseReader::RIGHT_ARM);
  return inOrder(right_arm_diffs);
}

bool DatabaseReaderTest::knnDifferenceDefaultPositionLeftArm()
{
  if(!database_reader->isDatabaseLoaded())
    throw std::runtime_error("Databases aren't loaded");

  robot_state_->setToDefaultValues();
  const Eigen::Affine3d left_eef_pose = robot_state_->getFrameTransform("l_gripper");

  std::vector<double> left_arm_diffs = getKnnDifferences(left_eef_pose, nao_whole_body_ik::DatabaseReader::LEFT_ARM);
  return inOrder(left_arm_diffs);
}

bool DatabaseReaderTest::knnDifferenceRandomPositionRightArm()
{
  if(!database_reader->isDatabaseLoaded())
    throw std::runtime_error("Databases aren't loaded");

  robot_state_->setToRandomPositions();
  const Eigen::Affine3d right_eef_pose = robot_state_->getFrameTransform("r_gripper");

  std::vector<double> right_arm_diffs = getKnnDifferences(right_eef_pose, nao_whole_body_ik::DatabaseReader::RIGHT_ARM);
  return inOrder(right_arm_diffs);
}

bool DatabaseReaderTest::knnDifferenceRandomPositionLeftArm()
{
  if(!database_reader->isDatabaseLoaded())
    throw std::runtime_error("Databases aren't loaded");

  robot_state_->setToRandomPositions();
  const Eigen::Affine3d left_eef_pose = robot_state_->getFrameTransform("l_gripper");

  std::vector<double> left_arm_diffs = getKnnDifferences(left_eef_pose, nao_whole_body_ik::DatabaseReader::LEFT_ARM);
  return inOrder(left_arm_diffs);
}

std::vector<double> DatabaseReaderTest::getKnnDifferences(const Eigen::Affine3d &eef_pose, nao_whole_body_ik::DatabaseReader::ARM arm)
{
  std::vector<double> eef_pose_vec;
  nao_whole_body_ik::CommonFunctions::EigenAffineToSTLVector2(eef_pose, eef_pose_vec);

  std::vector<int> dbs_to_search(1);
  dbs_to_search[0] = 2;

  std::vector<std::vector<double> > knns, dummy;
  database_reader->findNearestNeighbors(NUM_NEIGHBORS, dbs_to_search, eef_pose_vec, knns, dummy, arm);

  std::vector<double> diffs(NUM_NEIGHBORS);
  for(std::size_t i = 0; i < NUM_NEIGHBORS; ++i)
  {
    nao_whole_body_ik::distance_function<double> functor;
    diffs[i] = functor(eef_pose_vec, knns[i], 6);
  }

  return diffs;
}

bool DatabaseReaderTest::inOrder(const std::vector<double> &vals)
{
  for(std::size_t i = 1; i < vals.size(); ++i)
  {
    if((vals[i] - vals[i-1]) < -10e-5)
      return false;
  }

  return true;
}


