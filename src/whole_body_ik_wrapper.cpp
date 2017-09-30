#include <nao_whole_body_ik/whole_body_ik_wrapper.h>

#include <algorithm>
#include <boost/bind/bind.hpp>

#define NUM_NEIGHBORS 44

static bool compare_fn(const std::pair<int, double>& p1, const std::pair<int, double>& p2)
{
  return p1.second < p2.second;
}

namespace nao_whole_body_ik
{

WholeBodyWrapperIK::WholeBodyWrapperIK(const std::string &robot_description_param_name,
                                       NaoWholeBodyKinematics::ARMS_INCLUSION arm_inclusion,
                                       bool load_databases)
{
  robot_model_loader::RobotModelLoader model_loader(robot_description_param_name);
  robot_state_.reset(new robot_state::RobotState(model_loader.getModel()));
  nao_ik_.reset(new NaoWholeBodyKinematics(robot_state_, arm_inclusion));
  database_reader_.reset(new DatabaseReader());
  arm_inclusion_ = arm_inclusion;

  init(load_databases);
}

WholeBodyWrapperIK::WholeBodyWrapperIK(const moveit::core::RobotStatePtr &robot_state,
                                       NaoWholeBodyKinematics::ARMS_INCLUSION arm_inclusion,
                                       bool load_databases) :
  nao_ik_(new NaoWholeBodyKinematics(robot_state, arm_inclusion)),
  database_reader_(new DatabaseReader()),
  robot_state_(robot_state),
  arm_inclusion_(arm_inclusion)
{
  init(load_databases);
}

void WholeBodyWrapperIK::init(bool load_databases)
{
  if(load_databases && arm_inclusion_ == NaoWholeBodyKinematics::BOTH_ARMS)
  {
    database_reader_->loadMultiFootDatasets();
    num_neighbors_ = NUM_NEIGHBORS / 2;
  }
  else if(load_databases && arm_inclusion_ == NaoWholeBodyKinematics::RIGHT_ARM_ONLY)
  {
    database_reader_->loadMultiFootDatasets(true, false);
    num_neighbors_ = NUM_NEIGHBORS;
  }
  else if(load_databases && arm_inclusion_ == NaoWholeBodyKinematics::LEFT_ARM_ONLY)
  {
    database_reader_->loadMultiFootDatasets(false, true);
    num_neighbors_ = NUM_NEIGHBORS;
  }
}

WholeBodyWrapperIK::~WholeBodyWrapperIK()
{
}

bool WholeBodyWrapperIK::getWholeBodyIKSolution(const std::map<std::string, Eigen::Affine3d>& target_pose_map, std::vector<double> &ik_solution)
{
  //sanity checks
  assert(target_pose_map.find("l_sole") != target_pose_map.end());
  if(arm_inclusion_ == NaoWholeBodyKinematics::RIGHT_ARM_ONLY || arm_inclusion_ == NaoWholeBodyKinematics::BOTH_ARMS)
    assert(target_pose_map.find("r_gripper") != target_pose_map.end());
  if(arm_inclusion_ == NaoWholeBodyKinematics::LEFT_ARM_ONLY || arm_inclusion_ == NaoWholeBodyKinematics::BOTH_ARMS)
    assert(target_pose_map.find("l_gripper") != target_pose_map.end());

  std::vector<std::vector<double> > nearest_neighbors, dummy;
  std::vector<int> database_ids = findClosestDatabaseIds(target_pose_map.find("l_sole")->second);
  std::vector<double> eef_pose;

  if(arm_inclusion_ == NaoWholeBodyKinematics::RIGHT_ARM_ONLY)
  {
    CommonFunctions::EigenAffineToSTLVector2(target_pose_map.find("r_gripper")->second, eef_pose);
    database_reader_->findNearestNeighbors(num_neighbors_, database_ids, eef_pose, dummy, nearest_neighbors, DatabaseReader::RIGHT_ARM);
  }
  else if(arm_inclusion_ == NaoWholeBodyKinematics::LEFT_ARM_ONLY)
  {
    CommonFunctions::EigenAffineToSTLVector2(target_pose_map.find("l_gripper")->second, eef_pose);
    database_reader_->findNearestNeighbors(num_neighbors_, database_ids, eef_pose, dummy, nearest_neighbors, DatabaseReader::LEFT_ARM);
  }
  else if(arm_inclusion_ == NaoWholeBodyKinematics::BOTH_ARMS)
  {
    std::vector<std::vector<double> > right_arm_nn, left_arm_nn;
    CommonFunctions::EigenAffineToSTLVector2(target_pose_map.find("r_gripper")->second, eef_pose);
    database_reader_->findNearestNeighbors(num_neighbors_, database_ids, eef_pose, dummy, right_arm_nn, DatabaseReader::RIGHT_ARM);

    CommonFunctions::EigenAffineToSTLVector2(target_pose_map.find("l_gripper")->second, eef_pose);
    database_reader_->findNearestNeighbors(num_neighbors_, database_ids, eef_pose, dummy, left_arm_nn, DatabaseReader::LEFT_ARM);

    for(std::size_t i = 0; i < num_neighbors_; ++i)
    {
      std::vector<double> joint_values1(22), joint_values2(22);  //one will have leg values from right arm nn and the other from left arm nn
      for(std::size_t j = 0; j < 12; ++j)
      {
        joint_values1[j] = right_arm_nn[i][j];
        joint_values2[j] = left_arm_nn[i][j];
      }

      for(std::size_t j = 0; j < 5; ++j)
      {
        joint_values1[j+12] = right_arm_nn[i][j+12];
        joint_values1[j+17] = left_arm_nn[i][j+12];

        joint_values2[j+12] = right_arm_nn[i][j+12];
        joint_values2[j+17] = left_arm_nn[i][j+12];
      }

      nearest_neighbors.push_back(joint_values1);
      nearest_neighbors.push_back(joint_values2);
    }
  }
  else
    throw std::runtime_error("Invalid arm inclusion value");

  for(std::size_t i = 0; i < nearest_neighbors.size(); ++i)
  {
    if(getWholeBodyIKSolution(target_pose_map, nearest_neighbors[i], ik_solution))
      return true;
  }

  return false;
}

bool WholeBodyWrapperIK::getWholeBodyIKSolution(const std::map<std::string, Eigen::Affine3d> &target_pose_map,
                                                const std::vector<double> &seed_state, std::vector<double> &ik_solution)
{
  for(std::map<std::string, Eigen::Affine3d>::const_iterator it = target_pose_map.begin(); it != target_pose_map.end(); ++it)
    assert(it->first == "l_sole" || it->first == "r_gripper" || it->first == "l_gripper");

  return nao_ik_->getWholeBodyIKSolution(target_pose_map, seed_state, ik_solution);
}

void WholeBodyWrapperIK::setTSWeights(const std::map<std::string, std::vector<double> > &ts_weight_map)
{
  for(std::map<std::string, std::vector<double> >::const_iterator it = ts_weight_map.begin(); it != ts_weight_map.end(); ++it)
  {
    if(it->first == "l_sole")
      nao_ik_->setLeftFootTSWeight(it->second);
    else if(it->first == "l_gripper")
      nao_ik_->setLeftArmTSWeight(it->second);
    else if(it->first == "r_gripper")
      nao_ik_->setRightArmTSWeight(it->second);
  }
}

std::vector<int> WholeBodyWrapperIK::findClosestDatabaseIds(const Eigen::Affine3d& right_T_left)
{
  std::vector<std::vector<double> > database_poses = database_reader_->getDatabaseFootPoses();

  std::vector<double> right_foot_T_left_foot;
  CommonFunctions::EigenAffineToSTLVector(right_T_left, right_foot_T_left_foot);
  std::vector<std::pair<int, double> > foot_pose_differences(database_poses.size());

  for(std::size_t i = 0; i < database_poses.size(); ++i)
  {
    double difference = 0.75 * (database_poses[i][0] - right_foot_T_left_foot[0]) * (database_poses[i][0] - right_foot_T_left_foot[0]);
    difference += 0.75 * (database_poses[i][1] - right_foot_T_left_foot[1]) * (database_poses[i][1] - right_foot_T_left_foot[1]);
    difference += 0.25 * (database_poses[i][2] - right_foot_T_left_foot[2]) * (database_poses[i][2] - right_foot_T_left_foot[2]);

    difference = sqrt(difference);
    foot_pose_differences[i] = std::pair<int, double>(i, difference);
  }

  std::sort(foot_pose_differences.begin(), foot_pose_differences.end(), compare_fn);
  std::vector<int> sorted_ids(foot_pose_differences.size());

  //printf("Query foot pose: [%lf, %lf, %lf]\n", right_foot_T_left_foot[0], right_foot_T_left_foot[1], right_foot_T_left_foot[2]);
  for(std::size_t i = 0; i < foot_pose_differences.size(); ++i)
  {
    //std::vector<double>& foot_pose = database_poses[foot_pose_differences[i].first];
    //printf("ID: %d, Pose: [%lf, %lf, %lf]\n", foot_pose_differences[i].first, foot_pose[0], foot_pose[1], foot_pose[2]);
    sorted_ids[i] = foot_pose_differences[i].first;
  }

  return sorted_ids;
}

void WholeBodyWrapperIK::setDatabaseReader(const DatabaseReader2Ptr &database_reader)
{
  database_reader_ = database_reader;
  databases_loaded_ = true;
  num_neighbors_ = (arm_inclusion_ == nao_whole_body_ik::NaoWholeBodyKinematics::BOTH_ARMS ? NUM_NEIGHBORS / 2 : NUM_NEIGHBORS);
}

robot_state::RobotStatePtr WholeBodyWrapperIK::getRobotState() const
{
  return robot_state_;
}

} //end namespace nao_whole_body_ik
