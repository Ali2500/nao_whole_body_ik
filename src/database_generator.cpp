/*
* 	database_generator2.cpp
*
*   Created on: 26 Apr 2016
*      Author: Ali Athar
*/
#include <nao_whole_body_ik/database_generator.h>

#include <boost/filesystem.hpp>

#define NUM_OF_DATABASES 11
#define NUM_JOINTS 17
#define DIMENSIONALITY 6
#define RANDOMIZATION_COUNT 200
#define NUM_THREADS 4

namespace nao_whole_body_ik
{

DatabaseGenerator::DatabaseGenerator() : joint_interval_(5), right_arm_joint_limits_(5),
  database_file_names_(NUM_OF_DATABASES), database_file_names_left_(NUM_OF_DATABASES), left_foot_poses_(NUM_OF_DATABASES)
{

  robot_model_loader::RobotModelLoader model_loader("robot_description_right");
  robot_model::RobotModelPtr robot_model = model_loader.getModel();

  num_threads_ = boost::thread::hardware_concurrency();
  std::cout << num_threads_ << " threads will be used for database generation" << std::endl;
  recurs_vars_ = new RecursionVariables[num_threads_];

  for(int i = 0; i < num_threads_; i++)
  {
    recurs_vars_[i].planning_scene_.reset(new planning_scene::PlanningScene(robot_model));
    recurs_vars_[i].robot_state_.reset(new robot_state::RobotState(robot_model));
    recurs_vars_[i].nao_leg_kinematics_.reset(new NaoLegKinematics(recurs_vars_[i].robot_state_));
    recurs_vars_[i].joint_values_.resize(17);
  }

  for(int i = 0; i < 5; i++)
  {
    joint_interval_[i] = 0.75;
  }

  //const robot_state::JointModelGroup* right_leg = robot_state_->getJointModelGroup("right_leg");
  const robot_state::JointModelGroup* right_arm = recurs_vars_[0].robot_state_->getJointModelGroup("right_arm");

  //const moveit::core::JointBoundsVector& r_leg_lims = right_leg->getActiveJointModelsBounds();
  const moveit::core::JointBoundsVector& r_arm_lims = right_arm->getActiveJointModelsBounds();

  for(int i = 0; i < 5; i++)
  {
    //joint_limits[i] = std::pair<double, double>(r_leg_lims[i]->at(0).min_position_, r_leg_lims[i]->at(0).max_position_);
    right_arm_joint_limits_[i] = std::pair<double, double>(r_arm_lims[i]->at(0).min_position_, r_arm_lims[i]->at(0).max_position_);
  }

  std::string package_directory_path = ros::package::getPath("nao_whole_body_ik");
  std::string base_name = package_directory_path + "/databases/multi_foot_databases/r_gripper_full_pose_";
  std::string base_name_left = package_directory_path + "/databases/multi_foot_databases/l_gripper_full_pose_";
  for(int j = 0; j < NUM_OF_DATABASES; j++)
  {
    std::stringstream ss;
    ss << j+1;
    database_file_names_[j] = base_name + ss.str() + ".hdf5";
    database_file_names_left_[j] = base_name_left + ss.str() + ".hdf5";
  }

  left_foot_poses_[0].matrix() << 0.955136, 0.296168, 0.000180808, -0.00762697,
      -0.296168, 0.955136, -0.000304359, 0.107447,
      -0.000262837, 0.000237155, 1, 0.00046792,
      0,            0,            0,            1;

  left_foot_poses_[1].matrix() << 1, 2.65333e-05, 0.000690139, -0.0597693,
      -2.65143e-05, 1, -2.75802e-05, 0.100312,
      -0.00069014, 2.75619e-05, 1, 0.000386832,
      0, 0, 0, 1;

  left_foot_poses_[2].matrix() << 0.95542, -0.295249, 0.000461744, -0.0673697,
      0.295249, 0.95542, -2.84757e-05, 0.0838275,
      -0.000432752, 0.000163536, 1, -2.43985e-05,
      0, 0, 0, 1;

  left_foot_poses_[3].matrix() << 1, -0.000631114, -0.000502005, -0.0393647,
      0.000631081, 1, -6.58882e-05, 0.0799794,
      0.000502047, 6.55714e-05, 1, -7.17289e-06,
      0, 0, 0, 1;

  left_foot_poses_[4].matrix() << 0.999999, 0.000817013, 0.000749912, -0.0276675,
      -0.000816919, 1, -0.000125641, 0.127975,
      -0.000750014, 0.000125028, 1, -0.000293117,
      0, 0, 0, 1;

  left_foot_poses_[5].matrix() << 0.955336, -0.295521, 0.000116259, 0.0394683,
      0.295521, 0.955336, 3.1164e-06, 0.100082,
      -0.000111987, 3.13797e-05, 1, 1.42567e-05,
      0, 0, 0, 1;

  left_foot_poses_[6].matrix() << 1, 0.000131798, 0.000467948, 0.0595281,
      -0.000131767, 1, -6.66429e-05, 0.10041,
      -0.000467957, 6.65813e-05, 1, -0.000379858,
      0, 0, 0, 1;

  left_foot_poses_[7].matrix() << 0.955365, 0.295428, -0.000224253, 0.0387836,
      -0.295428, 0.955365, -0.000372615, 0.100482,
      0.000104163, 0.000422234, 1, -0.00044027,
      0, 0, 0, 1;

  left_foot_poses_[8].matrix() << 1, -0.000177302, -3.10734e-05, 0.0367916,
      0.000177305, 1, 9.02875e-05, 0.0814006,
      3.10574e-05, -9.0293e-05, 1, 0.000366702,
      0, 0, 0, 1;

  left_foot_poses_[9].matrix() << 1, -0.000430582, -0.000553707, 0.0276721,
      0.000430644, 1, 0.000112055, 0.118728,
      0.000553658, -0.000112294, 1, 0.000307347,
      0, 0, 0, 1;

  left_foot_poses_[10].matrix() << 1, 0, 0, 0,
      0, 1, 0, 0.1,
      0, 0, 1, 0,
      0, 0, 0, 1;

  const double safe_right_arm_values[] = { -2.0, -1.3, 0.0, 0.2, 0.0 };
  const double safe_left_arm_values[] = { -2.0, 1.3, 0.0, -0.2, 0.0 };
  safe_right_arm_values_ = std::vector<double>(safe_right_arm_values, safe_right_arm_values + 5);
  safe_left_arm_values_ = std::vector<double>(safe_left_arm_values, safe_left_arm_values + 5);
}

DatabaseGenerator::~DatabaseGenerator()
{
  delete[] recurs_vars_;
}

void DatabaseGenerator::generateMultiFootDatabase()
{

  long long approx_iters = 1;
  for(int i = 0; i < 5; i++)
  {
    approx_iters *= (right_arm_joint_limits_[i].second - right_arm_joint_limits_[i].first) / joint_interval_[i];
  }

  long long max_total_iters = approx_iters * NUM_OF_DATABASES * RANDOMIZATION_COUNT;
  std::cout << "There will be a maximum of " << approx_iters << "x" << NUM_OF_DATABASES << "x" <<
               RANDOMIZATION_COUNT << " = " << max_total_iters << " iterations. Continue (Y/N)? ";
  char option;
  while(true)
  {
    std::cin >> option;
    if (option == 'N' || option == 'n')
    {
      return;
    }
    else if (option == 'Y' || option == 'y')
    {
      break;
    }
  }

  for(iterator_ = 0; iterator_ < NUM_OF_DATABASES; iterator_++)
  {
    time_t start = time(0);

    for(int i = 0; i < num_threads_; i++)
    {
      recurs_vars_[i].thread_ = boost::thread(&DatabaseGenerator::startRecursion, this, i);
      std::cout << "Thread " << i << " started" << std::endl;
    }

    for(int i = 0; i < num_threads_; i++)
    {
      recurs_vars_[i].thread_.join();
      std::cout << "Thread " << i << " complete" << std::endl;
    }

    //all threads complete
    std::vector<std::vector<double> > combined_cartesian_configs, combined_cartesian_configs_left;
    std::vector<std::vector<double> > combined_joint_configs, combined_joint_configs_left;
    int total_ik_fails = 0;

    for(int i = 0; i < num_threads_; i++)
    {
      for(std::size_t j = 0; j < recurs_vars_[i].joint_configs_.size(); j++)
      {
        combined_joint_configs.push_back(recurs_vars_[i].joint_configs_[j]);
      }

      for(std::size_t j = 0; j < recurs_vars_[i].cartesian_configs_.size(); j++)
      {
        combined_cartesian_configs.push_back(recurs_vars_[i].cartesian_configs_[j]);
      }

      for(std::size_t j = 0; j < recurs_vars_[i].joint_configs_left_.size(); j++)
      {
        combined_joint_configs_left.push_back(recurs_vars_[i].joint_configs_left_[j]);
      }

      for(std::size_t j = 0; j < recurs_vars_[i].cartesian_configs_left_.size(); j++)
      {
        combined_cartesian_configs_left.push_back(recurs_vars_[i].cartesian_configs_left_[j]);
      }

      total_ik_fails += recurs_vars_[i].ik_fails_;

      recurs_vars_[i].joint_configs_.clear();
      recurs_vars_[i].cartesian_configs_.clear();
      recurs_vars_[i].joint_configs_left_.clear();
      recurs_vars_[i].cartesian_configs_left_.clear();
    }

    assert (combined_joint_configs.size() == combined_cartesian_configs.size() &&
            combined_joint_configs_left.size() == combined_cartesian_configs_left.size());
    int total_valid_configs = (int)combined_cartesian_configs.size();
    int total_valid_configs_left = (int)combined_cartesian_configs_left.size();

    double time_taken = difftime(time(0), start);

    std::cout << "Writing database " << iterator_+1 << " to file..." << std::endl;
    flann::Matrix<int> cartesian_dataset(new int[total_valid_configs * DIMENSIONALITY], total_valid_configs, DIMENSIONALITY);
    flann::Matrix<int> joint_dataset(new int[total_valid_configs * NUM_JOINTS], total_valid_configs, NUM_JOINTS);

    flann::Matrix<int> cartesian_dataset_left(new int[total_valid_configs_left * DIMENSIONALITY], total_valid_configs_left, DIMENSIONALITY);
    flann::Matrix<int> joint_dataset_left(new int[total_valid_configs_left * NUM_JOINTS], total_valid_configs_left, NUM_JOINTS);

    for(int i = 0; i < total_valid_configs; i++)
    {
      for(int j = 0; j < DIMENSIONALITY; j++)
      {
        cartesian_dataset[i][j] = static_cast<int>(combined_cartesian_configs[i][j] * 10000);
      }

      for(int j = 0; j < NUM_JOINTS; j++)
      {
        joint_dataset[i][j] = static_cast<int>(combined_joint_configs[i][j] * 10000);
      }
    }

    for(int i = 0; i < total_valid_configs_left; i++)
    {
      for(int j = 0; j < DIMENSIONALITY; j++)
      {
        cartesian_dataset_left[i][j] = static_cast<int>(combined_cartesian_configs_left[i][j] * 10000);
      }

      for(int j = 0; j < NUM_JOINTS; j++)
      {
        joint_dataset_left[i][j] = static_cast<int>(combined_joint_configs_left[i][j] * 10000);
      }
    }

    std::ifstream file_check(database_file_names_[iterator_].c_str());
    if (file_check.good())
    {
      int success = remove(database_file_names_[iterator_].c_str());
      if (success != 0)
      {
        ROS_ERROR_STREAM("Previous database file could not be deleted.");
      }
    }
    file_check.close();

    if(boost::filesystem::exists(database_file_names_left_[iterator_].c_str()))
      boost::filesystem::remove(boost::filesystem::path(database_file_names_left_[iterator_]));


    flann::save_to_file(cartesian_dataset, database_file_names_[iterator_], "cartesian_dataset");
    flann::save_to_file(joint_dataset, database_file_names_[iterator_], "joint_dataset");

    flann::save_to_file(cartesian_dataset_left, database_file_names_left_[iterator_], "cartesian_dataset");
    flann::save_to_file(joint_dataset_left, database_file_names_left_[iterator_], "joint_dataset");

    std::cout << std::endl << "Database " << iterator_+1 << " generated and stored successfully. Details:" << std::endl;
    std::cout << "Total time elapsed: " << time_taken << "s" << std::endl;
    std::cout << "IK fails: " << total_ik_fails << std::endl;
    std::cout << "Valid configurations (right arm): " << total_valid_configs << std::endl;
    std::cout << "Valid configurations (left arm): " << total_valid_configs_left << std::endl;
    std::cout << "*************************************************************" << std::endl;
    //std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    //std::cin.get();

    delete cartesian_dataset.ptr();
    delete joint_dataset.ptr();
    // joint_configs_.clear();
    // cartesian_configs_.clear();
  }

}

void DatabaseGenerator::startRecursion(int thread_id)
{
  recurs_vars_[thread_id].total_iters_ = 0;
  recurs_vars_[thread_id].valid_configs_ = 0;
  recurs_vars_[thread_id].valid_configs_left_ = 0;
  recurs_vars_[thread_id].ik_fails_ = 0;
  recurs_vars_[thread_id].depth_ = 0;
  recurs_vars_[thread_id].is_done_ = false;

  for(int i = 0; i < (RANDOMIZATION_COUNT / num_threads_); i++)
  {
    recurs_vars_[thread_id].robot_state_->setToRandomPositions();
    std::vector<double> right_leg_joint_values;
    recurs_vars_[thread_id].robot_state_->copyJointGroupPositions("right_leg", right_leg_joint_values);

    std::vector<double> ik_seed_state(12, 0.0), ik_solution;
    for(int j = 0; j < 5; j++)
    {
      ik_seed_state[j] = right_leg_joint_values[j];
      ik_seed_state[11-j] = -1.0 * right_leg_joint_values[j];
    }
    bool ik_success = recurs_vars_[thread_id].nao_leg_kinematics_->adjustLegJointValues(left_foot_poses_[iterator_], ik_seed_state, ik_solution);
    if (!ik_success)
    {
      recurs_vars_[thread_id].ik_fails_++;
      continue;
    }

    recurs_vars_[thread_id].robot_state_->setToDefaultValues();
    recurs_vars_[thread_id].robot_state_->setJointGroupPositions("right_to_left_leg", ik_solution);
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    collision_detection::AllowedCollisionMatrix acm = recurs_vars_[thread_id].planning_scene_->getAllowedCollisionMatrix();

    recurs_vars_[thread_id].planning_scene_->checkCollision(collision_request, collision_result, *recurs_vars_[thread_id].robot_state_, acm);
    if (collision_result.collision)
    {
      std::cout << std::cout << "[THREAD " << thread_id << "]: Leg collision detected. Rejecting config" << std::endl;
      //CommonFunctions::printSTLVector("Joint values", ik_solution, false);
      //std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      //std::cin.get();
      continue;
    }

    for(int j = 0; j < 12; j++)
    {
      recurs_vars_[thread_id].joint_values_[j] = ik_solution[j];
    }

    generateDatabaseRecursively(thread_id);
    recurs_vars_[thread_id].depth_ = 0;

    if ((i%10) == 0)
    {
      std::cout << "[THREAD " << thread_id << "]: " << i+1 << "/" << (int)(RANDOMIZATION_COUNT / num_threads_) << " iterations complete" << std::endl;
    }
  }

  recurs_vars_[thread_id].is_done_ = true;
}

void DatabaseGenerator::generateDatabaseRecursively(int thread_id)
{
  assert (recurs_vars_[thread_id].depth_ >= 0);

  if (recurs_vars_[thread_id].depth_ == 5)
  {
    recurs_vars_[thread_id].total_iters_++;
    recurs_vars_[thread_id].robot_state_->setJointGroupPositions("left_arm", safe_left_arm_values_);
    recurs_vars_[thread_id].robot_state_->setJointGroupPositions("right_arm", std::vector<double>(recurs_vars_[thread_id].joint_values_.begin() + 12, recurs_vars_[thread_id].joint_values_.end()));

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    collision_detection::AllowedCollisionMatrix acm = recurs_vars_[thread_id].planning_scene_->getAllowedCollisionMatrix();

    recurs_vars_[thread_id].planning_scene_->checkCollision(collision_request, collision_result, *recurs_vars_[thread_id].robot_state_, acm);
    //      if (collision_result.collision)
    //      {
    //        //std::cout << "Arm collision detected. Rejecting config" << std::endl;
    //        recurs_vars_[thread_id].depth_--;
    //        return;
    //      }

    if (!collision_result.collision)
    {
      recurs_vars_[thread_id].valid_configs_++;
      //CommonFunctions::printSTLVector("Joint values", joint_values_, false);
      recurs_vars_[thread_id].joint_configs_.push_back(recurs_vars_[thread_id].joint_values_);

      const Eigen::Affine3d& r_gripper_pose = recurs_vars_[thread_id].robot_state_->getGlobalLinkTransform("r_gripper");
      Eigen::Vector3d r_gripper_translation = r_gripper_pose.translation();
      Eigen::Vector3d r_gripper_rotation = r_gripper_pose.rotation().eulerAngles(0, 1, 2);

      std::vector<double> r_gripper_pose_vector(6);
      for(int i = 0; i < 3; i++)
      {
        r_gripper_pose_vector[i] = r_gripper_translation[i];
        r_gripper_pose_vector[i+3] = r_gripper_rotation[i];
      }

      recurs_vars_[thread_id].cartesian_configs_.push_back(r_gripper_pose_vector);
    }

    std::vector<double> left_arm_values(recurs_vars_[thread_id].joint_values_.begin() + 12, recurs_vars_[thread_id].joint_values_.end());
    left_arm_values[1] *= -1.0;
    left_arm_values[3] *= -1.0;

    recurs_vars_[thread_id].robot_state_->setJointGroupPositions("right_arm", safe_right_arm_values_);
    recurs_vars_[thread_id].robot_state_->setJointGroupPositions("left_arm", left_arm_values);
    collision_request = collision_detection::CollisionRequest();
    collision_result = collision_detection::CollisionResult();

    recurs_vars_[thread_id].planning_scene_->checkCollision(collision_request, collision_result, *recurs_vars_[thread_id].robot_state_, acm);

    if(!collision_result.collision)
    {
      recurs_vars_[thread_id].valid_configs_left_++;
      recurs_vars_[thread_id].joint_configs_left_.push_back(left_arm_values);

      const Eigen::Affine3d& l_gripper_pose = recurs_vars_[thread_id].robot_state_->getGlobalLinkTransform("l_gripper");
      Eigen::Vector3d l_gripper_translation = l_gripper_pose.translation();
      Eigen::Vector3d l_gripper_rotation = l_gripper_pose.rotation().eulerAngles(0, 1, 2);

      std::vector<double> l_gripper_pose_vector(6);
      for(int i = 0; i < 3; i++)
      {
        l_gripper_pose_vector[i] = l_gripper_translation[i];
        l_gripper_pose_vector[i+3] = l_gripper_rotation[i];
      }

      recurs_vars_[thread_id].cartesian_configs_left_.push_back(l_gripper_pose_vector);
    }

    recurs_vars_[thread_id].depth_--;
    return;
  }

  for(double val = right_arm_joint_limits_[recurs_vars_[thread_id].depth_].first; val <= right_arm_joint_limits_[recurs_vars_[thread_id].depth_].second; val += joint_interval_[recurs_vars_[thread_id].depth_])
  {
    recurs_vars_[thread_id].joint_values_[12 + recurs_vars_[thread_id].depth_] = val;
    recurs_vars_[thread_id].depth_++;
    generateDatabaseRecursively(thread_id);
  }

  recurs_vars_[thread_id].depth_--;
  return;
}

}
