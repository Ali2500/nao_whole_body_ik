/*
* 	database_reader2.cpp
*
*   Created on: 28 Apr 2016
*      Author: Ali Athar
*/
#include <nao_whole_body_ik/database_reader.h>

#include <boost/filesystem.hpp>

#define NUM_OF_DATABASES 11
#define DIMENSIONALITY 6
#define NUM_JOINTS 17

namespace nao_whole_body_ik
{

DatabaseReader::DatabaseReader() :
  db_search_vars_right_(NUM_OF_DATABASES),
  db_search_vars_left_(NUM_OF_DATABASES),
  database_loaded_(false)
{
  const double database_poses[][3] =
  {
    {-0.0382, 0.112, -0.3},
    { -0.06, 0.1, 0.0 },
    { -0.0382, 0.0882, 0.3 },
    { -0.04, 0.08, 0.0 },
    { -0.03, 0.13, 0.0 },
    { 0.04, 0.1, 0.3 },
    { 0.06, 0.1, 0.0 },
    { 0.04, 0.1, -0.3 },
    { 0.04, 0.08, 0.0 },
    { 0.03, 0.12, 0.0 },
    { 0.0, 0.1, 0.0}
  };

  for(int i = 0; i < sizeof(database_poses)/sizeof(database_poses[0]); i++)
  {
    database_foot_poses_.push_back(std::vector<double>(database_poses[i], database_poses[i] + 3));
  }
}

DatabaseReader::~DatabaseReader()
{
}

void DatabaseReader::loadMultiFootDatasetsAsync(bool load_right_arm, bool load_left_arm)
{
  boost::thread db_loading_thread(&DatabaseReader::loadMultiFootDatasets, this, load_right_arm, load_left_arm);
}

void DatabaseReader::loadMultiFootDatasets(bool load_right_arm, bool load_left_arm)
{
  std::string filepath = ros::package::getPath("nao_whole_body_ik");
  if(!load_right_arm && !load_left_arm)
  {
    printf("There is nothing to load!\n");
    return;
  }

  int total_right_arm_configs, total_left_arm_configs;
  if(load_right_arm)
    total_right_arm_configs = loadMultiFootDataset(filepath + "/databases/multi_foot_databases/r_gripper_full_pose_", RIGHT_ARM);

  if(load_left_arm)
    total_left_arm_configs = loadMultiFootDataset(filepath + "/databases/multi_foot_databases/l_gripper_full_pose_", LEFT_ARM);

  database_loaded_ = true;
  std::cout << "Loaded a total of " << total_right_arm_configs << " right arm configurations configurations across " << NUM_OF_DATABASES << " databases." << std::endl;
  std::cout << "Loaded a total of " << total_left_arm_configs << " left arm configurations configurations across " << NUM_OF_DATABASES << " databases." << std::endl;
}

int DatabaseReader::loadMultiFootDataset(const std::string &file_prefix, ARM arm)
{
  int total_configs = 0;

  for(int i = 0; i < NUM_OF_DATABASES; i++)
  {
    std::string filepath = file_prefix;
    std::stringstream ss;
    ss << i+1;
    filepath += ss.str() + ".hdf5";

    if(!boost::filesystem::exists(filepath))
    {
      ss.str(std::string());
      ss << "The file " << filepath << " was not found.";
      ROS_ERROR_STREAM(ss.str().c_str());
      continue;
    }

    if(arm == RIGHT_ARM)
    {
      std::cout << "Loading right arm datasets for file no." << i+1 << "..." << std::endl;
      flann::load_from_file(db_search_vars_right_[i].cartesian_dataset, filepath, "cartesian_dataset");
      flann::load_from_file(db_search_vars_right_[i].joint_dataset, filepath, "joint_dataset");

      total_configs += db_search_vars_right_[i].cartesian_dataset.rows;
      db_search_vars_right_[i].search_index = new flann::Index<distance_function<int> >(db_search_vars_right_[i].cartesian_dataset, flann::KDTreeIndexParams(4));
      db_search_vars_right_[i].search_index->buildIndex();
    }
    else if(arm == LEFT_ARM)
    {
      std::cout << "Loading left arm datasets for file no." << i+1 << "..." << std::endl;
      flann::load_from_file(db_search_vars_left_[i].cartesian_dataset, filepath, "cartesian_dataset");
      flann::load_from_file(db_search_vars_left_[i].joint_dataset, filepath, "joint_dataset");

      total_configs += db_search_vars_left_[i].cartesian_dataset.rows;
      db_search_vars_left_[i].search_index = new flann::Index<distance_function<int> >(db_search_vars_left_[i].cartesian_dataset, flann::KDTreeIndexParams(4));
      db_search_vars_left_[i].search_index->buildIndex();
    }
    else
      throw std::runtime_error("Invalid arm specified");
  }

  return total_configs;
}

void DatabaseReader::findNearestNeighbors(const int &num_neighbors, const std::vector<double> &query_state,
                                          std::vector<std::vector<double> > &cartesian_values,
                                          std::vector<std::vector<double> > &joint_values, ARM arm)
{
  std::vector<int> database_ids(NUM_OF_DATABASES);
  for(int i = 0; i < NUM_OF_DATABASES; ++i)
    database_ids[i] = i;

  findNearestNeighbors(num_neighbors, database_ids, query_state, cartesian_values, joint_values, arm);
}

void DatabaseReader::findNearestNeighbors(const int& num_neighbors, const std::vector<int>& database_ids,
                                          const std::vector<double>& query_state, std::vector<std::vector<double> >& cartesian_values,
                                          std::vector<std::vector<double> >& joint_values, ARM arm)
{
  assert (database_loaded_);
  assert ((int)query_state.size() == 6);

  if (num_neighbors % (int)database_ids.size() != 0)
  {
    std::cout << "Only " << (num_neighbors / (int)database_ids.size()) * num_neighbors << " neighbors will be returne"
                                                                                          "d because num_neighbors is"
                                                                                          " not an integer factor of "
                                                                                          "the length of database_ids" << std::endl;
    ROS_WARN_STREAM("Number of neighbors returned will not be exactly as requested.");
  }

  int neighbors_per_db = num_neighbors / (int)database_ids.size();

  std::vector<FlannVariables>& db_search_vars = (arm == RIGHT_ARM ? db_search_vars_right_ : db_search_vars_left_);
  for(int i = 0; i < (int)database_ids.size(); i++)
  {
    assert (database_ids[i] < NUM_OF_DATABASES);
    int db_id = database_ids[i];

    flann::Matrix<int> query(new int[1 * DIMENSIONALITY], 1, DIMENSIONALITY);
    flann::Matrix<int> indices(new int[query.rows * neighbors_per_db], query.rows, neighbors_per_db);
    flann::Matrix<float> distances(new float[query.rows * neighbors_per_db], query.rows, neighbors_per_db);

    for(int j = 0; j < DIMENSIONALITY; j++)
    {
      query[0][j] = query_state[j] * 10000;
    }

    db_search_vars[db_id].search_index->knnSearch(query, indices, distances, neighbors_per_db, flann::SearchParams(128));

    std::vector<std::vector<double> > cart_vals(neighbors_per_db), joint_vals(neighbors_per_db);

    for(int j = 0; j < neighbors_per_db; j++)
    {
      cart_vals[j].resize(DIMENSIONALITY);
      for(int k = 0; k < DIMENSIONALITY; k++)
      {
        cart_vals[j][k] = static_cast<double>(db_search_vars[db_id].cartesian_dataset[indices[0][j]][k]) / 10000.0;
      }

      joint_vals[j].resize(NUM_JOINTS);
      for(int k = 0; k < NUM_JOINTS; k++)
      {
        joint_vals[j][k] = static_cast<double>(db_search_vars[db_id].joint_dataset[indices[0][j]][k]) / 10000.0;
      }
    }

    cartesian_values.insert(cartesian_values.end(), cart_vals.begin(), cart_vals.end());
    joint_values.insert(joint_values.end(), joint_vals.begin(), joint_vals.end());
  }
}

}
