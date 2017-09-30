/*
* 	database_reader.h
*
*   Created on: 28 Apr 2016
*      Author: Ali Athar
*/

#ifndef _DATABASE_GENERATOR_H_
#define _DATABASE_GENERATOR_H_
#define LAMBDA 0.75

#include <nao_whole_body_ik/common_functions.h>
#include <ros/package.h>
#include <boost/thread.hpp>
#include <flann/flann.hpp>
#include <flann/io/hdf5.h>
#include <Eigen/Geometry>
#include <fstream>

namespace nao_whole_body_ik
{

template<class T>
/**
 * Distance functor that is used by FLANN to calculate the distance between two poses.
 */
struct distance_function
{
    typedef bool is_kdtree_distance;
    typedef T ElementType;
    typedef typename flann::Accumulator<T>::Type ResultType;
    template <typename Iterator1, typename Iterator2>
    ResultType operator()(Iterator1 a, Iterator2 b, size_t size,
                          ResultType /*worst_dist*/ = -1) const
    {
      ResultType result = ResultType();
      ResultType diff;

      assert (size == 6);

      for(size_t i = 0; i < size; ++i )
      {
        if (i < 3)
        {
          diff = LAMBDA * (a[i] - b[i]);
        }
        else
        {
          diff = (1 - LAMBDA) * (a[i] - b[i]);
        }

        result += diff*diff;
      }

      return result;
    }

    template <typename U, typename V>
    inline ResultType accum_dist(const U& a, const V& b, int) const
    {
      return (a-b)*(a-b);
    }
};

/**
 * The DatabaseReader class loads the stored database files and organizes them into KD trees using FLANN so that
 * the nearest neighbor query can be efficiently solved.
 */
class DatabaseReader
{
  public:
    /**
     * Enum to specify whether file/data related to right or left arm is being processed
     */
    enum ARM { RIGHT_ARM = 0, LEFT_ARM };

    /**
     * Default constructor
     */
    DatabaseReader();

    /**
      * Default destructor
      */
    ~DatabaseReader();

    /**
     * Get list of foot poses corresponding to the foot poses of configurations in the database files.
     * @return List of foot poses (right_foot_T_left_foot). Each database file contains various joint configurations and
     *  poses with the foot pose specified by this list.
     */
    inline std::vector<std::vector<double> > getDatabaseFootPoses() const;

    /**
     * Check is databases have been loaded
     * @return true if the database has been successfully loaded
     */
    inline bool isDatabaseLoaded() const;

    /**
     * Load the configurations without blocking the calling thread.
     * @param load_right_arm Flag specifying whether right arm configurations should be loaded.
     * @param load_left_arm Flag specifying whether left arm configurations should be loaded.
     */
    void loadMultiFootDatasetsAsync(bool load_right_arm = true, bool load_left_arm = true);

    /**
     * Loads the configurations while blocking the calling thread.
     * @param load_right_arm Flag specifying whether right arm configurations should be loaded.
     * @param load_left_arm Flag specifying whether left arm configurations should be loaded.
     */
    void loadMultiFootDatasets(bool load_right_arm = true, bool load_left_arm = true);

    /**
     * Finds joint configurations which have an end effector pose close to the query state. This is done by performing
     * a k-nearest neighbor search on the configurations that were loaded from the database. This function will search
     * @param num_neighbors Number of neighbors to find per database.
     * @param query_state The query pose of the end effector whose neighbors have to be found. This should be a 6-element vector in
     * the following format: [x, y, z, roll, pitch, yaw].
     * @param cartesian_values List of neighbors that will be populated. The neighbors are arranged in decreasing order of proximity i.e.
     *  the closes neighbor is at index 0. The format of the poses is the same as above.
     * @param joint_values List of joint configurations corresponding to the nearest neighbors. Each joint configuration has 17 values
     * corresponding to the right left, left leg, and arm joint positions.
     * @param arm Specify which end effector's pose has to be queried.
     */
    void findNearestNeighbors(const int& num_neighbors, const std::vector<double>& query_state,
                              std::vector<std::vector<double> >& cartesian_values,
                              std::vector<std::vector<double> >& joint_values, ARM arm);

    /**
     * Finds joint configurations which have an end effector pose close to the query state. This is done by performing
     * a k-nearest neighbor search on the configurations that were loaded from the database.
     * @param num_neighbors Total number of neighbors to find
     * @param database_ids A list of database IDs which will be searched for solving this query.
     * @param query_state The query pose of the end effector whose neighbors have to be found. This should be a 6-element vector in
     * the following format: [x, y, z, roll, pitch, yaw].
     * @param cartesian_values List of neighbors that will be populated. The neighbors are arranged in decreasing order of proximity i.e.
     *  the closes neighbor is at index 0. The format of the poses is the same as above.
     * @param joint_values List of joint configurations corresponding to the nearest neighbors. Each joint configuration has 17 values
     * corresponding to the right left, left leg, and arm joint positions.
     * @param arm Specify which end effector's pose has to be queried.
     */
    void findNearestNeighbors(const int& num_neighbors, const std::vector<int>& database_ids,
                              const std::vector<double>& query_state, std::vector<std::vector<double> >& cartesian_values,
                              std::vector<std::vector<double> >& joint_values, ARM arm);

  private:
    int loadMultiFootDataset(const std::string& file_prefix, ARM arm);

    struct FlannVariables
    {
        flann::Matrix<int> cartesian_dataset;
        flann::Matrix<int> joint_dataset;
        flann::Index<distance_function<int> >* search_index;
    };

    std::vector<FlannVariables> db_search_vars_right_, db_search_vars_left_;
    bool database_loaded_;
    std::vector<std::vector<double> > database_foot_poses_;

};

typedef boost::shared_ptr<DatabaseReader> DatabaseReader2Ptr;
typedef boost::shared_ptr<DatabaseReader const> DatabaseReader2ConstPtr;

inline std::vector<std::vector<double> > DatabaseReader::getDatabaseFootPoses() const
{
  return database_foot_poses_;
}

inline bool DatabaseReader::isDatabaseLoaded() const
{
  return database_loaded_;
}
}

#endif
