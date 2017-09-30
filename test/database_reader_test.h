#ifndef DATABASE_READER_TEST_H
#define DATABASE_READER_TEST_H

#include <gtest/gtest.h>
#include <nao_whole_body_ik/database_reader.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

/**
 * The DatabaseReaderTest class is a test fixture for the DatabaseReader class
 */
class DatabaseReaderTest : public ::testing::Test
{
  public:
    /**
     * Default constructor
     */
    DatabaseReaderTest();

    /**
      * Default destructor
      */
    ~DatabaseReaderTest();

    /**
     * Initializes the fixture.
     */
    void SetUp();

    /**
     * Check loading of databases
     * @return True if all databases were successfully loaded.
     */
    bool databaseLoadTest();

    /**
     * Check if neighbors can be found for the default right arm pose. A certain number of neighbors are queried and
     * it is verified that the neighbors are returned in the correct order in terms of distance to query state.
     * @return True if the neighbors are successfully retrieved and are in the correct order.
     */
    bool knnDifferenceDefaultPositionRightArm();

    /**
     * Check if neighbors can be found for the default left arm pose. A certain number of neighbors are queried and
     * it is verified that the neighbors are returned in the correct order in terms of distance to query state.
     * @return True if the neighbors are successfully retrieved and are in the correct order.
     */
    bool knnDifferenceDefaultPositionLeftArm();

    /**
     * Check if neighbors can be found for A randomly generated right arm pose. A certain number of neighbors are queried and
     * it is verified that the neighbors are returned in the correct order in terms of distance to query state.
     * @return True if the neighbors are successfully retrieved and are in the correct order.
     */
    bool knnDifferenceRandomPositionRightArm();

    /**
     * Check if neighbors can be found for A randomly generated left arm pose. A certain number of neighbors are queried and
     * it is verified that the neighbors are returned in the correct order in terms of distance to query state.
     * @return True if the neighbors are successfully retrieved and are in the correct order.
     */
    bool knnDifferenceRandomPositionLeftArm();

  protected:
    std::vector<double> getKnnDifferences(const Eigen::Affine3d& eef_pose, nao_whole_body_ik::DatabaseReader::ARM arm);

    bool inOrder(const std::vector<double>& vals);

    robot_state::RobotStatePtr robot_state_;
};

extern nao_whole_body_ik::DatabaseReader2Ptr database_reader;

#endif
