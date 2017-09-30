#include "database_reader_test.h"
#include "whole_body_ik_solver_test.h"

nao_whole_body_ik::DatabaseReader2Ptr database_reader;

TEST_F(DatabaseReaderTest, databaseLoadingTest)
{
  ASSERT_TRUE(databaseLoadTest());
}

TEST_F(DatabaseReaderTest, DefaultValuesRightArmKNNTest)
{
  ASSERT_TRUE(knnDifferenceDefaultPositionRightArm());
}

TEST_F(DatabaseReaderTest, DefaultValuesLeftArmKNNTest)
{
  ASSERT_TRUE(knnDifferenceDefaultPositionLeftArm());
}

TEST_F(DatabaseReaderTest, RandomValuesRightArmKNNTest)
{
  ASSERT_TRUE(knnDifferenceRandomPositionRightArm());
}

TEST_F(DatabaseReaderTest, RandomValuesLeftArmKNNTest)
{
  ASSERT_TRUE(knnDifferenceRandomPositionLeftArm());
}

TEST_F(WholeBodyIKSolverTest, RightArmIKFromDatabaseTest)
{
  SetUp(database_reader, nao_whole_body_ik::NaoWholeBodyKinematics::RIGHT_ARM_ONLY);
  ASSERT_TRUE(rightArmIKFromDatabaseTest());
}

TEST_F(WholeBodyIKSolverTest, RightArmIKFromSeedTest)
{
  SetUp(database_reader, nao_whole_body_ik::NaoWholeBodyKinematics::RIGHT_ARM_ONLY);
  ASSERT_TRUE(rightArmIKSeedTest());
}

TEST_F(WholeBodyIKSolverTest, LeftArmIKFromDatabaseTest)
{
  SetUp(database_reader, nao_whole_body_ik::NaoWholeBodyKinematics::LEFT_ARM_ONLY);
  ASSERT_TRUE(leftArmIKFromDatabaseTest());
}

TEST_F(WholeBodyIKSolverTest, LeftArmIKFromSeedTest)
{
  SetUp(database_reader, nao_whole_body_ik::NaoWholeBodyKinematics::LEFT_ARM_ONLY);
  ASSERT_TRUE(leftArmIKSeedTest());
}

TEST_F(WholeBodyIKSolverTest, BothArmsIKFromDatabaseTest)
{
  SetUp(database_reader, nao_whole_body_ik::NaoWholeBodyKinematics::BOTH_ARMS);
  ASSERT_TRUE(bothArmsIKFromDatabaseTest());
}

TEST_F(WholeBodyIKSolverTest, BothArmsIKFromSeedTest)
{
  SetUp(database_reader, nao_whole_body_ik::NaoWholeBodyKinematics::BOTH_ARMS);
  ASSERT_TRUE(bothArmsIKSeedTest());
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "gtest_node");

  int return_code RUN_ALL_TESTS();
  ros::shutdown();
  return return_code;
}
