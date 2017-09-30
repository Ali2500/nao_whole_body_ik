#ifndef WHOLE_BODY_IK_SOLVER_TEST_H
#define WHOLE_BODY_IK_SOLVER_TEST_H

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

#include <gtest/gtest.h>
#include <moveit/robot_state/robot_state.h>

#include <nao_whole_body_ik/whole_body_ik_wrapper.h>

/**
 * The WholeBodyIKSolverTest class is a test fixture for the WholeBodyIWRapper class
 */
class WholeBodyIKSolverTest : public ::testing::Test
{
  public:
    /**
     * Default constructor
     */
    WholeBodyIKSolverTest();

    /**
      * Default destructor
      */
    ~WholeBodyIKSolverTest();

    /**
     * Initializes the fixture
     * @param database_reader Shared pointer to DatabaseReader object with databases already loaded. This saves having to
     * perform time consuming loading operations for every test case.
     * @param arms
     */
    void SetUp(const nao_whole_body_ik::DatabaseReader2Ptr &database_reader,
               nao_whole_body_ik::NaoWholeBodyKinematics::ARMS_INCLUSION arms);

    /**
     * Checks IK solver's ability to solve for left arm end effector poses when given a nearby seed state. A random
     * joint configuration is generated, the corresponding left arm end effector pose is calculated and is given as the
     * target pose to the IK solver. As a seed state, the joint configuration is shifted by introducing a Gaussian error
     * to each joint value.
     * @return True if the IK solution was found.
     */
    bool leftArmIKSeedTest();

    /**
     * Checks IK solver's ability to solve for left arm end effector poses using nearby poses from the database as seeds.
     * A random joint configuration is generated, and the corresponding left arm end effector pose is calculated and is
     * given as the target pose to the IK solver. The seed states are obtained by querying the databases.
     * @return True if the IK solution was found.
     */
    bool leftArmIKFromDatabaseTest();

    /**
     * Checks IK solver's ability to solve for right arm end effector poses when given a nearby seed state. A random
     * joint configuration is generated, the corresponding right arm end effector pose is calculated and is given as the
     * target pose to the IK solver. As a seed state, the joint configuration is shifted by introducing a Gaussian error
     * to each joint value.
     * @return True if the IK solution was found.
     */
    bool rightArmIKSeedTest();

    /**
     * Checks IK solver's ability to solve for right arm end effector poses using nearby poses from the database as seeds.
     * A random joint configuration is generated, and the corresponding right arm end effector pose is calculated and is
     * given as the target pose to the IK solver. The seed states are obtained by querying the databases.
     * @return True if the IK solution was found.
     */
    bool rightArmIKFromDatabaseTest();

    /**
     * Checks IK solver's ability to solve for both left and right arm end effector poses simultaneously when given a
     * nearby seed state. A random joint configuration is generated, the corresponding left/right arm end effector poses are
     * calculated and given as the target poses to the IK solver. As a seed state, the joint configuration is shifted
     * by introducing a Gaussian error to each joint value.
     * @return True if the IK solution was found.
     */
    bool bothArmsIKSeedTest();

    /**
     * Checks IK solver's ability to solve for left and right arm end effector poses using nearby poses from the database as seeds.
     * A random joint configuration is generated, and the corresponding left/right arm end effector poses are calculated and
     * given as the target poses to the IK solver. The seed states are obtained by querying the databases.
     * @return True if the IK solution was found.
     */
    bool bothArmsIKFromDatabaseTest();

  protected:
    double getGaussianRandomNumber(double mean, double std);

    nao_whole_body_ik::WholeBodyWrapperIK::Ptr whole_body_ik_;
    robot_state::RobotStatePtr robot_state_;
    boost::mt19937 rng_;
};

#endif // WHOLE_BODY_IK_SOLVER_TEST_H
