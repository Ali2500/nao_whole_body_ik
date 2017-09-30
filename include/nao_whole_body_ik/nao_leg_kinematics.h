/*
*  nao_leg_kinematics.h
*
*  Created on: 9 Apr 2016
*      Author: Ali Athar
*/

#ifndef _NAO_LEG_KINEMATICS_H_
#define _NAO_LEG_KINEMATICS_H_

#include <nao_whole_body_ik/common_functions.h>
#include <moveit/robot_state/robot_state.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/treeiksolvervel_wdls.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <stdexcept>
#include <ros/ros.h>

namespace nao_whole_body_ik
{

/**
 * The NaoLegKinematics class is a helper class that helps to adjust leg joint values of the Nao robot.
 */
class NaoLegKinematics
{

  public:
    /**
     * The ChainIKVariablesContainer struct. Container for various variable used in internal operations.
     */
    struct ChainIKVariablesContainer
    {
      public:
        KDL::JntArray delta_q;
        KDL::Frames frames;
        KDL::Twists delta_twists;
        unsigned int maxiter;
        double eps;
    };

    /**
     * Constructor.
     * @param robot_state
     */
    NaoLegKinematics(const robot_state::RobotStatePtr& robot_state);

    /**
      * Default destructor
      */
    ~NaoLegKinematics();

    /**
     * Computes an IK solutation that will yield the specified target pose using the specified seed state.
     * @param target_pose Desired pose for the left foot w.r.t right foot (i.e. right_foot_T_left_foot)
     * @param seed_state A seed state for the IK solver
     * @param ik_solution The solution, if successfully computed, is populated in this vector.
     * @return true if the IK query was successful.
     */
    bool adjustLegJointValues(const Eigen::Affine3d& target_pose, const std::vector<double>& seed_state,
                              std::vector<double>& ik_solution);

    /**
     * @brief Computes an IK solutation that will yield the specified target pose using the specified current state as
     * seed state..
     * @param original_values The current joint configuration that needs to be adjusted.
     * @param fixed_joint_indices List of joint indexes whosr values are locked i.e. they remain unchanged by the IK solver.
     * @param leg_joint_values The solution, if successfully computed, is populated in this vector.
     * @return true if the IK query was successful.
     */
    bool adjustLegJointValues(const std::vector<double>& original_values, const std::vector<int>& fixed_joint_indices,
                              std::vector<double>& leg_joint_values);

  private:
    double calculateLegIKSolutionIteratively(const KDL::JntArray& q_init, const KDL::Frames& p_in, KDL::JntArray& q_out);

    KDL::JntArray min_joint_limits_;
    KDL::JntArray max_joint_limits_;
    boost::scoped_ptr<KDL::TreeIkSolverVel_wdls> leg_chain_vik_;
    boost::scoped_ptr<KDL::TreeFkSolverPos_recursive> leg_chain_fk_;
    ChainIKVariablesContainer chain_ik_vars_;
};

typedef boost::shared_ptr<NaoLegKinematics> NaoLegKinematicsPtr;

}

#endif
