/*
*  nao_whole_body_kinematics.h
*
*  Created on: 8 Apr 2016
*      Author: Ali Athar
*/

#ifndef _NAO_WHOLE_BODY_KINEMATICS_H_
#define _NAO_WHOLE_BODY_KINEMATICS_H_

#include <nao_whole_body_ik/common_functions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <kdl/treeiksolvervel_wdls.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <stdexcept>
#include <ros/ros.h>

namespace nao_whole_body_ik
{

/**
 * The NaoWholeBodyKinematics class is the main class used for computing whole body IK solutions. It wraps aound Tree
 * velocity IK solvers provided by KDL.
 */
class NaoWholeBodyKinematics
{
  public:

    /**
     * The ARMS_INCLUSION enum specifies whether an IK solver including only the right arm, left arm, or both is required.
     */
    enum ARMS_INCLUSION
    {
      RIGHT_ARM_ONLY = 0, LEFT_ARM_ONLY, BOTH_ARMS
    };

    /**
     * Container for variables used by internal operations.
     */
    struct TreeIKVariablesContainer
    {
      public:
        KDL::JntArray delta_q;
        KDL::Frames frames;
        KDL::Twists delta_twists;
        unsigned int maxiter;
        double eps;
    };

    /**
     * Constructor
     * @param robot_state
     * @param arm_inclusion
     */
    NaoWholeBodyKinematics(const robot_state::RobotStatePtr& robot_state, ARMS_INCLUSION arm_inclusion);

    /**
      * Default destructor
      */
    ~NaoWholeBodyKinematics();

    /**
     * Sets the weight for the importance of each of the 6 spatial axes (x, y, z, roll, pitch, yaw) for the end effector
     * attached to the right arm.
     * @param ts_weight A vector of 6 double type numbers in the range [0, 1] to indicate the degree of priority of each
     * spatial axis. For example if the yaw angle is irrelevant and all others are equally important, (1,1,1,1,1,0) should
     * be given. If all rotational axes are irrelevant, then (1,1,1,0,0,0) should be gien. By default, each axis has
     * equal importance and is equal to 1. Values between 0 and 1 will result in comparatively larger errors in the IK
     * solution for that axis.
     */
    void setRightArmTSWeight(const std::vector<double>& ts_weight);

    /**
     * Sets the weight for the importance of each of the 6 spatial axes (x, y, z, roll, pitch, yaw) for the end effector
     * attached to the left arm.
     * @param ts_weight A vector of 6 double type numbers in the range [0, 1] to indicate the degree of priority of each
     * spatial axis. For example if the yaw angle is irrelevant and all others are equally important, (1,1,1,1,1,0) should
     * be given. If all rotational axes are irrelevant, then (1,1,1,0,0,0) should be gien. By default, each axis has
     * equal importance and is equal to 1. Values between 0 and 1 will result in comparatively larger errors in the IK
     * solution for that axis.
     */
    void setLeftArmTSWeight(const std::vector<double>& ts_weight);

    /**
     * Sets the weight for the importance of each of the 6 spatial axes (x, y, z, roll, pitch, yaw) for the end effector
     * attached to the left leg.
     * @param ts_weight A vector of 6 double type numbers in the range [0, 1] to indicate the degree of priority of each
     * spatial axis. For example if the yaw angle is irrelevant and all others are equally important, (1,1,1,1,1,0) should
     * be given. If all rotational axes are irrelevant, then (1,1,1,0,0,0) should be gien. By default, each axis has
     * equal importance and is equal to 1. Values between 0 and 1 will result in comparatively larger errors in the IK
     * solution for that axis.
     */
    void setLeftFootTSWeight(const std::vector<double>& ts_weight);

    /**
     * Computes a forward kinematics (FK) solution for the provided joint configuration. The following order should
     * be used for joint values [right ankle --> right hip, left hip --> left ankle, right shoulder --> right wrist,
     * left shoulder --> left wrist]. For RIGHT_ARM_ONLY or LEFT_ARM_ONLY mode, the other arm's values should be omitted.
     * @param joint_values List of joint values in the appropriate order.
     * @param eef_name Name of the enf effector whose pose has to be found.
     * @param f_out The calculated pose is populated here.
     */
    void getWholeBodyFKSolution(const std::vector<double>& joint_values, const std::string& eef_name,
                                KDL::Frame& f_out);

    /**
     * Compures an Inverse Kinematics (IK) solution for the poses of the end effectors.
     * @param target_pose_map A map with the name and desired pose for all end effectors. Target poses for "l_sole" and
     * the required arm end effectors should be specified.
     * @param seed_state An initial state to serve as the seed for the IK solver.
     * @param joint_values The IK solution, if successfully computed, is populated here.
     * @return True if the IK solution was found.
     */
    bool getWholeBodyIKSolution(const std::map<std::string, Eigen::Affine3d>& target_pose_map,
                                const std::vector<double>& seed_state, std::vector<double>& joint_values);

    /**
     * Forces the IK solver to not alter the right arm's joint values when trying to compute a solution.
     */
    void lockRightArmJoints();

    /**
     * Forces the IK solver to not alter the right arm's joint values when trying to compute a solution.
     */
    void lockLeftArmJoints();

    /**
     * Makes the IK solver ignore orientation constraints in the target pose for right arm's end effector.
     */
    inline void freeRightArmOrientationConstraint();

    /**
     * Makes the IK solver ignore orientation constraints in the target pose for left arm's end effector.
     */
    inline void freeLeftArmOrientationConstraint();

    /**
     * Makes the IK solver ignore orientation constraints in the target pose for left foot's end effector.
     */
    inline void freeLeftFootOrientationConstraint();

  private:
    double calculateWholeBodyIKSolutionIteratively(const KDL::JntArray& q_init, const KDL::Frames& p_in,
                                                   KDL::JntArray& q_out);

    robot_state::RobotStatePtr robot_state_;
    KDL::Tree nao_kinematic_tree_;
    boost::scoped_ptr<KDL::TreeIkSolverVel_wdls> nao_tree_vik_;
    boost::scoped_ptr<KDL::TreeFkSolverPos_recursive> nao_tree_fk_;
    ARMS_INCLUSION arm_inclusion_;
    std::vector<std::string> end_effector_names_;
    KDL::JntArray min_joint_limits_, max_joint_limits_;
    TreeIKVariablesContainer tree_ik_vars_;
};

typedef boost::shared_ptr<NaoWholeBodyKinematics> NaoWholeBodyKinematicsPtr;

inline void NaoWholeBodyKinematics::freeRightArmOrientationConstraint()
{
  std::vector<double> ts_weight(6, 1.0);
  ts_weight[3] = ts_weight[4] = ts_weight[5] = 0.0;
  setRightArmTSWeight(ts_weight);
}

inline void NaoWholeBodyKinematics::freeLeftArmOrientationConstraint()
{
  std::vector<double> ts_weight(6, 1.0);
  ts_weight[3] = ts_weight[4] = ts_weight[5] = 0.0;
  setLeftArmTSWeight(ts_weight);
}

inline void NaoWholeBodyKinematics::freeLeftFootOrientationConstraint()
{
  std::vector<double> ts_weight(6, 1.0);
  ts_weight[3] = ts_weight[4] = ts_weight[5] = 0.0;
  setLeftFootTSWeight(ts_weight);
}

}

#endif
