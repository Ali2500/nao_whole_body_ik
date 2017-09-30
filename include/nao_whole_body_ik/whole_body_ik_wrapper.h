#ifndef WHOLE_BODY_IK_WRAPPER_H
#define WHOLE_BODY_IK_WRAPPER_H

#include <Eigen/Core>

#include "database_reader.h"
#include "nao_whole_body_kinematics.h"

namespace nao_whole_body_ik
{

/**
 * The WholeBodyWrapperIK class is a wrapper that encloses the DatabaseReader and NaoWholeBodyKinematics and provides an
 *  easy interface to compute IK solutions. It lacks some of the flexibility ihough, because some specific methods of the
 * underlying classes are not accessible to users of this class.
 */
class WholeBodyWrapperIK
{
  public:
    /**
     * Constructor.
     * @param robot_state_param_name Name of the param name on ROS server under which the robot description is.
     * @param arm_inclusion
     * @param load_databases If set to false, the poses from the databases will not be loaded.
     */
    WholeBodyWrapperIK(const std::string& robot_description_param_name,
                       NaoWholeBodyKinematics::ARMS_INCLUSION arm_inclusion = NaoWholeBodyKinematics::RIGHT_ARM_ONLY,
                       bool load_databases = true);

    /**
     * Constructor.
     * @param robot_state
     * @param arm_inclusion
     * @param load_databases If set to false, the poses from the databases will not be loaded.
     */
    WholeBodyWrapperIK(const robot_state::RobotStatePtr& robot_state,
                       NaoWholeBodyKinematics::ARMS_INCLUSION arm_inclusion = NaoWholeBodyKinematics::RIGHT_ARM_ONLY,
                       bool load_databases = true);

    /**
      * Default destructor.
      */
    ~WholeBodyWrapperIK();

    /**
     * Computes an IK solution for the specified end effectors and their respective poses. Internally, it first fetches
     * nearby configurations from the databases and uses each one as a seed state for the IK solver.
     * @param target_pose_map Target poses for each end effector. The target pose for "l_sole" must be specified, and also
     * "r_gripper" and "l_gripper" according to the arm_inclusion with which the class was initialized.
     * @param ik_solution he IK solution, if successfully computed, is populated here.
     * @return True if the IK solution was found.
     */
    bool getWholeBodyIKSolution(const std::map<std::string, Eigen::Affine3d>& target_pose_map, std::vector<double>& ik_solution);

    /**
     * Computes an IK solution for the specified end effectors and their respective poses by using the specified seed
     * state.
     * @param target_pose_map target_pose_map Target poses for each end effector. The target pose for "l_sole" must be specified, and also
     * "r_gripper" and "l_gripper" according to the arm_inclusion with which the class was initialized.
     * @param seed_state Seed state for the IK solver
     * @param ik_solution he IK solution, if successfully computed, is populated here.
     * @return True if the IK solution was found.
     */
    bool getWholeBodyIKSolution(const std::map<std::string, Eigen::Affine3d>& target_pose_map,
                                const std::vector<double>& seed_state, std::vector<double>& ik_solution);

    /**
     * Set the priority of each of the spatial axes of the target poses. See similar function in nao_whole_body_kinematics
     * for more details
     * @param ts_weight_map A map of the end effectors and the weight vector associated with each.
     */
    void setTSWeights(const std::map<std::string, std::vector<double> >& ts_weight_map);

    /**
     * Set a pre-initialized DatabaseReader object. This is not recommended, and is only used for unit testing to avoid
     * having to load all the databases for each new test.
     * @param database_reader
     */
    void setDatabaseReader(const DatabaseReader2Ptr& database_reader);

    /**
     * Get the robot state object
     * @return Shared pointer to robot state object
     */
    robot_state::RobotStatePtr getRobotState() const;

  private:
    void init(bool load_databases);

    std::vector<int> findClosestDatabaseIds(const Eigen::Affine3d &right_T_left);

    DatabaseReader2Ptr database_reader_;
    NaoWholeBodyKinematicsPtr nao_ik_;
    NaoWholeBodyKinematics::ARMS_INCLUSION arm_inclusion_;
    robot_state::RobotStatePtr robot_state_;
    bool databases_loaded_;
    int num_neighbors_;

  public:
    typedef boost::shared_ptr<WholeBodyWrapperIK> Ptr;
};

} //end namespace nao_whole_body_ik

#endif
