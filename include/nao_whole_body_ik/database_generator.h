/*
* 	database_generator.h
*
*   Created on: 26 Apr 2016
*      Author: Ali Athar
*/
#ifndef _DATABASE_GENERATOR_2_H_
#define _DATABASE_GENERATOR_2_H_

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <nao_whole_body_ik/nao_leg_kinematics.h>
#include <nao_whole_body_ik/common_functions.h>
#include <boost/thread.hpp>
#include <fstream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <ros/package.h>
#include <flann/flann.hpp>
#include <flann/io/hdf5.h>

namespace nao_whole_body_ik
{

/**
 * The DatabaseGenerator class creates databases of various valid, collision-free robot poses. These poses are then
 * saved to files inside the <package>/database directory.
 */
class DatabaseGenerator
{
  public:
    /**
     * Default constructor
     */
    DatabaseGenerator();

    /**
      * Default destructor
      */
    ~DatabaseGenerator();

    /**
     * Begin generation of database. This may take up to an hour depending on the system
     * specifications and the parameters.
     */
    void generateMultiFootDatabase();

  private:

    struct RecursionVariables
    {
        boost::thread thread_;
        planning_scene::PlanningScenePtr planning_scene_;
        robot_state::RobotStatePtr robot_state_;
        NaoLegKinematicsPtr nao_leg_kinematics_;

        std::vector<double> joint_values_;
        std::vector<std::vector<double> > joint_configs_;
        std::vector<std::vector<double> > cartesian_configs_;
        std::vector<std::vector<double> > joint_configs_left_;
        std::vector<std::vector<double> > cartesian_configs_left_;

        int total_iters_;
        int valid_configs_;
        int valid_configs_left_;
        int ik_fails_;
        int depth_;
        bool is_done_;
    };

    void generateDatabaseRecursively(int thread_id);
    void startRecursion(int thread_id);

    std::vector<double> joint_interval_;
    std::vector<std::pair<double, double> > right_arm_joint_limits_;
    std::vector<std::string> database_file_names_, database_file_names_left_;
    std::vector<Eigen::Affine3d> left_foot_poses_;

    //used when computing combinations recursively:
    int iterator_;
    unsigned int num_threads_;
    RecursionVariables* recurs_vars_;
    std::vector<double> safe_right_arm_values_, safe_left_arm_values_;
};

}

#endif
