#include <nao_whole_body_ik/whole_body_ik_wrapper.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh;

  //initialize the IK solver with the param name of the robot description on the param server
  nao_whole_body_ik::WholeBodyWrapperIK ik_solver("robot_description_right",
                                                  nao_whole_body_ik::NaoWholeBodyKinematics::RIGHT_ARM_ONLY,
                                                  true /* load databases */);

  //get the robot state so we can play around with it
  robot_state::RobotStatePtr robot_state = ik_solver.getRobotState();

  //set the robot to default values
  robot_state->setToDefaultValues();

  //get the right arm end effector pose
  const Eigen::Affine3d& right_arm_end_effector_pose = robot_state->getFrameTransform("r_gripper");

  //get the left foot pose
  const Eigen::Affine3d& left_foot_pose = robot_state->getFrameTransform("l_sole");

  //build target pose map
  std::map<std::string, Eigen::Affine3d> target_pose_map;
  target_pose_map["l_sole"] = left_foot_pose;
  target_pose_map["r_gripper"] = right_arm_end_effector_pose;

  //vector to store the IK solution in
  std::vector<double> ik_solution;

  //Use case #1: Apply IK using seed states from the database
  bool success = ik_solver.getWholeBodyIKSolution(target_pose_map, ik_solution);
  if(success)
    printf("Use case #1 was successful.\n");
  else
    printf("Use case #1 was unsuccessful.\n");

  //Use case #2: Apply IK by providing a seed state
  std::vector<double> seed_state;
  robot_state->copyJointGroupPositions("legs_with_right_hand", seed_state);

  //offset the joint values a bit (so we don't actually give the IK solution itself as a seed state!
  seed_state[1] += 0.1;
  seed_state[12] += 0.2;

  success = ik_solver.getWholeBodyIKSolution(target_pose_map, seed_state, ik_solution);
  if(success)
    printf("Use case #2 was successful.\n");
  else
    printf("Use case #2 was unsuccessful.\n");

  return EXIT_SUCCESS;
}

