/*
*  global_functions.h
*
*  Created on: 8 Apr 2016
*      Author: Ali Athar
*/
#ifndef _COMMON_FUNCTIONS_H_
#define _COMMON_FUNCTIONS_H_

#include <ros/ros.h>
#include <vector>
#include <map>
#include <iostream>
#include <stdexcept>
#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>
#include <kdl/treeiksolvervel_wdls.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treeiksolvervel_wdls.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

namespace nao_whole_body_ik
{
class CommonFunctions
{
  public:
    /**
     * Convert Eigen Affine frame to KDL frame
     * @param affine_frame
     * @param kdl_frame
     */
    static void EigenAffineToKDLFrame(const Eigen::Affine3d& affine_frame, KDL::Frame& kdl_frame)
    {
      KDL::Vector vector(affine_frame.translation()[0], affine_frame.translation()[1], affine_frame.translation()[2]);
      KDL::Rotation rotation(affine_frame.matrix()(0,0), affine_frame.matrix()(0,1), affine_frame.matrix()(0,2),
                             affine_frame.matrix()(1,0), affine_frame.matrix()(1,1), affine_frame.matrix()(1,2),
                             affine_frame.matrix()(2,0), affine_frame.matrix()(2,1), affine_frame.matrix()(2,2));
      kdl_frame.p = vector;
      kdl_frame.M = rotation;
    }

    /**
     * Convert KDL Frame to Eigen Affine
     * @param kdl_frame
     * @param affine_frame
     */
    static void KDLFrameToEigenAffine(const KDL::Frame& kdl_frame, Eigen::Affine3d& affine_frame)
    {
      for(int i = 0; i < 4; i++)
      {
        for(int j = 0; j < 4; j++)
        {
          affine_frame.matrix()(i, j) = kdl_frame(i, j);
        }
      }
    }

    /**
     * Convert a KDL Joint Array to an STL vector of double type
     * @param kdl_array
     * @param stl_vector
     */
    static void KDLJntArrayToSTLVector(const KDL::JntArray& kdl_array, std::vector<double>& stl_vector)
    {
      stl_vector.resize(kdl_array.rows());
      for(int i = 0; i < kdl_array.rows(); i++)
      {
        stl_vector[i] = kdl_array(i);
      }
    }

    /**
     * Convert an STL vector of double type to a KDL Joint Array
     * @param stl_vector
     * @param kdl_array
     */
    static void STLVectorToKDLJntArray(const std::vector<double>& stl_vector, KDL::JntArray& kdl_array)
    {
      kdl_array.resize((int)stl_vector.size());
      for(int i = 0; i < stl_vector.size(); i++)
      {
        kdl_array(i) = stl_vector[i];
      }
    }

    /**
     * Convert an STL vector of the form [x, y, yaw] to KDL frame
     * @param foot_pose_vector
     * @param foot_pose_frame
     */
    static void STLVectorToKDLFrame(const std::vector<double>& foot_pose_vector, KDL::Frame& foot_pose_frame)
    {
      assert ((int)foot_pose_vector.size() == 3);
      foot_pose_frame.p = KDL::Vector(foot_pose_vector[0], foot_pose_vector[1], 0.0);
      foot_pose_frame.M = KDL::Rotation::RPY(0.0, 0.0, foot_pose_vector[2]);
    }

    /**
     * Convert a KDL Frame to an STL vector of the form [x, y, yaw]
     * @param foot_pose_frame
     * @param foot_pose_vector
     */
    static void KDLFrameToSTLVector(const KDL::Frame& foot_pose_frame, std::vector<double>& foot_pose_vector)
    {
      foot_pose_vector.resize(3);
      double roll, pitch, yaw;
      foot_pose_frame.M.GetRPY(roll, pitch, yaw);
      foot_pose_vector[0] = foot_pose_frame.p(0);
      foot_pose_vector[1] = foot_pose_frame.p(1);
      foot_pose_vector[2] = yaw;
    }

    /**
     * Convert an Eigen Affine transformation to an STL vector of the form [x, y, theta]
     * @param eigen_foot_pose
     * @param vector_foot_pose
     */
    static void EigenAffineToSTLVector(const Eigen::Affine3d& eigen_foot_pose, std::vector<double>& vector_foot_pose)
    {
      KDL::Frame foot_pose_frame;
      EigenAffineToKDLFrame(eigen_foot_pose, foot_pose_frame);
      KDLFrameToSTLVector(foot_pose_frame, vector_foot_pose);
    }

    /**
     * Convert an Eigen Affine transformation to an STL Vector of the form [x, y, z, roll, pitch, yaw]
     * @param eigen_pose
     * @param vector_pose
     */
    static void EigenAffineToSTLVector2(const Eigen::Affine3d& eigen_pose, std::vector<double>& vector_pose)
    {
      vector_pose.resize(6);
      Eigen::Vector3d translation = eigen_pose.translation();
      Eigen::Vector3d rotation = eigen_pose.rotation().eulerAngles(0, 1, 2);
      for(std::size_t i = 0; i < 3; ++i)
      {
        vector_pose[i] = translation[i];
        vector_pose[i+3] = rotation[i];
      }
    }

    template <class T>
    /**
     * Generic function for printing elements of an STL vector. Normally useful for debugging
     * @param heading Text label to apply before printing the elements.
     * @param vect The vector whose elements to print
     * @param change_line If true, each element will be printed onto a separate line.
     */
    static void printSTLVector(const std::string& heading, const std::vector<T>& vect, bool change_line)
    {
      std::cout << heading << ": ";
      if (change_line)
      {
        std::cout << std::endl;
      }

      for(int i = 0; i < (int)vect.size(); i++)
      {
        std::cout << vect[i];
        if (change_line)
        {
          std::cout << std::endl;
        }
        else
        {
          std::cout << ", ";
        }
      }

      if (!change_line)
      {
        std::cout << std::endl;
      }
    }

};
}

#endif
