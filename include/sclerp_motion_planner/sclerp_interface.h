/* Author: Dasharadhan Mahalingam */

#pragma once

#include <ros/ros.h>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

#include <limits>
#include <vector>
#include <Eigen/Geometry>

#include "kinlib/kinlib_kinematics.h"
#include "kinlib/collision_utils.h"

namespace sclerp_interface
{

class ScLERPInterface
{
  public:
    ScLERPInterface();
    
    /*!
      \brief    Constructor to initialize class object
      
      \details  Initializes the kinematic model of the manipulator for which
                we need to plan the motion
                
      \param    base_link   Name of the manipulator base link
      \param    tip_link    Name of the manipulator tip link
      \param    nh          Node handle
      \param    urdf_path   Initializing with local urdf file if provided
    */
    ScLERPInterface(const std::string base_link,
                    const std::string tip_link,
                    const ros::NodeHandle &nh = ros::NodeHandle("~"),
                    const std::string &urdf_path = "");

    /*!
      \brief    To plan motion between initial and goal states

      \details  Returns the sequence of joint angles required for moving the
                manipulator from the given initial state to the required
                goal state

      \param    init_jnt_values   Initial joint encoder values of the
                                  manipulator
      \param    g_f               Required final pose of the manipulator's
                                  end-effector
      \param    jnt_trajectory    The required motion plan

      \return   Success/Failure of motion plan determination
    */
    bool solve( const Eigen::VectorXd &init_jnt_values,
                const Eigen::Matrix4d &g_f,
                trajectory_msgs::JointTrajectory &jnt_trajectory);
                
    /*!
      \brief    To plan motion between initial and goal states

      \details  Returns the sequence of joint angles required for moving the
                manipulator from the given initial state to the required
                goal state

      \param    init_jnt_values   Initial joint encoder values of the
                                  manipulator
      \param    g_f               Required final pose of the manipulator's
                                  end-effector
      \param    jnt_trajectory    The required motion plan
      \param    ee_trajectory     End effector trajectory

      \return   Success/Failure of motion plan determination
    */
    bool solve( const Eigen::VectorXd &init_jnt_values,
                const Eigen::Matrix4d &g_f,
                trajectory_msgs::JointTrajectory &jnt_trajectory,
                std::vector<geometry_msgs::Pose> &ee_trajectory);

    /*!
      \brief    To plan motion between initial and goal states while 
                considering collision avoidance

      \details  Returns the sequence of joint angles required for moving the
                manipulator from the given initial state to the required
                goal state

      \param    init_jnt_values   Initial joint encoder values of the
                                  manipulator
      \param    g_f               Required final pose of the manipulator's
                                  end-effector
      \param    num_links_ignore    Number of manipulator links to ignore.
      \param    obstacles           Collection of obstacles in the environment.
      \param    grasped_object      Grasped object (if any).
      \param    jnt_trajectory      Variable to store motion plan

      \return   Success/Failure of motion plan determination
    */
    bool solve( const Eigen::VectorXd &init_jnt_values,
                const Eigen::Matrix4d &g_f,
                const int num_links_ignore,
                const std::vector<std::shared_ptr<CollisionUtils::ObstacleBase>> &obstacles,
                const std::shared_ptr<CollisionUtils::ObstacleBase> &grasped_object,
                trajectory_msgs::JointTrajectory &jnt_trajectory);

    /*!
      \brief    The kinematic solver for the manipulator
    */
    kinlib::KinematicsSolver kinlib_solver_;

  protected:
    ros::NodeHandle nh_;
    std::string name_;    

  private:
  
    urdf::Model urdf_model;
    
    KDL::Tree robot_tree;
    KDL::Chain manip_chain;
    
    
    std::string robot_desc_string;
    
    std::map<std::string, urdf::JointSharedPtr> jnt_list;
    
    std::string base_link_name;
    std::string tip_link_name;
    std::string local_urdf_file;
    
    Eigen::Matrix4d t_ref;
    Eigen::Matrix4d t_jnt;
    Eigen::Matrix4d t_tip;

    Eigen::Vector4d p_jnt;
    Eigen::Vector4d v_jnt;
    
  
    bool init_failed;
};

} // namespace sclerp_interface