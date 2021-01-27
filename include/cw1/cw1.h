/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021-, Dimitrios Kanoulas
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef CW1_H_
#define CW1_H_

#include <ros/ros.h>

#include <iostream>
#include <vector>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

// HW1 Includes
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>

// cw1 Includes
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/** \brief HW 2.
  *
  * \author Dimitrios Kanoulas
  */
class CW1
{
  public:
    /** \brief Empty constructor.
      *
      * \input[in] nh the ROS node
      */
    CW1 (ros::NodeHandle &nh);
    
    /** \brief Lab 1: initialize the parameters. */
    void
    initParams ();
    
    /** \brief Load the parameters. */
    void
    updateParams (ros::NodeHandle &nh);
    
    /** \brief Lab 1: get the world frame.
      *
      * \return the world frame string
      */
    std::string
    getWorldFrame ();
    
    /** \brief Lab 1: get the robot frame.
      *
      * \return the robot frame string
      */
    std::string
    getRobotFrame ();
    
    /** \brief Lab 1: create the transformation between robot & world
     *  frames. 
     */
    void
    Lab1CreateFrames ();
    
    /** \brief Lab 1: publish the transformations between robot & world
      * frames.
      */
    void
    Lab1PublishFrames ();
    
    /** \brief CW1: helper function to implement a non-blocking key getter.
      *
      * \return 1 for key pressed, 0 for no pressed
      */
    int
    kbhit ();
    
    /** \brief CW1 Q1: create the transformations between frames {1}, {2},
     *  {3}, and {4}. 
     */
    void
    cw1Q1GenFrames ();
    
    /** \brief CW1 Q1: publish the transformations between frames {1}, {2},
     *  {3}, and {4}. 
     */
    void
    cw1Q1PubFrames ();
    
    /** \brief CW1 Q2: home the robot. 
     */
    void
    cw1Q2HomeRobot (ros::Publisher &joint_pub);
    
    /** \brief CW1 Q2: print the TFs.
     */
    void
    cw1Q2PrintTFs ();
    
    /** \brief CW1 Q2: rotate panda_joint2.
     */
    void
    cw1Q2RotJoint (ros::Publisher &joint_pub);
    
    /** \brief CW1 Q2: hand pose publisher.
     */
    void
    cw1Q2PubHandPose (ros::Publisher &ee_pose_pub);
    
    /** \brief CW1 Q2: add collision objects: table1, table2, object
      *
      * \input planning_scene_interface the MoveIt! PlanningSceneInterface
      */
    void
    cw1Q3AddColObj (moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);
    
    /** \brief CW1 Q3: publish the object's pose.
      *
      * \input obj_pub the object publisher
      */
    void
    cw1Q3PubObjPose (ros::Publisher &obj_pub);

    /** \brief CW1 Q3: publishing the ee_obj_close value.
      *
      * \input ee_obj_close_pub the ee-object close publisher
      */
    void
    cw1Q3PubEeObjClose (ros::Publisher &ee_obj_close_pub);
    
    /** \brief CW1 Q3: print the ee_obj_close. */
    void    
    cw1Q3EeObjClosePrint (moveit::planning_interface::MoveGroupInterface& move_group);
    
    /** \brief CW1 Q3: Reach object. */
    void
    cw1Q3ReachObj (moveit::planning_interface::MoveGroupInterface& move_group);
    
    /** \brief CW1 Q3: Select side grasp. */
    void
    cw1Q3SelectSideGrasp (moveit::planning_interface::MoveGroupInterface& move_group);
    
    /** \brief CW1 Q3: Select top grasp. */
    void
    cw1Q3SelectTopGrasp (moveit::planning_interface::MoveGroupInterface& move_group);
    
    /** \brief CW1 Q3: Grasp object. */
    void
    cw1Q3GraspObj (moveit::planning_interface::MoveGroupInterface& move_group);
    
    /** \brief CW1 Q3: Move object. */
    void
    cw1Q3MoveObj (moveit::planning_interface::MoveGroupInterface& move_group);
    
    /** \brief CW1 Q3: Detects if object is touching the table. */
    void    
    cw1Q3ObjTouch (moveit::planning_interface::MoveGroupInterface& move_group);
    
  public:
    /** \brief Node handle. */
    ros::NodeHandle nh_;
    
    /** \brief World and robot frames. */
    std::string world_frame_, robot_frame_;
    
    /** \brief Lab 1: TF transforms definition. */
    tf::Transform transf_;
    
    /** \brief Lab 1: TF transform broadcaster definitions. */
    tf::TransformBroadcaster tranf_br_; 
    
    /** \brief HW1-Q2: frame names definitions. */
    std::string frame_0_, frame_1_, frame_2_, frame_3_;
    
    /** \brief HW1-Q2: TF transforms definitions. */
    tf::StampedTransform transf_01_, transf_02_, transf_03_, transf_23_;
        
    /** \brief HW1-Q2: TF transform broadcaster definitions. */
    tf::TransformBroadcaster transf_01_br_, transf_02_br_, transf_03_br_;
    
    /** \brief HW1-Q2: TF listener definition. */
    tf::TransformListener listener_;
    
    /** \brief HW1-Q3: TF transforms definitions. */
    tf::StampedTransform q3_transf_0h_, q3_transf_2h_;
    
    /** \brief HW1-Q3: TF listener definition. */
    tf::TransformListener q3_listener_;
    
    /** \brief HW1-Q3: Joint state. */
    sensor_msgs::JointState joint_state_;
    
    
    /** \brief cw1-Q1: */
    bool ee_obj_close_;
    
  protected:
    /** \brief Debug mode. */
    bool debug_;
};
#endif
