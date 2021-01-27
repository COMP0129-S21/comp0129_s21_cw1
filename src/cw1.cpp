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

#include <cw1.h>

CW1::CW1 (ros::NodeHandle &nh):
  debug_ (false)
{
  nh_ = nh;
  initParams ();
  updateParams (nh);
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::initParams ()
{
  // Frames
  this->world_frame_ = "/world_frame";
  
  // Topics
  this->robot_frame_ = "/robot_frame";
  
  // HW1-Q2: setup the names for frames {0}, {1}, {2}, and {3}
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::updateParams (ros::NodeHandle &nh)
{
  // Frames
  nh.getParam("/frame_params/world_frame", this->world_frame_);
  nh.getParam("/frame_params/robot_frame", this->robot_frame_);
}


////////////////////////////////////////////////////////////////////////////////
std::string
CW1::getWorldFrame ()
{
  return (this->world_frame_);
}

////////////////////////////////////////////////////////////////////////////////
std::string
CW1::getRobotFrame ()
{
  return (this->robot_frame_);
}

////////////////////////////////////////////////////////////////////////////////
int
CW1::kbhit()
{
  struct termios oldt, newt;
  int ch;
  int oldf;
  
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
  
  ch = getchar();
  
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
  
  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
  
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::Lab1CreateFrames ()
{
  // generate a robot frame attached to the world frame (0,0,0)
  transf_.setOrigin (tf::Vector3(0.0, 0.0, 3.0));
  transf_.setRotation (tf::Quaternion(0.0, 0.0, 0.0, 1.0)); //quaternion
  
  // Note that the rotation can be given in various forms (rotation matrix,
  // axis-angle, etc), but the function setRotation gets a tf::Quaternion,
  // thus when a different type rotation is available, it should be converted
  // to a tf::Quaternion.
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::Lab1PublishFrames ()
{
  // publish world->robot
  tranf_br_.sendTransform(tf::StampedTransform(transf_,
                                               ros::Time::now(), 
                                               world_frame_,
                                               robot_frame_));
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q1GenFrames ()
{  
  // ToDo: create transformations
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q1PubFrames ()
{
  // ToDo: publish transformations
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q2HomeRobot (ros::Publisher &joint_pub)
{
  // ToDo: setup and publish the joint_state
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q2PrintTFs ()
{
  // ToDo: lookup the transformation and print it
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q2RotJoint (ros::Publisher &joint_pub)
{
  // ToDo: setup the joint_state
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q2PubHandPose (ros::Publisher &ee_pose_pub)
{
  // ToDo: create and publish the end effector pose message
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q3AddColObj (moveit::planning_interface::PlanningSceneInterface& 
                       planning_scene_interface)
{
  // BEGIN_SUB_TUTORIAL table1
  //
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(3);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "panda_link0";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type =
    collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.2;
  collision_objects[0].primitives[0].dimensions[1] = 0.4;
  collision_objects[0].primitives[0].dimensions[2] = 0.4;

  /* Define the pose of the table: center of the cube. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.5;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = 0.2;
  // END_SUB_TUTORIAL

  collision_objects[0].operation = collision_objects[0].ADD;

  // BEGIN_SUB_TUTORIAL table2
  // Add the second table where we will be placing the cube.
  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "panda_link0";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type =
    collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.4;
  collision_objects[1].primitives[0].dimensions[1] = 0.2;
  collision_objects[1].primitives[0].dimensions[2] = 0.4;

  /* Define the pose of the table: center of the cube. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0;
  collision_objects[1].primitive_poses[0].position.y = 0.5;
  collision_objects[1].primitive_poses[0].position.z = 0.2;
  // END_SUB_TUTORIAL

  collision_objects[1].operation = collision_objects[1].ADD;

  // BEGIN_SUB_TUTORIAL object
  // Define the object that we will be manipulating
  collision_objects[2].header.frame_id = "panda_link0";
  collision_objects[2].id = "object";

  /* Define the primitive and its dimensions. */
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type =
    collision_objects[1].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.02;
  collision_objects[2].primitives[0].dimensions[1] = 0.02;
  collision_objects[2].primitives[0].dimensions[2] = 0.2;

  /* Define the pose of the object: center of the cylinder. */
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.5;
  collision_objects[2].primitive_poses[0].position.y = 0;
  collision_objects[2].primitive_poses[0].position.z = 0.5;
  // END_SUB_TUTORIAL

  collision_objects[2].operation = collision_objects[2].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q3PubObjPose (ros::Publisher &obj_pub)
{
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q3PubEeObjClose (ros::Publisher &ee_obj_close_pub)
{
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q3EeObjClosePrint (moveit::planning_interface::MoveGroupInterface& move_group)
{
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q3ReachObj (moveit::planning_interface::MoveGroupInterface& move_group)
{
  return;
}
 
////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q3SelectSideGrasp (moveit::planning_interface::MoveGroupInterface& move_group)
{
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q3SelectTopGrasp (moveit::planning_interface::MoveGroupInterface& move_group)
{
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q3GraspObj (moveit::planning_interface::MoveGroupInterface& move_group)
{
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q3MoveObj (moveit::planning_interface::MoveGroupInterface& move_group)
{
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q3ObjTouch (moveit::planning_interface::MoveGroupInterface& move_group)
{
  return;
}
