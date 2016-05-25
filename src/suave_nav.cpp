/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "btMatrix3x3.h"
#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitionss
#include <time.h>   // time calls

int initial_pose_counter = 0;
geometry_msgs::PoseStamped initial_position;
geometry_msgs::PoseStamped current_position;
geometry_msgs::PoseStamped global_goal_position;
geometry_msgs::PoseStamped local_goal_position;
geometry_msgs::PoseStamped clearance;
geometry_msgs::PoseStamped rpy;
geometry_msgs::PoseStamped nxtway;


tf::Quaternion current_attitude;
bool initial_pose_initialized = false;
float x,y,x_prev=0, y_prev = 0, yaw_prev = 0, xdot = 0, ydot = 0, yawdot = 0,  yaw, holdx, holdy, fcl=0, rcl=0, lcl=0, del_phi = 0, del_th = 0, del_yaw = 0, yddlim = 0.05, xddlim = 0.15, yawddlim = 15;
float kp_theta = 28;  // roll
float kd_theta = 18;
float kp_phi = 23;    // pitch
float kd_phi = 15;
float kp_psi = 65;    // yaw
float kd_psi = 0;
double imuroll=0, imupitch=0, imuyaw=0;
double timestamp = 0;
float last_wp_x = 0, last_wp_y = 0, last_wp_yaw = 0, wp_x_dot = 0, wp_y_dot = 0, wp_yaw_dot = 0;
unsigned char state = 0;
float sc_x = 0.307;
float sc_y = 0.25;
int flag1 = 0, flag2 = 0, flag3 = 0, flagTR = 0;
int fleft = 0;
int fright = 0;
int ffwd = 0;
int value_p = 64;
int value_r = 64;
int value_y = 64;
float tr = 13.16;	// x position of target room (meters)
float openingYlow = -1.95;
float openingYhigh = 0.3;
float brc = 1.83; 	// obstacle b corner
float leftgo = 3.25; 	// go left this much
float gcupper = 1.1;	// grid clearance
float gclower = 0.1; 


int th_des = 0, phi_des = 0, yaw_des = 0;
float dt = 0.1, nxtX, nxtY, nxtYaw;

float navigation_resolution_x = 1.30; //resolution of our navigation algorithm occupancy grid in meters.
float navigation_resolution_y = 1.30; //resolution of our navigation algorithm occupancy grid in meters.
float local_goal_padding = 0.08; //tolerance padding around the local goal position in meters.  This is how close the robot must get to the local goal position before a new goal position will be issued.
bool in_transit = false;

class GoalPlanner
{
public:
  GoalPlanner()
  {
    //goal_position topic publisher.  This is the position we will move to as determined by the navigation algorithm.
    pub_local_goal_position_ = n_.advertise<geometry_msgs::PoseStamped>("/local_goal_position", 1);
    pub_global_goal_position_ = n_.advertise<geometry_msgs::PoseStamped>("/global_goal_position",1);
pub_clearance = n_.advertise<geometry_msgs::PoseStamped>("/clearance",1);

pub_attitude = n_.advertise<geometry_msgs::PoseStamped>("/attitude",1);


    //DEBUG. Used to see if we are adhering to our desired trajectory.
    pub_initial_position_ = n_.advertise<geometry_msgs::PoseStamped>("/initial_position",1);

    //slam_cloud topic subscription.
    sub_slam_cloud_ = n_.subscribe("/slam_cloud", 1, &GoalPlanner::slam_cloud_callback, this);

    //Topic you want to subscribe
    sub_slam_out_pose_ = n_.subscribe("/slam_out_pose", 1, &GoalPlanner::slam_out_pose_callback, this);
sub_next_way = n_.subscribe("/next_way",1, &GoalPlanner::wp_callback, this);
    
  }

//&& !in_transit 

  void slam_cloud_callback(const sensor_msgs::PointCloud& input)
  {
      if(initial_pose_initialized && !bool_arrived_at_global_goal()){ //if initial pose has been determined //(and we are not moving), then we can determine the next waypoint goal.
          local_goal_position = getNextWaypoint(input);
      }
      in_transit = bool_in_transit(local_goal_position);
      pub_local_goal_position_.publish(local_goal_position);

  }
  


  void slam_out_pose_callback(const geometry_msgs::PoseStamped& input)
  {
      current_position = input;//save current pose for use in other locations of the program.
      if(!initial_pose_initialized){//if inital pose has not been determined, determine it.
          getInitialPose(input);
          getDestinationPose(); //get global_goal_position
      }
      pub_initial_position_.publish(initial_position);//DEBUG
      pub_global_goal_position_.publish(global_goal_position);

  }


void wp_callback(const geometry_msgs::PoseStamped& input)
{
nxtX = input.pose.position.x;
nxtY = input.pose.position.y;
nxtYaw = input.pose.position.z;
}


public:
  ros::NodeHandle n_; 
  ros::Publisher pub_local_goal_position_;
  ros::Publisher pub_global_goal_position_;
  ros::Publisher pub_initial_position_;
  ros::Publisher pub_clearance;
  ros::Publisher pub_attitude;


  ros::Subscriber sub_next_way;
  ros::Subscriber sub_slam_cloud_;
  ros::Subscriber sub_slam_out_pose_;
  ros::Subscriber sub_imuattitude;

  void getInitialPose(const geometry_msgs::PoseStamped& input){
      if(initial_pose_counter < 10){ //lock initial pose after ten iterations to allow noise to settle.
          initial_position = input;
          initial_pose_initialized = true;
          initial_pose_counter++;
      }
  }

  void getDestinationPose(){
      global_goal_position = initial_position;
      global_goal_position.pose.position.x = initial_position.pose.position.x + 30; //set global destination 30m in front of initial position. TO-DO: Have this accept a clicked pose from RViz.
  }


  geometry_msgs::PoseStamped getNextWaypoint(const sensor_msgs::PointCloud& pointCloud){
      geometry_msgs::PoseStamped lcl_position;//our next waypoint in navigating from our initial position to the global goal position.
      bool left_occupied = false, right_occupied = false, front_occupied = false, self_occupied = false, left_front_occupied = false, right_front_occupied = false;//occupancy grid Booleans.
      float currPoseX = current_position.pose.position.x;//absolute x-position of laser.
      float currPoseY = current_position.pose.position.y;//absolute y-position of laser.
      float currPoseYaw = 2*asin(current_position.pose.orientation.z); //transform absolute laser yaw from quaternion to radians
      float currPointXabs, currPointYabs;//absolute x and y position of current point in pointcloud.

 timestamp = current_position.header.stamp.sec;




      //(START A)------------------------------
      // Determine whether adjacent block to left, right, right-front, left-front, and front of laser are occupied.  Change occupancy grid size with "navigation_resolution_" global variable.
      for(int i = 0; i<pointCloud.points.size(); i++){//iterate through all points in pointcloud.
          const geometry_msgs::Point32& currPoint(pointCloud.points[i]);//get current point in pointcloud.

          //calculate absolute position of current point by offsetting it relative to the absolute position of the laser.
          currPointXabs = currPoint.x + currPoseX;
          currPointYabs = currPoint.y + currPoseY;

int j = pointCloud.points.size();

const geometry_msgs::Point32& right(pointCloud.points[0]);
const geometry_msgs::Point32& front(pointCloud.points[j/2]);
const geometry_msgs::Point32& left(pointCloud.points[j-5]);

rcl = -right.y;
fcl = front.x;
lcl = left.y;


clearance.pose.position.x = rcl;
clearance.pose.position.y = fcl;
clearance.pose.position.z = lcl;


pub_clearance.publish(clearance);

          //determine our occupancy grid relative to the laser based on Cartesian pointcloud data.  The coefficient in front of "navigation_resolution_" sets the bounds of how much of the block to check in.  E.g. "1.5*navigation_resolution_" is "one and a half occupancy grid blocks".
          if(currPointYabs < (currPoseY + gcupper*navigation_resolution_y) && currPointYabs > (currPoseY + gclower*navigation_resolution_y) && currPointXabs > (currPoseX - gclower*navigation_resolution_x) && currPointXabs < (currPoseX + gclower*navigation_resolution_x)){
              left_occupied = true;
          }
          if(currPointYabs < (currPoseY + gcupper*navigation_resolution_y) && currPointYabs > (currPoseY + gclower*navigation_resolution_y) && currPointXabs > (currPoseX + gclower*navigation_resolution_x) && currPointXabs < (currPoseX + gcupper*navigation_resolution_x)){
              left_front_occupied = true;
          }
          if(currPointYabs < (currPoseY - gclower*navigation_resolution_y) && currPointYabs > (currPoseY - gcupper*navigation_resolution_y) && currPointXabs > (currPoseX - gclower*navigation_resolution_x) && currPointXabs < (currPoseX + gclower*navigation_resolution_x)){
              right_occupied = true;
          }
          if(currPointYabs < (currPoseY - gclower*navigation_resolution_y) && currPointYabs > (currPoseY - gcupper*navigation_resolution_y) && currPointXabs > (currPoseX + gclower*navigation_resolution_x) && currPointXabs < (currPoseX + gcupper*navigation_resolution_x)){
              right_front_occupied = true;
          }
          if(currPointYabs < (currPoseY + gclower*navigation_resolution_y) && currPointYabs > (currPoseY - gclower*navigation_resolution_y) && currPointXabs > (currPoseX + gclower*navigation_resolution_x) && currPointXabs < (currPoseX + gcupper*navigation_resolution_x)){
              front_occupied = true;
          }
          if(currPointYabs < (currPoseY + gclower*navigation_resolution_y) && currPointYabs > (currPoseY - gclower*navigation_resolution_y) && currPointXabs > (currPoseX - gclower*navigation_resolution_x) && currPointXabs < (currPoseX + gclower*navigation_resolution_x)){
              self_occupied = true;
          }
      }


      //DEBUG.  Use this block to check how previous for-loop is building the local occupancy grid.
      std_msgs::String msg;

      std::stringstream ss;

      //\DEBUG

      //(END A)------------------------------

      //(START B)------------------------------
      // Determine angular bearing from current position to global goal position.
      double global_goal_bearing = getGlobalGoalBearing(currPoseX, currPoseY);
      //(END B)--------------------------------



     
//(Algorithm FRONT)-------------------------------------------------------------------------------STRAIGHT FORWARD BEGINS---------------------------------------------------------------------------------------------
 if (flagTR == 0)
{

if ((!front_occupied) && (flag1 == 0) && (flag2 == 0) && (flag3 == 0))
{
//go forward
          ss << ", go forward";
          lcl_position = goForward(currPoseX, currPoseY, currPoseYaw);
}

if (front_occupied)
{
flag1 = 1;
}


if ((flag1 == 1) && (flag2 == 0) && (flag3 == 0))
{
                  //go right
                  ss << ", go right";
                  lcl_position = goRight(currPoseX, currPoseY, currPoseYaw);
}

if ((rcl < 2.5) && (!front_occupied) && (flag1 == 1))
{
flag2 = 1;
}

if ((flag1 == 1) && (flag2 == 1) && (flag3 == 0))
{
//go forward
          ss << ", go forward";
          lcl_position = goForward(currPoseX, currPoseY, currPoseYaw);
}

if ((front_occupied) && (flag1 == 1) && (flag2 == 1))
{
flag3 = 1;
}

if ((flag1 == 1) && (flag2 == 1) && (flag3 == 1) && (rcl < 3.5)){
                  //go left
          ss << ", go left";
                  lcl_position = goLeft(currPoseX, currPoseY, currPoseYaw);
}

if ((!front_occupied) && (flag1 == 1) && (flag2 == 1) && (flag3 == 1) && (rcl > 3.5))
{
//go forward
          ss << ", go forward";
          lcl_position = goForward(currPoseX, currPoseY, currPoseYaw);
}

if (currPoseX >= tr)
{//reached target room
flagTR = 1;
}
}

              
//(END Algorithm FRONT)-------------------------------------------------------------------------------STRAIGHT FORWARD ENDS-------------------------------------------------------------------------------------------


// (SWITCH TO TARGET ESTIMATION)
if (flagTR == 1)
{

lcl_position.pose.position.x = currPoseX;
lcl_position.pose.position.y = currPoseY;
lcl_position.pose.orientation.z = currPoseYaw;
return lcl_position;
 
}
}
  geometry_msgs::PoseStamped goLeft(float currPoseX, float currPoseY, float currPoseYaw){
      geometry_msgs::PoseStamped returnPose;

      returnPose = current_position;

if (fleft == 0)
{
holdx = returnPose.pose.position.x;
fleft = fleft+1;
}
      returnPose.pose.position.y = currPoseY + sc_y*navigation_resolution_y*cos(currPoseYaw);
      if(currPoseYaw > 0){
          //returnPose.pose.position.x = currPoseX + sc*navigation_resolution_*sin(currPoseYaw);
returnPose.pose.position.x = holdx;// + sc*navigation_resolution_*sin(currPoseYaw);
      }else{
          //returnPose.pose.position.x = currPoseX - sc*navigation_resolution_*sin(currPoseYaw);
returnPose.pose.position.x = holdx;// - sc*navigation_resolution_*sin(currPoseYaw);
      }
      returnPose.pose.orientation.z = sin(0);// transform yaw from radians to quaternion.
fright = 0;
ffwd = 0;

//returnPose.pose.position.x = initial_position.pose.position.x;
//returnPose.pose.position.y = initial_position.pose.position.y;

      return returnPose;
  }

  geometry_msgs::PoseStamped goRight(float currPoseX, float currPoseY, float currPoseYaw){
      geometry_msgs::PoseStamped returnPose;

      returnPose = current_position;
if (fright == 0)
{
holdx = returnPose.pose.position.x;
fright = fright+1;
}      returnPose.pose.position.y = currPoseY - sc_y*navigation_resolution_y*cos(currPoseYaw);
      if(currPoseYaw > 0){
         // returnPose.pose.position.x = currPoseX - sc*navigation_resolution_*sin(currPoseYaw);
returnPose.pose.position.x = holdx;// - sc*navigation_resolution_*sin(currPoseYaw);
      }else{
         // returnPose.pose.position.x = currPoseX + sc*navigation_resolution_*sin(currPoseYaw);
returnPose.pose.position.x = holdx;// + sc*navigation_resolution_*sin(currPoseYaw);
      }
      returnPose.pose.orientation.z = sin(0);// transform yaw from radians to quaternion.
fleft = 0;
ffwd = 0;

//returnPose.pose.position.x = initial_position.pose.position.x;
//returnPose.pose.position.y = initial_position.pose.position.y;

      return returnPose;
  }

  geometry_msgs::PoseStamped goForward(float currPoseX, float currPoseY, float currPoseYaw){
      geometry_msgs::PoseStamped returnPose;

      //set returnPose equal to current position and then adjust x & y offsets to move the drone forward.
      returnPose = current_position;
if (ffwd == 0)
{
holdy = returnPose.pose.position.y;
ffwd = ffwd+1;
}
      if(currPoseYaw > 0){
         // returnPose.pose.position.y = currPoseY - sc*navigation_resolution_*sin(currPoseYaw);
returnPose.pose.position.y = holdy;// - sc*navigation_resolution_*sin(currPoseYaw);
      }else{
         // returnPose.pose.position.y = currPoseY + sc*navigation_resolution_*sin(currPoseYaw);
returnPose.pose.position.y = holdy;// + sc*navigation_resolution_*sin(currPoseYaw);
      }
      returnPose.pose.position.x = currPoseX + sc_x*navigation_resolution_x*cos(currPoseYaw);
      returnPose.pose.orientation.z = sin(0);// transform yaw from radians to quaternion.
fright = 0;
fleft = 0;

//returnPose.pose.position.x = initial_position.pose.position.x;
//returnPose.pose.position.y = initial_position.pose.position.y;

      return returnPose;
  }

  geometry_msgs::PoseStamped Turn(float degrees){
      geometry_msgs::PoseStamped returnPose = current_position;
      double radians = (3.14159/180)*degrees;

      returnPose.pose.orientation.z = sin(radians);// transform yaw from radians to quaternion.

returnPose.pose.position.x = initial_position.pose.position.x;
returnPose.pose.position.y = initial_position.pose.position.y;

      return returnPose;
  }


  double getGlobalGoalBearing(float PosX, float PosY){// X & Y position from which to calculate the angular bearing to the global goal destination.
      return -atan(PosY/(global_goal_position.pose.position.x - PosX));
  }

  bool bool_in_transit(geometry_msgs::PoseStamped& lcl_position){
      bool response = false;
      float lgpX = lcl_position.pose.position.x;
      float lgpY = lcl_position.pose.position.y;
      float lgpYaw = lcl_position.pose.orientation.z;

      float cpX = current_position.pose.position.x;
      float cpY = current_position.pose.position.y;
      float cpYaw = current_position.pose.orientation.z;

      if(std::abs(lgpX - cpX) < local_goal_padding && std::abs(lgpY - cpY) < local_goal_padding && std::abs(lgpYaw - cpYaw) < local_goal_padding/8){//checking to see if current position matches local goal position within certain tolerance.  If so, issue new waypoint.
          response = false;
      }else{
          response = true;
      }
	//bool response = true;
      return response;
  }

  bool bool_arrived_at_global_goal(){
      bool response = false;
      float ggpX = global_goal_position.pose.position.x;
      float ggpY = global_goal_position.pose.position.y;
      float ggpYaw = global_goal_position.pose.orientation.z;

      float cpX = current_position.pose.position.x;
      float cpY = current_position.pose.position.y;
      float cpYaw = current_position.pose.orientation.z;

      if(cpX < ggpX){
          response = false;
      }else{
          response = true;
      }

      return response;
  }

};//End of class GoalPlanner


int main(int argc, char **argv)
{

ros::init(argc, argv, "goal_planner");
ros::NodeHandle n_;

//Create an object of class GoalPlanner that will take care of everything
  GoalPlanner GPObject;

	

ros::Publisher pub_local_goal_position_;
pub_local_goal_position_ = n_.advertise<geometry_msgs::PoseStamped>("/local_goal_position", 1);

ros::spin();

  return 0;
}
