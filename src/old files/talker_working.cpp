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
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "btMatrix3x3.h"
#include <SerialStream.h>
#include <sstream>
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
geometry_msgs::PoseStamped attitude;
sensor_msgs::Imu current_attitude;
bool initial_pose_initialized = false;
float x,y,yaw, fcl=0, rcl=0, lcl=0;
unsigned char address;
unsigned char value;
float sc = 2;

float navigation_resolution_ = 1; //resolution of our navigation algorithm occupancy grid in meters.
float local_goal_padding = 0.08; //tolerance padding around the local goal position in meters.  This is how close the robot must get to the local goal position before a new goal position will be issued.
bool in_transit = false;

void send_cmd(int, int);

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
sub_imuattitude = n_.subscribe("/mavros/imu/data",1000, &GoalPlanner::attitude_callback, this);

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

 void attitude_callback(const sensor_msgs::Imu& input)
{
current_attitude = input;

geometry_msgs::Quaternion q;

//check correct conversion...is it NED -> ENU??

q.x = current_attitude.orientation.x;
q.y = -current_attitude.orientation.y;
q.z = -current_attitude.orientation.z;
q.w = current_attitude.orientation.w;

tf::Quaternion quat;

tf::quaternionMsgToTF(q, quat);
double imuroll, imupitch, imuyaw;

tf::Matrix3x3(quat).getRPY(imuroll, imupitch, imuyaw);

geometry_msgs::PoseStamped rpy;
rpy.pose.position.x = imuroll;
rpy.pose.position.y = imupitch;
rpy.pose.position.z = imuyaw;

pub_attitude.publish(rpy);

}

public:
  ros::NodeHandle n_; 
  ros::Publisher pub_local_goal_position_;
  ros::Publisher pub_global_goal_position_;
  ros::Publisher pub_initial_position_;
  ros::Publisher pub_clearance;
  ros::Publisher pub_attitude;
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
      global_goal_position.pose.position.x = initial_position.pose.position.x + 5; //set global destination 5m in front of initial position. TO-DO: Have this accept a clicked pose from RViz.
  }

  geometry_msgs::PoseStamped getNextWaypoint(const sensor_msgs::PointCloud& pointCloud){
      geometry_msgs::PoseStamped lcl_position;//our next waypoint in navigating from our initial position to the global goal position.
      bool left_occupied = false, right_occupied = false, front_occupied = false, self_occupied = false, left_front_occupied = false, right_front_occupied = false;//occupancy grid Booleans.
      float currPoseX = current_position.pose.position.x;//absolute x-position of laser.
      float currPoseY = current_position.pose.position.y;//aboslute y-position of laser.
      float currPoseYaw = 2*asin(current_position.pose.orientation.z); //transform absolute laser yaw from quaternion to radians
      float currPointXabs, currPointYabs;//absolute x and y position of current point in pointcloud.





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

rcl = right.y;
fcl = front.x;
lcl = left.y;


clearance.pose.position.x = -rcl;
clearance.pose.position.y = fcl;
clearance.pose.position.z = lcl;


pub_clearance.publish(clearance);

          //determine our occupancy grid relative to the laser based on Cartesian pointcloud data.  The coefficient in front of "navigation_resolution_" sets the bounds of how much of the block to check in.  E.g. "1.5*navigation_resolution_" is "one and a half occupancy grid blocks".
          if(currPointYabs < (currPoseY + 1.5*navigation_resolution_) && currPointYabs > (currPoseY + 0.5*navigation_resolution_) && currPointXabs > (currPoseX - 0.5*navigation_resolution_) && currPointXabs < (currPoseX + 0.5*navigation_resolution_)){
              left_occupied = true;
          }
          if(currPointYabs < (currPoseY + 1.5*navigation_resolution_) && currPointYabs > (currPoseY + 0.5*navigation_resolution_) && currPointXabs > (currPoseX + 0.5*navigation_resolution_) && currPointXabs < (currPoseX + 1.5*navigation_resolution_)){
              left_front_occupied = true;
          }
          if(currPointYabs < (currPoseY - 0.5*navigation_resolution_) && currPointYabs > (currPoseY - 1.5*navigation_resolution_) && currPointXabs > (currPoseX - 0.5*navigation_resolution_) && currPointXabs < (currPoseX + 0.5*navigation_resolution_)){
              right_occupied = true;
          }
          if(currPointYabs < (currPoseY - 0.5*navigation_resolution_) && currPointYabs > (currPoseY - 1.5*navigation_resolution_) && currPointXabs > (currPoseX + 0.5*navigation_resolution_) && currPointXabs < (currPoseX + 1.5*navigation_resolution_)){
              right_front_occupied = true;
          }
          if(currPointYabs < (currPoseY + 0.5*navigation_resolution_) && currPointYabs > (currPoseY - 0.5*navigation_resolution_) && currPointXabs > (currPoseX + 0.5*navigation_resolution_) && currPointXabs < (currPoseX + 1.5*navigation_resolution_)){
              front_occupied = true;
          }
          if(currPointYabs < (currPoseY + 0.5*navigation_resolution_) && currPointYabs > (currPoseY - 0.5*navigation_resolution_) && currPointXabs > (currPoseX - 0.5*navigation_resolution_) && currPointXabs < (currPoseX + 0.5*navigation_resolution_)){
              self_occupied = true;
          }
      }

      //DEBUG.  Use this block to check how previous for-loop is building the local occupancy grid.
      std_msgs::String msg;

      std::stringstream ss;
      ss << "left: " << left_occupied;
      ss << ", left-front: " << left_front_occupied;
      ss << ", right: " << right_occupied;
      ss << ", right-front: " << right_front_occupied;
      ss << ", front: " << front_occupied;
      ss << ", self: " << self_occupied;
      //\DEBUG

      //(END A)------------------------------

      //(START B)------------------------------
      // Determine angular bearing from current position to global goal position.
      double global_goal_bearing = getGlobalGoalBearing(currPoseX, currPoseY);
      //(END B)--------------------------------
// 0x45 is zero. fwd is 0x38. right is 0x51. left is 0x38.
      //(START C)------------------------------
      // Determine which adjacent block to move into.  This is the grid-based control algorithm.
      if(!front_occupied){
          //go forward
          ss << ", go forward";
          lcl_position = goForward(currPoseX, currPoseY, currPoseYaw);
address = 0x84;
value = 40;
send_cmd(address,value);
      }else if(front_occupied){
          if(global_goal_bearing < 0){
              if(!left_front_occupied && !left_occupied){
                  //go left
                  ss << ", go left";
                  lcl_position = goLeft(currPoseX, currPoseY, currPoseYaw);
address = 0x85;
value = 40;
send_cmd(address,value);
              }else if(!right_front_occupied && !right_occupied){
                  //go right
                  ss << ", go right";
                  lcl_position = goRight(currPoseX, currPoseY, currPoseYaw);
address = 0x85;
value = 40;
send_cmd(address,value);
              }else if(!left_occupied){
                  //go left
                  ss << ", go left";
                  lcl_position = goLeft(currPoseX, currPoseY, currPoseYaw);
address = 0x85;
value = 40;
send_cmd(address,value);
              }else if(!right_occupied){
                  //go right
                  ss << ", go right";
                  lcl_position = goRight(currPoseX, currPoseY, currPoseYaw);
address = 0x85;
value = 40;
send_cmd(address,value);
              }else{
                  ss <<", boxed in, turn -90";
                  lcl_position = Turn(-90);
address = 0x83;
value = 40;
send_cmd(address,value);
              }
          }else if(global_goal_bearing > 0){
              if(!right_front_occupied && !right_occupied){
                  //go right
                  ss << ", go right";
                  lcl_position = goRight(currPoseX, currPoseY, currPoseYaw);
address = 0x85;
value = 40;
send_cmd(address,value);
             }else if(!left_front_occupied && !left_occupied){
                  //go left
                  ss << ", go left";
                  lcl_position = goLeft(currPoseX, currPoseY, currPoseYaw);
address = 0x85;
value = 40;
send_cmd(address,value);
             }else if(!right_occupied){
                  //go right
                  ss << ", go right";
                  lcl_position = goRight(currPoseX, currPoseY, currPoseYaw);
address = 0x85;
value = 40;
send_cmd(address,value);
              }else if(!left_occupied){
                  //go left
                  ss << ", go left";
                  lcl_position = goLeft(currPoseX, currPoseY, currPoseYaw);
address = 0x85;
value = 40;
send_cmd(address,value);
              }else{
                  ss <<", boxed in, turn 90";
                  lcl_position = Turn(90);
address = 0x83;
value = 40;
send_cmd(address,value);
              }
          }
      }

      //(END C)--------------------------------

      //DEBUG
      msg.data = ss.str();

      ROS_INFO("%s", msg.data.c_str());
      //\DEBUG

      return lcl_position;
 
}

  geometry_msgs::PoseStamped goLeft(float currPoseX, float currPoseY, float currPoseYaw){
      geometry_msgs::PoseStamped returnPose;

      returnPose = current_position;
      returnPose.pose.position.y = currPoseY + sc*navigation_resolution_*cos(currPoseYaw);
      if(currPoseYaw > 0){
          returnPose.pose.position.x = currPoseX + sc*navigation_resolution_*sin(currPoseYaw);
      }else{
          returnPose.pose.position.x = currPoseX - sc*navigation_resolution_*sin(currPoseYaw);
      }
      returnPose.pose.orientation.z = sin(0);// transform yaw from radians to quaternion.

      return returnPose;
  }

  geometry_msgs::PoseStamped goRight(float currPoseX, float currPoseY, float currPoseYaw){
      geometry_msgs::PoseStamped returnPose;

      returnPose = current_position;
      returnPose.pose.position.y = currPoseY - sc*navigation_resolution_*cos(currPoseYaw);
      if(currPoseYaw > 0){
          returnPose.pose.position.x = currPoseX - sc*navigation_resolution_*sin(currPoseYaw);
      }else{
          returnPose.pose.position.x = currPoseX + sc*navigation_resolution_*sin(currPoseYaw);
      }
      returnPose.pose.orientation.z = sin(0);// transform yaw from radians to quaternion.

      return returnPose;
  }

  geometry_msgs::PoseStamped goForward(float currPoseX, float currPoseY, float currPoseYaw){
      geometry_msgs::PoseStamped returnPose;

      //set returnPose equal to current position and then adjust x & y offsets to move the drone forward.
      returnPose = current_position;
      if(currPoseYaw > 0){
          returnPose.pose.position.y = currPoseY - sc*navigation_resolution_*sin(currPoseYaw);
      }else{
          returnPose.pose.position.y = currPoseY + sc*navigation_resolution_*sin(currPoseYaw);
      }
      returnPose.pose.position.x = currPoseX + sc*navigation_resolution_*cos(currPoseYaw);
      returnPose.pose.orientation.z = sin(0);// transform yaw from radians to quaternion.

      return returnPose;
  }

  geometry_msgs::PoseStamped Turn(float degrees){
      geometry_msgs::PoseStamped returnPose = current_position;
      double radians = (3.14159/180)*degrees;

      returnPose.pose.orientation.z = sin(radians);// transform yaw from radians to quaternion.

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


int fd;
int open_port(void)
{
	int fd; // file description for the serial port
	
	fd = open("/dev/ttyUSB0", O_RDWR| O_NONBLOCK | O_NDELAY);
	
	if(fd == -1) // if open is unsucessful
	{
		//perror("open_port: Unable to open /dev/ttyS0 - ");
		printf("open_port: Unable to open /dev/ttyUSB0. \n");
		//return 0;
	}

	if ( fd < 0 )
    	{
        	//cout << "Error " << errno << " opening " << "/dev/ttyUSB0" << ": " << strerror (errno) 			<< endl;
        	perror("USB ");
    	}

	else
	{
		fcntl(fd, F_SETFL, 0);
		printf("port is open.\n");
	}
	
	return(fd);
} //open_port



int configure_port(int fd)      // configure the port
{
	struct termios port_settings;      // structure to store the port settings in

	cfsetispeed(&port_settings, B57600);    // set baud rates
	cfsetospeed(&port_settings, B57600);

	port_settings.c_cflag &= ~PARENB;    // set no parity, stop bits, data bits
	port_settings.c_cflag &= ~CSTOPB;
	port_settings.c_cflag &= ~CSIZE;
	port_settings.c_cflag |= CS8;
	
	tcsetattr(fd, TCSANOW, &port_settings);    // apply the settings to the port
	return(fd);

} //configure_port

void send_cmd(int cmd, int value)
{
	unsigned char send_bytes[4];
	// header byte
	send_bytes[0]=0x81;
	// throttle=0x82, yaw=0x83, pitch=0x84, roll=0x85, mode=0x86;
	send_bytes[1]=cmd;
	// value bayte	
	send_bytes[2]=value;
	//CRC byte
	send_bytes[3]=send_bytes[0] ^ send_bytes[1] ^ send_bytes[2]; 
	write(fd, send_bytes,4);
	printf("done\n");
}


int main(int argc, char **argv)
{
	fd = open_port();
	configure_port(fd);

  //Initiate ROS
  ros::init(argc, argv, "goal_planner");
ros::NodeHandle n_;
  //Create an object of class GoalPlanner that will take care of everything
  GoalPlanner GPObject;

	

ros::Publisher pub_local_goal_position_;
pub_local_goal_position_ = n_.advertise<geometry_msgs::PoseStamped>("/local_goal_position", 1);

ros::spin();

  return 0;
}
/*
[n] 1.) Get navigation bearing from first slam_out_pose quaternion orientation.
[y] 2.) Get slam_cloud.
[y] 3.) Create occupancy grid relative to base_link.
[y] 4.) Check if any points from slam_cloud exist within occupancy grid squares.
[y] 5.) Set any occupied and unoccupied squares.
[n] 6.) Apply navigation algorithm to get desired destination square.
[n] 7.) Map desired destination square to absolute map position.
[n] 8.) Publish desired PoseStamped pose (abs position and navigation bearing) on goal_position topic.
[n] 9.) Test if current slam_out_pose matches goal_position (+- tolerance).  If it matches then we have arrived and can repeat the process starting at (2).
















*/
