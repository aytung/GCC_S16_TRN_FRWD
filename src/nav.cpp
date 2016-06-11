#ifndef NAV_CPP
#define NAV_CPP
#include <turtlebot/mymsg.h>
#include "nav.h"
#include "ros/ros.h"
#include <string>
#include <cstring>
#include <math.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <stdio.h>

// The amount of error in rotation we consider acceptable
const double ANGLE_ERR = 7;
// how much we need in velocity commands to move INCREMENT_AMT forward
// The amount of angular.z we publish to rotate turtlebot
const double ROTATION_VELOCITY = .7;
// The amount of linear.x we publish move turtlebot forward
const double FORWARD_VELOCITY = 0.2;
// how much we need to multiply radians by in order to get degrees
const double ANGLE_CONVERT = 57.2958;



// initializer for RoboState::State (with default values)
RoboState::RoboState(ros::NodeHandle rosNode): xCoord(0), yCoord(0), yaw(0), yawGoal(0), count(0), currentState(NEUTRAL), internalCount(0), acceptErr(.03)
{
  // declare which ROS node we use
  this->node = rosNode;
  // publishes velocity messages
  this->velocityPublisher = this->node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  // subscribes to messages from ui_node
  this->messageSubscriber= this->node.subscribe("my_msg", 1000, &RoboState::messageCallback, this);
  // subscribes to data from bumpers
  this->bumperSubscriber = this->node.subscribe("/mobile_base/sensors/core", 100, &RoboState::bumperCallback, this);
  // subscribes to encoders
  this->odomSubscriber = this->node.subscribe("/odom", 100, &RoboState::odomCallback, this);
}

// @return Whether or not we are done with rotation
// face the final destination, which is determined by yawGoal
bool RoboState::faceDestination()
{
  // By default, assume that we are not done with movement
  bool done = false;

  ROS_INFO("The yaw goal is set to %f", getYawGoal());
  ROS_INFO("Calling face destination.");
  
  // How much we are the desired direction
  double yawOffset = getYawGoal() - getYaw();
  
  // Our yawOffset was not within acceptable bounds
  if(yawOffset <= -ANGLE_ERR || yawOffset >= ANGLE_ERR){
    this->velocityCommand.linear.x = 0.0;
    // We rotate to the left
    if(yawOffset > 0){
      this->velocityCommand.angular.z = ROTATION_VELOCITY;
    }
    // We rotate to the right
    else{
      this->velocityCommand.angular.z = -ROTATION_VELOCITY;
    }
    
  }
  else {
    ROS_INFO("We are done with rotation.");
    
    // Don't move
    this->velocityCommand.linear.x = 0.0;
    this->velocityCommand.angular.z = 0.0;

    done = true;
  }

  velocityPublisher.publish(this->velocityCommand);
  
  return done;
}


// @return Whether or not we are done with forward mvoement
bool RoboState::goForward()
{
  bool done = false;
  
  // How much we are from the destination
  // Basic distance formula
  double offset = sqrt(pow(getX()-getXodom(),2) + pow(getY()-getYodom(),2));

  // Our offset was not within acceptable bounds
  if(offset <= -getErr() || offset >= getErr()){
    ROS_INFO("Moving forward because we are off by %f.", offset);
    this->velocityCommand.linear.x = FORWARD_VELOCITY;
  }
  // Our offset was within acceptable bounds
  else{
    ROS_INFO("Done with forward movement in the X direction.");
    this->velocityCommand.linear.x = 0.0;
    done = true;
  }
  

  this->velocityCommand.angular.z = 0.0;    
  velocityPublisher.publish(this->velocityCommand);

  return done;
}

// The function that we call whenever we recieve odometry data from turtlebot
void RoboState::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  
  // set odometry
  setXodom(msg->pose.pose.position.x);
  setYodom(msg->pose.pose.position.y);
  
  // convert from quaternions to angles
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, tempYaw;
  m.getRPY(roll, pitch, tempYaw);
  
  tempYaw*=ANGLE_CONVERT;
  // determine whether to send positive or negative angles
  bool isTempYawNeg = (tempYaw < 0);
  
  // we always want a positive yaw value (for consistency)
  if(isTempYawNeg){
    setYaw(360+tempYaw);
  }
  else{
    setYaw(tempYaw);
  }


  // we do not want to get spammed with messages about yaw.
  if(getInternalCount() % 100==1){
    ROS_INFO("The yaw was %f", getYaw());

    if(getY() >= 0){
      ROS_INFO("The yaw is supposed to be positive.");
    }
    else{
      ROS_INFO("The yaw is supposed to be negative.");
    }
  }
}

// Function that is called whenever we receive a message from ui_node
// If this function does not make sense, refer to the diagram at top of code
void RoboState::messageCallback(const turtlebot::mymsg::ConstPtr& msg)
{
  // only accept message if movement is not in progress
  if(getCurrentState()==NEUTRAL)
    {
	double messageX = msg->x;
	double messageY = msg->y;  
      if(msg->x==0 && msg->y==0){
	ROS_INFO("No reason to move a distance of 0. Message not sent.");
	}
      else{	  
	ROS_INFO("X and Y coordinates sent were: x:%f y:%f", msg->x, msg->y);

        if(messageX > 0){
          if(messageY == 0){
	    // Means we only want forward movement
          setYawGoal(0);
          }
          else if(messageY < 0){
	    // In order to go from negative to positive
          setYawGoal(360+atan(msg->x/msg->y)*ANGLE_CONVERT);
          }
          else{
	    // Arctangent works as expected
          setYawGoal(atan(msg->x/msg->y)*ANGLE_CONVERT);
          }
        }
        else if(messageX < 0){
	  // Means we only need to move backward
          if(messageY == 0){
          setYawGoal(180);
          }
          else{
	    // What we use to convert from below y axis to positive values
	    // that make sense
          setYawGoal(180+atan(msg->x/msg->y)*ANGLE_CONVERT);
          }
        }
        else if(messageX == 0){
          if(messageY > 0){
	    // Means that we only need to face +y
          setYawGoal(90);
          }
          else{
	    // Means that we only need to face -y
          setYawGoal(270);
          }
        }
      } 
        
	ROS_INFO("The yaw goal is set to %f", getYawGoal());

	// Need to account for prior movement
	setX(msg->x + getXodom());
	setY(msg->y + getYodom());

	ROS_INFO("xCoord is: %f. yCoord is: %f", getX(), getY());
	
	// Start off by rotating first
	setCurrentState(FACE_DESTINATION);


	setErr(.1);
      }


    
    
  else{
    ROS_INFO("Cannot accept message. Movement still in progress.");
  }
}

// Function that is called whenever we receive information from bumpers
void RoboState::bumperCallback(const create_node::TurtlebotSensorState::ConstPtr& msg)
{
  // if bumpers don't complain, don't run the loop
  if(msg->bumps_wheeldrops != 0){
    ROS_INFO("You hit an object! Motion terminating.");
    ROS_INFO("The remaining x was:%f and the remaining y was: %f.", getX(), getY());
    setX(0);
    setY(0);
    // allow RoboState::State to receive messages again
    setCurrentState(NEUTRAL);
  }
}
  
/*
  Section for dealing with internalCount

*/

void RoboState::incrementInternalCount()
{
  internalCount++;
}

int RoboState::getInternalCount()
{
  return internalCount;
}

/* 

   Get and set functions

*/

void RoboState::setCurrentState(State newState)
{
  currentState = newState;
}

State RoboState::getCurrentState()
{
  return currentState;
}

double RoboState::getX()
{
  return xCoord;
}

void RoboState::setX(double x)
{
  xCoord=x;
}

void RoboState::setY(double y)
{
  yCoord=y;
}

double RoboState::getY()
{
  return yCoord;
}

double RoboState::getXodom()
{
  return xOdom;
}

double RoboState::getYodom()
{
  return yOdom;
}

void RoboState::setXodom(double xOdomCurrent)
{
  xOdom = xOdomCurrent;
}

void RoboState::setYodom(double yOdomCurrent)
{
  yOdom = yOdomCurrent;
}

double RoboState::getYodomOld()
{
  return yOdomOld;
}

double RoboState::getXodomOld()
{
  return xOdomOld;
}

void RoboState::setYodomOld(double yOdomCurrent)
{
  yOdomOld = yOdomCurrent;
}

void RoboState::setXodomOld(double xOdomCurrent)
{
  xOdomOld = xOdomCurrent;
}

double RoboState::getErr()
{
  return acceptErr;
}

void RoboState::setErr(double err)
{
  acceptErr = err;
}

void RoboState::setYaw(double newYaw)
{
  yaw = newYaw;
}

double RoboState::getYaw()
{
  return yaw;
}

void RoboState::setYawGoal(double newYawGoal)
{
  yawGoal = newYawGoal;
}

double RoboState::getYawGoal()
{
  return yawGoal;
}
 
#endif
