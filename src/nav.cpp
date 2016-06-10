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
// the least amount the turtlebot can move, otherwise odometry breaks;
// serves as our acceptable error
const double ANGLE_ERR = 3.5;
// how much we need in velocity commands to move INCREMENT_AMT forward
const double MOVEMENT_MULTIPLE = 1.8; 
// how much we move forward/backward each increment
const double INCREMENT_AMT = .1;
// how much angular velocity we need to move right 90 degrees
const double LEFT_90 = 2.54629;
// how much angular velocity we need to move right 90 degrees
const double RIGHT_90 = -2.56;

const double ROTATION_VELOCITY = .8;

const double FORWARD_VELOCITY = 0.2;

// how much we need to multiply radians by in order to get degrees
const double ANGLE_CONVERT = 57.2958;



// initializer for RoboState::State (with default values)
RoboState::RoboState(ros::NodeHandle rosNode): xCoord(0), yCoord(0), xOdomOld(0), yOdomOld(0), yaw(0), yawGoal(0), count(0), currentState(NEUTRAL), internalCount(0), acceptErr(.1)
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

// face the final destination, which is determined by yawGoal
bool RoboState::faceDestination()
{
  bool done = false;
  ROS_INFO("Calling face destination.");
  double yawOffset = getYawGoal() - getYaw();
  
  if(yawOffset <= -ANGLE_ERR || yawOffset >= ANGLE_ERR){
    this->velocityCommand.linear.x = 0.0;
    if(yawOffset > 0){
    this->velocityCommand.angular.z = ROTATION_VELOCITY;
    }
    else{
      this->velocityCommand.angular.z = -ROTATION_VELOCITY;
    }
    
  }
  else {
    this->velocityCommand.linear.x = 0.0;
    this->velocityCommand.angular.z = 0.0;

    done = true;
  }
velocityPublisher.publish(this->velocityCommand);
  
  return done;
}



bool RoboState::goForward()
{
  bool done = false;
  
  double offset = sqrt(pow(getX()-getXodom(),2) + pow(getY()-getYodom(),2));

if(offset <= -getErr() || offset >= getErr()){
      ROS_INFO("Moving forward because we are off by %f.", offset);
      this->velocityCommand.linear.x = FORWARD_VELOCITY;
    }
    else{
      ROS_INFO("Done with forward movement in the X direction.");
      this->velocityCommand.linear.x = 0.0;
      done = true;
    }
  

  this->velocityCommand.angular.z = 0.0;    
      velocityPublisher.publish(this->velocityCommand);
  return done;
}

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

void RoboState::messageCallback(const turtlebot::mymsg::ConstPtr& msg)
{
  // only accept message if movement is not in progress
  if(getCurrentState()==NEUTRAL)
    {
      if(msg->x==0 && msg->y==0)
	ROS_INFO("No reason to move a distance of 0. Message not sent.");
      else{	  
	ROS_INFO("X and Y coordinates sent were: x:%f y:%f", msg->x, msg->y);

	setYawGoal(atan(msg->x/msg->y)*ANGLE_CONVERT);

	setX(msg->x + getXodom());
	setY(msg->y + getYodom());
	ROS_INFO("xCoord is: %f. yCoord is: %f", getX(), getY());
	
	// we don't need to face backward since initial movement is forward
	setCurrentState(FACE_DESTINATION);


	  setErr(.1);
      }
      // need to determine what direction we will ultimately face

      }
    
  else{
    ROS_INFO("Cannot accept message. Movement still in progress.");
  }
}

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

bool RoboState::currentCountOdd()
{
  if(getInternalCount()%2==1)
    return true;
  else
    return false;
}

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
