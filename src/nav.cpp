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
#define DEBUG 0
// the least amount the turtlebot can move, otherwise odometry breaks;
// serves as our acceptable error
const double ANGLE_ERR = 15;
// how much we need in velocity commands to move INCREMENT_AMT forward
const double MOVEMENT_MULTIPLE = 1.8; 
// how much we move forward/backward each increment
const double INCREMENT_AMT = .1;
// how much angular velocity we need to move right 90 degrees
const double LEFT_90 = 2.54629;
// how much angular velocity we need to move right 90 degrees
const double RIGHT_90 = -2.56;
// this is how long the TurtleBot takes to move INCREMENT_AMT distance forward
const double MOVEMENT_INTERVAL = 500000;
// how much we need to multiply radians by in order to get degrees
const double ANGLE_CONVERT = 57.2958;
// approximately Pi, used to indicate around 180 degrees (must be under pi, incase of overshooting
const double YAW_180 = 3.1415;

const double MIN_LEFT = .4;

const double MIN_RIGHT = -.4;

// initializer for RoboState::State (with default values)
RoboState::RoboState(ros::NodeHandle rosNode): xCoord(0), yCoord(0), xOdomOld(0), yOdomOld(0), yaw(0), yawGoal(0), count(0), currentState(NEUTRAL), internalCount(0)
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



void RoboState::turn_neg_x_1_2()
{
  rotateLeft_90();
  setCurrentState(TURN_NEG_X_2_2);
}

void RoboState::turn_neg_x_2_2()
{
  rotateLeft_90();
  setCurrentState(MOVE_FORWARD_X);
}


// move forward in x direction and update xCoord
void RoboState::goForwardX()
{
  // we move forward INCREMENT_AMT since we have to go
  // further than that 
  if(getX() <= -INCREMENT_AMT || getX() >= INCREMENT_AMT )
    {
      double xMoveCommand; 
      usleep(MOVEMENT_INTERVAL);
      
      // move forward
      xMoveCommand = INCREMENT_AMT*MOVEMENT_MULTIPLE;
      this->velocityCommand.linear.x = xMoveCommand;
      this->velocityCommand.angular.z = 0.0;
      velocityPublisher.publish(this->velocityCommand);

      // update xCoord
      double currentXodom = getXodom();
      double amountMoved = currentXodom-getXodomOld();
      setX(getX()-amountMoved);
      setXodomOld(currentXodom);
      usleep(MOVEMENT_INTERVAL/10);

      ROS_INFO("We moved %f", amountMoved);
      ROS_INFO("The remaining amount to move is %f", getX());
    }
  else if(getX() <= getErr() && getX() >= -getErr()){
    // our x coord is 'good enough'
    setCurrentState(FACE_DESTINATION);
  }
  else{
    ROS_INFO("Moving forward by %f.", getX());
    
    usleep(MOVEMENT_INTERVAL);

    // move forward
    double xMoveCommand = abs(getX())*MOVEMENT_MULTIPLE;
    this->velocityCommand.linear.x = xMoveCommand;
    this->velocityCommand.angular.z = 0.0;
    velocityPublisher.publish(this->velocityCommand);
    ROS_INFO("We moved forward %f", getX());

    // update xCoord
    double currentXodom = getXodom();
    double amountMoved = currentXodom-getXodomOld();
    setX(getX()-amountMoved);
    setXodomOld(currentXodom);
    usleep(MOVEMENT_INTERVAL/10);
    ROS_INFO("The remaining amount to move is %f (should be around zero)", getX());
  }
}

// face the final destination, which is determined by yawGoal
void RoboState::faceDestination()
{
  ROS_INFO("Calling face destination.");
  
  // how much we are off the goal
  double yawOffset = getYawGoal() - getYaw();

  // means that we are off by more than 90 degrees
  if(getY() > 0 && !initialXnegative) {
    ROS_INFO("We are turning left because we were off by %f)", yawOffset);
    ROS_INFO("(Should be more than 90 degrees)");
<<<<<<< HEAD
    rotateLeft_90();
    ROS_INFO("Assume that we turned 90 degrees to the leftproperly.");
    setCurrentState(MOVE_FORWARD_Y);
=======
#if !DEBUG
    rotateLeft_90();
#endif
>>>>>>> origin
  }
  // means that we are off by less than -90 degrees
  else if(getY() < 0 && !initialXnegative){
    ROS_INFO("We are turning right because yawOffset was %f", yawOffset);
    ROS_INFO("(Should be less than -90 degrees)");
<<<<<<< HEAD
    rotateRight_90();
    ROS_INFO("Assume that we turned 90 degrees to the right properly.");
    setCurrentState(MOVE_FORWARD_Y);
  }
  else if(getY() > 0 && initialXnegative){
    rotateRight_90();
    setCurrentState(MOVE_FORWARD_Y);
=======
#if !DEBUG   
    rotateRight_90();
#endif
>>>>>>> origin
  }
  else if(getY() < 0 && initialXnegative){
    rotateLeft_90();
    setCurrentState(MOVE_FORWARD_Y);
  }
  else{
    setCurrentState(NEUTRAL);
  }
  /*
  // means that we are off by between 90 and -90 degrees
  else if(yawOffset >= ANGLE_ERR || yawOffset <= -ANGLE_ERR){
    ROS_INFO("We are turning less because yawOffset was %f", yawOffset);
    ROS_INFO("(Should be between 90 and -90 degrees)");
    usleep(MOVEMENT_INTERVAL);
    this->velocityCommand.linear.x = 0;
    if(yawOffset >= 0){
      this->velocityCommand.angular.z = MIN_LEFT;
    }
    else
      this->velocityCommand.angular.z = MIN_RIGHT;


    velocityPublisher.publish(this->velocityCommand);
	  
  }
  else{ // means are we are close enough to facing 180 degrees 
    ROS_INFO("We should be facing %f degrees.", getYawGoal());
    setCurrentState(MOVE_FORWARD_Y);
  }
  */
}


// move forward in y axis and update yCoord
void RoboState::goForwardY()
{
 
  // we move forward by INCREMENT_AMT since we are still that amount
  // or further from the destination, and we want accurate, consistent
  // movement
  if(getY() <= -INCREMENT_AMT || getY() >= INCREMENT_AMT )
    {
      ROS_INFO("Moving forward by INCREMENT_AMT.");
      
      usleep(MOVEMENT_INTERVAL);
      
      // move forward by INCREMENT_AMT
      double xMoveCommand; 
      xMoveCommand = INCREMENT_AMT*MOVEMENT_MULTIPLE;
      this->velocityCommand.linear.x = xMoveCommand;
      this->velocityCommand.angular.z = 0.0;
      velocityPublisher.publish(this->velocityCommand);
      
      // update yCoord
      double currentYodom = getYodom();
      double amountMoved = currentYodom-getYodomOld();
      setY(getY()-amountMoved);
      setYodomOld(currentYodom);
      usleep(MOVEMENT_INTERVAL/10);

      ROS_INFO("We moved %f", amountMoved);
      ROS_INFO("The remaining amount to move is %f", getY());
    }
  else if(getY() <= getErr() && getY() >= -getErr()){
    // yCoord is 'good enough'
    ROS_INFO("We are done with y-movement.");
    setCurrentState(NEUTRAL);
  }
  else{
    ROS_INFO("Moving forward by %f.", getY());

    double xMoveCommand = abs(getY())*MOVEMENT_MULTIPLE;
    usleep(MOVEMENT_INTERVAL);
    this->velocityCommand.linear.x = xMoveCommand;
    this->velocityCommand.angular.z = 0.0;
    velocityPublisher.publish(this->velocityCommand);

    // update yCoord
    double currentYodom = getXodom();
    double amountMoved = currentYodom-getYodomOld();
    setY(getY()-amountMoved);
    setYodomOld(currentYodom);
    ROS_INFO("The remaining amount to move is %f (should be around zero)", getY());
  }
}

// means that we need to face backwards, since initial x was negative
// and that initial y was positive
void RoboState::rotate_180()
{
  // yaw is always positive, so we have to subtrace from positive 180 degrees
  double yawOffset = 0;
  
  // compensate for whether yaw is positive or negative
  if(getY() >= 0)
    yawOffset = 180 - getYaw();
  else
    yawOffset = -180 - getYaw();

  // means that we are off by more than 90 degrees
  if(yawOffset >= 90){
    ROS_INFO("We are turning because we were off by %f)", yawOffset);
    ROS_INFO("(Should be more than %f degrees)", ANGLE_ERR);
<<<<<<< HEAD

    rotateLeft();

=======
#if !DEBUG
    rotateLeft_90();
#endif
>>>>>>> origin
  }
  // means that we are off by less than -90 degrees
  else if(yawOffset <= -90){
    ROS_INFO("We are turning right because yawOffset was %f", yawOffset);
    ROS_INFO("(Should be less than -%f degrees)", ANGLE_ERR);
<<<<<<< HEAD

    rotateRight();

=======
#if !DEBUG
    rotateRight_90();
#endif
>>>>>>> origin
  }
  // means that we are off by between 90 and -90 degrees
  if(yawOffset >= ANGLE_ERR || yawOffset <= -ANGLE_ERR){
    ROS_INFO("We are turning less because yawOffset was %f", yawOffset);
    ROS_INFO("(Should be between 90 and -90 degrees)");
    usleep(MOVEMENT_INTERVAL);
    this->velocityCommand.linear.x = 0.0;
    if(yawOffset >= 0){
      this->velocityCommand.angular.z = MIN_LEFT;
    }
    else
      this->velocityCommand.angular.z = MIN_RIGHT;


    velocityPublisher.publish(this->velocityCommand);

  }
  else{ // means that we are basically facing backwards, with a margin of yawErr
    ROS_INFO("We should have turned 180 degrees backwards.");
    setCurrentState(MOVE_FORWARD_X);
  }
}

// assumes that x is zero and y is nonzero. May want to add cases for when method is called in error
void RoboState::determineYawGoal()
{
  ROS_INFO("Switching x and y coordinates.");
  usleep(MOVEMENT_INTERVAL);

  if( getY() > 0){ // means that destination is on right
    setYawGoal(90);
    ROS_INFO("Our yaw goal is %f", getYaw());
  }
  else if ( getY() < 0){ // means that destination is on right
    setYawGoal(-90);
  }
  /*
  else if (getX() < 0){ // means that we must face opposite initial direction
    setYawGoal(180);
  }
  */
  else
    setYawGoal(0); // means that we must face initial direction

  if(getY() > 0){
    ROS_INFO("Your y-coordinate is positive.");
  }
  else if(getY() < 0){
    ROS_INFO("Your y-coordinate is negative.");
  }
  else{
    ROS_INFO("Your y-coordinate is zero.");
  }
}

// Change the value of Movement multiple until turtlebot moves forward by INCREMENT_AMT.
// And stops for MOVEMENT_INTERVAL.
void RoboState::testForward()
{
  // may need to adjust value for whatever reason
  usleep(MOVEMENT_INTERVAL);
  double xMoveCommand; 
  // only move forward INCREMENT_AMT if the amount left to move is greater than INCREMENT_AMT

  // ideally, this should result in forward (or backward movement)
  // in the x direction by INCREMENT_AMT

  this->velocityCommand.linear.x = .1;
  this->velocityCommand.angular.z = 0;
	  
  velocityPublisher.publish(this->velocityCommand);
  // ideally, this is the amount that x has changed
  // we should wait until forward movement has finished before we go on
  usleep(MOVEMENT_INTERVAL);

}

void RoboState::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  
  // set odometry
  setXodom(msg->pose.pose.position.x);
  setYodom(msg->pose.pose.position.y);
  
  // convert from quaternions to angles
  tf::Quaternion q(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, tempYaw;
  m.getRPY(roll, pitch, tempYaw);
  
  tempYaw*=ANGLE_CONVERT;
  // determine whether to send positive or negative angles
  bool isTempYawNeg = (tempYaw < 0);
  
  // we always want a positive yaw value (for consistency)
  if(getY() >= 0){
    if(isTempYawNeg)
      setYaw(360+tempYaw);
    else
      setYaw(tempYaw);
  }
  // we always want a negative yaw value (for consistency)
  else{
    if(isTempYawNeg)
      setYaw(tempYaw);
    else
      setYaw(-360+tempYaw);
  }

  // we do not want to get spammed with messages about yaw.
  if(getInternalCount() % 100==1){
    ROS_INFO("The yaw was %f", getYaw());

    if(getY() >= 0)
      ROS_INFO("The yaw is supposed to be positive.");
    else
      ROS_INFO("The yaw is supposed to be negative.");
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
	setX(msg->x);
	setY(msg->y);
	ROS_INFO("xCoord is: %f. yCoord is: %f", getX(), getY());
	
	// we don't need to face backward since initial movement is forward
	if(getX() >= 0){
	  setCurrentState(MOVE_FORWARD_X);
	  initialXnegative = false;
	}
	// need to face backward since initial movement is backward
	// (want bumper sensors to be useful)
       	else{
	  initialXnegative = true;
	  //setCurrentState(TURN_NEG_X);
	  setCurrentState(TURN_NEG_X_1_2);
	}
	//setErr(sqrt(pow(getX(),2)+pow(getY(),2))*.1);
	setErr(.1);
      }
      // need to determine what direction we will ultimately face
      determineYawGoal();

    }
  else
    ROS_INFO("Cannot accept message. Movement still in progress.");
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
  

// we use this because it has been calibrated to turn 90 degrees left
void RoboState::rotateLeft_90()
{
  ROS_INFO("Rotating left.");
  usleep(MOVEMENT_INTERVAL);
  this->velocityCommand.linear.x = 0.0;
  this->velocityCommand.angular.z = LEFT_90;
  velocityPublisher.publish(this->velocityCommand);
  usleep(MOVEMENT_INTERVAL/5);
}

// we use this because it has been calibrated to turn 90 degrees right
void RoboState::rotateRight_90()
{
  ROS_INFO("Rotating right.");
  usleep(MOVEMENT_INTERVAL);
  this->velocityCommand.linear.x = 0.0;
  this->velocityCommand.angular.z = RIGHT_90;
  velocityPublisher.publish(this->velocityCommand);
  usleep(MOVEMENT_INTERVAL/5);
}


#if DEBUG
// we use this because it has been calibrated to turn 90 degrees left
void RoboState::rotateLeft(double velocity)
{
  ROS_INFO("The yaw was %f", getYaw());

  ROS_INFO("Enter the velocity for left");
  ROS_INFO("Rotating left.");
  usleep(MOVEMENT_INTERVAL);
  this->velocityCommand.linear.x = 0.0;
  this->velocityCommand.angular.z = velocity;
  velocityPublisher.publish(this->velocityCommand);
  usleep(MOVEMENT_INTERVAL/5);
}

// we use this because it has been calibrated to turn 90 degrees right
void RoboState::rotateRight(double velocity)
{
  ROS_INFO("The yaw was: %f", getYaw());
  ROS_INFO("Enter the velocity for right");
  ROS_INFO("Rotating right.");
  usleep(MOVEMENT_INTERVAL);
  this->velocityCommand.linear.x = 0.0;
  this->velocityCommand.angular.z = velocity;
  velocityPublisher.publish(this->velocityCommand);
  usleep(MOVEMENT_INTERVAL/5);
}
#endif

#if !DEBUG
// we use this because it has been calibrated to turn 90 degrees left
void RoboState::rotateLeft()
{
  ROS_INFO("Rotating left.");
  usleep(MOVEMENT_INTERVAL);
  this->velocityCommand.linear.x = 0.0;
  this->velocityCommand.angular.z = MIN_LEFT;
  velocityPublisher.publish(this->velocityCommand);
  usleep(MOVEMENT_INTERVAL/5);
}

// we use this because it has been calibrated to turn 90 degrees right
void RoboState::rotateRight()
{
  ROS_INFO("Rotating right.");
  usleep(MOVEMENT_INTERVAL);
  this->velocityCommand.linear.x = 0.0;
  this->velocityCommand.angular.z = MIN_RIGHT;
  velocityPublisher.publish(this->velocityCommand);
  usleep(MOVEMENT_INTERVAL/5);
}
#endif

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


/* We don't use this.

void RoboState::goRobotGo()
{
  incrementInternalCount();
  switch (getCurrentState()){
  case NEUTRAL:
    // do nothing
    break;
  case TURN_NEG_X:
    // ROS has difficulty with updating yaw
    if(currentCountOdd()){
      ROS_INFO("Rotating to face backwards.");
      rotate_180();
    }
    break;
    ROS_INFO("Moving forward in x coordinate.");
  case MOVE_FORWARD_X:
    goForwardX();
    break;
  case FACE_DESTINATION:
    // ROS has difficulty with updating yaw
    if(currentCountOdd()){
      ROS_INFO("Facing final destination.");
      faceDestination();
    }
    break;
  case MOVE_FORWARD_Y:
    ROS_INFO("Moving forward in y coordinate.");
    goForwardY();
    break;
  default:
    ROS_INFO("Something is broken. We should not reach this point.");
  }

}
*/
