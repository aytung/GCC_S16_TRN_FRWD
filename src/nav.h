#ifndef NAV_H
#define NAV_H
// ROS includes.
#include "ros/ros.h"
#include <turtlebot/mymsg.h>
#include <iostream>
//#include "node_example/listener.h"
#include "geometry_msgs/Twist.h"
#include "create_node/TurtlebotSensorState.h"
#include <string>
#include <cstring>
#include <math.h>
#include <cmath>
#include <nav_msgs/Odometry.h>

// change this to 0 when you want to use goRobotGo
// change this to 1 when you want to use nav_node.cpp
#define USE_MAIN 1
 /* This is a visualization of what the x and y coordinates represent on 
   relative to the direction that the turtlebot is facing.
|         X+        . (destination)
|        |
|        |
|      (forward)
|        __
|      /   \
|      |___|    
|        ____________ Y-
  */

using namespace std;

enum State{ NEUTRAL, TURN_NEG_X_1_2, TURN_NEG_X_2_2,  MOVE_FORWARD_X,FACE_DESTINATION, MOVE_FORWARD_Y};
    

class RoboState
  {
  public:

    // test function
    void testForward();
    void goRobotGo();
    // constructor
    RoboState(ros::NodeHandle rosNode);
    double getYaw();
    void turn_neg_x_1_2();
    void turn_neg_x_2_2();
    void rotate_180();
    void goForwardX();
    void goForwardY();
    State getCurrentState();
    void incrementInternalCount();
    void faceDestination();
    bool currentCountOdd();
    int getInternalCount();

#if DEBUG
    void rotateLeft(double velocity);
    void rotateRight(double velocity);
    void rotateLeft_90();
    void rotateRight_90();
#endif
    
  private:
    bool initialXnegative;
#if !DEBUG
    void rotateLeft();
    void rotateRight();
    void rotateLeft_90();
    void rotateRight_90();

#endif

    // the ros node being used by RoboState
    ros::NodeHandle node;    

    // publishers and subscribers 
    geometry_msgs::Twist velocityCommand;
    ros::Publisher velocityPublisher;
    geometry_msgs::Twist command;
    ros::Subscriber messageSubscriber;
    ros::Subscriber bumperSubscriber;

    State currentState;
    // various callback functions
    void bumperCallback(const create_node::TurtlebotSensorState::ConstPtr& msg);
    void messageCallback(const turtlebot::mymsg::ConstPtr& msg);
    void setMessageStatus(bool status);    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
    ros::Subscriber odomSubscriber;
    int count;
    int internalCount;
    void determineYawGoal();

    // these are private if we just call goRobotGo
#if !USE_MAIN
    int getInternalCount();
    void rotate_180();
    void goForwardX();
    void goForwardY();
    State getCurrentState();
    void incrementInternalCount();
    void faceDestination();
    bool currentCountOdd();
        double getYaw();
#endif

    // private variables
    double xTarget;
    double yTarget;
    double xOdom;
    double yOdom;
    double xOdomOld;
    double yOdomOld;
    double xCoord;
    double yCoord;
    double acceptErr;
    double yawErr;
    double yaw;
    double yawGoal;

    // get and set functions
    void setCurrentState(State newState);
    void setXodom(double xOdom);
    void setYaw(double newYaw);

    void setYodom(double yOdom);
    double getYawGoal();

    void setErr(double err);
    double getErr();
    double getXodom();
    double getYodom();
    double getXodomOld();
    double getYodomOld();
    void setXodomOld(double xOdomCurrent);
    void setYodomOld(double yOdomCurrent);
    void setYawGoal(double newYawGoal);
    double getX();
    void setX(double x);
    void setY(double y);
    double getY();

  };
#include "nav.cpp"
#endif // NAV_NODE_H
