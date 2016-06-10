#include "nav.h"
#include "ros/ros.h"
#include <string>
#include <cstring>
#include <math.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include <tf/transform_listener.h>

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


int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle nh;
  RoboState robot = RoboState(nh);
  geometry_msgs::Twist cmd_vel;
  
  ros::Rate loopRate(10); // 10 hz

  while(ros::ok())
    {
     
      robot.incrementInternalCount();
      //      if(robot.getInternalCount() % 5 == 0){
	switch (robot.getCurrentState()){

	case NEUTRAL:
	  ROS_INFO("Our yaw was %f", robot.getYaw());
	  break;


	case FACE_DESTINATION:
	  ROS_INFO("Facing final destination.");
	  if(robot.faceDestination()){
	    ROS_INFO("Done facing final destination.");
	    robot.setCurrentState(MOVE_FORWARD);
	  }
	  break;

	case MOVE_FORWARD:
	  ROS_INFO("Moving forward along x-axis.");
	  if(robot.goForward()){
	    robot.setCurrentState(FACE_ORIGINAL);
	    robot.setYawGoal(0);
	  }
	  break;


	case FACE_ORIGINAL:
	  ROS_INFO("Attempting to face original direction.");
	  if(robot.faceDestination()){
	    ROS_INFO("Done with movement.");
	    robot.setCurrentState(NEUTRAL);

	  }
	  break;

	default:

	  ROS_INFO("Something is broken. We should not reach this point.");
	}

	//      }

      loopRate.sleep();
      ros::spinOnce();
     
    }
	
  return 0;
}


