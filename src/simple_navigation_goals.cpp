#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
//define actionlib from move_base_msgs library
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
MoveBaseClient;
//Main function to control the robot move at four waypoints
//Update the map information
int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
	ROS_INFO("Waiting for the move_base action server to come up");
  }
  //Set up the goal point from MoveBaseGoal library
  move_base_msgs::MoveBaseGoal goal_point[4];
  //Set up the coordinates of wayppoints
  double wayppoints[4][7]={{1.386641,0.390903,0.997183},{1.410281,1.307093,0.676476},{0.410099,1.368677,-0.009813},{0.412467,0.420511,0.724897}};
  //we'll send a goal to the robot to move 1 meter forward
  for(int i = 0 ; i<4 ;i++){
	    //Read map information
            goal_point[i].target_pose.header.frame_id = "map";
 	    //Setup the sychronus time
	    goal_point[i].target_pose.header.stamp = ros::Time::now();
	    //Set x,y position and angular speed
            goal_point[i].target_pose.pose.position.x = wayppoints[i][0];
            goal_point[i].target_pose.pose.position.y = wayppoints[i][1];
            goal_point[i].target_pose.pose.orientation.w =wayppoints[i][2];
	    ROS_INFO("Sending goal");
	    //Sending the goal point to robot
	    ac.sendGoal(goal_point[i]);
	    //Get feedback from robot
  	    ac.waitForResult();
    }
  //Check Robot state
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  ROS_INFO("Hooray, the base moved 1 meter forward");
  else
  ROS_INFO("The base failed to move forward 1 meter for some reason");
  return 0;
}
