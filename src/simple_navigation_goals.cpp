#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

extern float md_volt;

//define actionlib from move_base_msgs library
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
MoveBaseClient;


// Global Functions for subscriber
//void chatterCallback(const std_msgs::Float32::data& msg)
//{
   
//}


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
  move_base_msgs::MoveBaseGoal goal_point[36];
  //Set up the coordinates of wayppoints
  double wayppoints[37][3]={{0.61,0.0,1},{1.22,0.0,1},{1.83,0.0,1},{1.83,0.0,0.7071},{1.83,0.56,0.7071},{2.7432,1.524,1},{3.3528,1.524,-0.7071},{3.3528,0.9144,1},{3.9624,0.9144,0.7071},{3.9624,1.524,0.7071},{3.9624,2.1336,0.7071},{3.9624,2.7432,0.7071},{3.9624,3.3528,0.7071},{3.9624,3.9628,-0.7071},{3.9624,2.7432,0.0},{0.2721,1.2322,6.2832},{0.2721,1.2322,6.2832},{0.2721,1.2322,6.2832},{0.2721,1.2322,6.2832},{0.2721,1.2322,6.2832},{0.2721,1.2322,6.2832},{0.2721,1.2322,6.2832},{0.2721,1.2322,6.2832},{0.2721,1.2322,6.2832},{0.2721,1.2322,6.2832},{0.2721,1.2322,6.2832},{0.2721,1.2322,6.2832},{0.2721,1.2322,6.2832},{0.2721,1.2322,6.2832},{0.2721,1.2322,6.2832},{0.2721,1.2322,6.2832},{0.2721,1.2322,6.2832},{0.2721,1.2322,6.2832},{0.2721,1.2322,6.2832},{0.2721,1.2322,6.2832},{0.2721,1.2322,6.2832},{0.2721,1.2322,6.2832}};
  //we'll send a goal to the robot to move 1 meter forward
  for(int i = 0 ; i<36 ;i++){

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
	    ac.waitForResult();
    }
  //Check Robot state
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  ROS_INFO("Hooray, the base moved 1 meter forward");
  else
  ROS_INFO("The base failed to move forward 1 meter for some reason");
  return 0;
}

