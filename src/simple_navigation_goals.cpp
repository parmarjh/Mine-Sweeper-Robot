#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalID.h>
#include <ros/console.h>
#include <unistd.h>

enum robot_state { move_to_target, disposal, backup, back2origin };

float md_voltage;
enum robot_state rs;
int count;


//define actionlib from move_base_msgs library
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
//typedef actionlib_msgs::GoalID<move_base::cancel> 

//typedef ros::Subscriber sub
ros::MultiThreadedSpinner spinner(6);

//Set up the goal point from MoveBaseGoal library
//move_base_msgs::MoveBaseGoal goal_point[38];

//all path
 double wayppoints[47][7]={

{0.9496,0.36,0,0,0,0,1}, 	//up 4	(1,0)
{1.5592,0.36,0,0,0,0,1}, 	//up 4	(2,0)
{1.5592,0.9496,0,0,0,0,1}, 	//up 4	(2,1)
{0.9496,0.9496,0,0,0,0,1}, 	//up 4	(1,1)
{0.34,0.9496,0,0,0,0,1}, 	//up 4	(0,1)
{1.5592,0.9496,0,0,0,0,1}, 	//up 4	(2,1)
{1.5592,1.5592,0,0,0,0,1}, 	//up 4	(2,2)
{1.5592,2.1288,0,0,0,0,1}, 	//up 4	(2,3)
{1.5592,2.7784,0,0,0,0,1}, 	//up 4	(2,4)
{0.9496,2.7784,0,0,0,0,1}, 	//up 4	(1,4)
{0.34,2.7784,0,0,0,0,1}, 	//up 4	(0,4)
{0.34,2.1288,0,0,0,0,1}, 	//up 4	(0,3)
{0.34,1.5592,0,0,0,0,1}, 	//up 4	(0,2)
{0.9496,1.5592,0,0,0,0,1}, 	//up 4	(1,2)
{0.9496,2.1288,0,0,0,0,1}, 	//up 4	(1,3)
{0.9496,2.7784,0,0,0,0,1}, 	//up 4	(1,4)
{2.7784,2.7784,0,0,0,0,1}, 	//up 4	(4,4)
{2.7784,2.1288,0,0,0,0,1}, 	//up 4	(4,3)
{2.7784,1.5592,0,0,0,0,1}, 	//up 4	(4,2)
{2.7784,0.9496,0,0,0,0,1}, 	//up 4	(4,1)
{2.7784,0.34,0,0,0,0,1}, 	//up 4	(4.0)
{2.7784,1.5592,0,0,0,0,1}, 	//up 4	(4.2) 	
{3.388,1.5592,0,0,0,0,1}, 	//up 4	(5.2)	
{3.9976,1.5592,0,0,0,0,1}, 	//up 4	(6,2)
{3.9976,2.1288,0,0,0,0,1}, 	//up 4	(6,3)
{3.9976,2.7784,0,0,0,0,1}, 	//up 4	(6,4)
{4.0376,3.388,0,0,0,0,1}, 	//up 4	(6.5)!
{4.0376,3.8976,0,0,0,0,1}, 	//up 4	(6.6)!
{3.9976,2.7784,0,0,0,0,1}, 	//up 4	(6.4)
{3.388,2.7784,0,0,0,0,1}, 	//up 4	(5.4)
{3.388,3.388,0,0,0,0,1}, 	//up 4	(5.5)
{3.388,3.8976,0,0,0,0,1}, 	//up 4	(5.6)!
{2.7784,3.9976,0,0,0,0,1}, 	//up 4	(4.6)
{2.1288,3.9976,0,0,0,0,1}, 	//up 4	(3.6)
{1.5592,3.8976,0,0,0,0,1}, 	//up 4	(2.6)
{1.5592,3.388,0,0,0,0,1}, 	//up 4	(2.5)
{0.9496,3.388,0,0,0,0,1}, 	//up 4	(1.5)!
{0.9996,3.8976,0,0,0,0,1}, 	//up 4	(1.6)!
{0.39,3.8976,0,0,0,0,1}, 	//up 4	(0.6)!
{0.39,3.438,0,0,0,0,1}, 	//up 4	(0.5)!
{2.1288,3.388,0,0,0,0,1}, 	//up 4	(3.5)
{2.7784,3.388,0,0,0,0,1}, 	//up 4	(4.5)
{3.388,2.7784,0,0,0,0,1}, 	//up 4	(5.4)
{3.388,2.1288,0,0,0,0,1}, 	//up 4	(5.3)
{3.388,0.9496,0,0,0,0,1}, 	//up 4	(5.1)
{3.9976,0.9496,0,0,0,0,1}, 	//up 4	(6.1)

{3.9976,0.34,0,0,0,0,1} 	//up 4	(6,0) Desposial
};



// Robot move_back from disposal point
/*
void move_back()
{
	ros::NodeHandle cmdh;
	ros::Publisher cmdpub= cmdh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
	ros::Rate r(10);
	geometry_msgs::Twist twist;
	geometry_msgs::Vector3 linear;
	linear.x=-0.1;
	linear.y=0;
	linear.z=0;
	geometry_msgs::Vector3 angular;
	angular.x=0;
	angular.y=0;
	angular.z=0;
	twist.linear=linear;
	twist.angular=angular;

	cmdpub.publish(twist);
	//cout<<"hello"<<endl;
	//ros::spinOnce();
	//sleep(1);	
}
*/
void cancel_goal()
{
	ros::NodeHandle cmdh;
	ros::Publisher cancel_pub = cmdh.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1, true);
	actionlib_msgs::GoalID first_goal;
	cancel_pub.publish(first_goal);
	//ros::spinOnce();
}

void moveback_Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	ROS_INFO("Enter back up function");
	if (rs == backup){
		if(md_voltage > 0){
			//Robot Move back, drop mine
			ROS_INFO("Success back up");
			ros::NodeHandle cmdh;
			ros::Publisher cmdpub= cmdh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
			ros::Rate r(60);
			geometry_msgs::Twist twist;
			twist.linear.x=-0.1;
			cmdpub.publish(twist);
			sleep(1);
		}
		if(md_voltage == 0){rs = back2origin;}
	}
	else if (rs == back2origin){
		MoveBaseClient ac("move_base", true);
		move_base_msgs::MoveBaseGoal goal_point[57];
		ac.sendGoal(goal_point[count]);
		ac.waitForResult();
		if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			rs = move_to_target;
		}
	}
}

// Check Robot whether need to go to disposal point
void md_voltCallback(const std_msgs::Float32::ConstPtr& msg)
{
	MoveBaseClient ac("move_base", true);
	//wait for the action server to come up
  	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
  	}
	move_base_msgs::MoveBaseGoal goal_point[57];
	md_voltage = msg->data;
	
	// Set goal to disposal point
	if (md_voltage > 4.9 && rs != backup) {
		//cancel_goal();
		rs = disposal;
		//Read map information
		goal_point[56].target_pose.header.frame_id = "map";
		//Setup the sychronus time
		goal_point[56].target_pose.header.stamp = ros::Time::now();
		//Set x,y position and angular speed
		goal_point[56].target_pose.pose.position.x = wayppoints[56][0];
		goal_point[56].target_pose.pose.position.y = wayppoints[56][1];
		goal_point[56].target_pose.pose.orientation.w = wayppoints[56][6];
		ROS_INFO("Sending disposal point");
		ac.sendGoal(goal_point[56]);
		ac.waitForResult();
		rs = backup;
		
	}

	
}

// Robot move to the goal
void waypointCallback(const std_msgs::String::ConstPtr& msg)
{
	MoveBaseClient ac("move_base", true);
	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
  	}
	move_base_msgs::MoveBaseGoal goal_point[57];

	if (md_voltage < 2 && rs == move_to_target) {
		//Read map information
		goal_point[count].target_pose.header.frame_id = "map";
		//Setup the sychronus time
		goal_point[count].target_pose.header.stamp = ros::Time::now();
		//Set x,y position and angular speed
		goal_point[count].target_pose.pose.position.x = wayppoints[count][0];
		goal_point[count].target_pose.pose.position.y = wayppoints[count][1];
		goal_point[count].target_pose.pose.orientation.w = wayppoints[count][6];
		ROS_INFO("Sending goal");
		//Sending the goal point to robot
		ac.sendGoal(goal_point[count]);
		ac.waitForResult();
		//Check Robot state
		if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
			ROS_INFO("Hooray, the base moved 1 meter forward");
			count++;
		}

	}
}


//Main function to control the robot move at four waypoints
//Update the map information
int main(int argc, char** argv) {

	ros::init(argc, argv, "simple_navigation_goals");
	//tell the action client that we want to spin a thread by default

	ros::NodeHandle n;

	//wait for the action server to come up
	//while (!ac.waitForServer(ros::Duration(5.0))) {
		//ROS_INFO("Waiting for the move_base action server to come up");
	//}

	count = 0;
	rs = move_to_target;

	//ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
	ros::Subscriber sub1 = n.subscribe("md_volt", 1, md_voltCallback);
	ros::Subscriber sub2 = n.subscribe("chatter", 1, waypointCallback);
	ros::Subscriber sub3 = n.subscribe("odom", 1, moveback_Callback);
	spinner.spin();

	return 0;
}

/*


#include <iostream>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <ros/console.h>
#include<unistd.h>
using namespace std;
int main(int argc,char** argv)
{
    ros::init(argc, argv, "cmdveltest");
     ros::NodeHandle cmdh;
    ros::Publisher cmdpub= cmdh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
    ros::Rate r(60);
    while(1){
        geometry_msgs::Twist twist;
        geometry_msgs::Vector3 linear;
        linear.x=-0.1;
        linear.y=0;
        linear.z=0;
        geometry_msgs::Vector3 angular;
        angular.x=0;
        angular.y=0;
        //直行
        //angular.z=0;
        //转圈
        angular.z=0;
        twist.linear=linear;
        twist.angular=angular;

        cmdpub.publish(twist);
        cout<<"hello"<<endl;
       // ros::spinOnce();
        //r.sleep();
        sleep(1);
    }
    return 0;
}
*/
