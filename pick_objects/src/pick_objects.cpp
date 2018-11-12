#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  int num_goals = 2;
  float goals[2][2] = { {-4.0, 4.0}, {6.0, -5.8} };

  for (int g = 0; g < num_goals; g++) {
    move_base_msgs::MoveBaseGoal goal;
    // set up the frame parameters
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    goal.target_pose.pose.position.x = goals[g][0];
    goal.target_pose.pose.position.y = goals[g][1];
    goal.target_pose.pose.orientation.w = 1.0;

     // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    // Wait an infinite time for the results
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Goal %d reached!", g);
    } else {
      ROS_INFO("Error reaching goal %d", g);
    }
    sleep(5);
  }

  sleep(30);

  return 0;
}
