#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  // publishers
  ros::NodeHandle nh;  
  ros::Publisher poseobject_pub = nh.advertise<geometry_msgs::Pose>("/poseobject",10);
  ros::Publisher pickup_pub = nh.advertise<geometry_msgs::Pose>("/pickup",10);
  ros::Publisher dropoff_pub = nh.advertise<geometry_msgs::Pose>("/dropoff",10);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // pickup location
  move_base_msgs::MoveBaseGoal pickup_goal;
  pickup_goal.target_pose.header.frame_id = "map";
  pickup_goal.target_pose.header.stamp = ros::Time::now();
  pickup_goal.target_pose.pose.position.x = 6.0; 
  pickup_goal.target_pose.pose.position.y = 5.0; 
  pickup_goal.target_pose.pose.orientation.w = 1.0;

  // drop off location
  move_base_msgs::MoveBaseGoal drop_goal;
  drop_goal.target_pose.header.frame_id = "map";
  drop_goal.target_pose.header.stamp = ros::Time::now();
  drop_goal.target_pose.pose.position.x = -4.0; 
  drop_goal.target_pose.pose.position.y = 5.0; 
  drop_goal.target_pose.pose.orientation.w = -0.5;

  while (poseobject_pub.getNumSubscribers() < 1 ){
    if (!ros::ok()){
      return 0;
    }
    ROS_WARN("Waiting for add_markers to subscribe...");
    sleep(1);
  }

  geometry_msgs::Pose msg;    
  msg.position.x = 6.0;
  msg.position.y = 5.0;
  msg.position.z = 0.0;
  msg.orientation.x = 0.0;
  msg.orientation.y = 0.0;
  msg.orientation.z = 0.0;
  msg.orientation.w = 1;    
  poseobject_pub.publish(msg);
  ROS_INFO("Published initial position x:%f, y:%f, w:%f",
    msg.position.x, msg.position.y, msg.orientation.w);


   // Send the pickup position and orientation for the robot to reach
  ROS_INFO("Sending goal for pickup location");
  ac.sendGoal(pickup_goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Hooray, the robot reached the pickup location...");

    geometry_msgs::Pose msg;
    msg.position.x = 6.0;
    msg.position.y = 5.0;
    msg.position.z = 0.0;
    msg.orientation.x = 0.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;
    msg.orientation.w = 1.0;
    pickup_pub.publish(msg);
    ROS_INFO("Publishing goal for pickup location x:%f, y:%f, w:%f",
      msg.position.x, msg.position.y, msg.orientation.w);

    ROS_INFO("Waiting for 5 seconds...");
    ros::Duration(5.0).sleep();
  }
  else {
    ROS_INFO("The robot failed to reach to the pickup location...");
    ROS_INFO("Exit the program...");
    return 0;
  }

  // Send the drop off goal position and orientation for the robot to reach
  ROS_INFO("Sending goal for drop off location");
  ac.sendGoal(drop_goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Hooray, the robot reached the drop off location...");

    geometry_msgs::Pose msg;
    msg.position.x = -4.0;
    msg.position.y = 5.0;
    msg.position.z = 0.0;
    msg.orientation.x = 0.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;
    msg.orientation.w = -0.5;
    dropoff_pub.publish(msg);
    ROS_INFO("Publishing goal for dropoff location x:%f, y:%f, w:%f",
      msg.position.x, msg.position.y, msg.orientation.w);

    ROS_INFO("object dropped off...");  
  }
  else{
    ROS_INFO("The robot failed to reach to the drop off location...");
    ROS_INFO("Exit the program...");
    return 0;
  }

  ros::spin();
}