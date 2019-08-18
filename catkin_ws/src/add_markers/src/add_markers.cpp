#include <ros/ros.h>
#include <visualization_msgs/Marker.h>


visualization_msgs::Marker marker;
ros::Publisher marker_pub ;

void poseobjectCallback(const geometry_msgs::Pose &msg) {    
  ROS_INFO("Object position received: x:%f vs %f, y:%f ",
    msg.position.x, msg.position.y, msg.orientation.z);

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker for pick up
  marker.pose.position.x = msg.position.x;
  marker.pose.position.y = msg.position.y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.7;
  marker.scale.y = 0.7;
  marker.scale.z = 0.7;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  marker_pub.publish(marker);

  ROS_INFO("Initial location action applied...");
}

void pickupCallback(const geometry_msgs::Pose &msg) {    
  ROS_INFO("Pickup location received: x:%f vs %f, y:%f ",
    msg.position.x, msg.position.y, msg.orientation.z);

  ROS_INFO("Hide the marker...");

  marker.action = visualization_msgs::Marker::DELETE;
  marker_pub.publish(marker);
  ROS_INFO("pickup location action applied...");
}

void dropoffCallback(const geometry_msgs::Pose &msg) {
  
  ROS_INFO("Dropoff location received: x:%f vs %f, y:%f ",
    msg.position.x, msg.position.y, msg.orientation.z);

   // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker for pick up
  marker.pose.position.x = msg.position.x;
  marker.pose.position.y = msg.position.y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.7;
  marker.scale.y = 0.7;
  marker.scale.z = 0.7;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  marker_pub.publish(marker);
  ROS_INFO("drop off location action applied...");
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;

  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  ros::Subscriber poseobject_sub = n.subscribe("/poseobject",10,poseobjectCallback);
  ros::Subscriber pickup_sub = n.subscribe("/pickup",10,pickupCallback);
  ros::Subscriber dropoff_sub = n.subscribe("/dropoff",10,dropoffCallback);

  // Set our shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  // Set the frame ID and timestamp.
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "add_markers";
  marker.id = 0;
  marker.type = shape;

  while (marker_pub.getNumSubscribers() < 1){
    if (!ros::ok()){
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }

  ros::spin();
  
}


