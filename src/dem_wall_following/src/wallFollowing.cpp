#include "dem_wall_following/wallFollowing.h"

#define SUBSCRIBER_BUFFER_SIZE 1  // Size of buffer for subscriber.
#define PUBLISHER_BUFFER_SIZE 1000  // Size of buffer for publisher.
#define WALL_DISTANCE 0.7
#define MAX_SPEED 0.3
#define P 5    // Proportional constant for controller
#define D 25     // Derivative constant for controller
#define ANGLE_COEF 1    // Proportional constant for angle controller
#define DIRECTION -1 // 1 for wall on the left side of the robot (-1 for the right side).
// #define PUBLISHER_TOPIC "/syros/base_cmd_vel"
#define PUBLISHER_TOPIC "/cmd_vel"
// #define SUBSCRIBER_TOPIC "/syros/laser_laser"
#define SUBSCRIBER_TOPIC "/laser/scan"

int main(int argc, char **argv)
{
  //Initialization of node
  ros::init(argc, argv, "wallFollowing");
  ros::NodeHandle n;

  //Creating publisher
  ros::Publisher pubMessage = n.advertise<geometry_msgs::Twist>(PUBLISHER_TOPIC, PUBLISHER_BUFFER_SIZE);
  ros::Publisher pubDetectionReady = n.advertise<std_msgs::Bool>("/wallFollowing/DetectionReady", 1);

  //Creating object, which stores data from sensors and has methods for
  //publishing and subscribing
  NodeWallFollowing *nodeWallFollowing = new NodeWallFollowing(pubMessage, pubDetectionReady, WALL_DISTANCE, MAX_SPEED, DIRECTION, P, D, 1);

  //Creating subscriber and publisher
  ros::Subscriber sub = n.subscribe(SUBSCRIBER_TOPIC, SUBSCRIBER_BUFFER_SIZE, &NodeWallFollowing::messageCallback, nodeWallFollowing);
  ros::Subscriber detectedObjectSub = n.subscribe("Object_detected", 1, &NodeWallFollowing::objectDetectedCallback, nodeWallFollowing);
  
  ros::spin();

  return 0;
}
