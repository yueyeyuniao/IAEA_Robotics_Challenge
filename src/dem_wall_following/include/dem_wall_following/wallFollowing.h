#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"

class NodeWallFollowing
{
public:

  /* Constructor:
   * pub   Publisher, which can send commands to robot.
   * wallDist Desired distance from the wall.
   * maxSp Maximum speed, that robot can go.
   * dir   1 for wall on the left side of the robot (-1 for the right side).
   * pr    P constant for PD controller.
   * di    D constant for PD controller.
   * an    Angle coefficient for P controller.
   */
  NodeWallFollowing(ros::Publisher pub, ros::Publisher pub_new, double wallDist, double maxSp, int dir, double pr, double di, double an);

  ~NodeWallFollowing();

  /* This method is generating commands for robot by
   * using data in variables (e, diffE, angleMin).
   */

  void publishMessage();

  /* This method finds minimum distance in data from
   * sensor and process these data into variables
   * (e, diffE, angleMin).
   * msg  Message, which came from robot and contains data from
   * laser scan.
   */

  void messageCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void objectDetectedCallback(const std_msgs::Bool::ConstPtr& msg);

  void ninetyDegreeTurn(double yaw_speed, double x_speed);

//variables
  double wallDistance; // Desired distance from the wall.
  double e;            // Difference between desired distance from the wall and actual distance.
  double e_left;
  double e_right;
  double diffE;     // Derivative element for PD controller;
  double maxSpeed;     // Maximum speed of robot.
  double P;            // k_P Constant for PD controller.
  double D;            // k_D Constant for PD controller.
  double angleCoef;    // Coefficient for P controller.
  int direction;      // 1 for wall on the right side of the robot (-1 for the left one).
  double angleMin;     // Angle, at which was measured the shortest distance.
  double distFront;    // Distance, measured by ranger in front of robot.
  double distLeft;     // Distance, measured by ranger in left of robot.
  double distRight;    // Distance, measured by ranger in right of robot.
  int follow_mode;     // follow_mode in publishMessage
  ros::Publisher pubMessage, pubDetectionReady;  // Object for publishing messages.
  geometry_msgs::Twist current_cmd; //Current cmd
  sensor_msgs::LaserScan laser_msg; //laser msg
  bool isturn,isDetectionObject;         // turn 90 degree for corner
};
