#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>
#include <vector>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "tf/transform_datatypes.h"
#include "sensor_msgs/LaserScan.h"

using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


#define P 2
#define D 5

class object
{
public:
  int ID;
  tf::Transform transform;
  object() {}
  object(int nID, tf::Transform ntransform)
  {
    ID = nID;
    transform = ntransform;
  }
};

class iaeatask1 : public object
{
public:
  double odom_yaw;
  int counts = 0; // real object counts
  //int flag = 0;
  int temp_count = 0;
  int min_dis_laser_index = 0;
  int mid_laser_index;
  int e_laser;
  int diff_e_laser;
  float fx, fy, fz;

  vector<int> pixel_x; // midpoint x of boundingbox
  vector<int> pixel_y; // midpoint y of boundingbox
  visualization_msgs::Marker points;
  image_geometry::PinholeCameraModel model1;
  sensor_msgs::LaserScan laser_msg; //laser msg
  geometry_msgs::Twist vel_msg;

  int waypoints_size = 5;                                              //  waypoints
  float arrayWaypointX[5] = {10.2, 10, 13.5, 14.7, 13.8}; //
  float arrayWaypointY[5] = {-8.3, -6.6, -7, -5.7, -4};   //

  vector<move_base_msgs::MoveBaseGoal> waypoints; // vector to save the waypoints
  vector<move_base_msgs::MoveBaseGoal> object_locations;

  ros::Subscriber sub_info, sub_image, sub_yolo, odom_sub, sub_laser;
  ros::NodeHandle n_;
  ros::Publisher vel_pub, yolo_pub, lift_pub;

  vector<object> objects;
  double minimum_threshold = 0.1;

  iaeatask1()
  {
    // subscribe to topics to calculate location
    sub_info = n_.subscribe("/camera/rgb/camera_info", 10, &iaeatask1::info_callback, this);
    sub_image = n_.subscribe("/camera/depth/image_rect", 10, &iaeatask1::depth_callback, this);
    sub_yolo = n_.subscribe("darknet_ros/bounding_boxes", 1, &iaeatask1::yolo_callback, this);
    sub_laser = n_.subscribe("/laser/scan", 1, &iaeatask1::laser_callback, this);
    //sub_wallFollow = n_.subscribe("/wallFollow/DetectionReady", 1, &iaeatask1::wallFollow_callback, this);
    // odom_sub = n_.subscribe("odom", 1, &iaeatask1::odom_callback, this);

    // publisher to publish calculated location
    //pub_location = n_.advertise<move_base_msgs::MoveBaseGoal>("/goandfind/object_location", 1);
    vel_pub = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    yolo_pub = n_.advertise<std_msgs::Bool>("Object_detected", 1);
    lift_pub = n_.advertise<std_msgs::Float32>("move_lift", 1);
  }

  ~iaeatask1()
  {
  }

  /* laser_callback 
   param: 
*/
  void laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
    laser_msg = *msg;
    int size = laser_msg.ranges.size();
    mid_laser_index = (int)size / 2;
    min_dis_laser_index = 0;
    for (int i = 0; i < size; i++)
    {
      if (laser_msg.ranges[i] < 0.002)
      {
        laser_msg.ranges[i] = 100;
      }
      if (laser_msg.ranges[i] < laser_msg.ranges[min_dis_laser_index] && laser_msg.ranges[i] > 0.0)
      {
        min_dis_laser_index = i;
      }
    }
    cout << "min index " << min_dis_laser_index << endl;
    cout << "mid index " << mid_laser_index << endl;
  }

  /* yolo_callback 
   param:  boundingboxes info: &msg, publisher:  vel_pub
   function: boundingbox callback, get the size of the bb, get the position of the bb, calculate the x and y coordinates of the bb in the picture
              && publish velocity 0 to the robot to stop the robot
*/

  void yolo_callback(const darknet_ros_msgs::BoundingBoxesConstPtr &msg)
  {
    temp_count = 0;
    pixel_x.clear(); //column
    pixel_y.clear(); //row

    int size = msg->boundingBoxes.size();
    for (int i = 0; i < size; i++)
    {
      if (msg->boundingBoxes[i].Class.compare("IAEATask2") == 0)
      {
        cout << "find an object" << endl;

        // change this later****************************************************************
        // geometry_msgs::Twist vel;
        // vel.linear.x = 0;
        // vel.linear.y = 0;
        // vel.linear.z = 0;
        // vel.angular.x = 0;
        // vel.angular.y = 0;
        // vel.angular.z = 0;
        // vel_pub.publish(vel);
        //***********************************************************************************

        bool isDuplicated = false;
        double temp_x = (msg->boundingBoxes[i].xmin + msg->boundingBoxes[i].xmax) / 2.0;
        double temp_y = (msg->boundingBoxes[i].ymin + msg->boundingBoxes[i].ymax) / 2.0;
        for (int j = 0; j < pixel_x.size(); j++)
        {
          if (fabs(temp_x - pixel_x[j]) < minimum_threshold && fabs(temp_y - pixel_y[j]) < minimum_threshold)
          {
            isDuplicated = true;
            break;
          }
        }
        if (isDuplicated == false)
        {
          pixel_x.push_back(temp_x);
          pixel_y.push_back(temp_y);
          temp_count++;
        }
      }
    }
  }


  void info_callback(const sensor_msgs::CameraInfoConstPtr &msg) // what the hell is this
  {
    model1.fromCameraInfo(msg);
  }

  void depth_callback(const sensor_msgs::ImageConstPtr &msg)
  {
    cout << "object depth info" << endl;

    double co_x; // midpoint coordinate x
    double co_y; // midpoint coordinate y
    double co_z; // midpoint coordinate z

    cv_bridge::CvImageConstPtr depth_img_cv;
    depth_img_cv = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);

    for (int i = 0; i < temp_count; i++)
    {
      move_base_msgs::MoveBaseGoal object_locations_temp;
      cv::Point2d pixel_point(pixel_x[i], pixel_y[i]);
      float depth = depth_img_cv->image.at<short int>(pixel_point);
      cv::Point3d xyz = model1.projectPixelTo3dRay(pixel_point);
      cv::Point3d coordinate = xyz * depth;

      if (depth > 0.01)
      {
        co_x = coordinate.x / 1000;
        co_y = coordinate.y / 1000;
        co_z = coordinate.z / 1000;
        cout << "x,y,z" << co_x << co_y << co_z << endl;
      }

      // test code
      // co_x = 400.0/1000.0 - 0.2;  // 20 cm in front of the objects
      // co_y = 0/1000;
      // co_z = 0/1000;
      //

      // new added

      fx = co_x;
      fy = co_y;
      fz = co_z;

      cout << "x,y,z" << fx << fy << fz;
      tf::Transform transform = tf_cal(co_x, co_y, co_z);
      tf::Quaternion quaternion_robot = tf_cal_robot();

      // object_locations.resize(counts);
      // object_locations_pair.resize(temp_count);

      object_locations_temp.target_pose.header.frame_id = "map";
      object_locations_temp.target_pose.header.stamp = ros::Time::now();
      object_locations_temp.target_pose.pose.position.x = transform.getOrigin().x() - 0.1;
      object_locations_temp.target_pose.pose.position.y = transform.getOrigin().y();
      object_locations_temp.target_pose.pose.position.z = transform.getOrigin().z();
      object_locations_temp.target_pose.pose.orientation.x = quaternion_robot.getX();
      object_locations_temp.target_pose.pose.orientation.y = quaternion_robot.getY();
      object_locations_temp.target_pose.pose.orientation.z = quaternion_robot.getZ();
      object_locations_temp.target_pose.pose.orientation.w = quaternion_robot.getW();

      //object_locations_pair_temp.push_back(make_pair(0,object_locations_temp[i]));
      // useful_temp_objects++;

      // geometry_msgs::Point p; // for marker
      // p.x = transform.getOrigin().x();  // for marker locations
      // p.y = transform.getOrigin().y();
      // p.z = 0;
      // points.points.push_back(p);

      // total

      // cout << "object_locations_temp.back().target_pose.pose.orientation.x=" << object_locations_temp.back().target_pose.pose.orientation.x << endl;
      // cout << "object_locations_temp.back().target_pose.pose.orientation.y=" << object_locations_temp.back().target_pose.pose.orientation.y << endl;
      // cout << "object_locations_temp.back().target_pose.pose.orientation.z=" << object_locations_temp.back().target_pose.pose.orientation.z << endl;

      bool isDuplicated = false;
      for (int j = 0; j < counts; ++j)
      {
        if (fabs(object_locations_temp.target_pose.pose.position.x - object_locations[j].target_pose.pose.position.x) < minimum_threshold &&
            fabs(object_locations_temp.target_pose.pose.position.y - object_locations[j].target_pose.pose.position.y) < minimum_threshold &&
            fabs(object_locations_temp.target_pose.pose.position.z - object_locations[j].target_pose.pose.position.z) < minimum_threshold)
        {
          isDuplicated = true;
          break;
        }
      }
      if (isDuplicated == false)
      {
        object_locations.push_back(object_locations_temp);
        counts++;
      }
    }

    cout << "total objects counts " << counts << endl;
    if (counts > 0)
    {
      std_msgs::Bool msg;
      msg.data = true;
      yolo_pub.publish(msg);
    }
  }

  /* tf_cal for getting 
    param: x,y,z  distance from the camera to the base
    function/; generate new frame for objects
*/
  tf::Transform tf_cal(float x, float y, float z) // subscribe the info of camera, and map, get the tf between them
  {
    //calculate transform from /map to /object
    tf::TransformListener listener;
    tf::StampedTransform stampedtransform;
    //ros::Rate rate(5.0);
    //rate.sleep();
    listener.waitForTransform("/map", "/camera_rgb_optical_frame", ros::Time::now(), ros::Duration(3.0));
    listener.lookupTransform("/map", "/camera_rgb_optical_frame", ros::Time(0), stampedtransform); // target frame source frame
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));
    transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    tf::Transform ntransform;
    ntransform = stampedtransform * transform;

    //float object_x = transform.getOrigin().x();
    //float object_y = transform.getOrigin().y(); //position of object in 2d map //to be saved
    //float result = {object_x, object_y};
    //return result;
    return ntransform;
  }

  /*
    was used to get robot quaternion values
*/

  tf::Quaternion tf_cal_robot() // subscribe the info of robot base_link to map
  {
    //calculate transform from /map to /object
    tf::TransformListener listener_robot;
    tf::StampedTransform stampedtransform_robot;
    listener_robot.waitForTransform("/map", "/base_link", ros::Time::now(), ros::Duration(3.0));
    listener_robot.lookupTransform("/map", "/base_link", ros::Time(0), stampedtransform_robot); // target frame source frame
    return stampedtransform_robot.getRotation();
  }

  // push back robot waypoints
  void RobotWaypoints()
  {
    move_base_msgs::MoveBaseGoal object_locations_temp2;
    for (int i = 0; i < waypoints_size; i++)
    {
      object_locations_temp2.target_pose.header.frame_id = "map";
      object_locations_temp2.target_pose.header.stamp = ros::Time::now();
      object_locations_temp2.target_pose.pose.position.x = arrayWaypointX[i];
      object_locations_temp2.target_pose.pose.position.y = arrayWaypointY[i];
      object_locations_temp2.target_pose.pose.position.z = 0;
      object_locations_temp2.target_pose.pose.orientation.x = 0;
      object_locations_temp2.target_pose.pose.orientation.y = 0;
      object_locations_temp2.target_pose.pose.orientation.z = 0;
      object_locations_temp2.target_pose.pose.orientation.w = 1;
      waypoints.push_back(object_locations_temp2);
    }
  }

  // go the waypoints and rotate
  void GotoWaypointsandRotate()
  {
    MoveBaseClient ac("move_base", true);
    for (int i = 0; i < waypoints_size; i++)
    {

      // Wait 60 seconds for the action server to become available
      ROS_INFO("Waiting for the move_base action server");
      ac.waitForServer(ros::Duration(10.0));
      ac.sendGoal(waypoints[i]);
      ac.waitForResult();
      if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("You have reached the goal!");
      else
        ROS_INFO("The base failed for some reason");

      //rotate 90 degree, 4 times

      NinetyDegreeTurn(0.5, 0);
      ros::spinOnce();
      ros::Duration(2).sleep();
      NinetyDegreeTurn(0.5, 0);
      ros::spinOnce();
      ros::Duration(2).sleep();
      NinetyDegreeTurn(0.5, 0);
      ros::spinOnce();
      ros::Duration(2).sleep();
      NinetyDegreeTurn(0.5, 0);
      ros::spinOnce();
      ros::Duration(2).sleep();
    }
  }

  // 90 degree turn
  void NinetyDegreeTurn(double yaw_speed, double x_speed)
  {
    double start_time = ros::Time::now().toSec();
    double current_time = start_time;
    geometry_msgs::Twist vel;
    while (current_time - start_time < 3.3)
    {
      vel.angular.z = yaw_speed;
      vel.linear.x = x_speed;
      vel_pub.publish(vel);
      ros::Duration(0.01).sleep();
      current_time = ros::Time::now().toSec();
      ROS_INFO("%f,%f", start_time, current_time);
    }
  }

  /* go to object locations
   param : movebasegoal, object_locations; velocity publisher
   function: go through all the saved object locations
*/

  void GotoObjectLocations()
  {
    MoveBaseClient ac("move_base", true);
    if (object_locations.size() != 0)
    {
      for (int i = 0; i < object_locations.size(); i++)
      {

        cout << "ready for movement" << endl;

        // go to the first object
        // create the action client
        // true causes the client to spin its own thread
        //  MoveBaseClient ac("move_base", true);  // put in the constructor

        // Wait 60 seconds for the action server to become available
        ROS_INFO("Waiting for the move_base action server");

        cout << "***************global goal***********" << object_locations[i].target_pose.pose.position.x;
        cout << object_locations[i].target_pose.pose.position.y;
        cout << object_locations[i].target_pose.pose.position.z;
        cout << object_locations[i].target_pose.pose.orientation.x;
        cout << object_locations[i].target_pose.pose.orientation.y;
        cout << object_locations[i].target_pose.pose.orientation.z;
        cout << object_locations[i].target_pose.pose.orientation.w << endl;

        ac.waitForServer(ros::Duration(10.0));
        ac.sendGoal(object_locations[i]);
        ac.waitForResult();
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
          ROS_INFO("You have reached the goal!");
        else
          ROS_INFO("The base failed for some reason");

        // face to the object
        e_laser = min_dis_laser_index - mid_laser_index;
        while (abs(min_dis_laser_index - mid_laser_index) > 10)
        {
          diff_e_laser = (min_dis_laser_index - mid_laser_index) - e_laser;
          e_laser = min_dis_laser_index - mid_laser_index;
          vel_msg.angular.z = 0.001 * (P * e_laser + D * diff_e_laser); //PD controller
          vel_pub.publish(vel_msg);
          ros::spinOnce();
        }

        // lifter
        if (object_locations[i].target_pose.pose.position.z <= 0.5)
        { // measure the minimum bound
          ROS_INFO("The object is lower than the minimum bound");
        }

        if (object_locations[i].target_pose.pose.position.z > 0.5 && object_locations[i].target_pose.pose.position.z <= 1)
        {
          // publish servo running time--
          // wait some seconds
          // moving the lifter back
          std_msgs::Float32 msg;
          msg.data = -3.5; // total height to 78 cm
          lift_pub.publish(msg);
          ros::Duration(2).sleep();
          msg.data = 3.5;
          lift_pub.publish(msg);
        }
      }
      // remove the first goal in the object_locations
      //  object_locations.erase(object_locations.begin());
    }
  }

}; // end class

int main(int argc, char **argv)
{
  ros::init(argc, argv, "iaeatask1");
  iaeatask1 iaeataskone;
  iaeataskone.RobotWaypoints();
  iaeataskone.GotoWaypointsandRotate();
  iaeataskone.GotoObjectLocations();
}
