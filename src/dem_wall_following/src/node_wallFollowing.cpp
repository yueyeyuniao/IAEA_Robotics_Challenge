#include "dem_wall_following/wallFollowing.h"
#include <math.h>
#define PI 3.141592

NodeWallFollowing::NodeWallFollowing(ros::Publisher pub, ros::Publisher pub_new, double wallDist, double maxSp, int dir, double pr, double di, double an)
{
	wallDistance = wallDist;
	maxSpeed = maxSp;
	direction = dir;
	P = pr;
	D = di;
	angleCoef = an;
	e = 0;
	e_left = 0;
	e_right = 0;
	angleMin = 0;  //angle, at which was measured the shortest distance
	pubMessage = pub;
	pubDetectionReady = pub_new;
	follow_mode = 0;
}

NodeWallFollowing::~NodeWallFollowing()
{
}

//Publisher
void NodeWallFollowing::publishMessage()
{
//preparing message
	geometry_msgs::Twist msg;
	int size;
	int minIndex;
	int maxIndex;

	int case3counter;


	if (follow_mode == 3 || follow_mode == 4){
		if ((direction == 1 && distLeft > 1.5*wallDistance) || (direction == -1 && distRight > 1.5*wallDistance)) //distFront > wallDistance && 
			follow_mode = 4;
		else 
			follow_mode = 3;
	}
	
	else{
		ROS_INFO("%f,%f,%f",distFront,distLeft,distRight);
		
		if (distFront > wallDistance && distLeft > wallDistance && distRight > wallDistance && follow_mode != 2)
			follow_mode = 1;
		if ((distFront < wallDistance || distLeft < wallDistance || distRight < wallDistance) && follow_mode !=2){
			if (distLeft < distRight){
				direction = 1;
			}
			follow_mode = 2;
		}
		if ((distLeft < 1.5*wallDistance && fabs(angleMin - PI*direction/2)<0.1) || (distRight < 1.5*wallDistance && fabs(angleMin - PI*direction/2)<0.1)){
			follow_mode = 3;
		}
	}

	ROS_INFO("%d",follow_mode);

	switch(follow_mode){
		case 1:
		msg.linear.x = maxSpeed * 0.5;
		break;
// follow move_base goal
		case 2:
// msg.angular.z = direction*(P*e + D*diffE) + angleCoef * (angleMin - PI*direction/2);    //PD controller
ROS_INFO("%f", angleMin);
msg.angular.z = angleCoef * (angleMin - PI*direction/2);
if(msg.angular.z<-0.5)
	msg.angular.z=-0.5;
if(msg.angular.z>0.5)
	msg.angular.z=0.5;
// msg.linear.x = maxSpeed - 4*maxSpeed*(wallDistance-distFront)/wallDistance;
msg.linear.x = 0;
break;
case 3:

case3counter++;
if (direction == 1){
	diffE = (distLeft - wallDistance) - e_left;
	e_left = distLeft - wallDistance;
	msg.linear.x = 0.3*maxSpeed;
    if (distRight < 0.3)
    {
    	msg.angular.z = 0.3*direction*(P*e_left + D*diffE)-(distRight - 0.3);    //PD controller
    }else{
    	msg.angular.z = 0.3*direction*(P*e_left + D*diffE);    //PD controller
    }
	ROS_INFO("left");
}
else if (direction == -1){
	diffE = (distRight - wallDistance) - e_right;
	e_right = distRight - wallDistance;
	msg.linear.x = 0.3*maxSpeed;
	if (distLeft < 0.3)
    {
		msg.angular.z = 0.3*direction*(P*e_right + D*diffE)+(distLeft - 0.3);    //PD controller
    }else{
    	msg.angular.z = 0.3*direction*(P*e_right + D*diffE);    //PD controller
    }
	ROS_INFO("right");
}

	std::cout << "case3counter"<< case3counter <<std::endl;

if (case3counter == 5) 
{	

	ROS_INFO("ready to detect");
	ninetyDegreeTurn(0.5*direction, 0);
	//publish 
	std_msgs::Bool msg_detected_object;
	msg_detected_object.data = true;
	pubDetectionReady.publish(msg_detected_object);
	ros::Duration(10.0).sleep();
	if (isDetectionObject == false)
	{
		msg_detected_object.data = false;
		pubDetectionReady.publish(msg_detected_object);
		ninetyDegreeTurn(-0.5*direction, 0);
	}else{
		msg.linear.x = 0;
		msg.angular.z = 0;
	}
}
isturn=false;
break;
case 4:
size = laser_msg.ranges.size();
minIndex = round((double)size*(double)(direction+1)/4.0 + 3.0*(double)(1+direction)*size/16.0);
maxIndex = round((double)size*(double)(direction+3)/4.0 - 3.0*(double)(1-direction)*size/16.0);
ROS_INFO("%d, %d,%d",size,minIndex,maxIndex);
for(int i = minIndex; i < maxIndex; i++){
	if (laser_msg.ranges[i] < laser_msg.ranges[minIndex] && laser_msg.ranges[i] > 0.0){
		minIndex = i;
	}}
	ROS_INFO("minimum_id: %d, minimum_distance: %f",minIndex,laser_msg.ranges[minIndex]);
if (laser_msg.ranges[minIndex] < 1.5*wallDistance) // there's still wall
msg.linear.x = 0.5*maxSpeed;   // more control might be needed
else{
	double start_time = ros::Time::now().toSec();
	double current_time = start_time;
	while(current_time - start_time < 3.3 && isturn == false){
		msg.angular.z = 0.5*direction;
		msg.linear.x = fabs(msg.angular.z*wallDistance);
		pubMessage.publish(msg);
		ros::Duration(0.01).sleep();
		current_time = ros::Time::now().toSec();
		ROS_INFO("%f,%f",start_time,current_time);
	} 
	isturn = true;
}
break;
default:
break;
}

//publishing message
//if(current_cmd.linear.x!=msg.linear.x || current_cmd.angular.z != msg.angular.z){
pubMessage.publish(msg);
current_cmd = msg;
// }
}

void NodeWallFollowing::ninetyDegreeTurn(double yaw_speed, double x_speed){
	double start_time = ros::Time::now().toSec();
	double current_time = start_time;
	geometry_msgs::Twist msg;
	while(current_time - start_time < 3.3 && isturn == false){
		msg.angular.z = yaw_speed;
		msg.linear.x = x_speed;
		pubMessage.publish(msg);
		ros::Duration(0.01).sleep();
		current_time = ros::Time::now().toSec();
		ROS_INFO("%f,%f",start_time,current_time);
	}
}

//Subscriber
void NodeWallFollowing::messageCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	if(isDetectionObject == false){
	laser_msg = *msg;
	int size = laser_msg.ranges.size();

//Variables whith index of highest and lowest value in array.
	int minIndex = size*(direction+1)/4;
	int maxIndex = size*(direction+3)/4;

//This cycle goes through array and finds minimum
	for(int i = minIndex; i < maxIndex; i++)
	{
		if (laser_msg.ranges[i]<0.002)
	{
		laser_msg.ranges[i]=100;
	}
		if (laser_msg.ranges[i] < laser_msg.ranges[minIndex] && laser_msg.ranges[i] > 0.0){
			minIndex = i;
		}
	}

//Calculation of angles from indexes and storing data to class variables.
	angleMin = (minIndex-size/2)*laser_msg.angle_increment;
	double distMin;
	distMin = laser_msg.ranges[minIndex];
	distFront = laser_msg.ranges[size/2];
	distLeft = laser_msg.ranges[size-1];
	distRight = laser_msg.ranges[0];
	// if (distFront < 0.02)
	// 	distFront = 100;
	// if (distLeft < 0.02)
	// 	distLeft = 100;
	// if (distRight < 0.02)
	// 	distRight = 100;
	diffE = (distMin - wallDistance) - e;
	e = distMin - wallDistance;

//Invoking method for publishing message
	publishMessage();
}
}

void NodeWallFollowing::objectDetectedCallback(const std_msgs::Bool::ConstPtr& msg){
	isDetectionObject = msg->data;
}
