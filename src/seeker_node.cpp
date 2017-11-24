#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/SetBool.h>
#include <math.h>

//global variables for scan data
int middle;//dead center reading

//PID constants
const double kp = .0015; //proportional constant
const double ki = 0.0000005; //integral constant
const double kd = .001; //derivative constant

//PID variables for angular velocity
const double tolerance = 55; //tolerance for error
double position = NAN; //position of the min value of the scan, initialize as not a number to enable seeking behavior
double error; //difference between position and middle
double last_error;//previous cycle error
double integral;//accumulation of error
double derivative;//change in error
const double max_integral = 500; //keep integral from getting out of control

//Velocity variables
double angular_velocity;//angular velocity of the turtlebot in rad/s
double linear_velocity;//linear velocity of the turtlebot in m/s

//Displacement variables
float displacement_x;//x displacement relative to laser
float displacement_y;//y displacement relative to laser
float displacement_z;//z displacement relative to laser

//enable service
bool enabled = false;//start robot as off

//config parameters
const double max_angular = 0.4;//max angular speed of robot in rad/s
const double max_linear = 0.75;//max linear speed of robot in m/s

//laser scan callback
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){

	//import parameters from the sensor     	
	float angle_min = scan->angle_min;//minimum angle read by scanner in radians
	float angle_max = scan->angle_max;//maximum angle read by scanner in radians
	float angle_increment = scan->angle_increment;//increment of each angle in radians
	int scan_size = (angle_max - angle_min) / angle_increment;//determine length of the scan
	middle = scan_size / 2;//find the value that corresponds to dead center
	float min_scan_read = scan->range_max;//set the minimum reading to max allowable range before determining actual min
	double scan_data[scan_size];//declare an array to store the scan data

	//get the ranges
	for(int i = 0; i < scan_size; i++){//cycle through scan data
		scan_data[i] = scan->ranges[i]; //import scan data 
		if(std::isfinite(scan_data[i]) && (scan_data[i] < min_scan_read)){//if scan data is finite, and less than current min
			min_scan_read = scan_data[i];//declare new min
			position = i;//current position of lowest scan value relative to center value
			float angle = angle_min + angle_increment * i;//calculate current angle of reading
			displacement_x = min_scan_read * cos(angle);//calculate x displacement relative to laser
			displacement_y = min_scan_read * sin(angle);//calculate y displacement relative to laser
			displacement_z = 0;//only 2d laser, so always 0	
		}
	}	
}

//enable service callback
bool enableCallback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response) {
    
	
	if(request.data){//if request is true
		response.success = true;//report success
		response.message = "Seeker Enabled!";//send response message
		enabled = true;//enable the bot
	}

	if(!request.data){//if request is false
		response.success = true;//report success
		response.message = "Seeker Disabled :(";//send response message
		enabled = false;//disable the bot
	}
	return true;
}


int main(int argc, char **argv){

	ros::init(argc, argv, "seeker_node");//initialize ros node

	ros::NodeHandle nh;//create the nodehandle

	//laser scan subscriber
	ros::Subscriber scanListener = nh.subscribe("scan", 1000, scanCallback);//subscribe to "scan" topic with input buffer of 1000

	//boolean service subscriber
	ros::ServiceServer enableService = nh.advertiseService("enable", enableCallback);//create the service

	//movement publisher
	ros::Publisher movementPublisher = nh.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity",1000);//publisher to "mobile_base/commands/velocity"

	//displacement publisher
	ros::Publisher displacementPublisher = nh.advertise<geometry_msgs::Vector3>("displacement",1000);//publisher to "displacement" of type "geometry_msgs/Vector3"

	//loop rate in hz
	ros::Rate rate(1000);

	
	while (ros::ok()) {//while ros node is running
		if(enabled){//if enable service has been called

			geometry_msgs::Twist move;//define move
			geometry_msgs::Vector3 displacement;//define displacement	
			
			//check to see if ball has been located
			if(std::isfinite(position)){

				//determine error
				error = position - middle;//magnitude of error, defined as the position of lowest value read relative to the center

				//calculate integral
				integral += error;//accumulation of error

				//check to see if integral is within range
				if(integral > max_integral){
					integral = max_integral;//keep integral at max
				}
	
				if(integral < -max_integral){
					integral = -max_integral;//keep integral at negative of max
				}
			
				//reset integral if within tolerance
				if(abs(error) < tolerance){
					integral = 0;//reset integral
				}
			
				//calculate derivative
				derivative = error - last_error;//change in error

				//calculate angular output
				angular_velocity = error * kp + integral * ki + derivative * kd;//calculate angular velocity output

				//set error for next iteration
				last_error = error;//current error set to next iteration's previous error

				//check limits of velocity
				if(angular_velocity > max_angular){//control max velocity
					angular_velocity = max_angular;
				}
	
				if(angular_velocity < -max_angular){//control max velocity
					angular_velocity = -max_angular;
				}
	
				//check if ball is centered
				if(abs(error) < tolerance){//if facing the ball
					linear_velocity = max_linear;//ram it
				}

				else{
					linear_velocity = 0;//don't move forward
				}
			}

			else{
				angular_velocity = max_angular;//search
			}
			

			//update angular velocity accordingly
			move.linear.x = linear_velocity;//set linear velocity
			move.angular.z = angular_velocity;//set angular velocity
			movementPublisher.publish(move);//publish velocity to topic

			//prevent false zero displacement readings
			if(displacement_x == 0){//if x is read as 0 (impossible due to min reading)
				displacement_x = NAN;//not a number
				displacement_y = NAN;
				displacement_z = NAN;
			}

			//update the displacement
			displacement.x = displacement_x;//set x displacement
			displacement.y = displacement_y;//set y displacement
			displacement.z = displacement_z;//set z displacement
			displacementPublisher.publish(displacement);//publish displacement to topic
		}
	
		ros::spinOnce();//update ros
		rate.sleep();//sleep to try to maintain frequency
	}
	return 0;//exit
}
