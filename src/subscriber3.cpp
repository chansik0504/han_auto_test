/**
  *******************************************************************
  * @file		subscriber.cpp
  * @author		Seunghan Han
  * @version	V.2.0	(version 1 : oroca_ros_tutorials)
  * @date		4-March-2018
  * @brief		This file provide get a keyboard command and pubilsh 
  *				mesage data.
  *******************************************************************
  */


/**
  * Define Header file
*/
#include "ros/ros.h"
#include <std_msgs/UInt16.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <math.h>
#include <time.h>
#include <ctime>

/**
  * Define a Global variable
*/


// Using LIDAR variable
int width_size_count = 20;	// Lidar Maximum measuring distance : 15m. When count is 20, space width : 1.35m.
int width_size_count_scaler = 10;
int space_availability = 0; // 1 : The farhest cluster space exist, 0: not exist.

int count_speed_low_val = 6;
int count_speed_high_val = 30;
int speed_val = 5;

int i = 0; 

float a=0;
float b=0;

float x_total = 0;
float y_total = 0;
float x_avg = 0;
float y_avg = 0;
float a_nominator = 0;
float a_denominator = 0;


float current_val_right[720]; 
float current_val_left[720];

float estimated_previous_val_right[720];
float estimated_previous_val_left[720];

float estimated_current_val_right[720];
float estimated_current_val_left[720];

float val_forward_sum = 0;
float val_forward;
float val_forward2;

float group_right_x[720];
float group_right_y[720];
float group_left_x[720];
float group_left_y[720];

float main_wall = 1; // 1 : rignt 0 : left

float distance_now = 0.5;
float theta_now = 0;
float distance_pre = 0;
float theta_pre = 0.5;

float theta_60method;
int range_value;

float farhest_data_degree[720];


float err_now = 0;
float err_pre = 0;
float theta_desired = 0;
float V_theta = 0;
int servo = 0;
int speed = 90;

float num;

int count_speed = 0;

int left_group_size = 0;
int right_group_size = 0;
int left_out = 0;
int right_out = 0;

float Kp = 14;
float Kd = 0.5;
float C = 0.05;
float L = 0.05;
float distance_desired = 1;
int servo_angle;

int key = 20499;

int mode = 2;	// 1 : Auto 2 : Keyboard control 3 : Emergency stop
int control_speed;
int control_angle;

int servo_data;

int count = 0;
float degree[720];
int continuous_farhest_distance_count[720];
float farhest_distance_width[720];
float center_degree[720];
float center_degree_scaling[720];

int	total_largest_count_value = 0;
float total_largest_space_center_degree_scaling = 0;

float i_value_theta_now;
float left_center_degree[720];
float left_center_degree_scaling;
float right_center_degree[720];
float right_center_degree_scaling;

int left_space = 0;
int right_space = 0;

int left_space_count_start_point = 0;
int right_space_count_start_point = 0;


int select_direction = 0;	// right
float select_theta_now;
float select_theta_pre;

float estimated_previous_select_theta_now = 0;
float estimated_current_select_theta_now = 0;

float distance_data_within_15m[720];

float right_largest_value;
float left_largest_value;
float right_largest_degree;
float left_largest_degree;

int right_corner_exist = 0;
int left_corner_exist = 0;

float right_largest_value_next[720];
float left_largest_value_next[720];
float left_largest_degree_next[720];
float right_largest_degree_next[720];

float right_corner_previous_value = 0;
float right_corner_previous_degree = 0;
float left_corner_previous_value = 0;
float left_corner_previous_degree = 0;


float right_largest_degree_scaling;
float left_largest_degree_scaling;
float x_left_largest_value;
float x_right_largest_value;
float y_left_largest_value;
float y_right_largest_value;


float right_corner_previous_degree_scaling = 0;
float left_corner_previous_degree_scaling = 0;

float val_forward_corner;

int straight_return = 0;
int left_return = 0;
int right_return = 0;

// Using IMU variable
float gyro_x_val = 0;
float gyro_y_val = 0;
float gyro_z_val = 0;

float accel_x_val = 0;
float accel_y_val = 0;
float accel_z_val = 0;

float dt = 0.002;
float gyro_x, gyro_y, gyro_z;
float gyro_angle_x, gyro_angle_y, gyro_angle_z;
float gyro_pre_angle_x, gyro_pre_angle_y, gyro_pre_angle_z;
float accel_x, accel_y, accel_z;
float complementary_angle_x, complementary_angle_y, complementary_angle_z;

float x_degree_pre;
float y_degree_pre;
float x_degree_validation;
float y_degree_validation;

float gyro_current_hpf_x;
float gyro_current_hpf_y;
float gyro_current_hpf_z;

float gyro_previous_hpf_x; 
float gyro_previous_hpf_y; 
float gyro_previous_hpf_z; 

float accel_current_lpf_x;
float accel_current_lpf_y;
float accel_current_lpf_z;

float accel_previous_lpf_x;
float accel_previous_lpf_y;
float accel_previous_lpf_z;

float gyro_pre_x; 
float gyro_pre_y; 
float gyro_pre_z; 

float error_now, error_pre;
float speed_error_now, speed_error_pre;

float current_accel_speed;
float current_speed;
/**
  * Define Function
*/
void laserscan_and_obtain_coordinate_data();
void RIDAR_Low_Pass_Filter();
void val_forward_based_on_wall();
void locate_the_farhest_data();
void farhest_crowded_place();
void parameterize_width_size_count();
void crowded_place_center_value();
void change_angle_base();
void select_largest_crowded_space_only();
void select_proximity_crowded_space_base_on_theta_now();
int identification_of_cluster_space();
void choose_cluster_point_front_of_the_car();
void choose_largest_cluster_point_base_on_theta_now();
void select_theta_now_Low_Pass_Filter();
void select_theta_now_High_Pass_Filter();

void collect_distance_data_within_15m();
void find_largest_distance_data_on_left_right_respectively();
void change_angle_base_2();
void calculate_val_forward_corner();

int find_main_wall();
void select_wall_and_driving_mode();
void select_path_and_PD_control();
void select_path_and_PD_control_test();
void select_path_and_PD_control_test2();
void speed_control();
void select_middle_value_between_left_and_right();

void IMU_Complementary_Filter();
void complementary_x_y_degree_varidation();
void gyro_High_Pass_Filter();
void accel_Low_Pass_Filter();

void speed_calculate();
void Steering_PID_Control_Using_Gyro();
void Speed_PID_Control_Using_Accel();
/**
  * Define Class
*/
class SubscribeAndPublish
{
	public:
	SubscribeAndPublish()
	{
		ros_tutorial_pub = nh.advertise<std_msgs::UInt16>("/servo", 100);
		ros_tutorial_sub = nh.subscribe("/scan", 100, &SubscribeAndPublish::callback, this);
		ros_tutorial_sub_key = nh.subscribe("/keyboard", 100, &SubscribeAndPublish::callback2, this);
		ros_tutorial_sub_imu = nh.subscribe("/imu", 100, &SubscribeAndPublish::callback3, this);
	}

	void callback(const sensor_msgs::LaserScan::ConstPtr& msg)
	{

		clock_t start = clock();

		distance_pre = distance_now;
		theta_pre = theta_now;
		err_pre = err_now;



		select_theta_pre = select_theta_now;
		estimated_previous_select_theta_now = estimated_current_select_theta_now;

		error_pre = error_now;
		speed_error_pre = speed_error_now;


		std_msgs::UInt16 servo;
		
		// Function : Convert to pervious estimate
		for(i = 0; i < 720; i++)
		{
			estimated_previous_val_right[i] = estimated_current_val_right[i];
			estimated_previous_val_left[i] = estimated_current_val_left[i];
		}

		// Function : LaserScan data collection
		for(i = 0; i < 720; i++){
			current_val_right[i] = msg->ranges[i+180];	// From 0 degree of the right to 180 degree of the left
		}

		for(i = 0; i < 720; i++){
			current_val_left[i] = msg->ranges[900-i];	// From 180 degree ot the left to 0 degree of the right
		}

		// Function : Pass the RIDAR data through a Low Pass Filter to get an estimate.
		// RIDAR_Low_Pass_Filter();


		for(i = 0; i < 720; i++)
		{
			estimated_current_val_right[i] = current_val_right[i];
			estimated_current_val_left[i] = current_val_left[i];
		}
		laserscan_and_obtain_coordinate_data();

		
		/******************************************************************************************************/


		/**
		  * SeungHan Code work space
		  * case 1. When the farhest crowded spaces exist,
		  * case 2. When the farhest crowded space does not exist,

		  */

		// Function : Calculation of frontal data based on wall	
		// val_forward_based_on_wall();
		// val_forward = msg->ranges[range_value];
		val_forward = msg -> ranges[540];
		ROS_INFO("val_forward is %f", val_forward);

		// Function : Locate the farhest data(Based on LIDAR)
		locate_the_farhest_data();

		// check
		// Function : Parameterize the "width_size_count" value.
		// parameterize_width_size_count();

		// Function : Data acquisition function to calculate clustering start point and width.
		farhest_crowded_place();


		// Function : Determine the presence or absence of a cluster space.
		space_availability = identification_of_cluster_space();
		 // ROS_INFO("space availability value is %d.", space_availability);
		


		// When the farhest free space is detected, the space is specified as path. => Calculate "select_theta_now" and "select_detection"
		/*
		if(space_availability == 1)
		{
			// Function : Calculate about angle of clustering point.
			crowded_place_center_value();

			// Function : Change the angle base.
			change_angle_base();
			
			// Function : Only select the largest crowded space.
			select_largest_crowded_space_only();

			// Function : Using data of "theta_now", centerd on "theta_now", select the proximity cluster spaces on the left and right. 
			// select_proximity_crowded_space_base_on_theta_now();

			// Function : Choose cluster point and Set a new_theta_now (select_theta_now).(Method 1)
			// choose_cluster_point_front_of_the_car();

			// check
			// Function : Choose largest_cluster point based on the "theta_now".
			// choose_largest_cluster_point_base_on_theta_now();

			// Function : Pass the "select_theta_now" data through a Low Pass Filter & High Pass Filter to get an estimate.

			select_theta_now_Low_Pass_Filter();
			// select_theta_now_High_Pass_Filter();

			
			// Function : Select main wall
			main_wall = find_main_wall();
			
			// Function : Linear equation "a" and "b" calculation in Straight mode And Cornering mode
			select_wall_and_driving_mode();
			

			Steering_PID_Control_Using_Gyro();
		}

		*/

		
		
		// When the farhest free space is not detected, use the Triangle technique to specify the path. => Calculate select_theta_now.
		// This work is used for cornering or grafting.
		// else if(space_availability == 0)
		
			// Function : Collect distance data within 15m. Approximately 15m is the actual maximum length thah LIDAR can mesure. 
			collect_distance_data_within_15m();

			// Function : Find the greatest distance data on the left and right, respectively.
			find_largest_distance_data_on_left_right_respectively();

			// Function : Change the angle base.
			change_angle_base_2();

			// Function : Select the middle value between left and right, Set a new_theta_now (select_theta_now)
			
			// calculate_val_forward_corner();

			// straight driving
			if(left_corner_exist == 0 && right_corner_exist == 0)
			{
				// speed control duty calculate
				// speed_val = 5;
				// count_speed_low_val = 6;
				// count_speed_high_val = 30;
				/*
				if(val_forward > 5)
				{
					count_speed_high_val = 60;
					// count_speed_low_val = 3;
				}
				
				else if(val_forward > 4 && val_forward < 5)
				{
					speed_val = 5;
					count_speed_high_val = 50;
					// count_speed_low_val = 4;
				}
				else if(val_forward > 3 && val_forward < 4)
				{
					speed_val = 5;
					count_speed_high_val = 40;
					// count_speed_low_val = 5;
				}
				else if(val_forward < 3)
				{
					speed_val = 5;
					count_speed_high_val = 30;
					// count_speed_low_val = 6;
				}
				else if(val_forward < 2)
				{
					speed_val = 5;
					count_speed_high_val = 30;
					// count_speed_low_val = 6;
				}
				*/

				distance_now = 0.3;

				// select main wall
				if(left_return == 0 && right_return == 0)
				{
					main_wall = 1;
				}
				else if(left_return == 1 && right_return == 0)
				{
					main_wall = 1;
				}
				else if(left_return == 0 && right_return == 1)
				{
					// main_wall = 0;
				}
				
				// over 3m, wall based driving
				if(val_forward > 3)
				{
					select_wall_and_driving_mode();
					select_path_and_PD_control();
				}

				// in 3m, triangle based driving
				else if(val_forward < 3 && val_forward > 1)
				{
					select_middle_value_between_left_and_right();
					select_theta_now_Low_Pass_Filter();
					Steering_PID_Control_Using_Gyro();
				}
				// cornering edge margin
				else if(val_forward < 1)
				{
					if(main_wall == 1)
					{
						distance_now = 3;
					}
					if(main_wall == 0)
					{
						distance_now = 6;
					}
					
					select_wall_and_driving_mode();
					select_path_and_PD_control();
					
				}

				// main_wall = find_main_wall();
				ROS_INFO("1speed: %d  angle: %d", speed, servo_angle);
				ROS_INFO("STRAIGHT MODE. Main_wall is %f", main_wall);
			}

			// corner driving
			else if(left_corner_exist == 1 && right_corner_exist == 0)
			{
				// speed_val = 5;
				// count_speed_low_val = 6;
				// count_speed_high_val = 20;


				distance_now = 0.3;

				left_return = 1;
				right_return = 0;
				main_wall = 0;

				if(val_forward > 1.5)
				{
					select_wall_and_driving_mode();
					select_path_and_PD_control();
				}
				else if(val_forward < 1.5 && val_forward > 1)
				{
					select_middle_value_between_left_and_right();
					select_theta_now_Low_Pass_Filter();
					Steering_PID_Control_Using_Gyro();
				}

				else if(val_forward < 1)
				{
					if(main_wall == 1)
					{
						distance_now = 3;
					}
					if(main_wall == 0)
					{
						distance_now = 6;
					}
					
					select_wall_and_driving_mode();
					select_path_and_PD_control();
					
				}
				
				/*
				select_middle_value_between_left_and_right();
				select_theta_now_Low_Pass_Filter();
				Steering_PID_Control_Using_Gyro();
				*/
				ROS_INFO("1speed: %d  angle: %d", speed, servo_angle);
				ROS_INFO("LEFT Cornering Mode. Main_wall is %f", main_wall);				
			}
			else if(left_corner_exist == 0 && right_corner_exist == 1)
			{
				speed_val = 5;

				// count_speed_low_val = 6;
				// count_speed_high_val = 20;

				distance_now = 0.3;

				left_return = 0;
				right_return = 1;
				main_wall = 1;

				if(val_forward > 1.5)
				{
					select_wall_and_driving_mode();
					select_path_and_PD_control();
				}
				else if(val_forward < 1.5 && val_forward < 1)
				{
					select_middle_value_between_left_and_right();
					select_theta_now_Low_Pass_Filter();
					Steering_PID_Control_Using_Gyro();
				}

				else if(val_forward < 1)
				{
					if(main_wall == 1)
					{
						distance_now = 3;
					}
					if(main_wall == 0)
					{
						distance_now = 6;
					}
					
					select_wall_and_driving_mode();
					select_path_and_PD_control();
					
				}
				
				/*
				select_middle_value_between_left_and_right();
				select_theta_now_Low_Pass_Filter();
				Steering_PID_Control_Using_Gyro();
				*/
				ROS_INFO("1speed: %d  angle: %d", speed, servo_angle);
				ROS_INFO("RIGHT Cornering Mode. Main_wall is %f", main_wall);
			}
			else if(left_corner_exist == 1 && right_corner_exist == 1)
			{
				if(left_largest_value > right_largest_value)
				{

					/*
					select_middle_value_between_left_and_right();
					select_theta_now_Low_Pass_Filter();
					Steering_PID_Control_Using_Gyro();
					*/
					ROS_INFO("1speed: %d  angle: %d", speed, servo_angle);
					ROS_INFO("SELECT LEFT Cornering");
				}
				else
				{

					/*
					select_middle_value_between_left_and_right();
					select_theta_now_Low_Pass_Filter();
					Steering_PID_Control_Using_Gyro();
					*/
					ROS_INFO("1speed: %d  angle: %d", speed, servo_angle);
					ROS_INFO("SELECT RIGHT Cornering");
				}

			}
		




		/******************************************************************************************************/


		// Function : Select Desired Path & Servo motor angle correction using PID control and IMU data(Filtered Gyro z data).



		// speed controller test point
		// speed_calculate();	
		// Speed_PID_Control_Using_Accel();


		// Function : Speed Control part
		speed_control();
		servo.data = servo_data;

		ros_tutorial_pub.publish(servo);
		// ROS_INFO("servo: %d", servo.data);	

		// ROS_INFO("IMU Angular Velocity x : [%f], y : [%f], z : [%f]", gyro_x_val, gyro_y_val, gyro_z_val);
		// ROS_INFO("IMU Linear Acceleration x : [%f], y : [%f], z : [%f]", accel_x_val, accel_y_val, accel_z_val);
		
		dt = (float)(clock() - start) / CLOCKS_PER_SEC;
		ROS_INFO("%0.5F\n", (float)(clock() - start) / CLOCKS_PER_SEC);
	}

	void callback2(const std_msgs::UInt16::ConstPtr& msg)
	{
		// ROS_INFO("key: %d", msg->data);
		key = msg->data;
	}

	void callback3(const sensor_msgs::Imu::ConstPtr& msg)
	{
		// using complementary filter
		x_degree_pre = complementary_angle_x;
		y_degree_pre = complementary_angle_y;

		// using complementary filter, gyro hpf
		gyro_pre_angle_x = gyro_angle_x;
		gyro_pre_angle_y = gyro_angle_y;
		gyro_pre_angle_z = gyro_angle_z;

		// gyro hpf
		gyro_previous_hpf_x = gyro_current_hpf_x; 
		gyro_previous_hpf_y = gyro_current_hpf_y; 
		gyro_previous_hpf_z = gyro_current_hpf_z;

		// accel lpf
		accel_previous_lpf_x = accel_current_lpf_x;
		accel_previous_lpf_y = accel_current_lpf_y;
		accel_previous_lpf_z = accel_current_lpf_z;

		gyro_pre_x = gyro_x;
		gyro_pre_y = gyro_y;
		gyro_pre_z = gyro_z;


		// get a raw data of gyro
		gyro_x_val = msg -> angular_velocity.x;
		gyro_y_val = msg -> angular_velocity.y;
		gyro_z_val = msg -> angular_velocity.z;

		// get a raw data of accel
		accel_x_val = msg -> linear_acceleration.x;
		accel_y_val = msg -> linear_acceleration.y;
		accel_z_val = msg -> linear_acceleration.z;
		
		//IMU_Complementary_Filter();	
		//complementary_x_y_degree_varidation();

		gyro_High_Pass_Filter();
		accel_Low_Pass_Filter();

		//ROS_INFO("IMU Seq : [%d]", msg->header.seq);
		//ROS_INFO("IMU Orientation x : [%f], y : [%f], z : [%f], w : [%f]", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
		//ROS_INFO("IMU Angular Velocity x : [%f], y : [%f], z : [%f]", msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
		//ROS_INFO("IMU Linear Acceleration x : [%f], y : [%f], z : [%f]", msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
		
	}

	private:
	ros::NodeHandle nh;
	ros::Publisher ros_tutorial_pub;
	ros::Subscriber ros_tutorial_sub;
	ros::Subscriber ros_tutorial_sub_key;
	ros::Subscriber ros_tutorial_sub_imu;
};

/**
  * Main Function
*/
int main(int argc, char **argv)
{

	ros::init(argc, argv, "ros_tutuorial_msg_subscriber"); 
	SubscribeAndPublish SAPObject;

	ros::spin();


	return 0;
}

/**************************************************************************************************/
/**
  * LaserScan
  * range : -45 degree ~ 225 degree (scan angle : 270 degree)
  * Resolution Capability : 0.25 degree
  * Scan speed : 0.25 ms
  * detection direction : Counter ClockWise
  * saving data value : estimated_current_val_right[i], estimated_current_val_left[i]
  * transfer medium : msg -> ranges[]
  * 0 degree :  msg -> range[180] 
*/

void laserscan_and_obtain_coordinate_data()
{
	for(i = 0; i < 720; i++)
	{
		group_right_x[i] = estimated_current_val_right[i]*cos(i*3.14/720);		// x axis value of the estimated_current_val_right[]
		group_right_y[i] = estimated_current_val_right[i]*sin(i*3.14/720);		// y axis value of the estimated_current_val_right[]
		group_left_x[i] = estimated_current_val_left[i]*cos(3.14-i*3.14/720);	// x axis value of the estimated_current_val_left[]
		group_left_y[i] = estimated_current_val_left[i]*sin(3.14-i*3.14/720);	// y axis value of the estimated_current_val_left[] 
	}
}

/**************************************************************************************************/
/**
  * Calculation of frontal data based on wall
  * math.h => All decimals discarded => floor();
*/

void val_forward_based_on_wall()
{
	// val_forward = msg->ranges[540];			// 90 degree (Farward)
	theta_60method = floorf(57.3 * theta_pre);
		
	range_value = 540 + ((int)4*theta_60method);
	/*
	ROS_INFO("range_value is %d", range_value);
	ROS_INFO("theta_60method is %f", theta_60method);
	*/
}

/**************************************************************************************************/
/**
  * Function purpose : Choose between left wall & right wall
  * Judgement value : Size comparison of value of the "left_group_size" & value of the "right_group_size"
  * Purpose of return value : This function computes the numerical value (left_group_size, right_group_size)
							  of which the difference in distance on the coordinate between the proximity scan values is less,
							  and selects the wall where the value is large.
  * Retrun value : 1 (right_group_size > left_group_size), 0 (left_group_size > right_group_size)
*/

int find_main_wall(void)
{
	left_group_size = 0;
	right_group_size = 0;
	left_out = 0;
	right_out = 0;

	for(i = 0; i < 719; i++)
	{
		if(sqrt((group_right_x[i-right_out]-group_right_x[i+1])*(group_right_x[i-right_out]-group_right_x[i+1])*(group_right_y[i-right_out]-group_right_y[i+1])*(group_right_y[i-right_out]-group_right_y[i+1])) < 0.2)
		{
			right_group_size = right_group_size+right_out + 1;
			right_out = 0;
		}
		else
		{
			right_out++;
		}
		if(right_out > 10)
		{
			break;
		}
	}

	for(i = 0; i < 719; i++)
	{
		if(sqrt((group_left_x[i-left_out]-group_left_x[i+1])*(group_left_x[i-left_out]-group_left_x[i+1])*(group_left_y[i-left_out]-group_left_y[i+1])*(group_left_y[i-left_out]-group_left_y[i+1])) < 0.2)
		{
			left_group_size = left_group_size+left_out + 1;
			left_out = 0;
		}
		else
		{
			left_out++;
		}
		if(left_out > 10)
		{
			break;
		}
	}
	// If right_group_size value is bigger than left_group_size value, return value is 1. Otherwise, return value is 0.
	return right_group_size > left_group_size;
}

/**************************************************************************************************/
/**
  * Function : Linear equation "a" and "b" calculation in Straight mode And Cornering mode
  * In straight mode : Select a left or right wall to calculate a straight line equation
					   by averaging the lidar data up to 45 degree from the horizontal direction
					   of the car body to the vertical direction
  * In Cornering mode : The equation of a straight line is made by using only two pieces of 
					    horizontal direction data and vertical direction data of the vehicle body.
*/

void select_wall_and_driving_mode()
{
	// Select Right wall
	if(main_wall == 1)
	{
		if(val_forward > 1.5)
		{
			x_total = 0;
			y_total = 0;
			a_nominator = 0;
			a_denominator = 0;

			for(i = 0; i < 180; i++)
			{
				x_total += group_right_x[i];
				y_total += group_right_y[i];
			}

			x_avg = x_total / 180;
			y_avg = y_total / 180;

			for(i = 0; i < 180; i++)
			{
				a_nominator += (group_right_x[i] - x_avg) * (group_right_y[i] - y_avg);
				a_denominator += (group_right_x[i] - x_avg) * (group_right_x[i] - x_avg);
			}

			a = a_nominator / a_denominator;
			b = y_avg - a * x_avg;
		}
		// Conering mode
		if(val_forward < 1.5)
		{
			a =- val_forward / estimated_current_val_right[0];
			b = val_forward;
		}
	}

	// Selsct Left wall
	else if(main_wall == 0)
	{
		if(val_forward > 1.5)
		{
			x_total = 0;
			y_total = 0;
			a_nominator = 0;
			a_denominator = 0;

			for(i = 0; i < 180; i++)
			{
				x_total += group_left_x[i];
				y_total += group_left_y[i];
			}
			x_avg = x_total / 180;
			y_avg = y_total / 180;

			for(i = 0; i < 180; i++)
			{
				a_nominator += (group_left_x[i] - x_avg) * (group_left_y[i] - y_avg);
				a_denominator += (group_left_x[i] - x_avg) * (group_left_x[i] - x_avg);
			}

			a = a_nominator / a_denominator;
			b = y_avg - a * x_avg;
		}

		//Conering mode
		if(val_forward < 1.5)
		{
			a = val_forward / estimated_current_val_left[0];
			b = val_forward;
		}
	}
}

/**************************************************************************************************/
/**
  * Select Desired Path & Servo motor angle correction using PD control
  
  * Method : 1. When the coordinate of lidar is assumed to be (0,0) by using the values 
				of "a" and "b" of the straight line equation, 
				the "distance_now" (distance value) is calculated.
			 2. We have a linear equation y = ax+b around the car. Therefore, when the center is the car, 
				the angle with the y-axis is defined as "theta_now", and theta_now is obtained through the "a" value.
			 3. The car is still running. Therefore, assuming that the car is moving by the value "L" specified by the front,
				"L*sin(theta_now)" can be obtained through theta_now angle value. 
			 4. The "err_now" value is selected based on the "desired_path". 
				The value obtained by subtracting "L*sin(theta_now)" and "distance_desired" from the "distance_now" value is "err_now".
			 5. Theta_desired value is calculated using the values of the constants "Kp" and "Kd" of the PD controller 
				and the additional correction constant "C" of the angular value.
			 6. "V_theta" is calculated by subtracting the current "theta_now" value from "theta_desired" which is the required angle value.
				These values will cause the angle of servo-motor required at each instant in the current position to turn more smoothly to the desired angle.
				Recalculate and transfer the "servo_angle" value to the Teensy board that controls the servo motor.
				It will sum up with the speed value data below and deliver the value in message type of "servo.data".
			 7. If the front distance is more than 2.5m, it is judged to be a straight line running and the angle change ratio of servo is reduced to 0.2 times.
				This improves the stability of straight running.
*/

void select_path_and_PD_control()
{	 
	distance_now = b / sqrt((a*a) + 1);
		
	if(distance_now < 0)
	{
		distance_now =- distance_now;
	}
	theta_now = atan(-1/a);

	// ROS_INFO("theta_now is %f", theta_now);
	err_now = distance_now - distance_desired - L*sin(theta_now);

	if(main_wall == 1)
	{
		theta_desired = (Kp*err_now + Kd*(err_now - err_pre) / 0.1) * C;
		// ROS_INFO("main_wall : right wall");
	}
	if(main_wall == 0)
	{
		theta_desired =- (Kp*err_now + Kd*(err_now - err_pre) / 0.1) * C;
		// ROS_INFO("main_wall : left wall");
	}

	V_theta = theta_desired - theta_now;

	if(val_forward > 2.5)
	{
		V_theta = (theta_desired - theta_now) * 0.4;
	}

	servo_angle = 99 + V_theta*180/3.14;

	/*
	ROS_INFO("1speed: %d  angle: %d", speed, servo_angle);
	ROS_INFO("distance: %f  theta: %f", distance_now, theta_now);
	ROS_INFO("err_now: %f  theta_desired: %f", err_now, theta_desired);
	
	ROS_INFO("left_group_size : %d right_group_size : %d", left_group_size, right_group_size);
	*/
}

void select_path_and_PD_control_test()
{	 
	distance_now = b / sqrt((a*a) + 1);
		
	if(distance_now < 0)
	{
		distance_now =- distance_now;
	}
	theta_now = atan(-1/a);

	// ROS_INFO("theta_now is %f", theta_now);
	err_now = distance_now - distance_desired - L*sin(theta_now);

	if(main_wall == 1)
	{
		theta_desired = (Kp*err_now + Kd*(err_now - err_pre) / 0.1) * C;
		// ROS_INFO("main_wall : right wall");
	}
	if(main_wall == 0)
	{
		theta_desired =- (Kp*err_now + Kd*(err_now - err_pre) / 0.1) * C;
		// ROS_INFO("main_wall : left wall");
	}

	V_theta = theta_desired - theta_now;

	if(val_forward > 2.5)
	{
		V_theta = (theta_desired - theta_now) * 0.2;
	}

	servo_angle = 99 + estimated_current_select_theta_now;

	/*
	ROS_INFO("1speed: %d  angle: %d", speed, servo_angle);
	ROS_INFO("distance: %f  theta: %f", distance_now, theta_now);
	ROS_INFO("err_now: %f  theta_desired: %f", err_now, theta_desired);
	
	ROS_INFO("left_group_size : %d right_group_size : %d", left_group_size, right_group_size);
	*/
}

void select_path_and_PD_control_test2()
{
	distance_now = b / sqrt((a*a) + 1);
		
	if(distance_now < 0)
	{
		distance_now =- distance_now;
	}
	theta_now = atan(-1/a);

	// ROS_INFO("theta_now is %f", theta_now);
	err_now = distance_now - distance_desired - L*sin(theta_now);

	if(main_wall == 1)
	{
		theta_desired = (Kp*err_now + Kd*(err_now - err_pre) / 0.1) * C;
		ROS_INFO("main_wall : right wall");
	}
	if(main_wall == 0)
	{
		theta_desired =- (Kp*err_now + Kd*(err_now - err_pre) / 0.1) * C;
		ROS_INFO("main_wall : left wall");
	}

	V_theta = theta_desired - theta_now;

	if(val_forward > 2.5)
	{
		V_theta = (theta_desired - theta_now) * 0.2;
	}

	servo_angle = 99 + V_theta*180/3.14;
	ROS_INFO("servo_angle is %d", servo_angle);
}

/**************************************************************************************************/
/**
  *	Speed Control part
*/

void speed_control()
{

	if(left_corner_exist == 0 && right_corner_exist == 0)
	{
		speed_val = 7;
		count_speed_high_val = 30;
		if(val_forward > 6)
		{
			speed_val = 6;
			count_speed_high_val = 30;
		}


		// count_speed_high_val = 60;
	}	
	else if(left_corner_exist == 1 && right_corner_exist == 0)
	{
		speed_val = 6;
		count_speed_high_val = 30;
	}
	else if(left_corner_exist == 0 && right_corner_exist == 1)
	{
		speed_val = 6;
		count_speed_high_val = 30;
	}

	mode = key / 10000;
	control_speed = key/1000 - mode*10;
	control_angle = key % 1000;
		
	if(mode == 1)
	{
		if(val_forward > 4)
		{
			count_speed = 0;
		}
		// stop count
		if(val_forward < 4)
		{
			count_speed++;
		}
		if(count_speed <= count_speed_low_val)
		{
			speed = 4;
		}
		if(count_speed > count_speed_low_val || count_speed == 0)
		{
			speed = speed_val;
		}
		
		if(count_speed == count_speed_high_val)
		{
			count_speed = 0;
		}
		servo_data = speed * 1000 + servo_angle;
	}

	if(mode == 2)
	{
		servo_data = control_speed*1000 + control_angle;
	}
	if(mode == 3)
	{
		servo_data = 4099;
	}
}

/**************************************************************************************************/
/**
   * Locate the farhest data(Based on LIDAR)
*/

void locate_the_farhest_data()
{
	for(i = 0; i <= 720; i++)
	{
		if(estimated_current_val_right[i] > 65)
		{
			farhest_data_degree[i] = 0.25 * i;
			// ROS_INFO("Farhest location degree is %f ", farhest_data_degree[i]);
		}
		else
		{
			farhest_data_degree[i] = 0;
		}
	}
}

void parameterize_width_size_count()
{
	width_size_count = 10;
	for(i = 0; i < 720; i++)
	{
		if(continuous_farhest_distance_count[i] > width_size_count)
		{
			width_size_count = continuous_farhest_distance_count[i] ;
		}
	}
	width_size_count = width_size_count - width_size_count_scaler;
	// ROS_INFO("width_size_count is %d", width_size_count);
}

/**************************************************************************************************/
/**
  * Data acquisition function to calculate clustering start point and width.
*/
void farhest_crowded_place()
{
	count = 0;

	for(i = 0; i <720; i++)
	{
		degree[i] = 0;
		continuous_farhest_distance_count[i] = 0;

		if(farhest_data_degree[i] > 0)
		{
			count++;
		}
		else if(farhest_data_degree[i] == 0 && count >= width_size_count)
		{
			degree[i-count] = farhest_data_degree[i-count];
			continuous_farhest_distance_count[i-count] = count;
			count = 0;
		}
		else if(farhest_data_degree[i] == 0 && count < width_size_count)
		{
			count = 0;
		}
	}

	for(i = 0; i < 720; i++)
	{
		farhest_distance_width[i] = 2 * 15 * sin(0.25 * continuous_farhest_distance_count[i] * 3.14 / 360); 
	}

	for(i = 0; i < 720; i++)
	{
		if(degree[i] > 0)
		{
			// ROS_INFO("degree is %f, %d", degree[i], i);
		}
		if(continuous_farhest_distance_count[i] > 0)
		{
			// ROS_INFO("continuous_farhest_distance_count is %d, %d", continuous_farhest_distance_count[i], i);
		}
		if(farhest_distance_width[i] > 0)
		{
			// ROS_INFO("width is %f, %d", farhest_distance_width[i], i);
		}
	}
}

// select largest space => parameterize the "width_size_count" value.


int identification_of_cluster_space()
{
	count = 0;

	for(i = 0; i < 720; i++)
	{
		if(degree[i] > 0)
		{
			count++;
		}
	}
	
	if(count > 0)
	{
		return 1;
	}

	return 0;
}

void crowded_place_center_value()
{
	// using data of degree[i] & continuous_farhest_distance_count[i]
	for(i = 0; i < 720; i++)
	{
		if(degree[i] > 0)
		{
			center_degree[i] = degree[i] + 0.25 * continuous_farhest_distance_count[i];
			// ROS_INFO("center degree is %f, %d", center_degree[i], i);
		}
	}
}

void change_angle_base()
{
	for(i = 0; i < 720; i++)
	{
		if(degree[i] > 0)
		{
			center_degree_scaling[i] =  -(center_degree[i] - 90);
			// ROS_INFO("center degree scaling is %f, %d", center_degree_scaling[i], i);
		}
	}
}

void select_largest_crowded_space_only()
{
	total_largest_count_value = 0;
	total_largest_space_center_degree_scaling = 0;

	for(i = 0; i < 720; i++){
		if(degree[i] > 0 && continuous_farhest_distance_count[i] > total_largest_count_value)
		{
			total_largest_count_value = continuous_farhest_distance_count[i];
			total_largest_space_center_degree_scaling = center_degree_scaling[i];
		}
	}
	select_theta_now = total_largest_space_center_degree_scaling;
	// ROS_INFO("total_largest_count_value is %d", total_largest_count_value);
	// ROS_INFO("total_largest_space_center_degree_scaling is %f, select_theta_now is %f", total_largest_space_center_degree_scaling, select_theta_now);
}

void select_proximity_crowded_space_base_on_theta_now()
{
	// theta_now angle => change i data using floorf function in "math.h" library
	// i_value_theta_now = floorf(4 * (-theta_now + 90));
	i_value_theta_now = floorf(4 * (V_theta + 90));

	// ROS_INFO("theta_now is %f. i_value_theta_now is %f.", theta_now, i_value_theta_now);
	// ROS_INFO("V_theta is %f. i_value_theta_now is %f.", V_theta, i_value_theta_now);

	// Selection of proximity "center_degree[i]" to the left and right of "theta_now"

	left_center_degree_scaling = 0;
	right_center_degree_scaling = 0;

	left_space = 0;
	right_space = 0;

	left_space_count_start_point = 0;
	right_space_count_start_point = 0;

	// choose only one left space
	for(i = i_value_theta_now; i < 720; i++)
	{
		if(degree[i] > 0)
		{
			left_center_degree[i] = center_degree[i];
			left_center_degree_scaling = center_degree_scaling[i];
			left_space = 1;
			left_space_count_start_point = i;
			// ROS_INFO("Left center drgree scaling is %f, %d.", left_center_degree_scaling, i);
			break;
		}
		else if(degree[i] == 0)
		{
			left_space = 0;
		}
	}
	
	// choose only one right space
	for(i = i_value_theta_now; i >= 0; i--)
	{
		if(degree[i] > 0)
		{
			right_center_degree[i] = center_degree[i];
			right_center_degree_scaling = center_degree_scaling[i];
			right_space = 1;
			right_space_count_start_point = i;
			// ROS_INFO("Right center drgree scaling is %f, %d.", right_center_degree_scaling, i);
			break;
		}
		else if(degree[i] == 0)
		{
			right_space = 0;
		}
	}
	// ROS_INFO("left space is %d, right space is %d.", left_space, right_space);
}

// Method 1
void choose_cluster_point_front_of_the_car()
{
	if(left_space == 1 && right_space == 1)
	{
		if(fabsf(left_center_degree_scaling) > fabsf(right_center_degree_scaling))
		{
			select_direction = 0;	// right
			select_theta_now = right_center_degree_scaling;
		}
		else if(fabsf(left_center_degree_scaling) < fabsf(right_center_degree_scaling))
		{
			select_direction = 1;	// left
			select_theta_now = left_center_degree_scaling;
		}
	}

	else if(left_space == 1 && right_space == 0)
	{
		select_direction = 1;	// left
		select_theta_now = left_center_degree_scaling;

	}
	else if(left_space == 0 && right_space == 1)
	{
		select_direction = 0;	// right
		select_theta_now = right_center_degree_scaling;
	}
	else
	{
		select_theta_now = theta_now;
	}

	// ROS_INFO("selection direction is %d", select_direction);
	// ROS_INFO("selection theta now is %f", select_theta_now);
}

// Method 2

void choose_largest_cluster_point_base_on_theta_now()
{
	// using continuous_farhest_distance_count[i], left_space_count_start_point, right_space_count_start_point.

	if(left_space == 1 && right_space == 1)
	{
		if(continuous_farhest_distance_count[left_space_count_start_point] < continuous_farhest_distance_count[right_space_count_start_point])
		{
			select_direction = 0;	// right
			select_theta_now = right_center_degree_scaling;
		}
		else if(continuous_farhest_distance_count[left_space_count_start_point] > continuous_farhest_distance_count[right_space_count_start_point])
		{
			select_direction = 1;	// left
			select_theta_now = left_center_degree_scaling;
		}
	}

	else if(left_space == 1 && right_space == 0)
	{
		select_direction = 1;	// left
		select_theta_now = left_center_degree_scaling;

	}
	else if(left_space == 0 && right_space == 1)
	{
		select_direction = 0;	// right
		select_theta_now = right_center_degree_scaling;
	}
	else
	{
		select_theta_now = theta_now;
	}

	// ROS_INFO("select_direction is %d. 0: right, 1 : left", select_direction);
	// ROS_INFO("left_center_degree_scaling is %f. right_center_degree_scaling is %f", left_center_degree_scaling, right_center_degree_scaling);
	// ROS_INFO("select_theta_now is %f.", select_theta_now);
}


void collect_distance_data_within_15m()
{
	for(i = 0; i <= 720; i++)
	{
		if(estimated_current_val_right[i] < 15)
		{
			distance_data_within_15m[i] = estimated_current_val_right[i];
			// ROS_INFO("distance data within 15m is %f. %f", distance_data_within_15m[i], (float)i/4);
		}
		else
		{
			distance_data_within_15m[i] = 0;
		}
	}
}

void find_largest_distance_data_on_left_right_respectively()
{
	right_largest_value = distance_data_within_15m[0];
	left_largest_value = distance_data_within_15m[361];
	
	right_corner_exist = 0;
	left_corner_exist = 0;

	// right largest value, degree
	for(i = 0; i < 360; i++)
	{
		if(distance_data_within_15m[i] >= right_largest_value)
		{
			right_largest_value = distance_data_within_15m[i];
			right_largest_degree = (float)i / 4;
		}
	}
	// ROS_INFO("ORIGINAL right_largest_value is %f, right_largest_degree is %f", right_largest_value, right_largest_degree);

	// extract corner value
	for(i = right_largest_degree*4 + 1; i > 0; i--)
	{
		if(distance_data_within_15m[i] == 0)
		{
			right_largest_value_next[i] = 0;
			right_largest_degree_next[i] = 0;
		}
		else if(distance_data_within_15m[i] != 0)
		{
			right_largest_value_next[i] = distance_data_within_15m[i];
			right_largest_degree_next[i] = (float)i / 4;

			// if corner exists,
			if (right_largest_value_next[i + 1] > 2 * right_largest_value_next[i])
			{
				right_largest_value = right_largest_value_next[i + 1];
				right_largest_degree = (float)(i + 1) / 4;
				right_corner_previous_value = right_largest_value_next[i];
				right_corner_previous_degree = (float)i / 4;

				right_corner_exist = 1;

				// ROS_INFO("NEW right_largest_value is %f, right_largest_degree is %f", right_largest_value, right_largest_degree);
				// ROS_INFO("NEW right_corner_previous_value is %f, right_corner_previous_degree is %f", right_corner_previous_value, right_corner_previous_degree);
				// ROS_INFO("right_corner_exist is %d", right_corner_exist);
				break;
			}
		}
	}

	// left largest value, degree
	for(i = 361; i < 720; i++)
	{
		if(distance_data_within_15m[i] >= left_largest_value)
		{
			left_largest_value = distance_data_within_15m[i];
			left_largest_degree = (float)i / 4;
		}
	}
	// ROS_INFO("ORIGINAL left_largest_value is %f, left_largest_degree is %f", left_largest_value, left_largest_degree);

	// extract corner value
	for(i = left_largest_degree*4 + 1; i < 720; i++)
	{
		if(distance_data_within_15m[i] == 0)
		{
			left_largest_value_next[i] = 0;
			left_largest_degree_next[i] = 0;
		}
		else if(distance_data_within_15m[i] != 0)
		{
			left_largest_value_next[i] = distance_data_within_15m[i];
			left_largest_degree_next[i] = (float)i / 4;

			// if corner exists,
			if (left_largest_value_next[i - 1] > 2 * left_largest_value_next[i])
			{
				left_largest_value = left_largest_value_next[i - 1];
				left_largest_degree = (float)(i - 1) / 4;
				left_corner_previous_value = left_largest_value_next[i];
				left_corner_previous_degree = (float)i / 4;

				left_corner_exist = 1;

				// ROS_INFO("NEW left_largest_value is %f, left_largest_degree is %f", left_largest_value, left_largest_degree);
				// ROS_INFO("NEW left_corner_previous_value is %f, left_corner_previous_degree is %f", left_corner_previous_value, left_corner_previous_degree);
				// ROS_INFO("left_corner_exist is %d", left_corner_exist);
				break;
			}
		}
	}
}

void change_angle_base_2()
{
	right_largest_degree_scaling =  -(right_largest_degree - 90);
	 // ROS_INFO("right largest degree scaling is %f.", right_largest_degree_scaling);

	left_largest_degree_scaling = -(left_largest_degree - 90);
	 // ROS_INFO("left largest degree scaling is %f.", left_largest_degree_scaling);
	
	right_corner_previous_degree_scaling =  -(right_corner_previous_degree - 90);
	if(right_corner_exist)
	{
		// ROS_INFO("right_corner_previous_degree_scaling is %f.", right_corner_previous_degree_scaling );
	}
	left_corner_previous_degree_scaling = -(left_corner_previous_degree - 90);
	if(left_corner_exist)
	{
		// ROS_INFO("left_corner_previous_degree_scaling is %f.", left_corner_previous_degree_scaling);
	}

}

void select_middle_value_between_left_and_right()
{
	x_left_largest_value = left_largest_value * sin(left_largest_degree_scaling * 3.14 / 180);	
	y_left_largest_value = left_largest_value * cos(left_largest_degree_scaling * 3.14 / 180);
	
	x_right_largest_value = right_largest_value * sin(right_largest_degree_scaling * 3.14 / 180);	
	y_right_largest_value = right_largest_value * cos(right_largest_degree_scaling * 3.14 / 180);
	
	select_theta_now = 180 / 3.14 * atan((x_left_largest_value + x_right_largest_value) / (y_left_largest_value + y_right_largest_value));
	//ROS_INFO("select_theta_now is %f.", select_theta_now);
}
/*
void calculate_val_forward_corner()
{
	if(left_largest_value > (2 * left_largest_value_next))
	{
		val_forward_corner = val_forward - (val_forward * left_largest_value_next / left_largest_value);
	}
	else if(right_largest_value > (2 * right_largest_value_next))
	{
		val_forward_corner = val_forward - (val_forward * right_largest_value_next / right_largest_value);
	}
	ROS_INFO("val_forward_corner is %f", val_forward_corner);

}
*/

void RIDAR_Low_Pass_Filter()
{
	float alpha = 0.7;

	for(i = 0; i < 720; i++)
	{
		estimated_current_val_right[i] = alpha * estimated_previous_val_right[i] + (1 - alpha) * current_val_right[i];
		estimated_current_val_left[i] = alpha * estimated_previous_val_left[i] + (1 - alpha) * current_val_left[i];	
	}
	// ROS_INFO("estimated_current_val_right[360] is %f", estimated_current_val_right[360]);
	// ROS_INFO("current_val_right[360] is %f", current_val_right[360]);
}


void select_theta_now_Low_Pass_Filter()
{
	float alpha = 0.7;
	
	estimated_current_select_theta_now = alpha * estimated_previous_select_theta_now + (1 - alpha) * select_theta_now;

	ROS_INFO("estimated_current_select_theta_now is %f", estimated_current_select_theta_now);
}

void IMU_Complementary_Filter()
{
	// gyro : rad/sec => deg/sec
	gyro_x = gyro_x_val * 57.3;
	gyro_y = gyro_y_val * 57.3;
	gyro_z = gyro_z_val * 57.3;

	gyro_angle_x = gyro_x * dt + gyro_pre_angle_x;
	gyro_angle_y = gyro_y * dt + gyro_pre_angle_y;
	gyro_angle_z = gyro_z * dt + gyro_pre_angle_z;

	// accel : The angle is obtained using the acceleration data.
	accel_x = atan(accel_y_val / sqrt(pow(accel_x_val, 2) + pow(accel_z, 2))) * 57.3;
	accel_y = atan(-1 * accel_x_val / sqrt(pow(accel_y_val, 2) + pow(accel_z, 2))) * 57.3;
	accel_z = 0;

	// Apply the Complementary Filter
	complementary_angle_x = 0.96 * (complementary_angle_x + gyro_angle_x * dt) + 0.04 * accel_x;
	complementary_angle_y = 0.96 * (complementary_angle_y + gyro_angle_y * dt) + 0.04 * accel_y;
	complementary_angle_z = complementary_angle_z + gyro_z * dt;
	// You can see how long the angle has been running while the code is running.
		
	// ROS_INFO("gyro_x : [%f], y : [%f], z : [%f]", gyro_x, gyro_y, gyro_z);
	// ROS_INFO("gyro_angle_x : [%f], y : [%f], z : [%f]", gyro_angle_x, gyro_angle_y, gyro_angle_z);
	// ROS_INFO("Accel+Gyro Complementary Filter output Angle x : [%f], y : [%f], z : [%f]", complementary_angle_x, complementary_angle_y, complementary_angle_z);
}

void select_theta_now_High_Pass_Filter()
{
	float alpha = 0.7;

	estimated_current_select_theta_now = alpha * estimated_previous_select_theta_now + alpha * (select_theta_now - select_theta_pre);
}

void complementary_x_y_degree_varidation()
{
	x_degree_validation = complementary_angle_x - x_degree_pre;
	y_degree_validation = complementary_angle_y - y_degree_pre;
	
	// ROS_INFO("x_degree_validation is [%f]. y_degree_validation is [%f]", x_degree_validation, y_degree_validation);

}

void gyro_High_Pass_Filter()
{
	float alpha = 0.7;

	// gyro : rad/sec => deg/sec
	gyro_x = gyro_x_val * 57.3;
	gyro_y = gyro_y_val * 57.3;
	gyro_z = gyro_z_val * 57.3;
	
	gyro_current_hpf_x = alpha * gyro_previous_hpf_x + alpha * (gyro_x - gyro_pre_x); 
	gyro_current_hpf_y = alpha * gyro_previous_hpf_y + alpha * (gyro_y - gyro_pre_y); 
	gyro_current_hpf_z = alpha * gyro_previous_hpf_z + alpha * (gyro_z - gyro_pre_z); 

	//  ROS_INFO("gyro_current_hpf_x is [%f], gyro_current_hpf_y is [%f], gyro_current_hpf_z is [%f].", gyro_current_hpf_x, gyro_current_hpf_y, gyro_current_hpf_z);

}

void Steering_PID_Control_Using_Gyro()
{
	float steering_Kp = 10;
	float steering_Ki = 5;
	float steering_Kd = 0.001;

	float P_control, I_control, D_control;

	float Control_PID_theta;

	error_now = gyro_current_hpf_z * dt - estimated_current_select_theta_now;
	
	P_control = steering_Kp * error_now;
	I_control = steering_Ki * error_now * dt;
	D_control = steering_Kd * (error_now - error_pre) / dt;

	if(space_availability == 1)
	{
		Control_PID_theta = (P_control + I_control + D_control) * 0.1;
	// ROS_INFO("Control_PID_theta is %f", Control_PID_theta);
	}
	else if (space_availability == 0)
	{
		Control_PID_theta = (P_control + I_control + D_control) * 0.07;
		 ROS_INFO("Control_PID_theta is %f", Control_PID_theta);
	}
	
	servo_angle = 99 - Control_PID_theta;
}

void accel_Low_Pass_Filter()
{
	float alpha = 0.5;

	accel_current_lpf_x = alpha * accel_previous_lpf_x + (1 - alpha) * accel_x_val;
	accel_current_lpf_y = alpha * accel_previous_lpf_y + (1 - alpha) * accel_y_val;
	accel_current_lpf_z = alpha * accel_previous_lpf_z + (1 - alpha) * accel_z_val;
	
	// ROS_INFO("accel_current_lpf_x is [%f], accel_current_lpf_y is [%f], accel_current_lpf_z is [%f].", accel_current_lpf_x, accel_current_lpf_y, accel_current_lpf_z);
}

void speed_calculate()
{
	if(accel_current_lpf_x >= 0)
	{
		current_accel_speed = sqrt(pow(accel_current_lpf_x, 2) + pow(accel_current_lpf_y, 2));
		// ROS_INFO("current_accel_speed is %f", current_accel_speed);
	}
	else if(accel_current_lpf_x < 0)
	{
		current_accel_speed = -sqrt(pow(accel_current_lpf_x, 2) + pow(accel_current_lpf_y, 2));
		// ROS_INFO("current_accel_speed is %f", current_accel_speed);
	}
	current_speed += current_accel_speed * dt;
	// ROS_INFO("current_speed is %f", current_speed);
}

void Speed_PID_Control_Using_Accel()
{
	float speed_Kp = 0;
	float speed_Ki = 0;
	float speed_Kd = 0;

	float P_control, I_control, D_control;

	float Control_PID_speed;

	speed_error_now = current_speed - speed;
	
	P_control = speed_Kp * speed_error_now;
	I_control = speed_Ki * speed_error_now * dt;
	D_control = speed_Kd * (speed_error_now - speed_error_pre) / dt;

	Control_PID_speed = (P_control + I_control + D_control) * 0.1;
	// ROS_INFO("Control_PID_speed is %f", Control_PID_speed);
	
	servo_data = Control_PID_speed * 1000 + servo_angle;
}


