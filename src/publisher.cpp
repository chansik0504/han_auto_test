/**
  *******************************************************************
  * @file		publisher.cpp
  * @author		Seunghan Han
  * @version	V.2.0	(version 1 : oroca_ros_tutorials)
  * @date		4-March-2018
  * @brief		This file provide get a keyboard command and pubilsh 
  *				mesage data.
  *******************************************************************
  */



#include <stdio.h>				// standard IO headerfile
#include "ros/ros.h"			// ROS fundermental headerfile
#include <std_msgs/UInt16.h>	// Headerfile containing ROS message type that is 16bit unsigned integer 
#include <termios.h>			// Headerfile containing functions that can control terminal properties




	// Define a macro constant at unistd.h
	// On Linux systems, the process refers to a file descripor for each file opened for file I/O.

	// When a process calls the open function, the kernel of the Linux system either opens an existing file or 
	// creates a new file and retruns a file descripter.
	// A file descriptor is an unsigned integer that guarantees a unique number for each unit
	// that performs file I/O in a process.
	// In particular, not the developer code,
	// a process running in the Linux sheel opens standard input, standard output, and standard error file in forming a process,
	// and their file descripters are 0, 1, and 2.

	// STDIN_FILENO : 0, STDOUT_FILENO : 1, STDERR_FILENO : 2
	// TCSANOW : Terminal attribute value changes effect as soon as tcsetattr is called.
	// ---------------------------------------------------------------------------------------
	// Utilize termios.h
	
	// get a terminal attrbute value.
	// int tcgetattr(int fd, struct termios *termios_p);
	// To change the terminal attribute value, first, the terminal's property value is received
	// current terminal value => fd
	// save the terminal attribute value at terminos structure => termios_p

	// tcgetattr -> tcsetattr : apply
	// int tcsetattr(int fd, int optional_actions, const struct termios *termios_p);
	// Using tcsetattr function, you can change the terminal attribute value
	// ---------------------------------------------------------------------------------------

	// getch function : 
	// Instead of using the buffer, 
	// it takes the value at the same time as you press and release the keyboard.

int getch()
{
	static struct termios oldt, newt;
	tcgetattr(STDIN_FILENO, &oldt);			// save old settings

	newt = oldt;
	newt.c_lflag &= ~(ICANON);					// disable buffering

	tcsetattr(STDIN_FILENO, TCSANOW, &newt);	// apply new settings

	int c = getchar();							// read character (non-blocking)
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);	// restore old settings

	return c;
}

// Node Main Function
int main(int argc, char **argv)
{
	ros::init(argc, argv, "publisher");	// Initialize Node Name 
	ros::NodeHandle nh;	// Declaration a Node handle for communicating the ROS system.		

	// Declaration a publisher,
	// Using UInt16 of std_msgs
	// Create the publisher : ros_tutorial_pub
	// Topic name : /keyboard
	// Queue size : 100
	ros::Publisher ros_tutorial_pub = nh.advertise<std_msgs::UInt16>("/keyboard", 100);
	ros::Rate loop_rate(10);			// Loop Period : 10Hz. (0.1sec)  


	// Data transfer means : message
	// 

	int servo = 99;						// initial servo angle value : center
	int speed = 4;						// initial speed value : stop 
	int mode = 2;							// initial mode value : control mode

	while (ros::ok())
	{
		std_msgs::UInt16 msg;
		int c = getch();

		// Keyboard Up, Down, Left, Right Control
		if(c == 65 && mode == 2 && speed < 7)		// Up
		{
			speed += 1;
			msg.data = servo + speed*1000 + mode*10000;
			ROS_INFO("send msg = %d", msg.data);
			ros_tutorial_pub.publish(msg);
		}
		if(c == 66 && mode == 2 && speed > 2)		// Down
		{
			speed -= 1;
			msg.data = servo + speed*1000 + mode*10000;
			ROS_INFO("send msg = %d", msg.data);
			ros_tutorial_pub.publish(msg);
		}



		if(c == 67 && mode == 2 && servo < 140)		// Right
		{
			servo += 5;
			msg.data = servo + speed*1000 + mode*10000;
			ROS_INFO("send msg = %d", msg.data);
			ros_tutorial_pub.publish(msg);
		}
		if(c == 68 && mode == 2 && servo > 55)		// Left
		{
			servo -= 5;
			msg.data = servo + speed*1000 + mode*10000;
			ROS_INFO("send msg = %d", msg.data);
			ros_tutorial_pub.publish(msg);
		}

		// Select Stop mode, Keyboard control mode, Autonomous mode
		if(c == 99)		// if c key in keyboard is pressed,
		{
			servo=99;
			if(mode == 1)
			{
				mode = 2;
			}
			else if(mode == 2)
			{
				mode = 1;
			}
			else if(mode == 3)
			{
				mode = 2;
			}
			msg.data = servo + speed*1000 + mode*10000;
			ROS_INFO("send msg = %d", msg.data);
			ros_tutorial_pub.publish(msg);
		}

		if(c == 32) // if spacebar in keyboard is pressed,
		{
			mode = 3;
			servo = 99;
			speed = 4;
			msg.data = servo + speed*1000 + mode*10000;
			ROS_INFO("send msg = %d", msg.data);
			ros_tutorial_pub.publish(msg);
		}

		loop_rate.sleep();
	}
	return 0;
}


