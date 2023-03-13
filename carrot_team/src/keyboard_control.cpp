#include "ros/ros.h"
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"
#include "carrot_team/orientation.hpp"
#include <iostream> // std
#include <cstring>	// memcpy
#include <signal.h>

#ifndef _WIN32
#include <termios.h> // tcgetattr, tcsetattr
#include <unistd.h>	 // read
#else
#include <windows.h>
#endif

#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_D 0x64
#define KEYCODE_F 0x66
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_Q 0x71

#define PI 3.14159265358979323846

class KeyboardReader
{
	/* This class represents the custom terminal
	 */

public:
	KeyboardReader()
		: fd(0)
	{
		// user mode in terminal
		tcgetattr(fd, &terattr);
		struct termios user;
		memcpy(&user, &terattr, sizeof(struct termios));
		user.c_lflag &= ~(ICANON | ECHO);

		// setting a new line, then end of file
		user.c_cc[VEOL] = 1;
		user.c_cc[VEOF] = 2;

		tcsetattr(fd, TCSANOW, &user);
	}

	void readOne(char *c)
	{
#ifndef _WIN32
		int rc = read(fd, c, 1); // char = 1byte, return fail = -1
		if (rc < 0)
		{
			throw std::runtime_error("read failed");
		}
#else
		for (;;)
		{
			HANDLE handle = GetStdHandle(STD_INPUT_HANDLE);
			INPUT_RECORD buffer;
			DWORD events;
			PeekConsoleInput(handle, &buffer, 1, &events);
			if (events > 0)
			{
				ReadConsoleInput(handle, &buffer, 1, &events);
				if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_UP)
				{
					*c = KEYCODE_UP;
					return;
				}
				else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_DOWN)
				{
					*c = KEYCODE_DOWN;
					return;
				}
				else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_LEFT)
				{
					*c = KEYCODE_LEFT;
					return;
				}
				else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_RIGHT)
				{
					*c = KEYCODE_RIGHT;
					return;
				}
				else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x44)
				{
					*c = KEYCODE_D;
					return;
				}
				else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x46)
				{
					*c = KEYCODE_F;
					return;
				}
				else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x41)
				{
					*c = KEYCODE_A;
				}
				else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x53)
				{
					*c = KEYCODE_S;
				}
				else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x51)
				{
					*c = KEYCODE_Q;
					return;
				}
			}
		}
#endif
	}

	void shutdown()
	{
#ifndef _WIN32
		tcsetattr(fd, TCSANOW, &terattr);
#endif
	}

private:
#ifndef _WIN32
	int fd;
	struct termios terattr;
#endif
};

// create terminal object
KeyboardReader input;

class TeleopDrone
{
	/* This class represents teleoperating control
	 */
public:
	TeleopDrone();
	void keyLoop();

private:
	ros::NodeHandle _nh;
	float _x, _y, _z;
	EulerAngles _angle;
	ros::Publisher _trajectory_point_pub;
};

TeleopDrone::TeleopDrone() : _x(10.0), _y(2.0), _z(2.0)
{
	_angle.yaw   = 0;
	_angle.roll  = 0; 
	_angle.pitch = 0;
	_trajectory_point_pub = _nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("red/position_hold/trajectory", 1000);
}

void quit(int sig)
{
	(void)sig;
	input.shutdown();
	ros::shutdown();
	exit(0);
}

void TeleopDrone::keyLoop()
{
	char c;
	bool dirty = false;

	puts("Reading from keyboard");
	puts("-------------------------");
	puts("Use arrow keys to move the drone. 'q' to quit.");
	puts("uparrow:    +1 in y axes    |    downarrow:  -1 in y axes");
	puts("rightarrow: +1 in x axes    |    leftarrow:  -1 in x axes");
	puts("keyboard F: +1 in z axes    |    keyboard D: -1 in z axes");
	puts("keyboard S: +30deg in yaw   |    keyboard A: -30deg in yaw");
	for (;;)
	{
		// get the next event from the keyboard
		try
		{
			input.readOne(&c);
		}
		catch (const std::runtime_error &)
		{
			perror("read():");
			return;
		}

		// _x=_y=_z=0; does not need initializing
		ROS_DEBUG("value: 0x%02X\n", c);

		switch (c)
		{
		case KEYCODE_UP:
			ROS_INFO("Y: +1[m]");
			_y += 1;
			dirty = true;
			break;
		case KEYCODE_DOWN:
			ROS_INFO("Y: -1[m]");
			_y += -1;
			dirty = true;
			break;
		case KEYCODE_RIGHT:
			ROS_INFO("X: +1[m]");
			_x += 1;
			dirty = true;
			break;
		case KEYCODE_LEFT:
			ROS_INFO("X: -1[m]");
			_x += -1;
			dirty = true;
			break;
		case KEYCODE_D:
			ROS_INFO("Z: -1[m]");
			_z += -1;
			dirty = true;
			break;
		case KEYCODE_F:
			ROS_INFO("Z: +1[m]");
			_z += +1;
			dirty = true;
			break;
		case KEYCODE_A:
			ROS_INFO("Yaw: -30[deg]");
			_angle.yaw -= PI/6;
			dirty = true;
			break;
		case KEYCODE_S:
			ROS_INFO("Yaw: +30[deg]");
			_angle.yaw += PI/6;
		case KEYCODE_Q:
			ROS_INFO("quit");
			return;
		}

		Quaternion q;
		q = ToQuaternion(_angle);

		trajectory_msgs::MultiDOFJointTrajectoryPoint point_msg;
		point_msg.transforms.resize(1);
		point_msg.velocities.resize(1);
		point_msg.accelerations.resize(1);

		point_msg.transforms[0].translation.x = _x;
		point_msg.transforms[0].translation.y = _y;
		point_msg.transforms[0].translation.z = _z;
		point_msg.transforms[0].rotation.x = q.x;
		point_msg.transforms[0].rotation.y = q.y;
		point_msg.transforms[0].rotation.z = q.z;
		point_msg.transforms[0].rotation.w = q.w;
		
		if (dirty == true)
		{
			_trajectory_point_pub.publish(point_msg);
			dirty = false;
		}
	}

	return;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "keyboard_control");
	TeleopDrone teleop_drone;

	signal(SIGINT, quit);

	teleop_drone.keyLoop();
	quit(0);

	return 0;
}
