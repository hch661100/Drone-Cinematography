#include "input.h"

using uav_utils::in_range;

RC_Data_t::RC_Data_t()
{
	rcv_stamp = ros::Time(0);
	
	last_mode = -1.0;
	set_default_mode(std::string("manual"));
	set_default_mode(std::string("noapi"));

	is_command_mode = false;
	enter_command_mode = false;
	is_api_mode = false;
	enter_api_mode = false;
}

void RC_Data_t::set_default_mode(std::string s)
{
	if (s=="manual")
	{
		last_gear = MANUAL_MODE_GEAR_VALUE;
	}
	else if (s=="command")
	{
		last_gear = COMMAND_MODE_GEAR_VALUE;
	}
	else if (s=="api")
	{
		last_mode = 0.8;
	}
	else if (s=="noapi")
	{
		last_mode = -0.8;
	}
	else
	{
		ROS_ASSERT_MSG(false, "Invalid mode for RC");
	}
}

void RC_Data_t::feed(sensor_msgs::JoyConstPtr pMsg)
{
	msg = *pMsg;
	rcv_stamp = ros::Time::now();

	roll = msg.axes[0];
	pitch = msg.axes[1];
	yaw = msg.axes[2];
	thr = msg.axes[3];
	mode = msg.axes[4];
	gear = msg.axes[5];

	check_validity();

	if (last_mode < API_MODE_THRESHOLD_VALUE && mode > API_MODE_THRESHOLD_VALUE) 
		enter_api_mode = true;
	else
		enter_api_mode = false;
	
	if (mode > API_MODE_THRESHOLD_VALUE) 
		is_api_mode = true;
	else
		is_api_mode = false;

	if (last_gear < GEAR_SHIFT_VALUE && gear > GEAR_SHIFT_VALUE)
	{
		enter_command_mode = true;
	}
	else if (gear < GEAR_SHIFT_VALUE)
	{
		enter_command_mode = false;
	}	

	if (gear > GEAR_SHIFT_VALUE)
		is_command_mode = true;
	else
		is_command_mode = false;

	last_mode = mode;
	last_gear = gear;
}

bool RC_Data_t::check_enter_command_mode()
{
	if (enter_command_mode)
	{
		enter_command_mode = false;
		return true;
	}
	else
	{
		return false;
	}
}

void RC_Data_t::check_validity()
{
	if (in_range(roll, 1.0)
		&& in_range(pitch, 1.0)
		&& in_range(yaw, 1.0)
		&& in_range(thr, 1.0)
		&& in_range(mode, -1.0, 1.0)
		&& in_range(gear, -1.0, 1.0)
	)
	{
		// pass
	}
	else
	{
		ROS_ERROR("RC data validity check fail.");
	}
}

Odom_Data_t::Odom_Data_t()
{
	rcv_stamp = ros::Time(0);
};

void Odom_Data_t::feed(nav_msgs::OdometryConstPtr pMsg)
{
	msg = *pMsg;
	rcv_stamp = ros::Time::now();

	uav_utils::extract_odometry(pMsg, p, v, q, w);
}

Imu_Data_t::Imu_Data_t()
{
	rcv_stamp = ros::Time(0);
}

void Imu_Data_t::feed(sensor_msgs::ImuConstPtr pMsg)
{
	msg = *pMsg;
	rcv_stamp = ros::Time::now();

	w(0) = msg.angular_velocity.x;
	w(1) = msg.angular_velocity.y;
	w(2) = msg.angular_velocity.z;
	
	a(0) = msg.linear_acceleration.x; 
	a(1) = msg.linear_acceleration.y;
	a(2) = msg.linear_acceleration.z;
	
	q.x() = msg.orientation.x;
	q.y() = msg.orientation.y;
	q.z() = msg.orientation.z;
	q.w() = msg.orientation.w;
}

Command_Data_t::Command_Data_t()
{
	rcv_stamp = ros::Time(0);
	trajectory_id = 0;
	trajectory_flag = 0;
}

template <typename T>
T _normalize_angle(T a)
{
    int cnt = 0;
    while(true)
    {
        cnt++;

        if (a < -M_PI)
        {
            a += M_PI*2;
        }
        else if (a > M_PI)
        {
            a -= M_PI*2;
        }

        if (-M_PI<=a && a <=M_PI)
        {
            break;
        };

        assert(cnt < 10 && "[uav_utils/geometry_msgs] INVALID INPUT ANGLE");
    }
   return a;
}

void Command_Data_t::feed(quadrotor_msgs::PositionCommandConstPtr pMsg)
{
	msg = *pMsg;
	rcv_stamp = ros::Time::now();

	p(0) = msg.position.x;
	p(1) = msg.position.y;
	p(2) = msg.position.z;

	v(0) = msg.velocity.x;
	v(1) = msg.velocity.y;
	v(2) = msg.velocity.z;

	a(0) = msg.acceleration.x;
	a(1) = msg.acceleration.y;
	a(2) = msg.acceleration.z;

	yaw  = _normalize_angle(msg.yaw);
	//yaw  = uav_utils::normalize_angle(msg.yaw);
	yaw_dot = msg.yaw_dot;

	trajectory_id = msg.trajectory_id;
	trajectory_flag = msg.trajectory_flag;
}


Idling_Data_t::Idling_Data_t()
{
	rcv_stamp = ros::Time(0);
	need_idling = false;
}

void Idling_Data_t::feed(geometry_msgs::Vector3StampedConstPtr pMsg)
{
	msg = *pMsg;
	rcv_stamp = ros::Time::now();

	need_idling = (msg.header.frame_id.compare(std::string("idling"))==0);
}
