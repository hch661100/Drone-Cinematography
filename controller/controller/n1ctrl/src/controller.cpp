#include "controller.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <uav_utils/converters.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <boost/format.hpp>


using namespace Eigen;
using std::cout;
using std::endl;
using namespace uav_utils;

Controller::Controller(Parameter_t& param_):
	param(param_)
{
	is_configured = false;
}

void Controller::config()
{
	config_gain(param.hover_gain);
	is_configured = true;
}

void Controller::config_gain(const Parameter_t::Gain& gain)
{
	Kp.setZero();
	Kv.setZero();
	Ka.setZero();
	Kr.setZero();
	Kw.setZero();
	Kp(0,0) = gain.Kp0;
	Kp(1,1) = gain.Kp1;
	Kp(2,2) = gain.Kp2;
	Kv(0,0) = gain.Kv0;
	Kv(1,1) = gain.Kv1;
	Kv(2,2) = gain.Kv2;
	Ka(0,0) = gain.Ka0;
	Ka(1,1) = gain.Ka1;
	Ka(2,2) = gain.Ka2;
	Kr(0,0) = gain.Kr0;
	Kr(1,1) = gain.Kr1;
	Kr(2,2) = gain.Kr2;
	Kw(0,0) = gain.Kw0;
	Kw(1,1) = gain.Kw1;
	Kw(2,2) = gain.Kw2;
}

void Controller::update(
	const Desired_State_t& des, 
	const Odom_Data_t& odom, 
	Controller_Output_t& u, 
	SO3_Controller_Output_t& u_so3
)
{
	ROS_ASSERT_MSG(is_configured, "Gains for controller might not be initialized!");
	std::string constraint_info("");
	Vector3d e_p, e_v, F_des;

	e_p = odom.p - des.p;
	e_v = odom.v - des.v;

	double yaw_des = des.yaw;
	double yaw_curr = get_yaw_from_quaternion(odom.q);

	Matrix3d wRc = rotz(yaw_curr);
	
	F_des = -Kp * e_p - Kv * e_v + 
		Vector3d(0, 0, param.mass * param.gra) + Ka * param.mass * des.a;
	
	if (F_des(2) < 0.5 * param.mass * param.gra)
	{
		constraint_info = boost::str(
			boost::format("thrust too low F_des(2)=%.3f; ")% F_des(2));
		F_des = F_des / F_des(2) * (0.5 * param.mass * param.gra);
	}
	else if (F_des(2) > 2 * param.mass * param.gra)
	{
		constraint_info = boost::str(
			boost::format("thrust too high F_des(2)=%.3f; ")% F_des(2));
		F_des = F_des / F_des(2) * (2 * param.mass * param.gra);
	}

	if (std::fabs(F_des(0)/F_des(2)) > std::tan(toRad(50.0)))
	{
		constraint_info += boost::str(boost::format("x(%f) too tilt; ")
			% toDeg(std::atan2(F_des(0),F_des(2))));
		F_des(0) = F_des(0)/std::fabs(F_des(0)) * F_des(2) * std::tan(toRad(30.0));
	}

	if (std::fabs(F_des(1)/F_des(2)) > std::tan(toRad(50.0)))
	{
		constraint_info += boost::str(boost::format("y(%f) too tilt; ")
			% toDeg(std::atan2(F_des(1),F_des(2))));
		F_des(1) = F_des(1)/std::fabs(F_des(1)) * F_des(2) * std::tan(toRad(30.0));	
	}
	// }

	{
		std_msgs::Header msg;
		msg = odom.msg.header;

		std::stringstream ss;

		if (constraint_info=="") constraint_info = "constraint no effect";
		ss << std::endl << constraint_info << std::endl;
		ss << "ep0 " << e_p(0) << " | ";
		ss << "ep1 " << e_p(1) << " | ";
		ss << "ep2 " << e_p(2) << " | ";
		ss << "ev0 " << e_v(0) << " | ";
		ss << "ev1 " << e_v(1) << " | ";
		ss << "ev2 " << e_v(2) << " | ";
		ss << "Fdes0 " << F_des(0) << " | ";
		ss << "Fdes1 " << F_des(1) << " | ";
		ss << "Fdes2 " << F_des(2) << " | ";

		msg.frame_id = ss.str();
		ctrl_dbg_pub.publish(msg);

		geometry_msgs::Vector3Stamped m;
		Vector3d d;
		
		m.header = odom.msg.header;

		d = -Kp * e_p;
		m.vector.x = d(0);
		m.vector.y = d(1);
		m.vector.z = d(2);
		ctrl_dbg_p_pub.publish(m);

		d = -Kv * e_v;
		m.vector.x = d(0);
		m.vector.y = d(1);
		m.vector.z = d(2);
		ctrl_dbg_v_pub.publish(m);
		
		d = param.mass * des.a;
		m.vector.x = d(0);
		m.vector.y = d(1);
		m.vector.z = d(2);
		ctrl_dbg_a_pub.publish(m);
	}

	Vector3d z_b_des = F_des / F_des.norm();
	
	/////////////////////////////////////////////////
	// Z-X-Y Rotation Sequence                
	// Vector3d x_c_des = Vector3d(std::cos(yaw_des), sin(yaw_des), 0.0);
	// Vector3d y_b_des = z_b_des.cross(x_c_des) / z_b_des.cross(x_c_des).norm();
	// Vector3d x_b_des = y_b_des.cross(z_b_des);
	/////////////////////////////////////////////////

	/////////////////////////////////////////////////
	// Z-Y-X Rotation Sequence                
	Vector3d y_c_des = Vector3d(-std::sin(yaw_des), std::cos(yaw_des), 0.0);
	Vector3d x_b_des = y_c_des.cross(z_b_des) / y_c_des.cross(z_b_des).norm();
	Vector3d y_b_des = z_b_des.cross(x_b_des);
	///////////////////////////////////////////////// 

	Matrix3d R_des1; // it's wRb
	R_des1 << x_b_des, y_b_des, z_b_des;
	
	Matrix3d R_des2; // it's wRb
	R_des2 << -x_b_des, -y_b_des, z_b_des;
	
	Vector3d e1 = R_to_ypr(R_des1.transpose() * odom.q.toRotationMatrix());
	Vector3d e2 = R_to_ypr(R_des2.transpose() * odom.q.toRotationMatrix());

	Matrix3d R_des; // it's wRb

	if (e1.norm() < e2.norm())
	{
		R_des = R_des1;
	}
	else
	{
		R_des = R_des2;
	}

	{	// so3 control
		u_so3.Rdes = R_des;
		u_so3.Fdes = F_des;

		Matrix3d wRb_odom = odom.q.toRotationMatrix();
		Vector3d z_b_curr = wRb_odom.col(2);
		u_so3.net_force = F_des.dot(z_b_curr);
	}

	{	// n1 api control in forward-left-up frame
		Vector3d F_c = wRc.transpose() * F_des;
		Matrix3d wRb_odom = odom.q.toRotationMatrix();
        //std::cout<<"Odom.q.v: "<<odom.q.vec()<<" Odom.q.w "<<odom.q.w()<<std::endl<<std::endl;
		Vector3d z_b_curr = wRb_odom.col(2);
		double u1 = F_des.dot(z_b_curr);
		double fx = F_c(0);
		double fy = F_c(1);
		double fz = F_c(2);
		u.roll  = std::atan2(-fy, fz);
		u.pitch = std::atan2( fx, fz);
		u.thrust = u1 / param.full_thrust;
        //std::cout<<z_b_curr<<" - "<<F_des<<" + "<<u1<<std::endl<<std::endl;
		u.mode = Controller_Output_t::VERT_THRU;
		u.yaw = des.yaw;
		// cout << "----------" << endl;
		// cout << z_b_curr.transpose() << endl;
		// cout << F_c.transpose() << endl;
		// cout << u1 << endl;
	}

	{
		Vector3d ypr_des = R_to_ypr(R_des);
		Vector3d ypr_real = R_to_ypr(odom.q.toRotationMatrix());
		geometry_msgs::Vector3Stamped m;
		m.header = odom.msg.header;
		m.vector.x = ypr_des(2);
		m.vector.y = ypr_des(1);
		m.vector.z = ypr_des(0);
		ctrl_dbg_att_des_pub.publish(m);
		m.header = odom.msg.header;
		m.vector.x = ypr_real(2);
		m.vector.y = ypr_real(1);
		m.vector.z = ypr_real(0);
		ctrl_dbg_att_real_pub.publish(m);

	}

	output_visualization(u);

};

void Controller::publish_ctrl(const Controller_Output_t& u, const ros::Time& stamp, const ros::Time& extra_stamp)
{
	sensor_msgs::Joy msg;

	msg.header.stamp = stamp;
	msg.header.frame_id = std::string("FRD");

	// need to translate to forward-right-down frame
	msg.axes.push_back(toDeg(u.roll));
	msg.axes.push_back(toDeg(-u.pitch));
	if (u.mode < 0)
	{
		msg.axes.push_back(u.thrust);
	}
	else
	{
		msg.axes.push_back(u.thrust*100);	
	}
	msg.axes.push_back(toDeg(-u.yaw));
	msg.axes.push_back(u.mode);

	//add time stamp for debug
    msg.buttons.push_back(100000);
    msg.buttons.push_back(extra_stamp.sec/msg.buttons[0]);
    msg.buttons.push_back(extra_stamp.sec%msg.buttons[0]);
    msg.buttons.push_back(extra_stamp.nsec/msg.buttons[0]);
    msg.buttons.push_back(extra_stamp.nsec%msg.buttons[0]);
	
    ctrl_pub.publish(msg);
}

void Controller::publish_so3_ctrl(const SO3_Controller_Output_t& u_so3, const ros::Time& stamp)
{
	Eigen::Vector3d T_w = u_so3.Fdes;
	Eigen::Quaterniond q(u_so3.Rdes);

	geometry_msgs::QuaternionStamped att_msg;

	att_msg.header.stamp = stamp;
	att_msg.header.frame_id = std::string("world");
	att_msg.quaternion.x = q.x();
	att_msg.quaternion.y = q.y();
	att_msg.quaternion.z = q.z();
	att_msg.quaternion.w = q.w();

	ctrl_so3_attitude_pub.publish(att_msg);

	geometry_msgs::WrenchStamped thr_msg;

	thr_msg.header.stamp = stamp;
	thr_msg.header.frame_id = std::string("body");
	thr_msg.wrench.force.z = u_so3.net_force / param.full_thrust;

	ctrl_so3_thrust_pub.publish(thr_msg);

	// quadrotor_msgs::SO3Command msg;
	// msg.header.stamp = stamp;
	// msg.header.frame_id = std::string("body");

	// msg.force.x = T_w(0);
	// msg.force.y = T_w(1);
	// msg.force.z = T_w(2);

	// msg.orientation.x = q.x();
	// msg.orientation.y = q.y();
	// msg.orientation.z = q.z();
	// msg.orientation.w = q.w();

	// msg.kR[0] = Kr(0,0);
	// msg.kR[1] = Kr(1,1);
	// msg.kR[2] = Kr(2,2);

	// msg.kOm[0] = Kw(0,0);
	// msg.kOm[1] = Kw(1,1);
	// msg.kOm[2] = Kw(2,2);

	// msg.aux.kf_correction = 0.0;
	// msg.aux.angle_corrections[0] = 0.0;
	// msg.aux.angle_corrections[1] = 0.0;
	// msg.aux.enable_motors = true;
	// msg.aux.use_external_yaw = false;

	// ctrl_so3_pub.publish(msg);
}

void Controller::output_visualization(const Controller_Output_t& u)
{
	double fn = u.thrust;
	double tan_r = std::tan(u.roll);
	double tan_p = std::tan(u.pitch);
	double fz = std::sqrt(fn*fn/(tan_r*tan_r+tan_p*tan_p+1));
	double fx = fz * tan_p;
	double fy = -fz * tan_r;

	sensor_msgs::Imu msg;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = std::string("intermediate");
	msg.linear_acceleration.x = fx;
	msg.linear_acceleration.y = fy;
	msg.linear_acceleration.z = fz;

	ctrl_vis_pub.publish(msg);
};
