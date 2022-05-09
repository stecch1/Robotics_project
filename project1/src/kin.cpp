#include "ros/ros.h"
#include <string.h>
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "project1/Cust_Wheel_Speed.h"
#include <math.h>
#include <dynamic_reconfigure/server.h>
#include "project1/parametersConfig.h"
#include "project1/SetOdometry.h"
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

class Omni_robot
{

	public:

		//Constructor
		Omni_robot(): previous_position_tick_front_left{0.0}, previous_position_tick_front_right{0.0}, previous_position_tick_rear_right{0.0}, previous_position_tick_rear_left{0.0},
		vel_tick_front_left{0.0}, vel_tick_front_right{0.0}, vel_tick_rear_right{0.0}, vel_tick_rear_left{0.0},
		x_pos{0.0}, y_pos{0.0},theta{0.0},runge_kutta{0}, V_tick{0.0}, w_tick{0.0},v_x_tick{0.0},v_y_tick{0.0}, dt{0.0}, v_x{0.0}, v_y{0.0}, r{0.070}, l{0.200}, wl{0.169}, N{42}, 			integration_method{0}
		{

		n.getParam("starting_x_param",x_pos);
		n.getParam("starting_y_param",y_pos);
		n.getParam("starting_theta_param",theta);


  		this-> subscribe2 = this->n.subscribe("wheel_states", 1000, &Omni_robot::callback2, this);
  		this-> subscribe3 = this->n.subscribe("cmd_vel", 1000, &Omni_robot::callback3, this);

		this-> geometry_pub = this->n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 10);
		this-> wheel_speed_pub = this->n.advertise<project1::Cust_Wheel_Speed>("wheels_rpm", 10);

		pub_odom = n.advertise<nav_msgs::Odometry>("/computed_odometry",10);

		/*registering services*/
		this->setService = n.advertiseService("set_odometry", &Omni_robot::setOdometry,this);

  		
  		}
 

	private:

		float vel_tick_front_left;
		float vel_tick_front_right;
		float vel_tick_rear_right;
		float vel_tick_rear_left;
		float previous_position_tick_front_left;
		float previous_position_tick_front_right;
		float previous_position_tick_rear_right;
		float previous_position_tick_rear_left;
 		float x_pos;
		float y_pos;
		float theta;
		float v_x_tick;
		float v_y_tick;
		float V_tick;
		float w_tick;
		float v_x;
		float v_y;
		
		float r;
		float l;
		float wl;
		int N;
 
		bool runge_kutta;
		int integration_method;

 		ros::Time time_ = ros::Time(0,0);
 		float dt = 0.0;
 
 		ros::NodeHandle n;
 		ros::Subscriber subscribe; 
 		ros::Subscriber subscribe2; 
 		ros::Subscriber subscribe3; 
		
		ros::Publisher geometry_pub;
		ros::Publisher wheel_speed_pub;
 		ros::Publisher pub_odom;


		ros::ServiceServer setService;

		
		tf2_ros::TransformBroadcaster br;
		geometry_msgs::TransformStamped transformStamped;

		geometry_msgs::Vector3 linear = geometry_msgs::Vector3();
		geometry_msgs::Vector3 angular = geometry_msgs::Vector3();


	public:


		void callback2(const sensor_msgs::JointState::ConstPtr& msg) {
		  
		  	dt = (msg->header.stamp - time_).toSec();
		  	time_ = msg->header.stamp; 
		  
			vel_tick_front_left = ((((msg->position[0]-previous_position_tick_front_left)/dt)/N)/5) * 2 * 3.1415;
			vel_tick_front_right = ((((msg->position[1]-previous_position_tick_front_right)/dt)/N)/5) * 2 * 3.1415;		
			vel_tick_rear_left = ((((msg->position[2]-previous_position_tick_rear_left)/dt)/N)/5) * 2 * 3.1415;
			vel_tick_rear_right = ((((msg->position[3]-previous_position_tick_rear_right)/dt)/N)/5) * 2 * 3.1415;
		  
			v_x_tick = ((this->vel_tick_front_left + this->vel_tick_front_right) * r)/2;
			v_y_tick= ((this->vel_tick_rear_left - this->vel_tick_front_left) * r)/2;

		 	previous_position_tick_front_left = msg->position[0];
		 	previous_position_tick_front_right = msg->position[1];
		 	previous_position_tick_rear_left = msg->position[2];
			previous_position_tick_rear_right = msg->position[3];
  			
  		  	calculate();



		}

		void callback3(const geometry_msgs::TwistStamped::ConstPtr& msg){

		
		
			float v_x = msg->twist.linear.x;
			float v_y = msg->twist.linear.y;
			float wa = msg->twist.angular.z;
			
			float rpm_fl = (1/r) * (((-l-wl) * wa) + (v_x) - (v_y));
			float rpm_fr = (1/r) * (((l+wl) * wa) + (v_x) + (v_y));
			float rpm_rr = (1/r) * (((l+wl) * wa) + (v_x) - (v_y)); 
			float rpm_rl = (1/r) * (((-l-wl) * wa) + (v_x) + (v_y)); 
			
			project1::Cust_Wheel_Speed msgPub;
			msgPub.header.frame_id = "print_velocity_rpm_wheels";
			msgPub.rpm_fl = rpm_fl;
			msgPub.rpm_fr = rpm_fr;
			msgPub.rpm_rr = rpm_rr;
			msgPub.rpm_rl = rpm_rl;
			
			wheel_speed_pub.publish(msgPub);
		
		
		
		}

		void publishVelocity(geometry_msgs::Vector3 linear, geometry_msgs::Vector3 angular) {

			auto msg = geometry_msgs::TwistStamped();
			
			msg.header.frame_id = "print_velocity";
			msg.header.stamp = ros::Time::now();
			
			msg.twist.angular = angular;
			msg.twist.linear = linear;
					
			
			geometry_pub.publish(msg);


		}

		void calculate() {
			
			x_pos += v_x * dt;
			y_pos += v_y * dt;

			if (integration_method == 0){

				v_x = v_x_tick * std::cos(theta + ((w_tick * dt) / 2)) - v_y_tick * std::sin(theta + ((w_tick * dt )/ 2));
				v_y = v_x_tick * std::sin(theta + ((w_tick * dt )/ 2)) + v_y_tick * std::cos(theta + ((w_tick * dt )/ 2));
				
			
			}else{

				v_x = v_x_tick * std::cos(theta) - v_y_tick * std::sin(theta);
				v_y = v_x_tick * std::sin(theta) + v_y_tick * std::cos(theta);
				
				
			}
			
			theta += w_tick* dt;
			

		  	V_tick = sqrt(pow(v_x_tick, 2.0) + pow(v_y_tick, 2.0));
		  	w_tick = ((this->vel_tick_rear_left - this->vel_tick_front_right) * r)/(2 * (-l-wl));
		  	
		  	this->linear.x = v_x_tick;
			this->linear.y = v_y_tick;
			this->linear.z = 0;
		
			this->angular.x = 0;
			this->angular.y = 0;
			this->angular.z = w_tick;

			publishVelocity(linear, angular);

	
			publishOdomMessage();

			


		}


		void publishOdomMessage(){

			nav_msgs::Odometry odom;
	    	odom.header.stamp = ros::Time::now();
	    	odom.header.frame_id = "odom";
	    	odom.child_frame_id = std::move("base_link");

	    	odom.pose.pose.position.x = x_pos;
	    	odom.pose.pose.position.y = y_pos;
	    	odom.pose.pose.position.z = 0.0;
	    	
	    	tf2::Quaternion z;
	    	z.setRPY(0, 0, theta);
	    	odom.pose.pose.orientation.x = z.x();
	    	odom.pose.pose.orientation.y = z.y();
	    	odom.pose.pose.orientation.z = z.z();
	    	odom.pose.pose.orientation.w = z.w();

	    	odom.twist.twist.linear.x = v_x;
	    	odom.twist.twist.linear.y = v_y;
	    	odom.twist.twist.angular.z = w_tick;

	    	pub_odom.publish(odom);

			broadCastTF();

		}



		void broadCastTF(){
			
			transformStamped.header.stamp= ros::Time::now();
			transformStamped.header.frame_id= "odom";
			transformStamped.child_frame_id= "base_link";
			transformStamped.transform.translation.x= x_pos;
			transformStamped.transform.translation.y= y_pos;
			transformStamped.transform.translation.z= 0.0;
			tf2::Quaternion q;
			q.setRPY(0, 0, theta);
			transformStamped.transform.rotation.x= q.x();
			transformStamped.transform.rotation.y= q.y();
			transformStamped.transform.rotation.z= q.z();
			transformStamped.transform.rotation.w= q.w();
			br.sendTransform(transformStamped);
		}



		bool setOdometry(project1::SetOdometry::Request  &req,
                     project1::SetOdometry::Response &res){
    	
		
			this->x_pos = req.x;
			this->y_pos = req.y;
			this->theta = req.theta;
			res.outcome = "Odometry has been set";
        	return true;
		}
	

		void setPositionX(float x){this->x_pos = x;}
		float getPositionX(){return this->x_pos;}
		void setPositionY(float y){this->y_pos = y;}
		//void setIntegrationMethod(bool value){this->runge_kutta=value;}
		void setIntegrationMethod(int value){
		this->integration_method=value;
		ROS_INFO("integration_method: %d", this->integration_method);
		}
		void setr(float value){this->r=value;}
		void setl(float value){this->l=value;}
		void setwl(float value){this->wl=value;}
		void setN(float value){this->N=value;}
		ros::NodeHandle getNodeHandler(){return this->n;}

};


void reconfigureOdometryParams(const project1::parametersConfig &params, uint32_t level, Omni_robot &rob);


int main(int argc, char **argv){
  	
	ros::init(argc, argv, "kin");

	Omni_robot rob;
	
	dynamic_reconfigure::Server<project1::parametersConfig> server;
	
	server.setCallback(boost::bind(&reconfigureOdometryParams, _1, _2, std::ref(rob)));

  	ros::spin();

  	return 0;


}


void reconfigureOdometryParams(const project1::parametersConfig &params, uint32_t level, Omni_robot &rob){

    rob.setIntegrationMethod(params.integration_method);
   
   
}

