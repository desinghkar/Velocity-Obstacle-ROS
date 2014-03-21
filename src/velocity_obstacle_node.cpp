#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>
#include <velocity_obstacle/Pose.h>
#include <geometry_msgs/Twist.h>
#include <velocity_obstacle/Pose.h>
#include <velocity_obstacle/Velocity.h>
#include <math.h>

#define R 2000  //Double of bot dimension in mm
#define PI 3.1416

using namespace std;

class VelocityObstacle
{
	ros::NodeHandle n;

	ros::Publisher robot_vel_pub; //Velocity publishing to the robot
	ros::Publisher obs_vel_pub; //Velocity publishing to the obstacle
	
	geometry_msgs::Twist robot_vel;
	geometry_msgs::Twist obs_vel;
	
	velocity_obstacle::Pose initial_robot_pose;
	velocity_obstacle::Pose initial_obs_pose;
	velocity_obstacle::Pose goal_pose;

	public:
	VelocityObstacle()
	{
		cout<<"Obstacle pose in x y th order"<<endl;
		cin>>initial_obs_pose.x>>initial_obs_pose.y>>initial_obs_pose.theta;
		initial_obs_pose.theta = initial_obs_pose.theta*180/PI;
		cout<<"Robot pose in x y th order"<<endl;
		cin>>initial_robot_pose.x>>initial_robot_pose.y>>initial_robot_pose.theta;
		initial_robot_pose.theta = initial_robot_pose.theta*180/PI;
		cout<<"Goal pose in x y th order"<<endl;
		cin>>goal_pose.x>>goal_pose.y;

		robot_vel.linear.x = 2;
		robot_vel.angular.z = 0;
		
		obs_vel.linear.x = 2;
		obs_vel.angular.z = 0;

		//time = 1;
//		iteration = 0;
		robot_vel_pub=n.advertise<geometry_msgs::Twist>("Robot/rosaria/cmd_vel", 1000);
		obs_vel_pub=n.advertise<geometry_msgs::Twist>("Obstacle/rosaria/cmd_vel", 1000);
	}
	~VelocityObstacle()
	{
	}
	
	void calc_pose()
	{
		initial_robot_pose.theta = atan2((goal_pose.y - initial_robot_pose.y), (goal_pose.x - initial_robot_pose.x));
	}

	void publish_pose()
	{
		calc_pose();
		estimatepose();
//		checkforcollosion();
		ros::Rate loop_rate(5);
		while(ros::ok())
		{
			if(robot_vel.linear.x!=0||robot_vel.angular.z!=0)
				VelocityObstacle::estimatepose();
			robot_vel_pub.publish(robot_vel);
			obs_vel_pub.publish(obs_vel);
			ros::spinOnce();
			loop_rate.sleep();
		}
	}

	bool checkforcollision(double beta1, double beta2, double zeta, double gama)
	{
		float Va_l = robot_vel.linear.x;
		float Va_a = robot_vel.angular.z;
		float Vb_l = obs_vel.linear.x;
		float Vb_a = obs_vel.angular.z;

		double Oa = zeta;
		double Ob = initial_obs_pose.theta;
/*		if(Ob<0)
			Ob+=360;
*/
		double angle = atan2((Va_l*sin(Oa)-Vb_l*sin(Ob)),(Va_l*cos(Oa)-Vb_l*cos(Ob)));
		
/*		if(angle<0)
			angle+=360;
		angle = (int)angle%360;
		*/
//		if(beta2>360)
//		{
//			beta1=beta1-360;
//			beta2=beta2-360;
//		}
		cout<<"angle is "<<angle*180/PI<<endl;
		cout<<"beta1 is "<<beta1*180/PI<<endl;
		cout<<"beta2 is "<<beta2*180/PI<<endl;
	
		if(abs((beta2)-(beta1+2*gama))<0.1*180/PI)
		{
			if(angle>beta1&&angle<beta2)
				return true;
			else
				return false;
		}
		else
		{
			if(angle<beta2)
				return true;
			else if(angle>beta1)
				return true;
			else
				return false;				
		}

	}

	void calc_amigo_velocities()
	{
		
	}

	void estimatepose()
	{
		float x1 = initial_robot_pose.x;
		float y1 = initial_robot_pose.y;

		float x2 = initial_obs_pose.x;
		float y2 = initial_obs_pose.y;

		float xg = goal_pose.x;
		float yg = goal_pose.y;
		//Calculating the goal biased angle of motion
		double zeta  = atan2((yg-y1), (xg-x1));
/*		if(zeta<0)
			zeta+=360;
		zeta = (int)zeta%360;
*/
		//Calculating the velocity obstacle at closest side
		double alpha = atan2((y2-y1),(x2-x1));
/*		if(alpha<0)
			alpha+=360;
		alpha = (int)alpha%360;
*/		
//		cout<<"Angle between the robots "<<alpha<<endl;

		float dist = sqrt(pow((x2-x1),2) + pow((y2-y1), 2));
		double gama = asin(2*R/dist);

		cout<<"Alpha is "<<alpha*180/PI<<endl;
		cout<<"gamma is "<<gama*180/PI<<endl;
		double beta1 = alpha - gama;
/*		if(beta1<0)
			beta1 = beta1+360;
//		beta1 = (int)beta1%360;
*/
		double beta2 = alpha + gama;
/*		if(beta2<0)
			beta2 = beta2+360;
		beta2 = (int)beta2%360;
*/
		double beta;
		float vx = robot_vel.linear.x*cos(zeta);
		float vy = robot_vel.linear.x*sin(zeta);
		if(checkforcollision(beta1, beta2, zeta, gama))
		{
			cout<<"Yes collision"<<endl;
			if(abs(beta1-zeta) < abs(beta2-zeta))
				beta = beta1;
			else 
				beta = beta2;
			vx =1;
			vy = tan(beta)*(1-(obs_vel.linear.x)*cos(initial_obs_pose.theta)) + obs_vel.linear.x*sin(initial_obs_pose.theta);

			robot_vel.linear.x = sqrt(pow(vx, 2)+pow(vy, 2));		

			initial_robot_pose.theta = beta;
			calc_amigo_velocities();
//			vx = robot_vel.linear.x*cos(robot_pose.theta*PI/180);
//			vy = robot_vel.linear.x*sin(robot_pose.theta*PI/180);

		//	time++;
		//	x1=x1+vx*time;
		//	y1=y1+vy*time;

		}
		else
		{
			cout<<"No collision"<<endl;
			beta = zeta;
//			vy = tan(beta*PI/180)*(1-(obs_vel.linear.x)*cos(obst_pose.theta*PI/180)) + obs_vel.linear.x*sin(obst_pose.theta*PI/180);

			robot_vel.linear.x = sqrt(pow(vx, 2)+pow(vy, 2));		
//			robot_vel.linear.x = initial_robot_vel.linear.x;
			initial_robot_pose.theta = zeta;
			vx = robot_vel.linear.x*cos(initial_robot_pose.theta);
			vy = robot_vel.linear.x*sin(initial_robot_pose.theta);
			calc_amigo_velocities();
		//	time++;
		//	x1=x1+vx*time;
		//	y1=y1+vy*time;
	//		angle2goal();
		}

		cout<<"theta = "<<initial_robot_pose.theta*180/PI<<endl;
	//	robot_pose.x = x1;
	//	robot_pose.y = y1;
	}



};
	
int main(int argc, char **argv)
{
	ros::init(argc, argv, "velocity_obstacle_node");
	VelocityObstacle robot_n;
	robot_n.publish_pose();

	return 0;
}
