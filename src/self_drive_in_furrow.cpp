#include <ros/ros.h>
#include <vector>
#include "std_msgs/Bool.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

using namespace std;


///////////////////////////////////////////////////////////////////////////	var. declaration
#define PI 3.141592

std_msgs::Bool fur_flag;							// existance of furrow
geometry_msgs::Twist cmd_vel;						// velocity command to be published

bool sub_flag=false;								// lidar data subscription flag

float inf=1/0.0;									// infinite number

vector<float> lidar;								// lidar ranges [m]
vector<float> lidar_i;								// interpolated lidar ranges
int l_num;											// number of elements of lidar ranges

vector<float> height;								// height [m]
int h_num;											// number of elements of height

float d_th;											// delta theta [deg]


///////////////////////////////////////////////////////////////////////////	parameters
float max_rng;										// lidar max range [m]

int th_r;											// lidar data processing range [deg]

float p_gain;										// control p gain

float tread;										// robot tread [m]
float vel;											// default velocity [rpm]


///////////////////////////////////////////////////////////////////////////	sub, pub class
class sub_pub
{
private:
	ros::NodeHandle nh;
	ros::Publisher fur_pub;
	ros::Publisher vel_pub;
	ros::Subscriber ldr_sub;

public:
	sub_pub()										// class constructor
	{
		fur_pub=nh.advertise<std_msgs::Bool>("fur_exi", 1);
		vel_pub=nh.advertise<geometry_msgs::Twist>("vel_sel", 1);
		ldr_sub=nh.subscribe("/scan", 10, &sub_pub::lidar_callback, this);
		
		nh.getParam("/self_drive_in_furrow/max_rng", max_rng);	// load parameters
		nh.getParam("/self_drive_in_furrow/th_r", th_r);
		nh.getParam("/self_drive_in_furrow/p_gain", p_gain);
		nh.getParam("/self_drive_in_furrow/tread", tread);
		nh.getParam("/self_drive_in_furrow/vel", vel);
	}
	
	void furrow_publish()							// furrow existance publish func.
	{
		fur_pub.publish(fur_flag);
	}
	
	void vel_publish()								// velocity command publish func.
	{
		vel_pub.publish(cmd_vel);
	}
	
	void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& msg)	// lidar call back func.
	{
		sub_flag=true;
		
		lidar.clear();
		lidar=msg->ranges;
		
		l_num=lidar.size();
		
		for(int i=0; i<l_num; i++)
		{
			if(lidar[i]==inf)
				lidar[i]=max_rng;
		}
	}
};


///////////////////////////////////////////////////////////////////////////	interpolation func.
void interpolation()
{
	lidar_i.clear();
	
	lidar_i.push_back(((lidar[l_num-1]+lidar[1])*0.5+lidar[0])*0.5);
	for(int i=1; i<l_num-2; i++)
	{
		lidar_i.push_back(((lidar[i-1]+lidar[i+1])*0.5+lidar[i])*0.5);
	}
	lidar_i.push_back(((lidar[l_num-2]+lidar[0])*0.5+lidar[l_num-1])*0.5);
}


///////////////////////////////////////////////////////////////////////////	std deviation func.
float std_deviation()
{
	float mean=0.0;
	float dev;

	for(int i=0; i<h_num; i++)						// find mean
	{
		mean+=height[i];
	}
	mean/=h_num;

	for(int i=0; i<h_num; i++)						// find standard deviation
	{
		dev+=(height[i]-mean)*(height[i]-mean);
	}
	dev=sqrt(dev/h_num);
	
	return dev;
}


///////////////////////////////////////////////////////////////////////////	furrow ex. func.
bool furrow_existance()
{
	float std_dev;									// standard deviation
	
	interpolation();								// interpolate lidar ranges
	
	d_th=360.0/(float)l_num;						// delta theta
	h_num=2*th_r/(int)d_th;							// number of elements of height
	
	height.clear();
	for (int i=0; i<h_num; i++)
	{
		height.push_back(lidar_i[i+(l_num-h_num)/2]*cos(((float)i*d_th-(float)th_r)*PI/180.0));
	}
	
	std_dev=std_deviation();						// get standard deviation of height
	
	if(std_dev>0.05)
		return true;
	else
		return false;
}


///////////////////////////////////////////////////////////////////////////	normalization func.
void normalization()
{
	float min=max_rng;
	float max=0.0;
	
	for(int i=0; i<h_num; i++)						// find minimum & maximum value
	{
		if(height[i]!=0)
		{
			min=(height[i]<min) ? (height[i]):(min);
			max=(height[i]>max) ? (height[i]):(max);
		}
	}
	
	for(int i=0; i<h_num; i++)						// normalize
	{
		height[i]=(height[i]-min)/(max-min);
	}
}


///////////////////////////////////////////////////////////////////////////	control func.
void self_drive_control()
{
	float pos_idx;
	float h_p_sum=0.0;
	float h_sum=0.0;
	float pos;
	float curv;
	
	normalization();								// normalize height data
	
	for(int i=0; i<h_num; i++)						// find position by centroid method
	{
		pos_idx=tan(((float)i*d_th-(float)th_r)*PI/180.0);
		h_p_sum+=pos_idx*height[i];
		h_sum+=height[i];
	}
	pos=h_p_sum/h_sum;
	
	curv=1/(p_gain*pos);							// find control output(curvature)
	
	cmd_vel.linear.x=vel;
	cmd_vel.angular.z=vel/curv;
}


///////////////////////////////////////////////////////////////////////////	main func.
int main(int argc, char **argv)
{
	ros::init(argc, argv, "self_drive_in_furrow");
	sub_pub sp;
	
	ros::Rate loop_rate(5);
	
	fur_flag.data=false;							// initialize furrow existance flag
	cmd_vel.linear.x=0;								// initialize velocity command
	cmd_vel.linear.y=0;
	cmd_vel.linear.z=0;
	cmd_vel.angular.x=0;
	cmd_vel.angular.y=0;
	cmd_vel.angular.z=0;
	
	while(ros::ok())
	{
		ros::spinOnce();
		
		if(sub_flag==true)
		{
			fur_flag.data=furrow_existance();
			self_drive_control();
		}
		
		sp.furrow_publish();
		sp.vel_publish();
		
		loop_rate.sleep();
	}
	
	return 0;					
}
