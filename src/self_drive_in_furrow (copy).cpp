#include <ros/ros.h>
#include <math.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/LaserScan.h"

#define PI 3.141592

bool ctrl_flag=false;						// control flag
bool weed_flag;						// weeding flag

float std_dev;							// standard deviation

float ldr_r[360];						// lidar ranges array
int ldr_idx;							// lidar index

int th_r=80;							// -80~80deg range
int h_idx;							// height index
float d_th;							// delta theta
float h_c;							// center height

float pos_coef;						// position array
float pos_sum;							// pos*height_norm sum
float h_norm_sum;						// height_norm sum
float pos;							// position(error)

float p_gain=15;						// control p gain
float ctrl_out;						// control output(gyration radius)
float tread=0.12;						// tread
float vel=5000.0;						// motor velocity

class publisher_subscriber					// class for pub, sub
{
	public:						// public member declaration
		publisher_subscriber()		// substitute topics to publisher & subscriber
		{
			vel_l_pub=nh.advertise<std_msgs::Float32>("vel_l", 1);
			vel_r_pub=nh.advertise<std_msgs::Float32>("vel_r", 1);
			flag_pub=nh.advertise<std_msgs::String>("log", 1);
			flag_sub=nh.subscribe("ctrl_flag", 1000, &publisher_subscriber::ctrl_flag_callback, this);
			ldr_sub=nh.subscribe("scan", 1000, &publisher_subscriber::sub_callback, this);
		}
		
		void ctrl_flag_callback(const std_msgs::Bool::ConstPtr& flag) // ctrl flag call back func.
		{
			if(flag->data==true)
    			{
    				ctrl_flag=!ctrl_flag;
    				ROS_INFO("%s", ctrl_flag ? "true" : "false");
    			}
		}
		
		void sub_callback(const sensor_msgs::LaserScan::ConstPtr& topic)
		{	
			std_msgs::Float32 vel_l;			// Float32 data type var.
			std_msgs::Float32 vel_r;			// Float32 data type var.
			std_msgs::String weeding_flag;		// String data type var.
									
			for(int i=0; i<360; i++)			// substitute topic 
			{
				ldr_r[i]=0.0;
				ldr_r[i]=((topic->ranges[i])<12.0) ? (topic->ranges[i]):(0.0);
			}
			
			for(int i=359; i>0; i--)			// find lidar index num.
			{
				if((ldr_r[i]>0.1) && (ldr_r[i]<12.0))
				{
					ldr_idx=i;
					break;
				}
			}
			
			interpolation(ldr_r, ldr_idx);		// interpolate lidar data
			
			d_th=360.0/ldr_idx;				// delta theta
			h_idx=2*th_r*ldr_idx/360;			// height index
			
			float height[h_idx];				// height array
			
			for(int i=0; i<h_idx; i++)			// calculate height
			{
				height[i]=ldr_r[i+(ldr_idx-h_idx)/2]*cos((i*d_th-th_r)*PI/180);
			}
			
			std_dev=std_deviation(height, h_idx);		// get standard deviation
			
			ROS_INFO("std deviation: %f", std_dev);
			
			if(std_dev>0.05)				// whether to weed or not
			{
				weed_flag=true;
				weeding_flag.data="weeder operating";
				ROS_INFO("weeder operating");
			}
			else
			{
				weed_flag=false;
				weeding_flag.data="finished a row";
				ROS_INFO("finished a row");
			}
			
			flag_pub.publish(weeding_flag);		// publish weeding flag
			
			h_c=10*height[h_idx/2];			// center height
			
			normalization(height, h_idx);			// normalize height
			
			pos_sum=0.0;					// initialize sum var.
			h_norm_sum=0.0;
			
			for(int i=0; i<h_idx; i++)
			{
				pos_coef=h_c*tan((i*d_th-th_r)*PI/180.0);
				pos_sum+=pos_coef*height[i];
				h_norm_sum+=height[i];
			}
			
			pos=pos_sum/h_norm_sum;
			
			ROS_INFO("position: %f", pos);
			
			ctrl_out=1/(p_gain*pos);			// control output
			
			vel_l.data=(ctrl_out-tread)*vel/ctrl_out;	// left motor velocity
    			vel_r.data=(ctrl_out+tread)*vel/ctrl_out;	// right motor velocity
    			
    			ROS_INFO("left: %f", vel_l.data);
    			ROS_INFO("right: %f", vel_r.data);
			
			if((ctrl_flag==true) && (weed_flag==true))
			{
				vel_l_pub.publish(vel_l);		// publish the pos topic
				vel_r_pub.publish(vel_r);		// publish the pos topic
				ROS_INFO("sent command");
			}
		}
		
		void interpolation(float arr[], int idx)		// interpolation func.
		{
			arr[0]=((arr[idx]+arr[1])*0.5+arr[0])*0.5;
		
			for(int i=1; i<idx; i++)
			{
				arr[i]=((arr[i-1]+arr[i+1])*0.5+arr[i])*0.5;
			}
			
			arr[idx]=((arr[idx-1]+arr[0])*0.5+arr[idx])*0.5;
		}
		
		float std_deviation(float arr[], int idx)		// standard deviation func.
		{
			float mean=0.0;
			float dev;
			
			for(int i=0; i<idx; i++)
			{
				mean+=arr[i];
			}
			
			mean/=idx;					// find mean
			
			for(int i=0; i<idx; i++)
			{
				dev+=(arr[i]-mean)*(arr[i]-mean);
			}
			
			dev=sqrt(dev/idx);				// find standard deviation
			
			return dev;
		}
		
		void normalization(float arr[], int idx)		// normalization func.
		{
			float min=12.0;
			float max=0.0;
		
			for(int i=0; i<=idx; i++)
			{
				if(arr[i]!=0)
				{
					min=(arr[i]<min) ? (arr[i]):(min);
					max=(arr[i]>max) ? (arr[i]):(max);
				}
			}
			
			for(int i=0; i<=idx; i++)
			{
				arr[i]=100*(arr[i]-min)/(max-min);
			}
		}
		
	private:					// private member declaration
	  	ros::NodeHandle nh; 			// declare node handle
		ros::Publisher vel_l_pub;		// declare publisher
		ros::Publisher vel_r_pub;
		ros::Publisher flag_pub;
  		ros::Subscriber ldr_sub;		// declare subscriber
  		ros::Subscriber flag_sub;
};

int main(int argc, char **argv)			// main function
{
	ros::init(argc, argv, "self_drive_in_furrow");	// ros initialization
	publisher_subscriber pub_sub;			// class object delaration
	
	ros::Rate loop_rate(10);			// set 10ms loop rate
	
	while(ros::ok())				// while loop
	{
		ros::spinOnce();			// run ros once 
		
		loop_rate.sleep();			// sleep to keep the loop rate
	}
	
	return 0;					
}
