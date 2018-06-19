#include<ros/ros.h>
#include <geometry_msgs/Twist.h>
#include<nav_msgs/Odometry.h>
#include <math.h>
#include<stdlib.h>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <angles/angles.h>
#include <cstring>
#include <vector>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Point1
	{
		double x;
		double y;
	};

Point1 current = {0,0};

double yaw_;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	 current.x=msg->pose.pose.position.x;
	 current.y=msg->pose.pose.position.y;
	 yaw_ = tf::getYaw(msg->pose.pose.orientation);
	 ROS_INFO("X:%f, Y:%f",current.x,current.y);		
}

double getYaw(){
	return yaw_;
}
/*
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    std::vector<float> ranges(msg->ranges.begin(), msg->ranges.end());
    std::vector<float>::iterator range_it = std::min_element(ranges.begin()+300, ranges.end()-300);

    //ROS_INFO("Smallest range is %lf", *range_it);

    if(*range_it < min_range_) obstacle_ = true;
    else obstacle_ = false;
}
*/
void setSpeed(double linear_speed, double angular_speed,ros::Publisher cmd_vel_pub_ )
{
    geometry_msgs::Twist msg;
    msg.linear.x = linear_speed;
    msg.angular.z = angular_speed;
    cmd_vel_pub_.publish(msg);
}
/*
public static double getAngle(){
    double radian = Math.atan2(ZCord, XCord);
    double angle = radian * 180.0 / Math.PI;
    return angle;
  }//end of getAngle
  
  public static double getBearingInDegree(){
    double[] north = compass.getValues();
    double rad = Math.atan2(north[0], north[2]);
    double bearing = (rad - 1.5708) / Math.PI * 180.0;
    
    if(bearing < 0.0)
      bearing = bearing + 360.0;
    return bearing;
  }//end of getBearing
*/

  double get_destination_angle_in_degrees(double x,double y, double currentX, double currentY){
    //double currentX = robot.x();
    //double currentZ = robot.y();
    double rad = atan2(x-currentX,y-currentY);
    double w = x-currentX;
    double h = y-currentY;
    double atan = (rad) / M_PI * 180.0;
    if (w < 0 || h < 0)
        atan += 180;
    if (w > 0 && h < 0)
        atan -= 180;
    if (atan < 0)
        atan += 360;
	atan = (int)atan % 360;
    return atan;
  } 

  double difference(double currentBearings,double destinationBearings){
    double diff= destinationBearings - currentBearings;
    if (diff>180.0){
      diff=diff-360.0;
    }
    if(diff<-180.0){
      diff=diff+360.0;
    }
    return fabs(diff);
  }


double distance(double a, double x,double b, double y){
	double distance1=0.0;
	distance1 = sqrt(pow((a-x),2.0)+pow((b-y),2.0));
	return distance1;
}

int main(int argc, char **argv){
	Point1 goal = {2,2};
	
	ros::init(argc,argv,"linear_movement");
	ros::NodeHandle n;	
	ros::Subscriber sub = n.subscribe("odom",100,odomCallback);
	ros::Publisher movement = n.advertise<geometry_msgs::Twist>("cmd_vel",1);

	//ros::Publisher cmd_vel_pub_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    	  //ros::Subscriber laser_sub_ = n.subscribe("scan", 10, laserCallback);
    	  //odom_sub_ = n.subscribe("odom", 10, &Robot::odomCallback, this);	
	  ros::Rate rate(10);
	
	ros::Time start_time = ros::Time::now();	
	
	double dist_from_goal;
	double last_set_yaw;
	double goal_angle = get_destination_angle_in_degrees(goal.x, goal.y,current.x,current.y);
	double target_yaw=angles::normalize_angle(goal_angle);
	 while(ros::ok())
	{	
		double curr_angle = (angles::normalize_angle(getYaw()) - 1.5708) / M_PI * 180.0;
		if (curr_angle < 0.0){
			curr_angle=360+curr_angle;
			
		}
		curr_angle = (curr_angle-270);
		ROS_INFO("Current and goal angles are: %0.2f, %0.2f", curr_angle, goal_angle);
		//double goal_angle = get_destination_angle_in_degrees(goal.x, goal.y,current.x,current.y);
   		//angles::normalize_angle(getYaw()+ M_PI);
		
		
		if((curr_angle - goal_angle)>2 || (curr_angle - goal_angle)<-2){
			setSpeed(0, 0.2,movement);
			ROS_INFO("I SHOULD ROTATE! Difference: %0.2f",(curr_angle - goal_angle));
					
		}
		//while((curr_angle - goal_angle)<=5){
		//if((fabs(target_yaw-angles::normalize_angle(getYaw())))<0.017){
		else{
			dist_from_goal = distance(current.x,goal.x,current.y,goal.y);
			if(dist_from_goal<0.5){
				//setSpeed(0.0,0.0,movement);
				ROS_INFO("I SHOULD STOP NOW!!!!!"); 
				setSpeed(0, 0*angles::shortest_angular_distance(getYaw(), target_yaw)*1,movement);
				break;
				
			}
			else{				
				target_yaw=angles::normalize_angle(getYaw());
				//setSpeed(2.0,0,movement);
				ROS_INFO("I SHOULD GO STRAIGHT because distance is: %0.2f!",dist_from_goal); 
				setSpeed(0.5, 0.5*angles::shortest_angular_distance(getYaw(), target_yaw)*1,movement);
			}
		}
		ros::spinOnce();
		//rate.sleep();
	}
		
	return 0;

}




