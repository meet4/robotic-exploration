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
#include <geometry_msgs/Pose2D.h>

//geometry_msgs::Pose2D current_pose;
//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Point1
	{
		double x;
		double y;
	};

Point1 current;
Point1 hitpoint = {0,0};
double curr_angle;
double yaw_;
double min_range_ = 0.2;
bool obstacle_;
double direction_vector_[360];
double min;
bool line;
std::vector <Point1> v;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	 current.x=msg->pose.pose.position.x;
	 current.y=msg->pose.pose.position.y;
	 yaw_ = tf::getYaw(msg->pose.pose.orientation);
	 ROS_INFO("X:%f, Y:%f",current.x,current.y);		
}

void laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  uint16_t scan_angle[360]; //= {0, 120, 300};
  for(int i=0;i<360;i++){scan_angle[i]=i;}		
  for (int num = 0; num < 360; num++)
  {
    if (std::isinf(msg->ranges.at(scan_angle[num])))
    {
      direction_vector_[num] = msg->range_max;
    }
    else
    {
      direction_vector_[num] = msg->ranges.at(scan_angle[num]);
    }
  }
	min=direction_vector_[0];
  for(int i=0;i<360;i++){
	if(direction_vector_[i]<min){
		min=direction_vector_[i];
	}      
  }

}


double getYaw(){return yaw_;}

bool inLine(){
	
return true;
}

void setSpeed(double linear_speed, double angular_speed,ros::Publisher cmd_vel_pub_ )
{
    geometry_msgs::Twist msg;
    msg.linear.x = linear_speed;
    msg.angular.z = angular_speed;
    cmd_vel_pub_.publish(msg);
}

double distance(double m,double x,double n,double y){
	double distance1=0.0;
	distance1 = sqrt(pow((m-x),2.0)+pow((n-y),2.0));
	return distance1;
}

double calculate_current_angle(){

		curr_angle = (angles::normalize_angle(getYaw()) - 1.5708) / M_PI * 180.0;
		if (curr_angle < 0.0){

			curr_angle=360+curr_angle;
			
		}
	return curr_angle;
}

void deal_with_obstacle(double angle,ros::Publisher cmd_vel_pub_ ,ros::Subscriber laser){
	
	v.push_back(current);
	//turn left or right
	
	while(ros::ok() && angle>180){
		
		ROS_INFO("direction vector min :%0.2f",min);
		angle=calculate_current_angle();
		setSpeed(0,-0.2,cmd_vel_pub_);
		ROS_INFO("Turning left");				
		ROS_INFO("current angle %0.2f:",angle);
		ros::spinOnce();
	}
	
	//move forward
	
	while(min<0.32 && ros::ok()){
		
		//ROS_INFO("direction vector min :%0.2f",min);
		ROS_INFO("current position %0.2f,%0.2f",current.x,current.y);		
		
		setSpeed(0.2,0,cmd_vel_pub_);
		//ROS_INFO("moving ahead");
		ros::spinOnce();
	}
	v.push_back(current);
	double temp_dist = distance(v[1].x,v[0].x,v[1].y,v[0].y);	

	//turning right
	while(ros::ok() && angle<270){
		
		ROS_INFO("direction vector min :%0.2f",min);
		
		angle=calculate_current_angle();
		setSpeed(0,0.2,cmd_vel_pub_);
		ROS_INFO(" TURNING RIGHT");
		ROS_INFO("current angle %0.2f:",angle);
		ros::spinOnce();
	}
	
	//again moving forward
	while(min<=0.41 && ros::ok()){
		
		ROS_INFO("direction vector min :%0.2f",min);
		setSpeed(0.2,0,cmd_vel_pub_);
		ROS_INFO("moving ahead");
		ros::spinOnce();
	}
	double m=current.x;double n=current.y;
	
	//turning right again
	while(ros::ok() && angle<358){
		ROS_INFO("direction vector min :%0.2f",min);
		
		angle=calculate_current_angle();
		setSpeed(0,0.2,cmd_vel_pub_);
		ROS_INFO("again TURNING RIGHT");
		ROS_INFO("current angle %0.2f:",angle);
		ros::spinOnce();
	}
	//getting in line
	while(ros::ok() && distance(current.x,m,current.y,n)<temp_dist){
		
		//ROS_INFO("direction vector min :%0.2f",min);
		ROS_INFO("current position %0.2f,%0.2f",current.x,current.y);		
		
		setSpeed(0.2,0,cmd_vel_pub_);
		//ROS_INFO("moving ahead");
		ros::spinOnce();
	}	
	
	// getting back in orientation
	
	while(ros::ok() && angle>271){
		//ROS_INFO("direction vector min :%0.2f",min);
		
		angle=calculate_current_angle();
		setSpeed(0,-0.2,cmd_vel_pub_);
		//ROS_INFO("again TURNING RIGHT");
		ROS_INFO("current angle %0.2f:",angle);
		ros::spinOnce();
	}
		v.clear();
}


 double get_destination_angle_in_degrees(double x,double y, double currentX, double currentY){
  
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





int main(int argc, char **argv){
	
	std::vector<Point1> vec;

	vec.push_back(hitpoint);
	//vec[0].x=current.x;
	//vec[0].y=current.y;
	int count=1;
		
	
	ros::init(argc,argv,"basic_exploration_obstacle");
	ros::NodeHandle n;	
	ros::Subscriber sub = n.subscribe("odom",100,odomCallback);
	ros::Subscriber laser_sub = n.subscribe("scan",100,laserScanMsgCallBack);
	ros::Publisher movement = n.advertise<geometry_msgs::Twist>("cmd_vel",100);
	//ros::Publisher pub_pose2d;
	ros::Rate rate(10);
	
	ros::Time start_time = ros::Time::now();	
	//double a=hitpoint.x; double b= hitpoint.y;
        //while(ros::ok())
	double a,b;
	
	while(ros::ok()){
		ROS_INFO("direction vector 0:%0.2f",direction_vector_[0]);
		ROS_INFO("direction vector 1:%0.2f",direction_vector_[1]);
		ROS_INFO("direction vector 2:%0.2f",direction_vector_[2]);
		sleep(2);
	if(min!=0){
		
		while(min>0.25 && ros::ok() && distance(current.x,0,current.y,0)<4){
			/*ROS_INFO("direction vector 0:%0.2f",direction_vector_[0]);
			ROS_INFO("direction vector 1:%0.2f",direction_vector_[1]);
			ROS_INFO("direction vector 2:%0.2f",direction_vector_[2]);*/
			ROS_INFO("obstacle not detected...carry on");
			setSpeed(0.2,0,movement);
			 ros::spinOnce();
		}	
		
		setSpeed(0,0,movement);
		ROS_INFO("obstacle detected....");
		deal_with_obstacle(calculate_current_angle(),movement,laser_sub);
		setSpeed(0,0,movement);
		//ros::spinOnce();
			
		}	
	   ros::spinOnce();	
	}

	/*while(count<6){	

		if(calculate_current_angle()<275 && calculate_current_angle()>265){
			a=vec.back().x;b=vec.back().y;	
			
			while(ros::ok() && distance(current.x,a,current.y,b)<2){
				
				setSpeed(0.2,0,movement);
				ROS_INFO("MOVING IN +X DIRECTION");
				ROS_INFO("current position %0.2f,%0.2f:",current.x,current.y);
				ros::spinOnce();
				
			}
			setSpeed(0,0,movement);
			while(ros::ok() && calculate_current_angle()>180){
				setSpeed(0,-0.2,movement);
				ROS_INFO("Turning left");				
				ROS_INFO("current angle %0.2f:",calculate_current_angle());
				ros::spinOnce();

			}
			setSpeed(0,0,movement);
			a=hitpoint.x = current.x;
			b=hitpoint.y = current.y;
			vec.push_back(hitpoint);
			ROS_INFO("hitpoint updated: %0.2f,%0.2f",hitpoint.x,hitpoint.y);
			count=count+1;
				
			while(ros::ok() && distance(current.x,a,current.y,b)<1){
				setSpeed(0.2,0,movement);
				ROS_INFO("MOVING in +y direction");			
				ROS_INFO("current position %0.2f,%0.2f:",current.x,current.y);
				ros::spinOnce();
			}
			while(ros::ok() && calculate_current_angle()>90){
				setSpeed(0,-0.2,movement);
				ROS_INFO("again turning left");
				ROS_INFO("current angle %0.2f:",calculate_current_angle());
				ros::spinOnce();

			}
			setSpeed(0,0,movement);
			a=hitpoint.x = current.x;
			b=hitpoint.y = current.y;
			vec.push_back(hitpoint);
			ROS_INFO("hitpoint updated: %0.2f,%0.2f",hitpoint.x,hitpoint.y);
			count=count+1;	
		}

		else if(calculate_current_angle()<95 && calculate_current_angle()>85){
			
			double a=hitpoint.x; double b= hitpoint.y;
			while(ros::ok() && distance(current.x,a,current.y,b)<2){
				setSpeed(0.2,0,movement);
				ROS_INFO("MOVING IN -X DIRECTION");
				ROS_INFO("current position %0.2f,%0.2f:",current.x,current.y);
				ros::spinOnce();
			} 
			while(ros::ok() && (calculate_current_angle())<180){
				setSpeed(0,0.2,movement);
				ROS_INFO("turning right");
				ROS_INFO("current angle %0.2f:",calculate_current_angle());
				ros::spinOnce();

			}
			setSpeed(0,0,movement);
			a=hitpoint.x = current.x;
			b=hitpoint.y = current.y;
			vec.push_back(hitpoint);
			ROS_INFO("hitpoint updated: %0.2f,%0.2f",hitpoint.x,hitpoint.y);
			count=count+1;	
			while(ros::ok() && distance(current.x,a,current.y,b)<1){
				setSpeed(0.2,0,movement);
			        ROS_INFO("MOVING in +y direction");
				ROS_INFO("current position %0.2f,%0.2f:",current.x,current.y);
				ros::spinOnce();

			}
			while(ros::ok() && calculate_current_angle()<270){
				setSpeed(0,0.2,movement);
				ROS_INFO("again TURNING RIGHT");
				ROS_INFO("current angle %0.2f:",calculate_current_angle());
				ros::spinOnce();
			}

			setSpeed(0,0,movement);
			a=hitpoint.x = current.x;
			b=hitpoint.y = current.y;
			vec.push_back(hitpoint);
			ROS_INFO("hitpoint updated: %0.2f,%0.2f",hitpoint.x,hitpoint.y);
			count=count+1;	
		}

		if(count>=6){
			setSpeed(0,0,movement);
			ROS_INFO("event successfull!!");
			for(int i=0;i<vec.size()  ;i++){
				ROS_INFO("VECTOR CONTENTS: %0.2f,%0.2f",vec[i].x,vec[i].y);
			}
			break;
		}

		
		//ros::spinOnce();
		
	}*/
		
	return 0;

   }


