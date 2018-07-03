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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

struct Point1
	{
		double x;
		double y;
	};

Point1 current;

/*for robotic movement*/
double curr_angle;
double yaw_;
bool obstacle_;
double direction_vector_[360];
double min;
bool line;
std::vector <Point1> v;

/* for mdp calculation */	
int i=0,j=0;
int state[4][4];
int currentState[2];
double reward[4][4];
double uPrime[4][4];
double utility[4][4];
char policy[4][4];
double direction[5];
int n=0;
int loopCount=100;
int id=0;
int max=1;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	 current.x=msg->pose.pose.position.x;
	 current.y=msg->pose.pose.position.y;
	 yaw_ = tf::getYaw(msg->pose.pose.orientation);
	 ROS_INFO("X:%f, Y:%f",current.x,current.y);		
}

double getYaw(){return yaw_;}

void setSpeed(double linear_speed, double angular_speed,ros::Publisher cmd_vel_pub_ )
{
    geometry_msgs::Twist msg;
    msg.linear.x = linear_speed;
    msg.angular.z = angular_speed;
    cmd_vel_pub_.publish(msg);
}

/*double distance(double m,double x,double n,double y){
	double distance1=0.0;
	distance1 = sqrt(pow((m-x),2.0)+pow((n-y),2.0));
	return distance1;
}*/

double calculate_current_angle(){

	curr_angle = (angles::normalize_angle(getYaw()) - 1.5708) / M_PI * 180.0;
	if (curr_angle < 0.0){
		curr_angle=360+curr_angle;	
	}
   return curr_angle;
}

void move_ahead(ros::Publisher cmd_vel_pub){
	time_t start, end;
	double elapsed;  // seconds
   	start = time(NULL);
   	int terminate = 1;
   	while (terminate) {
     		end = time(NULL);
     		elapsed = difftime(end, start);
	        setSpeed(0.2,0,cmd_vel_pub);   
  		if (elapsed >= 5.70  /* seconds */){terminate = 0;}
		     	
	 }	
}

void turnNorth(/*double angle,*/ros::Publisher cmd_vel_pub){
	double angle=calculate_current_angle();
	ROS_INFO("%0.2f",angle);
	if(angle>180){
		while(angle<359.5 && ros::ok()){		
			angle=calculate_current_angle();
			setSpeed(0,+0.2,cmd_vel_pub);
			ROS_INFO("%0.2f",angle);
			ROS_INFO("hi");
			ros::spinOnce();
		}
	}
	else{
		while(angle>1 && ros::ok()){		
			angle=calculate_current_angle();
			setSpeed(0,-0.2,cmd_vel_pub);
			//ROS_INFO("HEADING NORTH");
			ROS_INFO("%0.2f",angle);
			ros::spinOnce();		
		}

	}
}
void turnSouth(double angle,ros::Publisher cmd_vel_pub){
	angle=calculate_current_angle();
	if(angle>180.5){
		while(angle>180 && ros::ok()){		
			angle=calculate_current_angle();
			setSpeed(0,0.2,cmd_vel_pub);
			ROS_INFO("HEADING SOUTH");
			ros::spinOnce();				
		}
	}
	else if(angle<179.5){
		while(angle<180 && ros::ok()){		
			angle=calculate_current_angle();
			setSpeed(0,-0.2,cmd_vel_pub);
			ROS_INFO("HEADING SOUTH");
			ros::spinOnce();
		}

	}
	
}
void turnEast(/*double angle,*/ros::Publisher cmd_vel_pub){
	double angle=calculate_current_angle();
	if(angle>270){
		while(angle>270 && ros::ok()){		
			angle=calculate_current_angle();
			setSpeed(0,-0.2,cmd_vel_pub);
			ROS_INFO("hi east");
			ros::spinOnce();
		}
	}
	else{
		while(angle<270 && ros::ok()){		
			angle=calculate_current_angle();
			setSpeed(0,+0.2,cmd_vel_pub);
			ROS_INFO("HEADING EAST");
			ros::spinOnce();
		}

	}

	
}
void turnWest(double angle,ros::Publisher cmd_vel_pub){
	angle=calculate_current_angle();
	if(angle>90){
		while(angle>90 && ros::ok()){		
			angle=calculate_current_angle();
			setSpeed(0,0.2,cmd_vel_pub);
			ROS_INFO("HEADING WEST");
			ros::spinOnce();
		}
	}
	else{
		while(angle<90 && ros::ok()){		
			angle=calculate_current_angle();
			setSpeed(0,-0.2,cmd_vel_pub);
			ROS_INFO("HEADING WEST");
			ros::spinOnce();
		}

	}
}



double dirNorth(int i, int j){
	if((i==0)){
		return uPrime[i][j];
	}
	else {
		return uPrime[i-1][j];
	}
}

double dirSouth(int i, int j/*,int array1[],int array2[]*/){
	if((i==3)){
		return uPrime[i][j];
	}
	else {
		return uPrime[i+1][j];
	}
}

double dirEast(int i, int j){
	if((j==3)){
		return uPrime[i][j];
	}
	else {
		return uPrime[i][j+1];
	}
}

double dirWest(int i, int j){

	if((j==0)){
		return uPrime[i][j];
	}
	else {
		return uPrime[i][j-1];
	}
}

void copy(double source[4][4],double destination[4][4]){
  int a=0;
  int b=0;
  for(a=0;a<4;a++){
    for(b=0;b<4;b++){
      destination[a][b]=source[a][b];
      }
  }
}

int maxUtility(double direction[]){
	int b=0;
	int a=0;
	for(a=0;a<4;a++){
    	    if(direction[b]>direction[a]){
      		b=b;
   	    }
	    else{
		b=a;
	    }
  	}
 return b;
}

void updatePrime(int i,int j){
  
  if((i==3)&&(j==3)){
    uPrime[i][j]=reward[i][j];
  }
  else{
    
    direction[0] = 0.8*dirNorth(i,j) + 0.1*dirWest(i,j) + 0.1*dirEast(i,j);
    direction[1] = 0.8*dirSouth(i,j) + 0.1*dirWest(i,j) + 0.1*dirEast(i,j);
    direction[2] = 0.8*dirWest(i,j) + 0.1*dirSouth(i,j) + 0.1*dirNorth(i,j);
    direction[3] = 0.8*dirEast(i,j) + 0.1*dirSouth(i,j) + 0.1*dirNorth(i,j);

      max=maxUtility(direction);
      uPrime[i][j]=reward[i][j]+direction[max];
	
	  if(max==0){
      		policy[i][j]='N';
	  }
	  else if(max==1){
      		policy[i][j]='S';
	  }
	  else if(max==2){
      		policy[i][j]='W';
	  }
	  else if(max == 3){
      		policy[i][j]='E';
	  }
    
    policy[3][3]='+';
    } 
}

void load_mdp(){

double difference,delta;

/*initialize uPrime*/
  for(i=0;i<4;i++){
    for(j=0;j<4;j++){
	  uPrime[i][j]=0;
	  state[i][j]=0;
    }
  }
 
  /*Reward Initialization*/
  for(i=0;i<4;i++){
    for(j=0;j<4;j++){
      reward[i][j]=-0.040;
    }
  } 
  reward[3][3]=+1; //+1 reward state.
  
  //Value Iteration Algorithm

  do {
  	
      copy(uPrime,utility); 
	  n++;

      for (i=3; i>=0; i--) 
      {
        for (j=3; j>=0; j--) 
        {
		
        	updatePrime(i,j/*,array1,array2*/);
        	difference=abs(uPrime[i][j]-utility[i][j]);
		
	
        	if (difference > delta){
			delta = difference;
		}
		
        }
			
     }
	 
 } while ((delta>0.000001) && n<=loopCount);
   
}


int main(int argc, char** argv){

	load_mdp();  // loading the mdp matrix
	
	ros::init(argc,argv,"MDP_implementation");
	ros::NodeHandle n;	
	ros::Subscriber sub = n.subscribe("odom",100,odomCallback);
	//ros::Subscriber laser_sub = n.subscribe("scan",100,laserScanMsgCallBack);
	ros::Publisher movement = n.advertise<geometry_msgs::Twist>("cmd_vel",100);
	ros::Rate rate(10);
	
	int a=0;int b=0;
	while(ros::ok()){
		if((current.x> a-0.3||current.x< a+0.3) && (current.y> b-0.3||current.y< b+0.3)){
			if(policy[b][a]=='N'){
				turnSouth(calculate_current_angle(),movement);
				move_ahead(movement);
				b=b-1;
			}
			else if(policy[b][a]=='S'){
									
				turnNorth(/*calculate_current_angle()*/movement);
				move_ahead(movement);
				b=b+1;
			}
			else if(policy[b][a]=='W'){
				turnWest(calculate_current_angle(),movement);
				move_ahead(movement);
				a=a-1;
			}
			else if(policy[b][a]=='E'){
				turnEast(/*calculate_current_angle()*/movement);
				move_ahead(movement);
				a=a+1;
			}
			else if(policy[b][a]=='+'){
				setSpeed(0,0,movement);
				ROS_INFO("The destination has arrived");
				break;
			}
		}

	   ros::spinOnce();	
	}	
return 0;
}
  
