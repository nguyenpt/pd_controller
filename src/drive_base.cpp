#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include <time.h>

class RobotDriver
{
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  ros::Publisher cmd_vel_pub_;
  ros::Subscriber laserSub;
 
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);  


public:
  //! ROS node initialization
  RobotDriver(ros::NodeHandle &nh)
  {
    m = 0;
    nh_ = nh;
    //set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
    laserSub = nh_.subscribe("/scan", 1, &RobotDriver::scanCallback, this);
  }
  float m;
  double dE=0;
  double error=0;
  double distanceFront=0;
  double distanceMin=0;
  double angleMinIndex=0;
  double angleInitIndex=0;
  double dist=0;
  double angleError=0;
  double diffAngleError=0;
  void  drive();
  void  followWall();
  void  outputInfo(); 
  const static double FORWARD_SPEED = .4;
  const static double NINETYDEGREES = M_PI / 2;
  const static double ANGULAR_SPEED = .25;
  const static double SAFETY_WALL_DISTANCE = 1;
  const static double DESIRED_ANGLE =-.48; 
  const static double DESIRED_WALL_DISTANCE = 1.5;
  const static double KP = .3;
  const static double KD = .25;
  const static double K_ANGLE = .4;
}; 
/**
 *Callback function that processes the incoming laser scan message.
 */  
void RobotDriver::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) 
  {
    

    int length = scan->ranges.size(); 
    int minIndex = 0;

    while (isnan(scan->ranges[minIndex]))
    {
       if (minIndex < length/2)
         minIndex++;
    }
    
    int maxIndex = length/2;


    /*goes through array and finds the minimum value in the corresponding
    * quadrant, since we are following the right wall we use the latter half.
    */ 
      
    for(int i = minIndex; i < maxIndex; i++)
    {
       if (!isnan(scan->ranges[i]))
       {
         if(scan->ranges[i] > 0 && scan->ranges[i] < scan->ranges[minIndex])
           minIndex = i;
       }
    }
    /*
    *Calculates the appropiate errors between desired distance and actual 
    *distance, as well as the derivative of the error with respect to time
    */
    angleMinIndex= scan->angle_min + (minIndex) * scan->angle_increment;
    distanceMin = scan->ranges[minIndex];
    dE = (distanceMin - DESIRED_WALL_DISTANCE) - error; 
    error = distanceMin - DESIRED_WALL_DISTANCE;
    diffAngleError = (angleMinIndex - DESIRED_ANGLE) - angleError;
    angleError = angleMinIndex - DESIRED_ANGLE;
    
    if (!isnan(scan->ranges[length/2])) 
      distanceFront = scan->ranges[length /2];
    

    followWall();      
    
    
  }

  void RobotDriver::outputInfo(){
    
    ROS_INFO_STREAM("error is " << error);
    ROS_INFO_STREAM("diffangleError is " << diffAngleError);
    ROS_INFO_STREAM("angleError is " << angleError);

    ROS_INFO_STREAM("distanceFront is " << distanceFront);
    ROS_INFO_STREAM("distanceMin is  " << distanceMin);
    ROS_INFO_STREAM("desired angle is  " << DESIRED_ANGLE);
    ROS_INFO_STREAM("angleMinIndex is " << angleMinIndex);

  }




/*
 *Defines a function where it adjusts the angular and linear velocities
 *of the robot in order to follow the right wall.
 */
  void RobotDriver::followWall()
  {
    geometry_msgs::Twist base_cmd;
    
     base_cmd.angular.z = -1*(KP * error + KD * dE)+ K_ANGLE * (angleError); 

     /* 
     *Checks if the distance in front of robot is too close to wall.
     *If too close, stop robot and rotate until wall distance in front
     *is appropiate enough again to move robot forward.
    */ 
    
    if (distanceFront < SAFETY_WALL_DISTANCE) 
    {
       base_cmd.linear.x = 0;
       base_cmd.angular.z = ANGULAR_SPEED;
    }
    else if (distanceFront < SAFETY_WALL_DISTANCE* 2)
    {
       base_cmd.linear.x = .25 *  FORWARD_SPEED;
    }
    else
       base_cmd.linear.x = FORWARD_SPEED;
    
    cmd_vel_pub_.publish(base_cmd);
  }
/*
 *function that pubishes to cmd_vel in order to move the robot
 */
  void RobotDriver::drive()
  {
    geometry_msgs::Twist base_cmd; 


    while(nh_.ok() ){

      ros::spin();
    }
    
  }

/*
 *Initializes the robot_driver node.
 */
int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh;
  

  RobotDriver driver(nh);
  driver.drive();

}

