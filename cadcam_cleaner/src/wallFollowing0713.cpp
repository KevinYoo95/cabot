#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#define RAD2DEG(x) ((x)*180./M_PI)
#include <sstream>



float kp = 0.005;
float ki = 0.00001;
float kd = 0.001;

float dist_cur = 0;
float dist_des = 0.22;
float error = 0;
float e_d = 0;
float e_p = 0;
float e_i = 0;
float e_prev = 0;
float pid_val = 0;

float max_speed = 0.3;
float initial_speed = 0.15;

float max_lin_vel0 = 0.2;
float max_lin_vel1 = 0.150;
float max_lin_vel2 = 0.100;
float lin_vel =0.0;
float ang_vel =0.0;
float offset1 = 0.020;
float offset2 = 0.040;
float obs_thr = 0.3;

float rect_dist = 0.0;
float rect_dist1 = 0.0;
float rect_dist2 = 0.0;
float rect_dist_front = 0.0;
float accepter = 0;

float left_sp = 0.0;
float right_sp = 0.0;

int stop_flag = 0;
float counts = 0.0;
float sum = 0.0;
/////////////////////////////////////////////////////////////////
void wheelController(float L_vel,float R_vel)//Unit : m/s
{
 lin_vel = (L_vel + R_vel)/2.0;
 ang_vel = (R_vel-L_vel)/2.0/0.12;
}
////////////////////////////////////////////////////////////////
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{

    float min_obs=1.0;
    float degree_min=0;
    int count = scan->scan_time / scan->time_increment;
    long aa = sizeof(scan->ranges);
    long bb = sizeof(scan->ranges[0]);
    printf("%d\n",count);
    printf("[YDLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
    printf("[YDLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
    printf("[YDLIDAR INFO]: [%f]\n", scan->ranges[0]);
for(int i = 0; i < count; i = i+5) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
    	if(degree > -180 && degree< 180)

       	//printf("[YDLIDAR INFO]: angle-distance : [%f, %f, %i]\n", degree, scan->ranges[i], i);

        if( (scan->ranges[i] >= 0.1) && (scan->ranges[i]<1.0)){
          if (scan->ranges[i]<min_obs){
          min_obs = scan->ranges[i];
          degree_min = degree;
        }
        }
	
       }
	
	for(int j = 0; j<10 ; j++){
       if(scan->ranges[j] != 0){
float degree = RAD2DEG(scan->angle_min + scan->angle_increment * j);
        sum += scan->ranges[j];
        counts = counts + 1.0;
printf("[YDLIDAR INFO]: angle-distance : [%f, %f, %i]\n", degree, scan->ranges[j], j);
        }
       }
       rect_dist = sum/counts; 
       sum = 0.0;
       counts = 0.0;  // 90 degree

	printf("[YDLIDAR INFO]: -180 [%f, %f]\n", -180.0, rect_dist);

       for(int j = 498; j<508 ; j++){
       if(scan->ranges[j] != 0){
float degree = RAD2DEG(scan->angle_min + scan->angle_increment * j);
        sum += scan->ranges[j];
        counts = counts + 1.0;
	printf("[YDLIDAR INFO]: angle-distance : [%f, %f, %i]\n", degree, scan->ranges[j], j);
        }
       }
       rect_dist = sum/counts; 
       sum = 0.0;
       counts = 0.0;  // 90 degree
	printf("[YDLIDAR INFO]: -90 [%f, %f]\n", -90.0, rect_dist);

	for(int j = 995; j<1005 ; j++){
       if(scan->ranges[j] != 0){
float degree = RAD2DEG(scan->angle_min + scan->angle_increment * j);
        sum += scan->ranges[j];
        counts = counts + 1.0;

printf("[YDLIDAR INFO]: angle-distance : [%f, %f, %i]\n", degree, scan->ranges[j], j);
        }
       }
       rect_dist = sum/counts; 
       sum = 0.0;
       counts = 0.0;  // 90 degree
	printf("[YDLIDAR INFO]: 0 [%f, %f]\n", 0.0, rect_dist);
	for(int j = 1493; j<1503 ; j++){
       if(scan->ranges[j] != 0){
float degree = RAD2DEG(scan->angle_min + scan->angle_increment * j);
        sum += scan->ranges[j];
        counts = counts + 1.0;
printf("[YDLIDAR INFO]: angle-distance : [%f, %f, %i]\n", degree, scan->ranges[j], j);
        }
       }
       rect_dist = sum/counts; 
       sum = 0.0;
       counts = 0.0;  // 90 degree
	printf("[YDLIDAR INFO]: 90 [%f, %f]\n", 90.0, rect_dist);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
    for(int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
    if(degree > -180 && degree< 180)
       // printf("[YDLIDAR INFO]: angle-distance : [%f, %f, %i]\n", degree, scan->ranges[i], i);

        if( (scan->ranges[i] >= 0.1) && (scan->ranges[i]<1.0)){
          if (scan->ranges[i]<min_obs){
          min_obs = scan->ranges[i];
          degree_min = degree;
        }
        }
       }
       for(int j = 537; j<544 ; j++){
       if(scan->ranges[j] != 0){
        sum += scan->ranges[j];
        counts = counts + 1.0;
        }
       }
       rect_dist = sum/counts; 
       sum = 0.0;
       counts = 0.0;  // 90 degree
       
        for(int j = 497; j<504 ; j++){
       if(scan->ranges[j] != 0){
        sum += scan->ranges[j];
        counts = counts + 1.0;
        }
       }
       rect_dist1 = sum/counts; 
       sum = 0.0;
       counts = 0.0;
       
       for(int j = 577; j<584 ; j++){
       if(scan->ranges[j] != 0){
        sum += scan->ranges[j];
        counts = counts + 1.0;
        }
       }
       rect_dist2 = sum/counts; 
       sum = 0.0;
       counts = 0.0;
       for(int j = 357; j<364 ; j++){
       if(scan->ranges[j] != 0){
        sum += scan->ranges[j];
        counts = counts + 1.0;
        }
       }
       rect_dist_front = sum/counts; 
       sum = 0.0;
       counts = 0.0;

       dist_cur =  rect_dist + 0.5*sin(3.141592*(degree_min-90.0)/180.0) ;// rect_dist ;
       //printf("[YDLIDAR INFO]: minimum_obstacle : [%f, %f]\n", degree_min, min_obs);


   if(rect_dist_front > obs_thr){
      stop_flag =0;
            /////rect_1 left_upper, rect_2 left_lower   //// rect2>rect1 -> left , rect2<rect1 -> right
        if(dist_cur-dist_des<-offset1){
            if(dist_cur-dist_des<-offset2)
            {
                error = -5;
            }
            else{
                error = -1;
            }
        }
        else if(dist_cur-dist_des>offset1){
            if(dist_cur-dist_des>offset2)
            {
                error = 5;
            }
            else{
                error = 1;
            }
        }
        else{
            error = 0;
        }
        e_p = error;
        e_i = e_i + error;
        e_d = error - e_prev;
        
        pid_val = kp*e_p + ki*e_i + kd*e_d;
        e_prev = error;

        left_sp  = initial_speed - pid_val;
        right_sp = initial_speed + pid_val;
        if(left_sp>0.05){
            if(left_sp>max_speed){
                left_sp = max_speed;
            }
        }
        else{
            left_sp = 0.05;
        }
        if(right_sp>0.05){
            if(right_sp>max_speed){
                right_sp = max_speed;
            }
        }
        else{
            right_sp = 0.05;
        }
        wheelController(left_sp,right_sp);
    }
   else{
      if(stop_flag>3){
      lin_vel = 0;//max_lin_vel0;
       ang_vel = -max_speed/0.12;///2.0;;
      }
      else{
           wheelController(0,0);
       }
           stop_flag++;
   }


*/
}
int count = 0;
int main(int argc, char **argv)
{

    ros::init(argc, argv, "wallFollowing0713");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Rate loop_rate(100);
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);
 while (ros::ok())
  {
    geometry_msgs::Twist msg;
    msg.linear.x = lin_vel;
    msg.angular.z = ang_vel;
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    //printf("%d\n", count);
    ++count;

  }
    return 0;
}

