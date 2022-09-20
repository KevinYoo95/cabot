#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#define RAD2DEG(x) ((x)*180./M_PI)
#include <sstream>
#include <iostream>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

float kp = 0.006;
float ki = 0.00001;
float kd = 0.001;
float kp1 = 0.005;
float ki1 = 0.00001;
float kd1 = 0.001;

float dist_cur = 0;
float dist_des = 0.25;
float ang_cur1 = 0;
float ang_des1 = 0;

float error = 0;
float e_d = 0;
float e_p = 0;
float e_i = 0;
float error1 = 0;
float e_d1 = 0;
float e_p1 = 0;
float e_i1 = 0;
float e_prev = 0;
float pid_val = 0;
float e_prev1 = 0;
float pid_val1 = 0;

float max_speed = 0.3;
float min_speed = 0.05;
float initial_speed = 0.15;

float max_lin_vel0 = 0.2;
float max_lin_vel1 = 0.150;
float max_lin_vel2 = 0.100;
float lin_vel =0.0;
float ang_vel =0.0;
float offset1 = 0.020;
float offset2 = 0.040;
float offset1a = 0.05;
float offset2a = 0.20;
float obs_thr = 0.35;

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
float line_gap =0.2;

///////////////////////////////////////////////////////////////
int turn_flag = 0;
int set_flag = 0;
int up_down_flag = 0;
float x_ref = 0.0;
float y_ref = 0.0;
float w_ref = 0.0;

int turning_flag = 0;
int wall_follow_flag = 0;
float wall_thr = 0.01;

float left_side = 0.0;
float right_side = 0.0;
float front = 0.0;

float left_60 = 0.0;
float left_30 = 0.0;
float right_30 = 0.0;
float left_upper = 0.0;
float left_lower = 0.0;
float right_upper = 0.0;
float right_lower = 0.0;

float cur_x = 0.0;
float cur_y = 0.0;

//float line_tar = 0.0;
float max_limit = 0.8;
float min_limit = 0.0;
double roll, pitch, yaw;

float u_ang_vel = 0.07;
double diRection;
float spd = 0.0;
int t_flag = 0;
int u_flag = 0;
int tactile_flag = 0;

int big_flag = 1;
/////////////////////////////////////////////////////////////////

float avg_sum(float arry[],int start, int end){
    float a_sum = 0.0;
    float a_count = 0.0;
    float return_val = 0.0;

    for(int j = start; j<end+1 ; j++){
       if((arry[j]!= 0)){
        a_sum += arry[j];
        a_count = a_count + 1.0;
        }
    }
    return_val = a_sum/a_count;
    return return_val;
}

void wheelController(float L_vel,float R_vel)//Unit : m/s
{
    lin_vel = (L_vel + R_vel)/2.0;
    ang_vel = (R_vel - L_vel)/2.0/0.12;
}

void upNdown(double dir,float speed)//Unit : m/s
{

ang_des1 = dir;
initial_speed = speed;
   if(yaw < 0){
        ang_des1 = -ang_des1;
        }

error1= ang_des1 - yaw;

 if(error1 <-offset1a){
        if(error1<-offset2a){
            error1 = -8;
        }
        else{
            error1 = -2;
        }
        }
        else if(error1>offset1a){
            if(error1>offset2a){
                error1 = 8;
            }
            else{
                error1 = 2;
            }
        }
        else{
            error1 = 0;
        }
        e_p1 = error1;
        e_i1 = e_i1 + error1;
        e_d1 = error1 - e_prev1;
        
        pid_val1 = kp1*e_p1 + ki1*e_i1 + kd1*e_d1;
        e_prev1 = error1;

        left_sp  = initial_speed - pid_val1;
        right_sp = initial_speed + pid_val1;
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
     
}

float ang_dist(float ang_ar[],float dist_ar[],float start, float end)//Unit : m/s
{
    float a_sum = 0.0;
    float a_count = 0.0;
    float return_val = 0.0;
    
    if(end > start){

    }

    for(int i = 0; i < 2000 ;i++){
        if((ang_ar[i] > start)&&(ang_ar[i] < end)){
            if((dist_ar[i]!= 0)){
            a_sum += dist_ar[i];
            a_count = a_count + 1.0;
            }
        }
    }

    return_val = a_sum/a_count;
    return return_val;
}

////////////////////////////////////////////////////////////////
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    float min_obs = 1.0;
    float degree_min = 0;
    int count = scan->scan_time / scan->time_increment;
    float sum = 0.0;
    counts = 0.0;
    float angle_arr[3000] = {0,}; 
    float dist_arr[3000] = {0,};

    for(int i = 0; i < count; i++){
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        angle_arr[i] = degree;
        dist_arr[i] = scan->ranges[i];
    }
    
    left_side = ang_dist(angle_arr,dist_arr,87.0,93.0);
    right_side = ang_dist(angle_arr,dist_arr,-93.0,-87.0);
    front = ang_dist(angle_arr,dist_arr,-5.0,5.0);
    left_60 = ang_dist(angle_arr,dist_arr,57.0,63.0);
    left_30 = ang_dist(angle_arr,dist_arr,27.0,33.0);
    left_upper = ang_dist(angle_arr,dist_arr,42.0,48.0);
    left_lower = ang_dist(angle_arr,dist_arr,132.5,137.5);
    right_upper = ang_dist(angle_arr,dist_arr,-48.5,-42.5);
    right_lower = ang_dist(angle_arr,dist_arr,-137.5,-132.5);
    
    /*
    left_side  = avg_sum(arrary_test,537,544);
    right_side = avg_sum(arrary_test,175,184);
    front = avg_sum(arrary_test,355,365);
    left_60 = avg_sum(arrary_test,477,484);
    left_30 = avg_sum(arrary_test,417,424);
    right_30 = avg_sum(arrary_test,297,304);

    left_upper = avg_sum(arrary_test,445,455);
    left_lower = avg_sum(arrary_test,625,635);
    right_upper = avg_sum(arrary_test,265,275);
    right_lower = avg_sum(arrary_test,85,95);
    */

    for(int i = 0; i < count; i++){
            float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
            if(degree > -180 && degree< 180){
                if( (scan->ranges[i] >= 0.1) && (scan->ranges[i]<1.0)){
                    if (scan->ranges[i]<min_obs){
                        min_obs = scan->ranges[i];
                        degree_min = degree;
    }}}}

    /////

    if(big_flag == 0){
        if(front < obs_thr){
            if(stop_flag>2){
                    lin_vel = 0;//max_lin_vel0;
                    ang_vel = -max_speed/0.12/2;///2.0;;
                    turn_flag = 21;
                }
                else{
                    wheelController(0,0);
                }
                stop_flag++;
        }
        else{
            wheelController(0.1,0.1);
        }
    }

    else if(big_flag == 1){
        if(((y_ref-cur_y)*(y_ref-cur_y)+(x_ref-cur_x)*(x_ref-cur_x))<wall_thr){
            wall_follow_flag++;
        }
        if(wall_follow_flag<20){
            if(front < obs_thr){
                if(stop_flag>2){
                    lin_vel = 0;//max_lin_vel0;
                    ang_vel = -max_speed/0.12/2;///2.0;;
                    turn_flag = 11;
                }
                else{
                    wheelController(0,0);
                }
                stop_flag++;
            }
            else{
                if(turn_flag ==0){
                    dist_cur =  left_side + 0.5*sin(3.141592*(degree_min-90.0)/180.0);
                if((left_side > 0.35) && (left_upper > 0.7)){
                    //turn_flag = 1;
                    if(stop_flag>2){
                        turn_flag = 1;
                    }
                    else{
                        wheelController(0.0,0.0);
                    }
                    stop_flag++;
                }     
                else{
                    stop_flag =0;
                    if(dist_cur-dist_des<-offset1){
                        if(dist_cur-dist_des<-offset2){
                            error = -5;
                        }
                        else{
                            error = -1;
                        }
                    }
                    else if(dist_cur-dist_des>offset1){
                        if(dist_cur-dist_des>offset2){
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
                    if(left_sp>min_speed){
                        if(left_sp>max_speed){
                            left_sp = max_speed;
                        }
                    }
                    else{
                        left_sp = min_speed;
                    }
                    if(right_sp>min_speed){
                        if(right_sp>max_speed){
                            right_sp = max_speed;
                        }
                    }
                    else{
                        right_sp = min_speed;
                    }
                        wheelController(left_sp,right_sp);
                    }
                
            }
            else if(turn_flag == 1){//////corner 돌기
                if(left_60<0.32){
                    turn_flag = 0;
                }
                else{
                    //right_sp = initial_speed/3*3.4;//max_lin_vel0;
                    //left_sp = initial_speed/6;
                    //wheelController(left_sp,right_sp);
                    right_sp = 0.12;//max_lin_vel0;
                    left_sp = 0.02;
                    wheelController(left_sp,right_sp);
                }              
            }
            else if(turn_flag == 2){//////corner 돌기
                if(((degree_min-45)*(degree_min-45)<6)){
                    turn_flag = 3;
                }
                else{
                    //right_sp = initial_speed/3*3.4;//max_lin_vel0;
                    //left_sp = initial_speed/6;
                    //wheelController(left_sp,right_sp);
                    lin_vel = 0;//max_lin_vel0;
                    ang_vel = max_speed/0.12/1.5;///2.0;;
                }              
            }
            else if(turn_flag == 3){//////corner 돌기
                if(left_side < 0.35){
                    turn_flag = 0;
                }
                else{
                    right_sp = initial_speed/6;//max_lin_vel0;
                    left_sp = initial_speed/6;
                    wheelController(left_sp,right_sp);
                }                            
            }
            else if(turn_flag == 11){ ///front wall & turn right
                if(left_side < 0.28){
                    turn_flag = 12;
                    stop_flag = 0;
                }
                else{
                    lin_vel = 0;//max_lin_vel0;
                    ang_vel = -max_speed/0.12/1.5;///2.0;;
                }    
            }
            else if(turn_flag == 12){
                stop_flag++;
                if(stop_flag > 2){
                    turn_flag = 0;
                    stop_flag = 0;
                }
                else{
                    lin_vel = 0;//max_lin_vel0;
                    ang_vel = -max_speed/0.12/1.5;///2.0;;
                }
            }

            else if(turn_flag == 21){
                stop_flag++;
                if(stop_flag > 3){
                    turn_flag = 22;
                    stop_flag = 0;
                }
                else{
                    lin_vel = 0;//max_lin_vel0;
                    ang_vel = 0;///2.0;;
                }
            }
            else if(turn_flag == 22){
                stop_flag++;
                if(stop_flag > 3){
                    turn_flag = 0;
                    stop_flag = 0;
                }
                else{
                    lin_vel = 0;//max_lin_vel0;
                    ang_vel = 0;///2.0;;
                }
            }
            }
            
    }
    else if((wall_follow_flag>19)&&(wall_follow_flag<50)){
        wheelController(0,0);
    }
    else{
        if((w_ref-yaw)*(w_ref-yaw)>0.05){
            lin_vel = 0;
            ang_vel = -max_speed/0.12/4;
        }
        else{
            wheelController(0,0);
            big_flag = 2;
            std::cout << "start " << yaw << std::endl;
            //wheelController(0,0);
        }
    }
    }
    
    //// turn_flag /// 0 - wall_following, 1 - line_tracing , 2 - left , 3 - right , 4 - stop
}

int count = 0;
bool start_flag = 0;

int main(int argc, char **argv)
{   
    ////////////////////////////////////////////////
    //double roll, pitch, yaw;
   /////////////////////////////////////////////////
    ros::init(argc, argv, "wallFollowing0812");
    ros::NodeHandle n;
    
    ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Rate loop_rate(100);
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 100, scanCallback);
    tf::TransformListener listener;
    
    bool dir_check = 0;
   while (ros::ok())//ros::ok())
  {
    tf::StampedTransform transform;
    
    try{
      listener.lookupTransform("/map", "/base_footprint",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    tf::Quaternion q(
       transform.getRotation().getX(),
       transform.getRotation().getY(),
       transform.getRotation().getZ(),
       transform.getRotation().getW());
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    cur_x = transform.getOrigin().y();
    cur_y = transform.getOrigin().x();
    if((start_flag==0)&&(big_flag==0)){
        if(x_ref < transform.getOrigin().y()){
            x_ref = transform.getOrigin().y();
        }
        else{
            
        }
        if(y_ref < transform.getOrigin().x()){
            y_ref = transform.getOrigin().x();
        }
    }

    if((start_flag==0)&&(big_flag == 1)){
        x_ref = transform.getOrigin().y() - 0.15;
        y_ref = transform.getOrigin().x();
        w_ref = yaw;
        start_flag = 1;
    }
    ros::spinOnce();
    if((left_side<0.25)||(right_side<0.25)){
        tactile_flag = 0;
    }
    else{
        tactile_flag = 1;
    }

    if(big_flag == 2){
        std::cout << "kk " << yaw << std::endl; 
        if(t_flag == 0){ /// upward
            if((((((left_30 < 0.35)||(right_30 < 0.35))&&(tactile_flag == 1))||(front < 0.45)))&&(u_flag == 0)){
                u_flag = 1;
                right_sp = 0.0;
                left_sp = 0.1;
                if((right_side < 0.4)&&(right_side >= 0.2)){
                    right_sp = u_ang_vel*((right_side-0.12)-0.35);
                    left_sp  = u_ang_vel*((right_side-0.12)+0.35);
                }
                else if((right_side >= 0.01)&&(right_side < 0.2)){
                    right_sp = -u_ang_vel;
                    left_sp  = u_ang_vel;
                }
                else{
                    right_sp = 0.0;
                    left_sp = u_ang_vel;
                }
                
            }
            if(u_flag == 0){ 
            diRection = 0;
            spd = 0.15; 
            upNdown(diRection, spd);
            } 
            if(yaw < -3.10 || yaw > 3.10){
                t_flag = 1;
                u_flag = 0;
            } 
        }
        else if(t_flag == 1){ ///downward
        if((((((left_30 < 0.35)||(right_30 < 0.35))&&(tactile_flag == 1))||(front < 0.45)))&&(u_flag == 0)){
                u_flag = 1;
                right_sp = 0.1;
                left_sp = 0.0;
                if((left_side < 0.4)&&(left_side >= 0.2)){
                    right_sp = u_ang_vel*((left_side-0.12)+0.35);
                    left_sp  = u_ang_vel*((left_side-0.12)-0.35);
                }
                else if((left_side >= 0.01)&&(left_side < 0.2)){
                    right_sp = u_ang_vel;
                    left_sp  = -u_ang_vel;
                }
                else{
                    right_sp = u_ang_vel;
                    left_sp = 0.0;
                }
            }
            if(u_flag == 0){ 
                diRection = 3.1415;
                spd = 0.15; 
                upNdown(diRection, spd);
            } 
            if(yaw > -0.04 && yaw < 0.04){
                t_flag = 0;
                u_flag = 0;
            }  
        }
        wheelController(left_sp,right_sp);
    }
    ///////////////////////////////////////////////////////////////////////
    geometry_msgs::Twist msg;

    msg.linear.x = lin_vel;
    msg.angular.z = ang_vel;
    chatter_pub.publish(msg);
    
    loop_rate.sleep();
    //printf("%d\n", count);
    ++count;
    

    }
    return 0;
}
