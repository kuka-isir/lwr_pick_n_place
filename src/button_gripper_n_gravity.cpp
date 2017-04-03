#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include "serial/serial.h"
#include <mutex>

std::mutex mtx;
std_msgs::Bool run_gravity, run_gripper;

bool setGravityOn(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
  mtx.lock();
  run_gravity.data = true;
  mtx.unlock();
  return true;
}

bool setGravityOff(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
  mtx.lock();
  run_gravity.data = false;
  mtx.unlock();
  return true;
}

bool setGripperClosed(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
  mtx.lock();
  run_gripper.data = true;
  mtx.unlock();
  return true;
}

bool setGripperOpened(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
  mtx.lock();
  run_gripper.data = false;
  mtx.unlock();
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "button_gripper_n_gravity");
  ros::NodeHandle nh;
  ros::Publisher pub_gravity = nh.advertise<std_msgs::Bool>("/activate_gravity",1);
  ros::Publisher pub_gripper = nh.advertise<std_msgs::Bool>("/activate_gripper",1);
  ros::ServiceServer service_gravity_on = nh.advertiseService("/set_gravity_on", setGravityOn);
  ros::ServiceServer service_gravity_off = nh.advertiseService("/set_gravity_off", setGravityOff);
  ros::ServiceServer service_gripper_on = nh.advertiseService("/set_gripper_closed", setGripperClosed);
  ros::ServiceServer service_gripper_off = nh.advertiseService("/set_gripper_opened", setGripperOpened);
  
  
  serial::Serial serial("/dev/ttyACM0", 9600);
  
  ros::Rate r(100);
  
  bool button_active = false, gripper_last_open = true, long_press = false;
  run_gripper.data = false;  
  run_gravity.data = false;  
  ros::Time button_timer;
  ros::Duration long_press_time(1,0);
  ros::Duration short_press_time(0.1);
  
  while(ros::ok()){
    std::string line = serial.readline();
    if ((line != "0" ) && (line != "1" ))
      continue;
        
    if (line =="1"){
      if(!button_active){
        button_active = true;
        button_timer = ros::Time::now();
      }
      ros::Duration time_spent_active = ros::Time::now() - button_timer;
      if(time_spent_active > long_press_time && !long_press){
        run_gripper.data = !run_gripper.data;
        long_press = true;
      }
    }
    else{
      long_press = false;
      if(button_active){
        ros::Duration time_spent_active = ros::Time::now() - button_timer;
        if((time_spent_active < long_press_time) && (time_spent_active > short_press_time)){
          run_gravity.data = !run_gravity.data;
        }
        button_active = false;
      }
    }
   
    pub_gripper.publish(run_gripper);
    pub_gravity.publish(run_gravity);
    ros::spinOnce();
    r.sleep();
  }
  
  ros::shutdown();
  return 1;
}