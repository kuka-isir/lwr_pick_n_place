#include <lwr_pick_n_place/lwr_pick_n_place.hpp>

int main(int argc, char** argv){
  ros::init(argc, argv, "demo0");
  ros::NodeHandle nh;
  LwrPickNPlace lwr_pick_n_place;
  
  // Open the gripper on start
  lwr_pick_n_place.openGripper();
  
  // Start from the start
  lwr_pick_n_place.moveToStart();
  
  while(ros::ok()){
    // Go above 
    if(lwr_pick_n_place.objectFoundRecently("coke"))
      lwr_pick_n_place.moveAboveObject("coke");
    else
      continue;
    
    // Get object 
    lwr_pick_n_place.closeGripper();
    
    // Look if bucket is placed on the table  
    if(lwr_pick_n_place.checkBucket())
      lwr_pick_n_place.moveToBucket();
    else
      lwr_pick_n_place.moveToZone();
    
    // Release object
    lwr_pick_n_place.openGripper();
  }
  ros::shutdown();
  return 0;
}