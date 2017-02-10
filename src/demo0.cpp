#include <lwr_pick_n_place/lwr_pick_n_place.hpp>

int main(int argc, char** argv){
  ros::init(argc, argv, "demo0");
  ros::NodeHandle nh;
  LwrPickNPlace lwr_pick_n_place;
  
  // Start from the start
  lwr_pick_n_place.moveToStart();
//   usleep(3.0*1e06);
//     
//   // For each objects of the list
//   for(int i=0; i<lwr_pick_n_place.objects_list_.size();i++){
//     
//     lwr_pick_n_place.updateObjectsPosition();
//     
//     // Move above the object
//     lwr_pick_n_place.moveAboveObject(lwr_pick_n_place.objects_list_[i]);
//     
//     // Close gripper
//     lwr_pick_n_place.closeGripper();
//     
//     // lwr_pick_n_place.moveObject();
//     
//     // Open gripper
//     lwr_pick_n_place.openGripper();
//   }  
  
  
  lwr_pick_n_place.updateCurrentPose();
  
  geometry_msgs::Pose pose;
  pose = lwr_pick_n_place.current_pose_;
  pose.position.z += 0.1;
  lwr_pick_n_place.moveToCartesianPose(pose);
//   usleep(3.0*1e06);
  
  pose = lwr_pick_n_place.current_pose_;
  pose.position.z -= 0.1;
  lwr_pick_n_place.moveToCartesianPose(pose);
  
  lwr_pick_n_place.moveAboveObject(0);
  
  ros::shutdown();
  return 0;
}