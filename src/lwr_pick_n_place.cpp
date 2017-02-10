#include <lwr_pick_n_place/lwr_pick_n_place.hpp>

LwrPickNPlace::LwrPickNPlace(){

  ros::NodeHandle nh, nh_param("~");
  nh_param.param<std::string>("base_frame", base_frame_ , "link_0");
  nh_param.param<std::string>("ee_frame", ee_frame_, "ati_link");
  nh_param.param<double>("gripping_offset", gripping_offset_, 0.1);
  
  // Initialize start_pose
  // TODO Read start pose from a param
  start_pose_.position.x = -0.052;
  start_pose_.position.y = -0.561;
  start_pose_.position.z = 0.420;
  start_pose_.orientation.x = -0.70711;
  start_pose_.orientation.y = 0.70711;
  start_pose_.orientation.z = 0.0;
  start_pose_.orientation.w = 0.0;
  
  // Wait until the required ROS services are available
  trajectory_service_client_ = nh.serviceClient<cart_opt_ctrl::UpdateWaypoints>("/KDLTrajCompute/updateWaypoints");
//   gripper_service_client_ = nh.serviceClient();
  current_pose_service_client_ = nh.serviceClient<cart_opt_ctrl::GetCurrentPose>("/CartOptCtrl/getCurrentPose");
  while(!trajectory_service_client_.exists() || !current_pose_service_client_.exists()){
    ROS_INFO("Waiting for services to be ready ...");
    sleep(1.0);
  }
  ROS_INFO("Services ready !");
  
  objects_list_.push_back("coke");
  geometry_msgs::Pose pose;
  pose.orientation.x = -0.70711;
  pose.orientation.y = 0.70711;
  pose.orientation.z = 0.0;
  pose.orientation.w = 0.0;
  pose.position.x =-0.4;
  pose.position.y =-0.4;
  pose.position.z =0.0;
  objects_pose_.push_back(pose);
}

void LwrPickNPlace::moveAboveObject(const std::string& name){
  return moveAboveObject(getIdFromName(name));
}

void LwrPickNPlace::moveAboveObject(const int& id){
  // Check if id is possible
  if((id < 0) || (id >objects_list_.size()-1)){
    ROS_WARN_STREAM("Object with id " << id << " is not in the objects list !");
    return;
  }
  
  // Update the robot current cartesian pose
  updateCurrentPose();
  
  geometry_msgs::Pose waypoint, gripping_pose = objects_pose_[id];
  gripping_pose.position.z += gripping_offset_;
  
  cart_opt_ctrl::UpdateWaypoints kdl_traj_service;
  kdl_traj_service.request.waypoints.header.frame_id = base_frame_;
  kdl_traj_service.request.waypoints.header.stamp = ros::Time::now();
  kdl_traj_service.request.waypoints.poses.push_back(current_pose_);
  
  waypoint = current_pose_;
  waypoint.position.z = 0.5;
  kdl_traj_service.request.waypoints.poses.push_back(waypoint);
  
  waypoint = gripping_pose;
  waypoint.position.z = 0.5;
  kdl_traj_service.request.waypoints.poses.push_back(waypoint);  
  kdl_traj_service.request.waypoints.poses.push_back(gripping_pose);
  
  trajectory_service_client_.call(kdl_traj_service);
}

void LwrPickNPlace::moveToCartesianPose(const geometry_msgs::Pose target_pose){
  // Update the robot current cartesian pose
  updateCurrentPose();
  
  cart_opt_ctrl::UpdateWaypoints kdl_traj_service;
  kdl_traj_service.request.waypoints.header.frame_id = base_frame_;
  kdl_traj_service.request.waypoints.header.stamp = ros::Time::now();
  kdl_traj_service.request.waypoints.poses.push_back(current_pose_);  
  kdl_traj_service.request.waypoints.poses.push_back(target_pose);
  
  trajectory_service_client_.call(kdl_traj_service);
}

void LwrPickNPlace::moveToStart(){  
  moveToCartesianPose(start_pose_);
}

void LwrPickNPlace::openGripper(){
//   gripper_service_client_.call();
}

void LwrPickNPlace::closeGripper(){
//   gripper_service_client_.call();
}

void LwrPickNPlace::updateObjectsPosition(){
  objects_pose_.resize(objects_list_.size());
  
  // TODO Read in tf the poses of the objects  
  for(int i =0; i<objects_list_.size(); i++){
    objects_pose_[i];
  }
}

const int LwrPickNPlace::getIdFromName(const std::string& name){
  // Find the object id from the name
  for(int i = 0; i<objects_list_.size();i++){
    if(objects_list_[i] == name)
      return i;
  }
  ROS_WARN_STREAM("Object " << name << " was not found in the objects list !");
  return -1;
}

void LwrPickNPlace::updateCurrentPose(){
  cart_opt_ctrl::GetCurrentPose cart_opt_service;
  current_pose_service_client_.call(cart_opt_service);
  
  // TODO check for failure
  current_pose_ =  cart_opt_service.response.current_pose;
  
}
