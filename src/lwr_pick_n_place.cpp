#include <lwr_pick_n_place/lwr_pick_n_place.hpp>

LwrPickNPlace::LwrPickNPlace() :
  gripper_ac_("gripper")
{
  ros::NodeHandle nh, nh_param("~");
  nh_param.param<std::string>("base_frame", base_frame_ , "base_link");
  nh_param.param<std::string>("ee_frame", ee_frame_, "ati_link");
  nh_param.param<double>("gripping_offset", gripping_offset_, 0.1);
  nh_param.param<double>("zone_release_offset", zone_release_offset_, 0.15);
  nh_param.param<double>("bucket_release_offset", bucket_release_offset_, 0.2);
  
  // Initialize start_pose
  XmlRpc::XmlRpcValue start_pose_param;
  if(nh_param.getParam("start_pose", start_pose_param) && start_pose_param.size()==7){
    start_pose_.position.x = start_pose_param[0];
    start_pose_.position.y = start_pose_param[1];
    start_pose_.position.z = start_pose_param[2];
    start_pose_.orientation.x = start_pose_param[3];
    start_pose_.orientation.y = start_pose_param[4];
    start_pose_.orientation.z = start_pose_param[5];
    start_pose_.orientation.w = start_pose_param[6];
  }
  else{
    start_pose_.position.x = 0.4;
    start_pose_.position.y = 0.0;
    start_pose_.position.z = 0.4;
    start_pose_.orientation.x = 0.70711;
    start_pose_.orientation.y = 0.70711;
    start_pose_.orientation.z = 0.0;
    start_pose_.orientation.w = 0.0;
  }
  
  // Initialize objects list
  XmlRpc::XmlRpcValue objects_param;
  if(nh_param.getParam("objects", objects_param)){
    for(int i =0; i < objects_param.size(); i++)
      objects_list_.push_back(objects_param[i]);
  }
  else
    objects_list_.push_back("coke");
  
  // Initialize objects poses
  objects_pose_.resize(objects_list_.size());
  objects_pose_outdated_.resize(objects_list_.size(), true);
  
  // Wait until the required ROS services are available
  trajectory_service_client_ = nh.serviceClient<cart_opt_ctrl::UpdateWaypoints>("/KDLTrajCompute/updateWaypoints");
  current_pose_service_client_ = nh.serviceClient<cart_opt_ctrl::GetCurrentPose>("/CartOptCtrl/getCurrentPose");
  while(!trajectory_service_client_.exists() || !current_pose_service_client_.exists()){
    ROS_INFO("Waiting for services to be ready ...");
    sleep(1.0);
  }
  ROS_INFO("Services ready !");
  
  ROS_INFO("Waiting for the gripper action server to be ready ...");
  gripper_ac_.waitForServer();
  ROS_INFO("Gripper action server ready !");
  
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
  updateObjectPosition(id);
  
  geometry_msgs::Pose waypoint, gripping_pose = objects_pose_[id];
  gripping_pose.position.z += gripping_offset_;
  gripping_pose.orientation.x = 0.70711;
  gripping_pose.orientation.y = 0.70711;
  gripping_pose.orientation.z = 0.0;
  gripping_pose.orientation.w = 0.0;
  
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

geometry_msgs::Pose LwrPickNPlace::getStartPose(){
  return start_pose_;
}

geometry_msgs::Pose LwrPickNPlace::getCurrentPose(){
  updateCurrentPose();
  return current_pose_;
}

void LwrPickNPlace::updateZonePose(){
  tf::StampedTransform transform;
  try{
    tf_listener_.lookupTransform(base_frame_, "zone", ros::Time(0), transform);
  }
  catch(tf::TransformException ex){
  }
  zone_pose_.position.x = transform.getOrigin().getX();
  zone_pose_.position.y = transform.getOrigin().getY();
  zone_pose_.position.z = transform.getOrigin().getZ();
  zone_pose_.orientation.x = 0.70711;
  zone_pose_.orientation.y = 0.70711;
  zone_pose_.orientation.z = 0.0;
  zone_pose_.orientation.w = 0.0;
  
  // TODO Check that the bucket pose is actually above the table or too far away
}

void LwrPickNPlace::moveToZone(){  
  // Update the robot current cartesian pose
  updateCurrentPose();
  
  // Try to update zone pose
  updateZonePose();
  
  geometry_msgs::Pose waypoint;
  
  cart_opt_ctrl::UpdateWaypoints kdl_traj_service;
  kdl_traj_service.request.waypoints.header.frame_id = base_frame_;
  kdl_traj_service.request.waypoints.header.stamp = ros::Time::now();
  kdl_traj_service.request.waypoints.poses.push_back(current_pose_);
  
  waypoint = current_pose_;
  waypoint.position.z = 0.5;
  kdl_traj_service.request.waypoints.poses.push_back(waypoint);
  
  waypoint = zone_pose_;
  waypoint.position.z = 0.5;
  kdl_traj_service.request.waypoints.poses.push_back(waypoint);  
  
  waypoint.position.z = zone_pose_.position.z + zone_release_offset_;
  kdl_traj_service.request.waypoints.poses.push_back(waypoint);
  
  trajectory_service_client_.call(kdl_traj_service);
}

void LwrPickNPlace::moveToBucket(){  
  // Update the robot current cartesian pose
  updateCurrentPose();
  
  geometry_msgs::Pose waypoint;
  
  cart_opt_ctrl::UpdateWaypoints kdl_traj_service;
  kdl_traj_service.request.waypoints.header.frame_id = base_frame_;
  kdl_traj_service.request.waypoints.header.stamp = ros::Time::now();
  kdl_traj_service.request.waypoints.poses.push_back(current_pose_);
  
  waypoint = current_pose_;
  waypoint.position.z = 0.5;
  kdl_traj_service.request.waypoints.poses.push_back(waypoint);
  
  waypoint = bucket_pose_;
  waypoint.position.z = 0.5;
  kdl_traj_service.request.waypoints.poses.push_back(waypoint);  
  
  waypoint.position.z = zone_pose_.position.z + bucket_release_offset_;
  kdl_traj_service.request.waypoints.poses.push_back(waypoint);
  
  trajectory_service_client_.call(kdl_traj_service);
}


void LwrPickNPlace::openGripper(){
  lwr_gripper::GripperGoal open_goal;
  open_goal.close = false;
  gripper_ac_.sendGoalAndWait(open_goal);
}

void LwrPickNPlace::closeGripper(){
  lwr_gripper::GripperGoal close_goal;
  close_goal.close = true;
  gripper_ac_.sendGoalAndWait(close_goal);
}

void LwrPickNPlace::updateObjectsPosition(){  
  tf::StampedTransform transform;
  for(int i =0; i<objects_list_.size(); i++){
    try{
//       tf_listener_.lookupTransform(objects_list_[i], base_frame_, ros::Time(0), transform);
      tf_listener_.lookupTransform(base_frame_, objects_list_[i], ros::Time::now(), transform);
    }
    catch(tf::TransformException ex){
//       ROS_ERROR("%s",ex.what());
      objects_pose_outdated_[i] = true;
      continue;
    }
    objects_pose_[i].position.x = transform.getOrigin().getX();
    objects_pose_[i].position.y = transform.getOrigin().getY();
    objects_pose_[i].position.z = transform.getOrigin().getZ();
    objects_pose_outdated_[i] = false;
  }
  
  // TODO handle orientation issues ?
}

void LwrPickNPlace::updateObjectPosition(const int& id){  
  tf::StampedTransform transform;
  try{
//     tf_listener_.lookupTransform(objects_list_[id], base_frame_, ros::Time(0), transform);
    tf_listener_.lookupTransform(base_frame_, objects_list_[id], ros::Time::now(), transform);
  }
  catch(tf::TransformException ex){
//       ROS_ERROR("%s",ex.what());
    objects_pose_outdated_[id] = true;
    return;
  }
  objects_pose_[id].position.x = transform.getOrigin().getX();
  objects_pose_[id].position.y = transform.getOrigin().getY();
  objects_pose_[id].position.z = transform.getOrigin().getZ();
  objects_pose_outdated_[id] = false;
  
  // TODO handle orientation issues ?
}

void LwrPickNPlace::updateObjectPosition(const std::string& name){
  updateObjectPosition(getIdFromName(name));
}

bool LwrPickNPlace::objectFoundRecently(const int& id){
  // Update the objects pose
  updateObjectsPosition();

  // Check if id is compatible
  if((id < 0) || (id >objects_list_.size()-1)){
    ROS_WARN_STREAM("Object with id " << id << " is not in the objects list !");
    return false;
  }
  
  return !objects_pose_outdated_[id];
}

bool LwrPickNPlace::objectFoundRecently(const std::string& name){
  return objectFoundRecently(getIdFromName(name));
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

bool LwrPickNPlace::checkBucket(){
  tf::StampedTransform transform;
  try{
    tf_listener_.lookupTransform(base_frame_, "bucket", ros::Time::now(), transform);
  }
  catch(tf::TransformException ex){
    return false;
  }
  bucket_pose_.position.x = transform.getOrigin().getX();
  bucket_pose_.position.y = transform.getOrigin().getY();
  bucket_pose_.position.z = transform.getOrigin().getZ();
  bucket_pose_.orientation.x = 0.70711;
  bucket_pose_.orientation.y = 0.70711;
  bucket_pose_.orientation.z = 0.0;
  bucket_pose_.orientation.w = 0.0;
  
  // TODO Check that the bucket pose is actually above the table or too far away
  
  return true; 
}