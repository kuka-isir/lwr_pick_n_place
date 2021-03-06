#ifndef LWRPICKNPLACE_LWRPICKNPLACE_HPP_
#define LWRPICKNPLACE_LWRPICKNPLACE_HPP_

#include <ros/ros.h>
#include <ros/time.h>

#include <tf/transform_listener.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <cart_opt_ctrl/UpdateWaypoints.h>
#include <cart_opt_ctrl/GetCurrentPose.h>
#include <lwr_gripper/GripperAction.h>

#define M_PI 3.14159265358979323846  /* pi */

class LwrPickNPlace{
  public:
    //*** Class public functions ***//
    // Constructor.
    LwrPickNPlace();
    
    // Retrieve the poses of all the objects
    void updateObjectsPosition();
    
    // Retrieve the pose of an object
    bool updateObjectPosition(const std::string& name);
    bool updateObjectPosition(const int& id);
    
    // Check if object has been found recently or if the pose is outdated
    bool objectFoundRecently(const std::string& name);
    bool objectFoundRecently(const int& id);
    
    // The robot tries to go to the (x,y,z) position
    bool moveToCartesianPose(const geometry_msgs::Pose target_pose);
    
    // Simple getter for the start pose
    geometry_msgs::Pose getStartPose();
    
    // Simple getter for the current pose
    geometry_msgs::Pose getCurrentPose();
    
    // The robot tries to go to its home position
    bool moveToStart();

    // Go on top of an object
    bool moveAboveObject(const std::string& name);
    bool moveAboveObject(const int& id);
    
    // Move to put down an object at a specific pose
    bool putDownObject(const geometry_msgs::Pose& pose);
    
    // The robot goes to the bucket's position
    bool moveToBucket();
    
    // The robot goes to the zone's position
    bool moveToZone();
    
    // Close gripper
    bool closeGripper();
    
    // Open gripper
    bool openGripper();
    
    // Retrieve the current pose from the controller
    void updateCurrentPose();
    
    // Retrieve the pose of the zone
    void updateZonePose();
    
    // Check if the bucket is placed on the table
    bool checkBucket();
    
  private:
    //*** Class private functions ***//
    // Retrieve the object id from its name
    const int getIdFromName(const std::string& name);

  public:  
    //*** Class public variables ***//
    std::vector<std::string> objects_list_;
    geometry_msgs::Pose current_pose_;
  
  private:
    //*** Class private variables ***//
    ros::ServiceClient trajectory_service_client_, gripper_service_client_, current_pose_service_client_;
    actionlib::SimpleActionClient<lwr_gripper::GripperAction> gripper_ac_;
    
    tf::TransformListener tf_listener_;
    
    std::string base_frame_, ee_frame_;
    double gripping_offset_, zone_release_offset_, bucket_release_offset_;
    geometry_msgs::Pose start_pose_, bucket_pose_, zone_pose_;
    
    std::vector<geometry_msgs::Pose> objects_pose_;    
    std::vector<bool> objects_pose_outdated_;
};

#endif // LWRPICKNPLACE_LWRPICKNPLACE_HPP_
