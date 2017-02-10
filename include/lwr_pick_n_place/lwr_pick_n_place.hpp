#ifndef LWRPICKNPLACE_LWRPICKNPLACE_HPP_
#define LWRPICKNPLACE_LWRPICKNPLACE_HPP_

#include <ros/ros.h>
#include <ros/time.h>

#include <tf/transform_listener.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <cart_opt_ctrl/UpdateWaypoints.h>
#include <cart_opt_ctrl/GetCurrentPose.h>

#define M_PI 3.14159265358979323846  /* pi */

class LwrPickNPlace{
  public:
    //*** Class public functions ***//
    // Constructor.
    LwrPickNPlace();
    
    // Retrieve the poses of all the objects
    void updateObjectsPosition();
    
    // The robot tries to go to the (x,y,z) position
    void moveToCartesianPose(const geometry_msgs::Pose target_pose);
    
    // The robot tries to go to its home position
    void moveToStart();

    // Go on top of an object
    void moveAboveObject(const std::string& name);
    void moveAboveObject(const int& id);
    
    // Close gripper
    void closeGripper();
    
    // Open gripper
    void openGripper();
    
    // Retrieve the current pose from the controller
    void updateCurrentPose();
    
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
    
    std::string base_frame_, ee_frame_;
    double gripping_offset_;
    geometry_msgs::Pose start_pose_;
    
    std::vector<geometry_msgs::Pose> objects_pose_;
};

#endif // LWRPICKNPLACE_LWRPICKNPLACE_HPP_
