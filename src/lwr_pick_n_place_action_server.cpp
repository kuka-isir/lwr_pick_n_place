#include <actionlib/server/simple_action_server.h>
#include <lwr_pick_n_place/lwr_pick_n_place.hpp>
#include <lwr_pick_n_place/GoToStartAction.h>
#include <lwr_pick_n_place/GoToObjectAction.h>


class LwrPickNPlaceActionServer {
  public:
    LwrPickNPlaceActionServer(ros::NodeHandle& nh, LwrPickNPlace& lwr_pick_n_place) :
      nh_(nh), lwr_pick_n_place_(lwr_pick_n_place),
      go_to_start_as_(nh, "lwr_pick_n_place/go_to_start", boost::bind(&LwrPickNPlaceActionServer::goToStartGoalCb, this, _1), false),
      go_to_object_as_(nh, "lwr_pick_n_place/go_to_object", boost::bind(&LwrPickNPlaceActionServer::goToObjectGoalCb, this, _1), false) {
//       go_to_start_as_.registerPreemptCallback(boost::bind(&LwrPickNPlaceActionServer::goToStartPreemptCb, this));
//       go_to_object_as_.registerPreemptCallback(boost::bind(&LwrPickNPlaceActionServer::goToObjectPreemptCb, this));
      go_to_start_as_.start();
      go_to_object_as_.start();
    }

  protected:
    ros::NodeHandle nh_;
    LwrPickNPlace& lwr_pick_n_place_;

    actionlib::SimpleActionServer<lwr_pick_n_place::GoToStartAction> go_to_start_as_;
    lwr_pick_n_place::GoToStartFeedback go_to_start_feedback_;
    lwr_pick_n_place::GoToStartResult go_to_start_result_;
    bool go_to_start_action_finished_, go_to_start_action_failed_;

    actionlib::SimpleActionServer<lwr_pick_n_place::GoToObjectAction> go_to_object_as_;
    lwr_pick_n_place::GoToObjectFeedback go_to_object_feedback_;
    lwr_pick_n_place::GoToObjectResult go_to_object_result_;
    bool go_to_object_action_finished_, go_to_object_action_failed_;

    void goToStartGoalCb(const lwr_pick_n_place::GoToStartGoalConstPtr& goal) {
      ros::Duration r(0.01);
      ros::Timer spawner_thread = nh_.createTimer(r, boost::bind(&LwrPickNPlaceActionServer::executeGoToStart, this, goal, _1), true);

      go_to_start_action_finished_ = false;
      go_to_start_action_failed_ = false;

      while (ros::ok()) {
        if (go_to_start_action_finished_) {
          ROS_INFO("GoToStart action complete !");
          go_to_start_result_.result = lwr_pick_n_place::GoToStartResult::SUCCESS;
          go_to_start_as_.setSucceeded(go_to_start_result_);
          return;
        }
        if (go_to_start_action_failed_) {
          ROS_ERROR("GoToStart action failed !");
          go_to_start_result_.result = lwr_pick_n_place::GoToStartResult::ABORTED;
          go_to_start_as_.setAborted(go_to_start_result_);
          return;
        }
        // 	if(go_to_start_as_.isPreemptRequested()) {
        // 	  ROS_INFO("Preempting GoToStart...");
        // 	  spawner_thread.stop();
        // 	  return;
        // 	}
        sendGoToStartFeedback();
        ros::spinOnce();
        r.sleep();
      }
    }

    void executeGoToStart(const lwr_pick_n_place::GoToStartGoalConstPtr& goal, const ros::TimerEvent& te) {
      // TODO Check if moveToStart function fails
      // go_to_start_result_.result = lwr_pick_n_place::GoToStartResult::ABORTED;
      // go_to_start_as_.setAborted(go_to_start_result_);

      if(lwr_pick_n_place_.moveToStart())
        go_to_start_action_finished_ = true;
      else
        go_to_start_action_failed_ = true;
    }

//     void goToStartPreemptCb(){
//       go_to_start_as_.setPreempted();
//     }

    void sendGoToStartFeedback() {
      go_to_start_feedback_.current_pose = lwr_pick_n_place_.getCurrentPose();
      go_to_start_as_.publishFeedback(go_to_start_feedback_);
    }

    void goToObjectGoalCb(const lwr_pick_n_place::GoToObjectGoalConstPtr& goal) {
      ros::Duration r(0.01);
      ros::Timer spawner_thread = nh_.createTimer(r, boost::bind(&LwrPickNPlaceActionServer::executeGoToObject, this, goal, _1), true);

      go_to_object_action_finished_ = false;
      go_to_object_action_failed_ = false;

      while (ros::ok()) {
        if (go_to_object_action_finished_) {
          ROS_INFO("GoToObject action complete !");
          go_to_object_result_.result = lwr_pick_n_place::GoToObjectResult::SUCCESS;
          go_to_object_as_.setSucceeded(go_to_object_result_);
          return;
        }
        if (go_to_object_action_failed_) {
          ROS_ERROR("GoToStart action failed !");
          go_to_object_result_.result = lwr_pick_n_place::GoToStartResult::ABORTED;
          go_to_object_as_.setAborted(go_to_object_result_);
          return;
        }
        // 	if(go_to_start_as_.isPreemptRequested()) {
        // 	  ROS_INFO("Preempting GoToStart...");
        // 	  spawner_thread.stop();
        // 	  return;
        // 	}
        sendGoToObjectFeedback();
        ros::spinOnce();
        r.sleep();
      }
    }

    void executeGoToObject(const lwr_pick_n_place::GoToObjectGoalConstPtr& goal, const ros::TimerEvent& te) {
      // TODO Check if moveToStart function fails
      // TODO Reject goal if the object is not in the database
      lwr_pick_n_place_.updateObjectPosition(goal->object_name);
      if(lwr_pick_n_place_.moveAboveObject(goal->object_name))
        go_to_object_action_finished_ = true;
      else
        go_to_object_action_failed_ = true;
    }

//     void goToObjectPreemptCb(){
//       go_to_object_as_.setPreempted();
//     }

    void sendGoToObjectFeedback() {
      go_to_object_feedback_.current_pose = lwr_pick_n_place_.getCurrentPose();
      go_to_object_as_.publishFeedback(go_to_object_feedback_);
    }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "lwr_pick_n_place_action_server");
  ros::NodeHandle nh;
  usleep(1000*1000);

  LwrPickNPlace lwr_pick_n_place;
  LwrPickNPlaceActionServer lwr_pick_n_place_as(nh, lwr_pick_n_place);

  ros::spin();
  ros::shutdown();
  return 0;
}
// kate: indent-mode cstyle; indent-width 2; replace-tabs on; 
