#include "ROSPostureState.h"

#include "../SampleCommunicationController.h"
#include <mc_rtc/ros.h>
#include <ros/ros.h>

void ROSPostureState::rosSpinner()
{
  mc_rtc::log::info("Creating ROS joint subscriber on topic {}", topic_);
  jointsSubscriber_ = mc_rtc::ROSBridge::get_node_handle()->subscribe(topic_, 1, &ROSPostureState::processJoints, this);
  while(ros::ok() && running_)
  {
    ros::spinOnce();
  }
}

void ROSPostureState::processJoints(const sensor_msgs::JointState::ConstPtr& js)
{
  lock_t l(jointsMutex_);
  joints_.clear();
  for (unsigned int i = 0; i < js->name.size(); i++)
  {
    const auto & name = js->name[i];
    const auto & value = js->position[i];
    joints_[name] = value;
  }
  jointsUpdated_ = true;
}


void ROSPostureState::configure(const mc_rtc::Configuration & config)
{
  config("topic", topic_);
}

void ROSPostureState::start(mc_control::fsm::Controller & ctl)
{
  spinThread_ = std::thread(std::bind(&ROSPostureState::rosSpinner, this));
}

bool ROSPostureState::run(mc_control::fsm::Controller & ctl)
{
  if(jointsUpdated_)
  {
    auto & robot = ctl.robot();
    auto postureTask = ctl.getPostureTask(robot.name()); // Get global posture task
    auto postureTarget = postureTask->posture(); // Get current posture
    { // Update joint target from ROS data
      lock_t l(jointsMutex_);
      for(const auto & j : joints_)
      {
        if(ctl.robot().hasJoint(j.first))
        {
          auto jIdx = ctl.robot().jointIndexByName(j.first);
          if(ctl.robot().mb().joint(jIdx).dof() == 1)
          {
            postureTarget[jIdx] = {j.second};
          }
          else
          {
            mc_rtc::log::error("[{}] Attempted to set 1-DoF joint value for \"{}\" but this joint has {}-DoF", name(), j.first, ctl.robot().mb().joint(jIdx).dof());
          }
        }
        else
        {
          mc_rtc::log::error("[{}] Attempted to set joint \"{}\" but this joint is not part of robot \"{}\"", name(), j.first, ctl.robot().name());
        }
      }
    };
    postureTask->posture(postureTarget); // set new posture target
    jointsUpdated_ = false;
  }
  output("OK");
  return true;
}

void ROSPostureState::teardown(mc_control::fsm::Controller & ctl)
{
  running_ = false;
  spinThread_.join();
}

EXPORT_SINGLE_STATE("ROSPostureState", ROSPostureState)
