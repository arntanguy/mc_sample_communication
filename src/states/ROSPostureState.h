#pragma once

#include <mc_control/fsm/State.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

struct ROSPostureState : mc_control::fsm::State
{
    void configure(const mc_rtc::Configuration & config) override;

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;

    void rosSpinner();
    void processJoints(const sensor_msgs::JointState::ConstPtr& _js);


private:
    /**
     * Handing the ROS thread
     * @{
     */
    std::thread spinThread_;
    std::mutex spinMutex_;
    std::condition_variable spinCV_;
    bool running_ = true;
    /// @}

    /**
     * Joint subscriber
     * @{
     */
    std::string topic_ = "/mc_sample_communication/joints_state_input";
    ros::Subscriber jointsSubscriber_;
    std::atomic<bool> jointsUpdated_{false};

    using lock_t = std::lock_guard<std::mutex>;
    std::mutex jointsMutex_;
    std::map<std::string, double> joints_; ///< Written to by ROS thread, read by controller thread
    /// @}
};
