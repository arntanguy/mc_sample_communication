add_fsm_state_simple(ROSPostureState)
target_link_libraries(ROSPostureState PUBLIC mc_rtc::mc_rtc_ros mc_sample_communication::ROS)

add_fsm_data_directory(data)
