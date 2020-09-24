## Communication sample for mc_rtc


### Sending joint state over ROS

```
rostopic pub -1 /mc_sample_communication/joints_state_input sensor_msgs/JointState '{header: auto, name: ['NECK_P', 'NECK_Y'], position: [-1, -0.5418], velocity: [], effort: []}'
```
