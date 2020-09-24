# Communication sample for `mc_rtc`

`mc_rtc` does not (currently) provide predefined inputs (such as ROS topics and services, OpenRTM ports and services, etc). It is however simple to implement your own communication interface, and define the logic of what to do with these inputs (creating tasks, providing targets to existing tasks, triggering state transitions in an FSM, etc). In this controller, we will see a few ways to achieve this through a toy example: sending joint angles while maintaining stabilization.

There are multiple levels where these inputs can be defined: 
- As a plugin: these are global components always running before and after each iteration of the controller (ex: `mc_rtc` has a ROS plugin continuously publishing the state of the robot(s) over ROS)
- As a state: you can write a standalone FSM state that handles communication and defines what to do with the data.
- As part of your `mc_control::fsm::Controller` implementation
- By implementing your own GUI client 

## Using a state to change the robot posture over ROS

This section describes how to write a state that can receive joint input from ROS provided as a `sensor_msgs/JointState` message, and use these joint inputs to update `mc_rtc`'s `PostureTask` targets.
This target will then be achieved at best by the whole-body QP (depending on the choice of gains, and how it conflicts with other tasks).

The general principle is as follows:
- The state creates a ROS thread that defines a ROS subscriber and listens for `sensor_msgs/JointState` messages. The thread is necessary here as ROS communication has no real-time guarantee. 
- This thread updates the desired posture according to the message's data
- At each iteration, the state checks whether it needs to update the posture task targets, and do so if needed.

This is implemented in `ROSPostureState`. You can try it by running the controller, and sending a message to the `mc_sample_communication/joints_state_input` topic. For example, to move the robot neck:

```
rostopic pub -1 /mc_sample_communication/joints_state_input sensor_msgs/JointState '{header: auto, name: ['NECK_P', 'NECK_Y'], position: [-1, -0.5418], velocity: [], effort: []}'
```

## Sending joint state as a CLI client for the GUI

In this section, we will see how to implement a very simple CLI client to send requests to the GUI server provided by `mc_rtc`. This allows you to interact with any element visible in the GUI.

The general principle is that mc_rtc provides GUI elements identitifed by a unique identifier, composed of the `{category}, element name}`. For example, the posture task targets are shown as joint sliders identified as `{"Tasks", "FSM_posture_jvrc1", "Targets"}, "Joint Name"}`

By inheriting from `mc_control::ControllerClient`, you can write your own client to interact with the gui server. This is demonstrated in `tools/posture_cli_client.cpp`, that implements a simple command line interface to change joint targets and weight/stiffness of the posture task.

```
./build/tools/posture_cli_client
(command) set_joint_angle NECK_P 0.5
(command) set_joint_angle NECK_Y -0.5
(command) set_posture_stiffness 20
(command) set_joint_angle L_SHOULDER_P -0.5
```

