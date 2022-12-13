# ros_sockets

This [Robot Operating System (ROS)](https://www.ros.org/) package provides nodes that can be used for sending and receiving data via TCP sockets. These nodes allow for outside applications to remotely interact with ROS. For example, a robot could be controlled from a computer running a controller written in Julia.

## Installation

On a machine with ROS installed, start `roscore`:

```sh
roscore
```

In a new shell, navigate to the `catkin` package workspace:

```sh
roscd
cd ../src
```

For a default ROS install, your current directory should now be `~/catkin_ws/src`. Clone the repo to this location:

```sh
git clone https://github.com/CLeARoboticsLab/ros_sockets.git
```

Note: if you cannot clone the repo, download it manually from [github](https://github.com/CLeARoboticsLab/ros_sockets) and copy it to this location. After cloning or copying, on a default ROS install, the path to the repo should be:
`~/catkin_ws/src/ros_sockets` .

Navigate to the `catkin` workspace:

```sh
roscd
cd ../
```

For a default ROS install, your current directory should now be `~/catkin_ws`. Build the package:

```sh
catkin_make --only-pkg-with-deps ros_sockets
```

Refresh ROS:

```sh
source devel/setup.bash
rospack profile
```

## Usage

### Velocity Control

This node runs a server that listens for velocity commands to move a robot. The `controls` commands are received and published to the ROS topic `/cmd_vel`.

#### Launching the node

On a machine with ROS installed, start `roscore` if it is not already started:

```sh
roscore
```

In a new shell, launch the node with the following:

```sh
roslaunch ros_sockets velocity_control.launch port:=<PORT> timestep:=<TIMESTEP>
```

Arguments:

`<PORT>`: TCP port the node will listen on. Default is `42421` if left blank.

`<TIMESTEP>`: Timestep duration in seconds. This duration will elapse between each velocity command when a vector of velocity commands is sent to the node. This argument should be set to match the timestep from the solver that is sending control input sequences to this node. Default is `1.0` sec if left blank.

#### Stopping the node

Stop the node by pressing `Ctrl+C` on the shell the node was launched from. When this is done, the node will publish a zero velocity command to the robot and shutdown safely.

#### Sending velocity commands to the node

If sending commands using Julia, the [RosSockets.jl](https://github.com/CLeARoboticsLab/RosSockets.jl) package may be used to easily accomplish the below.

Connect to the `<IP_ADDRESS>:<PORT>` that the node is running on, via TCP.

Send a sequence of linear and angular velocity pairs to the node by writing to this port using the JSON format. The property name needs to be `"controls"` and the sequence needs to be formatted as an array of commands, where each command is an array containing a linear and angular command. Here is an example of a JSON formattted command:

```json
{ "controls": [[0.1, 0.2], [0.15, 0.21], [0.17, 0.23]] }
```

The sequence will be published to the ROS topic `/cmd_vel` at the rate specified by the `roslaunch` argument `<TIMESTEP>`. Note: a zero velocity command will be appended to every sequence to ensure the robot stops after it completes the sequence.

When a new sequence is sent to the node, the node will discard any remaining commands of the sequence it is currently executing and will immediately start executing the new sequence. In this fashion, the node is primed for receding-horizon type control.

When done, be sure to close the TCP connection.

### Motion Tracker Simulation

This node tracks the pose of a model in Gazebo and publishes it to the topic `/<tracker_name>/pose`. Used to simulate the [vrpn_client_ros](http://wiki.ros.org/vrpn_client_ros) node that is used with motion capture systems like Vicon and OptiTrack.

#### Launching the node

Edit the parameters in the launch file at: `/launch/gazebo_tracker.launch`.

- `tracker_name`: name that is assigned to the tracker. The topic that pose data will be published to is `/<tracker_name>/pose`.
- `model_name`: name of the model in Gazebo that will be tracked.
- `update_frequency`: rate in Hz to publish pose data.

After editing the launch file, launch the node with:

```sh
roslaunch ros_sockets gazebo_tracker.launch
```

## Acknowledgments

This package is heavily based on infrastructure from [@schmidma](https://github.com/schmidma) and [@lassepe](https://github.com/lassepe).
