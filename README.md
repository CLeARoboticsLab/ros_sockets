# ros_sockets

This [Robot Operating System (ROS)](https://www.ros.org/) package provides nodes that can be used for sending and receiving data via TCP sockets.
These nodes allow for outside applications to remotely interact with ROS.
For example, a robot could be controlled from a computer running a controller written in Julia.

## Installation

On a machine with ROS installed, navigate to the `catkin` package workspace:

```sh
roscd
cd ../src
```

For a typical ROS installation, your current directory should now be `~/catkin_ws/src`.
Clone the repo to this location:

```sh
git clone https://github.com/CLeARoboticsLab/ros_sockets.git
```

Note: if you cannot clone the repo due to permissions issues, download it manually from [github](https://github.com/CLeARoboticsLab/ros_sockets) and copy it to this location.
After cloning or copying, on a typical ROS installation, the path to the repo should be:
`~/catkin_ws/src/ros_sockets` .

Navigate to the `catkin` workspace:

```sh
roscd
cd ../
```

For a typical ROS installation, your current directory should now be `~/catkin_ws`.
Build the package:

```sh
catkin_make --only-pkg-with-deps ros_sockets
```

Refresh ROS:

```sh
source devel/setup.bash
rospack profile
```

## Usage

Nodes within this package are started using launch files.
The launch files pass in user-selected parameters to the nodes; these parameters are selected by editing the applicable launch files.

The nodes may be individually launched, or a launch file may be created which launches multiple nodes simultaneously.
The section below describes each node, how they are configured, and how to launch them individually.
The section which follows that one provides examples for simultaneous launching of nodes.

## Individual Node Descriptions and Usage

### Velocity Control

This node runs a server that listens for velocity commands to move a robot. The `controls` commands are received and published to the ROS topic `/cmd_vel`.

#### Configuration

Edit the values of the following parameters located in the `launch/velocity_control.launch` file.

Parameters:

- `port`: TCP port the node will listen on. This must be an integer between 1024 and 65535 and match the port that control commands are being sent to. It is recommended to select a port around 40000 to 60000.

- `timestep`: Timestep duration in seconds. This duration will elapse between each velocity command when a sequence (array) of velocity commands is sent to the node. This argument should be set to match the timestep from the solver that is sending control input sequences to this node.

#### Launching the node

On a machine with ROS installed, start `roscore` if it is not already started:

```sh
roscore
```

In a new shell, launch the node with the following:

```sh
roslaunch ros_sockets velocity_control.launch
```

#### Stopping the node

Stop the node by pressing `Ctrl+C` on the shell the node was launched from.
When this is done, the node will publish a zero velocity command to the robot and shutdown safely.

#### Sending velocity commands to the node

If sending commands using Julia, the [RosSockets.jl](https://github.com/CLeARoboticsLab/RosSockets.jl) package may be used to easily accomplish the below.

With the node already started, connect to the `<IP_ADDRESS>:<PORT>` that the node is running on, via TCP.

Send a sequence of linear and angular velocity pairs to the node by writing to this port using the JSON format, ending the line with the newline character (typically `\n`).
The property name needs to be `"controls"` and the sequence needs to be formatted as an array of commands, where each command is an array containing a linear and angular command.
Here is an example of a JSON formattted command:

```json
{ "controls": [[0.1, 0.2], [0.15, 0.21], [0.17, 0.23]] }\n
```

The sequence will be published to the ROS topic `/cmd_vel` at the rate specified by the `timestep` parameter.
*Note: a zero velocity command will be appended to every sequence to ensure the robot stops after it completes the sequence.*

When a new sequence is sent to the node, the node will discard any remaining commands of the sequence it is currently executing and will immediately start executing the new sequence.
In this fashion, the node is primed for receding-horizon type control.

When done, be sure to close the TCP connection.

### State Feedback

This node runs a server that holds pose and twist data obtained from a motion capture system.
A command is sent to the server to retrieve the latest pose and twist data.
Designed to be used with the [vrpn_client_ros](http://wiki.ros.org/vrpn_client_ros) node that is used with motion capture systems like Vicon and OptiTrack.

#### Configuration

Edit the values of the following parameters located in the `launch/state_feedback.launch` file.

Parameters:

- `tracker_name`: name that is assigned to the tracker using [vrpn_client_ros](http://wiki.ros.org/vrpn_client_ros). The pose and twist topics that will be subscribed to will be `/<tracker_name>/pose` and `/<tracker_name>/twist`.

- `port`: TCP port the node will listen on. This must be an integer between 1024 and 65535 and match the port that control commands are being sent to. It is recommended to select a port around 40000 to 60000.

#### Launching the node

On a machine with ROS installed, start `roscore` if it is not already started:

```sh
roscore
```

In a new shell, launch the node with the following:

```sh
roslaunch ros_sockets state_feedback.launch
```

#### Stopping the node

Stop the node by pressing `Ctrl+C` on the shell the node was launched from.

#### Receiving data from the node

If receiving data using Julia, the [RosSockets.jl](https://github.com/CLeARoboticsLab/RosSockets.jl) package may be used to easily accomplish the below.

With the node already started, connect to the `<IP_ADDRESS>:<PORT>` that the node is running on, via TCP.

Send a command to get feedback data by writing to this port using the JSON format, ending the line with the newline character (typically `\n`).
The property name needs to be `"action"` and the command needs to be `"get_feedback_data"`:

```json
{ "action": "get_feedback_data" }\n
```

Feedback data is sent back using the the JSON format. Four key/value pairs are sent, which represent the pose and twist of the tracked object:

- `position`: a 3d vector representing the x, y, and z position of the object

- `orientation`: a 4d vector that is the w, x, y, and z quaternion that represents the orientation of the object.

- `linear_vel`: a 3d vector representing the linear x, y, and z velocity of the object.

- `angular_vel`: a 3d vector representing the angular x, y, and z velocity of the object.

When done, be sure to close the TCP connection.

### Motion Tracker Simulation

This node tracks the pose and twist of a model in Gazebo and publishes them to the topics `/<tracker_name>/pose` and `/<tracker_name>/twist`.
It is used to simulate the [vrpn_client_ros](http://wiki.ros.org/vrpn_client_ros) node that is used with motion capture systems like Vicon and OptiTrack.

#### Configuration

Edit the values of the following parameters located in the `launch/gazebo_tracker.launch` file.

Parameters:

- `tracker_name`: name that is assigned to the tracker. The topics that pose and twist data will be published to are `/<tracker_name>/pose` and `/<tracker_name>/twist`.

- `model_name`: name of the model in Gazebo that will be tracked.

- `update_frequency`: rate in Hz to publish data.

#### Launching the node

On a machine with ROS installed, start `roscore` if it is not already started:

```sh
roscore
```

In a new shell, launch the node with the following:

```sh
roslaunch ros_sockets gazebo_tracker.launch
```

#### Stopping the node

Stop the node by pressing `Ctrl+C` on the shell the node was launched from.

## Multiple Node Launch File Examples

Multiple nodes can be launched simultaneously to simplify the launch process.
In this manner, both feedback and control (and simulation) can be started for one or multiple agents with just one launch file.
This section refers to a few examples found in the `launch/` directory.
Parameters within the launch file will need to be modified as described in the previous sections.

### Feedback Control Example

This launch file starts a velocity control node and a state feedback node.
Launch it with:

```sh
roslaunch ros_sockets feedback_control_example.launch
```

### Gazebo Simulation Example

Like feedback_control_example.launch, this launch file starts a velocity control node and a state feedback node.
It also starts the Gazebo Tracker node to simulate motion capture.
Launch it with:

```sh
roslaunch ros_sockets gazebo_simulation_example.launch
```

### Multi-Agent Simulation Example

This launch file starts velocity control, state feedback, and Gazebo tracking
for 3 robots.
Launch it with:

```sh
roslaunch ros_sockets multi_agent_example.launch
```

## Acknowledgments

The velocity control node is heavily based on infrastructure from [@schmidma](https://github.com/schmidma) and [@lassepe](https://github.com/lassepe).
