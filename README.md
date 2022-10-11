# event_recording
Programmatically record a rosbag with a list of selected topics when an event happened and exporting the metrics with
Promethesus via HTTP for monitoring and visualizing. 

# Rosbag_Snapshot Recording

## General Guide to Snapshots

In the event that some error or event happens on the robot, and no rosbag was being recorded. Recovering that data may not be straightforward, but important for debugging purposes. However, recording rosbags all the time isn’t sustainable for storage. The rosbag_snapshot package could be used to trigger rosbag recordings based on events, and make those bags accessible and connected to metrics.

Using the [rosbag_snapshot](https://github.com/DiuLaMaX/rosbag_snapshot/tree/feature/clear_buffer_option) package with a few adjustments, it’s easy to save 
snapshots of topics and write them to a rosbag file. The tool buffers data from topics in RAM, and when called, writes the 
data to a rosbag file. It also uses [Prometheus](https://prometheus.io/) to record metrics and link to the recorded rosbags 
capturing the same event.

### A non blocking call to rosbag_snapshot
1. ##### **Adding rosparam functionality for Grouping topics**
    
    The original package works great, but it takes in a list of topics to write to bag when triggered. However, not all the topics need to be recorded if an event happened. For example, don't record the cameras topics if it's localization that failed. Therefore, we created multiple groups, and each has its own list of topics to be recorded when triggered.

2. ##### **Ros Services are blocking and can’t handle a queue**
    
    The original package uses a rosservice to trigger writing to bag and then doesn’t return until after the bag file has been written. In the case of a small file, this isn’t a problem. However, 5GB of data could take a long time. This would also block other nodes to trigger a new recording until the data has been written. To solve this problem, the wrapper uses pub/sub which is non-blocking and has a built in queue for multiple requests sequentially. 

We have a [pull request](https://github.com/ros/rosbag_snapshot/pull/26) to include these changes upstream. Here is the original repository of [rosbag_snapshot](https://github.com/ros/rosbag_snapshot). 
## Installation

First, clone **2** repositories into your workspacce:

- [https://github.com/polymathrobotics/event_recording](https://github.com/polymathrobotics/event_recording)
- [https://github.com/polymathrobotics/rosbag_snapshot](https://github.com/polymathrobotics/rosbag_snapshot)

Go to the top directory of your catkin workspace, where the source code of the ROS packages you'd like to use are. Then use:
[rosdep](http://wiki.ros.org/rosdep) to install the dependencies:
```commandline
rosdep install --from-paths src --ignore-src -r -y
```
Finally, build your workspace and source it.

## Starting the Data Buffer and Wrapper
In the **event_recording** repository, there is a launch file called `snapshot_buffer.launch`, which starts the buffer for rosbag
recording. Run the following command to launch the snapshot buffer:

```bash
roslaunch event_recording snapshot_buffer.launch
```
Inside `snapshot_buffer.launch`, we spawn two nodes. `snapshot_buffer` starts a buffer on the topics listed in a yaml file.  Sending the trigger topic containing 
the desired group name will trigger the buffer to write to a list of selected topics in a bag. For example, we can 
create a `snapshot_group` named `robot_state` and trigger the rosbag recording as:
```commandline
rostopic pub /trigger_snapshot std_msgs/String "data: 'snapshot_groups/robot_state'" 
```
snapshot_trigger is a wrapper node with a subscriber that listens for event triggers and forwards them to the rosbag_snapshot node.
```xml
<!-- launch/snapshot_buffer.launch -->
<launch>
  <!-- parameters for topics added to the buffer are all in a
       rosparam in config/recording_params.yaml -->
  <!-- to begin writing to bag, publish to the trigger topic -->
  <!-- adapted from https://github.com/ros/rosbag_snapshot -->
  <rosparam file="$(find event_recording)/recording_params.yaml" command="load" />
  <node name="snapshot_buffer" pkg="rosbag_snapshot" type="snapshot" args="" output="screen" />
  <node name="snapshot_trigger" pkg="event_recording" type="snapshot_trigger_node" output="screen" />
</launch>
```

In `recording_params.yaml` under the config dir, we can define the topics to be buffered along with
the duration and memory limits under `snapshot_buffer`. We can pick all or a portion of the topics to be recorded. In this 
example, we create a group named `robot_state` and select the topics to be recorded under `snapshot_group/robot_state/topics`.
If an abosolute path is not provided, the rosbag will be saved in ~/.ros/NAME_OF_BAG.bag.

```yaml
snapshot_buffer:
  default_duration_limit: 10        # Maximum time difference between newest and oldest message, seconds, overrides -d flag
  default_memory_limit: 100         # (-1 = no limit) Maximum memory used by messages in each topic's buffer, MB, override -s flag
  clear_buffer: true                # Option to clear buffer after writing to bag. override -n flag
  trigger_topic: "trigger_snapshot" # topic to call to trigger snapshot
  topics:
      - /cmd_vel
      - /joint_states
      - /tf
      - /tf_static

# Preset a list of topics to write to bag
snapshot_groups:
  robot_state:
    filename: "bag_"
    topics:
      - /cmd_vel
      - /joint_states
      - /tf
      - /tf_static
```


## Programmatically triggering the buffer to write to bag

Here is an example on how to trigger the trigger topic using a Python script. In the code below, topic and group are defined
as `"/trigger_snapshot"` and `"snapshot_groups/robot_state"`, respectively. Publishing the group to the trigger topic will
automatically save the rosbag file with a ROS Time. This will write whatever is in the buffer with the topics listed under 
the `robot_state` to the path specified in the yaml file.

```python
import rospy
from std_msgs.msg import String

def trigger_nav():
    # the topic param gets loaded with the launch file for the buffer
    topic = "/trigger_snapshot"
    # name of the nav group
    group = "snapshot_groups/robot_state"
    pub = rospy.Publisher(topic, String, queue_size=10)
    # send the trigger message
    # This will write the topics under the "snapshot_groups/nav/topics" rosparam
    # to a bag file to the filename specified "snapshot_groups/robot_state/filename" + ROS::TIME NOW
    pub.publish(group)

if __name__ == '__main__':
    rospy.init_node('trigger', anonymous=True)
    try:
        trigger_nav()
    except:
        pass
```
## Example with multiple snapshot_group

Here is an example of how to create multiple snapshot_groups with custom topics for navigation and localization. Topic names
might be varied for different robots.
```yaml
snapshot_buffer:
  default_duration_limit: 10
  default_memory_limit: 200
  clear_buffer: true
  trigger_topic: "trigger_snapshot"
  topics:
    - /cmd_vel
    - /camera/camera_info
    - /camera/rgb/image_raw
    - /gps/fix
    - /imu/data
    - /joint_states
    - /move_base/global_costmap/costmap
    - /move_base/local_costmap/costmap
    - /odom
    - /odometry/filtered
    - /odometry/global
    - /pointclouds
    - /tf 
    - /tf_static
    
snapshot_groups:
  navigation:
    filename: "bag_"
    topics:
       - /cmd_vel
       - /gps/fix
       - /imu/data
       - /joint_states
       - /move_base/global_costmap/costmap
       - /move_base/local_costmap/costmap
       - /odom
       - /odometry/filtered
       - /odometry/global
       - /pointclouds
       - /tf 
       - /tf_static

  localization:
    filename: "bag_"
    topics:
       - /cmd_vel
       - /gps/fix
       - /imu/data
       - /joint_states
       - /odom
       - /odometry/filtered
       - /odometry/global
       - /tf 
       - /tf_static
```
## Command Line Tools for rosbag_snapshot

There are more [command line tools](https://github.com/DiuLaMaX/rosbag_snapshot/tree/feature/clear_buffer_option). These command line tools use the rosservice instead of the custom pub/sub wrapper.

### Write all buffered data to ‘<datetime>.bag’

```bash
rosrun rosbag_snapshot snapshot -t
```

### Write all buffered data to a bag file

```bash
rosrun rosbag_snapshot snapshot -t -O NAME_OF_BAG
```

### View the status of the buffer

```bash
rostopic echo /snapshot_status
```

Which returns something like:

```bash
topics:
  -
    topic: "/test"
    node_pub: ''
    node_sub: "/snapshot_1527185221120605085"
    window_start:
      secs: 62516
      nsecs: 761000000
    window_stop:
      secs: 62521
      nsecs: 984000000
    delivered_msgs: 524
    dropped_msgs: 0
    traffic: 2096
    period_mean:
      secs: 0
      nsecs:         0
    period_stddev:
      secs: 0
      nsecs:         0
    period_max:
      secs: 0
      nsecs:         0
    stamp_age_mean:
      secs: 0
      nsecs:         0
    stamp_age_stddev:
      secs: 0
      nsecs:         0
    stamp_age_max:
      secs: 0
      nsecs:         0
enabled: True
```

## Prometheus Client

[Prometheus client](https://github.com/prometheus/client_python/) is a useful tool to monitor and exporting the metrics of 
a robot, detecting faults and gaining overall insight into the performance of systems that help us improve them operationally.
Furthermore, we can link rosbag_snapshot to Prometheus. That is trigger the rosbag recording if we observe the metrics of 
the system are in an undesired state. 

### Installation
Prometheus client should be installed with rosdep. If not, you can 
install it with pip
```commandline
pip install prometheus-client
```
alternatively, install from source
```commandline
git clone https://github.com/prometheus/client_python
```

### Types of metrics
The four main types of [metric](http://prometheus.io/docs/concepts/metric_types/) are counter, gauge, summary, and histogram.
It's important to know which metric types to use for a given metric, and here is the [instrumentation best practices](https://prometheus.io/docs/practices/instrumentation/#counter-vs-gauge-summary-vs-histogram)
for picking the metric types. 

### Exporting 
There are [several ways](https://github.com/prometheus/client_python#exporting) to export the metrics. 
It's easiest to expose metrics over the prometheus server using [HTTP](https://github.com/prometheus/client_python#http).
```python
from prometheus_client import start_http_server

start_http_server(9000)
```
### Linking rosbags to metrics in Prometheus
In this example, [counter](https://github.com/prometheus/client_python#counter) is used to record
the number of the Estop presses and [gauge](https://github.com/prometheus/client_python#gauge) is used for linear and angular velocity.
Estop presses, linear and angular velocity are exported to http://localhost:9000/ by running this launch file.:

```bash
roslaunch event_recording prometheus_exporter.launch
```

In our example, rosbag_snapshot node is linked to an Estop. First, launch the rosbag_snapshot node if it is not launched:
```bash
roslaunch event_recording snapshot_buffer.launch
```

### Use Python script to simulate EStop
If you don't have an EStop, you can run this script to send a Estop message:
```commandline
rosrun event_recording fake_estop.py
```

Send Normal to set the ESTOP to released state:
```commandline
rostopic pub /estop std_msgs/Header "seq: 0
stamp:
  secs: 0
  nsecs: 0
frame_id: 'NORMAL'" 

```

Send ESTOP to press the EStop and trigger the rosbag snapshot:
```commandline
rostopic pub /estop std_msgs/Header "seq: 0
stamp:
  secs: 0
  nsecs: 0
frame_id: 'ESTOP'" 

```

### Using a physical EStop
Connect EStop with [rosserial](http://wiki.ros.org/rosserial_server) and press the EStop to trigger the rosbag_snapshot. : 
```commandline
rosrun rosserial_server serial_node name:=estop
```

### Echo the state of the EStop
```commandline
rostopic echo /estop
```
## Play Rosbags
You can play the rosbag without waiting with the following command:
```bash
rosbag play -i NAME_OF_BAG.bag
```
More [rosbag commands ](http://wiki.ros.org/rosbag/Commandline).