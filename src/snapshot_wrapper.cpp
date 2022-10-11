#include <ros/ros.h>

#include "event_recording/snapshot_wrapper.h"
#include <rosbag_snapshot_msgs/TriggerSnapshot.h>
#include <std_msgs/String.h>

#include <boost/bind.hpp>
#include <string>
#include <vector>

SnapshotTrigger::SnapshotTrigger()
{ 
  // set up client before subscribing
  client_ = nh_.serviceClient<rosbag_snapshot_msgs::TriggerSnapshot>("trigger_snapshot");
  client_.waitForExistence();
  
  std::string topic;
  if (!nh_.getParam("/snapshot_buffer/trigger_topic", topic))
  {
    ROS_ERROR("Snapshot Trigger Topic not found. Is the buffer running?");
  }
  sub_ = nh_.subscribe(topic, 100, &SnapshotTrigger::snapshotCb, this);
  ROS_INFO("subscribing to %s", topic.c_str());
}

SnapshotTrigger::~SnapshotTrigger(void) {}


void SnapshotTrigger::snapshotCb(const std_msgs::String::ConstPtr& msg)
{
  // wrapper around the service trigger callback
  rosbag_snapshot_msgs::TriggerSnapshotRequest req;
  rosbag_snapshot_msgs::TriggerSnapshot srv;

  // get the filename
  std::string filename;
  if (!nh_.getParam(msg->data + "/filename", filename))
  {
    ROS_ERROR("Config filename incorrect. Attempted param path: %s", (msg->data + "/filename").c_str());
    return;
  }

  req.filename = filename;
  req.start_time = ros::Time(0);
  req.stop_time = ros::Time(0);

  // convert topic group to list of topics
  std::vector<std::string> topic_list;
  if (!nh_.getParam((msg->data + "/topics"), topic_list))
  {
    ROS_ERROR("Writing to bag failed. Error: Topic list %s does not exist", (msg->data + "/topics").c_str());
    return;
  }
  req.topics = topic_list;

  // use rosservice to write bag
  srv.request = req;
  if (client_.call(srv))
  {
    if (srv.response.success)
    {
      ROS_INFO("Successfully wrote group %s to bag.", msg->data.c_str());
    }
    else 
    {
      ROS_ERROR("Failed to write to bag. Error: %s", srv.response.message.c_str());
      return;
    }
  }
  else
  {
    ROS_ERROR("Failed to call rosbag_snapshot service");
    return;
  }
}
  

int main(int argc, char** argv)
{
  ros::init(argc, argv, "snapshot_trigger_node");
  ROS_INFO("creating snapshot trigger node");
  SnapshotTrigger snapshot;
  ros::spin();
  return 0;
}
