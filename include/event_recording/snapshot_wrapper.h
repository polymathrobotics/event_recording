#ifndef EVENT_RECORDING_SNAPSHOT_WRAPPER_H
#define EVENT_RECORDING_SNAPSHOT_WRAPPER_H

#include <ros/ros.h>

#include <rosbag_snapshot_msgs/TriggerSnapshot.h>
#include <std_msgs/String.h>

#include <boost/bind.hpp>
#include <string>
#include <vector>

class SnapshotTrigger
{
protected:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::ServiceClient client_;

  // callback for trigger subscriber
  void snapshotCb(const std_msgs::String::ConstPtr& msg);

public:
  SnapshotTrigger();
  ~SnapshotTrigger(void);
};
  
#endif // EVENT_RECORDING_SNAPSHOT_WRAPPER_H