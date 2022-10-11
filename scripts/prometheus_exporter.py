#!/usr/bin/env python3
import rospy
from prometheus_client import Counter, Gauge, start_http_server

from std_msgs.msg import String, Float64, Header
from geometry_msgs.msg import Twist


class MetricCollector:
    def __init__(self):
        rospy.loginfo('starting metrics server to expose metrics')
        # Start up the server to expose the metrics.
        start_http_server(9000)
        rospy.loginfo('metrics server started')
        self.robot_label_ = 'robot'
        # amount of time when new event is considered (seconds)
        self.event_cutoff_time_sec_ = 10  # seconds

        # stats counters
        self.estop_c_ = Counter('estop_presses', 'Count of E-Stop presses', ['robot'])
        self.speed_h_ = Gauge('vehicle_speed_mph', 'Monitors the speed of the vehicle', ['robot'])
        self.steering_h_ = Gauge('vehicle_steering_angle', 'Monitors the steering angle of the vehicle', ['robot'])

        self.estop_pressed_ = True
        self.estop_timer_ = rospy.Time(0)
        # publisher for triggering rosbag snapshots
        self.snapshot_pub = rospy.Publisher("/trigger_snapshot", String, queue_size=10)

        # subscribers
        self.estop_sub_ = rospy.Subscriber('/estop', Header, self.estop_cb, queue_size=1)
        self.speed_sub_ = rospy.Subscriber('/cmd_vel', Twist, self.speed_cb, queue_size=1)

        self.run()

    def trigger_snapshot(self, group):
        rospy.loginfo("Estop pressed while driving. Writing data to bag")
        self.snapshot_pub.publish(group)

    # Callback Functions
    def speed_cb(self, data):
        # record the linear velocity
        self.speed_h_.labels(robot=self.robot_label_).set(data.linear.x)
        # record the steering angle
        self.steering_h_.labels(robot=self.robot_label_).set(data.angular.z)

    def estop_cb(self, data):
        # count change from NORMAL to ESTOP if there has been (self.event_cutoff_time_sec_) seconds or more between
        # presses
        if data.frame_id == 'ESTOP' and self.estop_pressed_ is False and (
                rospy.Time.now().to_sec() - self.estop_timer_.to_sec()) > self.event_cutoff_time_sec_:
            self.estop_pressed_ = True
            self.estop_timer_ = rospy.Time.now()
            self.estop_c_.labels(robot=self.robot_label_).inc()
            self.trigger_snapshot("snapshot_groups/robot_state")

        # estop released, change state. Don't change this state until after (self.event_cutoff_time_sec_) seconds
        if data.frame_id == 'NORMAL' and self.estop_pressed_ is True and (
                rospy.Time.now().to_sec() - self.estop_timer_.to_sec()) > self.event_cutoff_time_sec_:
            self.estop_pressed_ = False
            return

    @staticmethod
    def run():
        rate = rospy.Rate(0.5)
        while not rospy.is_shutdown():
            # every 2 seconds update regardless of scraping interval
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('prometheus_exporter_node', anonymous=True)
    rospy.loginfo('started prometheus_exporter_node')
    rospy.loginfo('waiting 2 seconds to begin recording metrics')
    rospy.sleep(2)
    collector = MetricCollector()
