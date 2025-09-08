#!/usr/bin/env python
import rospy
import rostest
import unittest
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

class SlamTest(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(SlamTest, self).__init__(*args, **kwargs)
        self.poses = []
        self.cloud = None

    def setUp(self):
        rospy.Subscriber('/Odometry', Odometry, self.pose_cb)
        rospy.Subscriber('/cloud_global_map', PointCloud2, self.cloud_cb)

    def pose_cb(self, msg):
        self.poses.append(msg.header.stamp)

    def cloud_cb(self, msg):
        self.cloud = msg

    def test_pose_frequency(self):
        timeout = rospy.Duration(2.0)
        rospy.sleep(timeout)
        expected_freq = 10.0
        rate = len(self.poses) / timeout.to_sec() if timeout.to_sec() > 0 else 0
        self.assertGreaterEqual(rate, expected_freq * 0.9, 'SLAM pose frequency too low')

    def test_mapping_cloud(self):
        try:
            rospy.wait_for_message('/cloud_global_map', PointCloud2, timeout=5.0)
        except rospy.ROSException:
            self.fail('No point cloud received')
        self.assertIsNotNone(self.cloud)
        points = self.cloud.width * self.cloud.height
        self.assertGreater(points, 0, 'Point cloud is empty')

if __name__ == '__main__':
    rostest.rosrun('gslivm', 'slam_test', SlamTest)
