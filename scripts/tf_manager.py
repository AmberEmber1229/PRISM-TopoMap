#!/usr/bin/env python

import rospy
import numpy as np
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation
from flow_trace import FlowTracer

class OdometryPublisher:
    def __init__(self):
        rospy.init_node('odometry_publisher')
        self.publish_odom_from_tf = rospy.get_param('~publish_odom_from_tf', False)
        self.source_frame = rospy.get_param('~source_frame', 'map')
        self.target_frame = rospy.get_param('~target_frame', 'base_footprint')
        self.publish_tf_from_odom = rospy.get_param('~publish_tf_from_odom', False)
        self.odometry_topic = rospy.get_param('~odometry_topic', '/odom')
        self.tf_from_odom_target_frame = rospy.get_param('~tf_from_odom_target_frame', 'base_link')
        self.trace = FlowTracer(
            enabled=rospy.get_param('~trace_data_flow', False),
            every_n=rospy.get_param('~trace_every_n_processed_frames', 1))
        self.publish_count = 0
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.rate = rospy.Rate(10)
        self.odometry_publisher = rospy.Publisher('/odom_gt', Odometry, latch=True, queue_size=100)
        if self.publish_tf_from_odom:
            odom_sub = rospy.Subscriber(self.odometry_topic, Odometry, self.odom_callback)
            self.tfbr = tf2_ros.TransformBroadcaster()
        self.trace.log(
            'TF', force=True, RESULT='TF_MANAGER_STARTED',
            publish_odom_from_tf=self.publish_odom_from_tf,
            source_frame=self.source_frame,
            target_frame=self.target_frame,
            output_topic='/odom_gt',
            publish_tf_from_odom=self.publish_tf_from_odom,
            input_odometry_topic=self.odometry_topic,
            tf_from_odom_target_frame=self.tf_from_odom_target_frame)

    def odom_callback(self, msg):
        tf_msg = TransformStamped()
        tf_msg.header = msg.header
        tf_msg.child_frame_id = self.tf_from_odom_target_frame
        try:
            tf_sensor_to_base = self.tf_buffer.lookup_transform(msg.child_frame_id, self.tf_from_odom_target_frame, rospy.Time(0))
                                                        #self.tf_listener.getLatestCommonTime(self.source_frame, self.target_frame))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print('Could not get transform from {} to {}'.format(msg.child_frame_id, self.tf_from_odom_target_frame))
            self.trace.log(
                'TF', stamp=msg.header.stamp, force=True,
                RESULT='LOOKUP_FAILED',
                source_frame=msg.child_frame_id,
                target_frame=self.tf_from_odom_target_frame)
            return
        tf_matrix_odom_to_sensor = np.eye(4)
        tf_matrix_odom_to_sensor[:3, 3] = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]
        R = Rotation.from_quat([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]).as_matrix()
        tf_matrix_odom_to_sensor[:3, :3] = R
        tf_matrix_sensor_to_base = np.eye(4)
        tf_matrix_sensor_to_base[:3, 3] = [
            tf_sensor_to_base.transform.translation.x,
            tf_sensor_to_base.transform.translation.y,
            tf_sensor_to_base.transform.translation.z
        ]
        R = Rotation.from_quat([
            tf_sensor_to_base.transform.rotation.x,
            tf_sensor_to_base.transform.rotation.y,
            tf_sensor_to_base.transform.rotation.z,
            tf_sensor_to_base.transform.rotation.w
        ]).as_matrix()
        tf_matrix_sensor_to_base[:3, :3] = R
        tf_matrix_odom_to_base = tf_matrix_odom_to_sensor @ tf_matrix_sensor_to_base
        tf_msg.transform.translation.x, \
        tf_msg.transform.translation.y, \
        tf_msg.transform.translation.z = tf_matrix_odom_to_base[:3, 3]
        q = Rotation.from_matrix(tf_matrix_odom_to_base[:3, :3]).as_quat()
        tf_msg.transform.rotation.x, \
        tf_msg.transform.rotation.y, \
        tf_msg.transform.rotation.z, \
        tf_msg.transform.rotation.w = q
        self.tfbr.sendTransform(tf_msg)
        self.extract_tf_and_publish_odom()

    def extract_tf_and_publish_odom(self):
        odom_msg = Odometry()
        odom_msg.header.frame_id = self.source_frame
        odom_msg.child_frame_id = self.target_frame
        try:
            transform_stamped = self.tf_buffer.lookup_transform(self.source_frame, self.target_frame, rospy.Time(0))
                                                        #self.tf_listener.getLatestCommonTime(self.source_frame, self.target_frame))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print('Could not get transform from {} to {}'.format(self.source_frame, self.target_frame))
            self.trace.log(
                'TF', force=True, RESULT='LOOKUP_FAILED',
                source_frame=self.source_frame,
                target_frame=self.target_frame)
            return
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.pose.pose.position.x = transform_stamped.transform.translation.x
        odom_msg.pose.pose.position.y = transform_stamped.transform.translation.y
        odom_msg.pose.pose.position.z = 0
        odom_msg.pose.pose.orientation = transform_stamped.transform.rotation
        self.odometry_publisher.publish(odom_msg)
        self.publish_count += 1
        if self.trace.enabled and (self.publish_count == 1 or self.publish_count % 100 == 0):
            self.trace.log(
                'TF', stamp=odom_msg.header.stamp, force=True,
                RESULT='ODOM_GT_PUBLISHED',
                publish_count=self.publish_count,
                source_frame=self.source_frame,
                target_frame=self.target_frame,
                output_topic='/odom_gt')

    def run(self):
        if self.publish_tf_from_odom:
            rospy.spin()
        else:
            while not rospy.is_shutdown():
                self.extract_tf_and_publish_odom()
                self.rate.sleep()

if __name__ == '__main__':
    odometry_publisher = OdometryPublisher()
    odometry_publisher.run()
