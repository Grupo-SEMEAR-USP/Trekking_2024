#!/usr/bin/env python3
import rospy

import tf_conversions

import tf2_ros
import geometry_msgs.msg
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry

class OdometryClass:
    def __init__(self): #Constructor of the class
        self.vehicle_name = rospy.get_param('~vehicle_name', 'ackermann_vehicle')
        self.global_frame_id = rospy.get_param('~global_frame_id', 'world')
        self.odom_publisher = rospy.Publisher('/odom', Odometry, queue_size=1)
        rospy.Subscriber('/gazebo/model_states',
                        ModelStates,
                        self.handle_vehicle_pose,
                        self.vehicle_name) #Gets the info of Pose and Twist from this topic, directly from gazebo

        self.vehicle_index = 0 #It's the correct index of the robot in the ModelStates msg

        self.odom_msg = Odometry()
        self.br = tf2_ros.TransformBroadcaster()
        self.t = geometry_msgs.msg.TransformStamped()

        self.rate = rospy.Rate(1) #Defining the rate that odom_msg will be published and the transform information will be sent

        self.odom_tf_pub()

    def handle_vehicle_pose(self, msg, vehicle_name): #Callback function to the topic gazebo/model_states
        self.vehicle_index = msg.name.index(vehicle_name)
        '''br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()'''

        #Filling the transform message with the information from gazebo
        self.t.header.stamp = rospy.Time.now()
        self.t.header.frame_id = self.global_frame_id
        self.t.child_frame_id = self.vehicle_name
        self.t.transform.translation.x = msg.pose[self.vehicle_index].position.x
        self.t.transform.translation.y = msg.pose[self.vehicle_index].position.y
        self.t.transform.translation.z = 0.0
        self.t.transform.rotation = msg.pose[self.vehicle_index].orientation

        #Filling the odometry message with the information (Pose and Twist) from gazebo
        self.odom_msg.header.stamp = rospy.Time.now()
        self.odom_msg.header.frame_id = self.global_frame_id
        self.odom_msg.child_frame_id = vehicle_name
        self.odom_msg.pose.pose = msg.pose[self.vehicle_index]
        self.odom_msg.twist.twist = msg.twist[self.vehicle_index]
    
    def odom_tf_pub(self):
        while not rospy.is_shutdown():
            self.br.sendTransform(self.t) #Updating the transform tree
            self.odom_publisher.publish(self.odom_msg) #Publishing the odometry message into the topic /odom

            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('odom_pub_simul')
    rospy.loginfo("Odometry node initialized")
    
    oc = OdometryClass()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
