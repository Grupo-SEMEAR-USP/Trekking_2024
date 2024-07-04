#!/usr/bin/env python3
import rospy

import tf

import tf2_ros
import geometry_msgs.msg
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry

class OdometryClass:
    def __init__(self): #Constructor of the class
        self.vehicle_name = rospy.get_param('~vehicle_name', 'ackermann_vehicle')
        self.global_frame_id = rospy.get_param('~global_frame_id', 'odom')
        self.odom_publisher = rospy.Publisher('/odom', Odometry, queue_size=1)
        rospy.Subscriber('/gazebo/model_states',
                        ModelStates,
                        self.handle_vehicle_pose,
                        self.vehicle_name) #Gets the info of Pose and Twist from this topic, directly from gazebo

        self.vehicle_index = 0 #It's the correct index of the robot in the ModelStates msg

        self.odom_msg = Odometry()
        self.br = tf2_ros.TransformBroadcaster()

        #Defining the initial conditions of the transform between the odom and the base_link frames
        self.t = geometry_msgs.msg.TransformStamped()
        self.t.header.stamp = rospy.Time.now()
        self.t.header.frame_id = "odom"
        self.t.child_frame_id = "base_link"
        self.t.transform.translation.x = 0.0
        self.t.transform.translation.y = 0.0
        self.t.transform.translation.z = 0.0
        self.t.transform.rotation.x = 0.0
        self.t.transform.rotation.y = 0.0
        self.t.transform.rotation.z = 0.0
        self.t.transform.rotation.w = 1

        self.rate = rospy.Rate(1) #Defining the rate that odom_msg will be published and the transform information will be sent

        #Defining the map frame and creating the transform between it and the odom frame
        self.t_map_odom = geometry_msgs.msg.TransformStamped()
        self.t_map_odom.header.stamp = rospy.Time.now()
        self.t_map_odom.header.frame_id = "map"
        self.t_map_odom.child_frame_id = "odom"
        self.t_map_odom.transform.translation.x = 0.0
        self.t_map_odom.transform.translation.y = 0.0
        self.t_map_odom.transform.translation.z = 0.0
        self.t_map_odom.transform.rotation.x = 0.0
        self.t_map_odom.transform.rotation.y = 0.0
        self.t_map_odom.transform.rotation.z = 0.0
        self.t_map_odom.transform.rotation.w = 1.0

        #Defining virtual frames to help in some geometry calculations in the ackermann_controller node
        #Left virtual frame
        self.t_left = geometry_msgs.msg.TransformStamped()
        self.t_left.header.stamp = rospy.Time.now()
        self.t_left.header.frame_id = "base_link"
        self.t_left.child_frame_id = "left_virtual_frame"
        self.t_left.transform.translation.x = 0.10378 
        self.t_left.transform.translation.y = -0.21397
        self.t_left.transform.translation.z = -0.0371
        self.t_left.transform.rotation.x = 0.0
        self.t_left.transform.rotation.y = 0.0
        self.t_left.transform.rotation.z = 0.0
        self.t_left.transform.rotation.w = 1.0

        #Right virtual frame
        self.t_right = geometry_msgs.msg.TransformStamped()
        self.t_right.header.stamp = rospy.Time.now()
        self.t_right.header.frame_id = "base_link"
        self.t_right.child_frame_id = "right_virtual_frame"
        self.t_right.transform.translation.x = -0.10624 
        self.t_right.transform.translation.y = -0.21396 
        self.t_right.transform.translation.z = -0.0371
        self.t_right.transform.rotation.x = 0.0
        self.t_right.transform.rotation.y = 0.0
        self.t_right.transform.rotation.z = 0.0
        self.t_right.transform.rotation.w = 1.0

        self.odom_tf_pub()

    def handle_vehicle_pose(self, msg, vehicle_name): #Callback function to the topic gazebo/model_states
        self.vehicle_index = msg.name.index(vehicle_name)

        #Filling the transform message with the information from gazebo
        self.t.header.stamp = rospy.Time.now()
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
            self.br.sendTransform(self.t) #Updating the transform tree with the transform between the base_link and the world
            self.odom_publisher.publish(self.odom_msg) #Publishing the odometry message into the topic /odom

            self.br.sendTransform(self.t_map_odom) #Updating the transform between the map and the odom frames

            #Broadcasting the transforms involving the virtual frames
            self.br.sendTransform(self.t_left)
            self.br.sendTransform(self.t_right)

            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('odom_pub_simul')
    rospy.loginfo("Odometry node initialized")
    
    oc = OdometryClass()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
