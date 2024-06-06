#! /usr/bin/env python

import rospy
from math import pi, sin, cos
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3
from navigation.msg import encoders
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
import tf
from tf.broadcaster import TransformBroadcaster

class OdometryClass:

    def __init__(self):
        self.ticks_sub = rospy.Subscriber('/encoders', encoders, self.getTicks)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.odom_broadcaster = TransformBroadcaster()
        self.odom = Odometry()
        self.rate = rospy.Rate(1)
        self.lastLeftTicks = 0
        self.lastRightTicks = 0
        self.currentLeftTicks = 0
        self.currentRightTicks = 0
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.L = 0.3 #Distance between both wheels
        self.R = 0.1 #Radius of the wheels
        self.N = 360 #Number of encoder ticks in a complete revolution
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        rospy.wait_for_message('/clock', Clock)
        self.updatePose()

    def getTicks(self, msg):
        self.currentLeftTicks = msg.encoderTicks[1]
        self.currentRightTicks = msg.encoderTicks[0]
        
    def updatePose(self):

        while not rospy.is_shutdown():
            #Calculating the variation of ticks compared to the last time
            delta_l = self.currentLeftTicks - self.lastLeftTicks
            delta_r = self.currentRightTicks - self.lastRightTicks
            #Determining the space variation of each wheel
            d_l = 2 * pi * self.R * delta_l / self.N
            d_r = 2 * pi * self.R * delta_r / self.N

            #Updating the number of ticks
            self.lastLeftTicks = self.currentLeftTicks
            self.lastRightTicks = self.currentRightTicks
            
            #Determinig the time variation
            self.current_time = rospy.Time.now()
            dt = (self.current_time - self.last_time).to_sec()
            
            if dt == 0.0: #Assuring that dt is not 0, in order to make the further calculations doable
                continue
            
            #---------- ESSA PARTE PRECISA ADAPTAR PARA O MODELO ACKERMANN --------
            v = ((d_r + d_l) / 2) / dt
            w = ((d_r - d_l) / self.L) / dt

            # compute odometry in a typical way given the velocities of the robot
            delta_x = v * cos(self.th) * dt
            delta_y = v * sin(self.th) * dt
            delta_th = w

            self.x += delta_x
            self.y += delta_y
            self.th += delta_th

            #------------------------------------------------------------------------

            # since all odometry is 6DOF we'll need a quaternion created from yaw
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

            # optional: broadcast the transform over tf
            self.odom_broadcaster.sendTransform(
                (self.x, self.y, 0.),
                odom_quat,
                self.current_time,
                "link_chassis", #ATUALIZAR NOME DO FRAME
                "odom"
            )

            # publish the odometry message over ROS
            odom = Odometry()
            odom.header.stamp = self.current_time
            odom.header.frame_id = "odom"

            # set the position
            odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(odom_quat))

            # set the velocity
            odom.child_frame_id = "link_chassis" #ATUALIZAR NOME DO FRAME
            odom.twist.twist = Twist(Vector3(v, 0, 0), Vector3(0, 0, w))

            # publish the message
            self.odom_pub.publish(odom)

            self.last_time = self.current_time
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node('odom_pub')
    rospy.loginfo("Odometry node initialized.")
    oc = OdometryClass()
    rospy.spin()