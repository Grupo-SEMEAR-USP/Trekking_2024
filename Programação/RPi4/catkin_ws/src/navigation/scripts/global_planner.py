#!/usr/bin/env python3
import rospy

from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

#Auxiliar functions
def init_pose_variable(frame_id): #Set default values to a pose variable
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = frame_id
    pose.pose.position.x = 0.0
    pose.pose.position.y = 0.0
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 1.0

    return pose

class GlobalPlanner:
    def __init__(self):
        #Defining the subscribers and publishers
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.update_pose, queue_size=1)
        self.path_pub = rospy.Publisher("GlobalPlanner/plan", Path, queue_size=1) #Topic where the global trajectoy will be published
        self.goal_pose_sub = rospy.Subscriber("goal", PoseStamped, self.update_goal_pose, queue_size=1) #Topic where the global planner will read the goal pose
        self.teste_pub = rospy.Publisher("teste", PoseStamped, queue_size=1)
        
        #TF listener

        #Defining the pose variable of the robot
        self.cur_pose = PoseStamped()
        self.cur_pose = init_pose_variable("odom")

        #Defining the goal pose variable
        self.goal_pose = PoseStamped()
        self.goal_pose = init_pose_variable("odom")

    def update_pose(self, msg): #Callback function to the topic odom -> updates the current pose of the robot
        self.cur_pose.header.stamp = rospy.Time.now()
        self.cur_pose.pose = msg.pose.pose
    
    def update_goal_pose(self, msg): #Callback function to the topic goal -> updates the goal pose
        self.goal_pose.header.stamp = rospy.Time.now()
        self.goal_pose.pose = msg.pose

        #self.teste_pub.publish(self.goal_pose)

        self.create_path() #Creating a path to reach the new goal pose
    
    def create_path(self): #Creates and publishes a linear path from the cur_pose to the goal_pose
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "odom"

        num_points = 10 #Number of poses of the path

        self.teste_pub.publish(self.cur_pose)

        cur_pose_copy = self.cur_pose #Assuring that the base_link pose will not change while the creation of the trajectory

        for i in range(num_points): #Creating the linear path
            ratio = float(i)/(num_points-1) #Sample number * discretization step

            mid_pose = PoseStamped() #Represents the PoseStamped that composes the PoseStamped[] within the Path msg

            #Calculating the mid pose according to the current sample
            mid_pose.pose.position.x = cur_pose_copy.pose.position.x + (ratio * (self.goal_pose.pose.position.x - cur_pose_copy.pose.position.x))
            mid_pose.pose.position.y = cur_pose_copy.pose.position.y + (ratio * (self.goal_pose.pose.position.y - cur_pose_copy.pose.position.y))
            mid_pose.pose.position.z = cur_pose_copy.pose.position.z + (ratio * (self.goal_pose.pose.position.z - cur_pose_copy.pose.position.z))

            mid_pose.pose.orientation.x = cur_pose_copy.pose.orientation.x + (ratio * (self.goal_pose.pose.orientation.x - cur_pose_copy.pose.orientation.x))
            mid_pose.pose.orientation.y = cur_pose_copy.pose.orientation.y + (ratio * (self.goal_pose.pose.orientation.y - cur_pose_copy.pose.orientation.y))
            mid_pose.pose.orientation.z = cur_pose_copy.pose.orientation.z + (ratio * (self.goal_pose.pose.orientation.z - cur_pose_copy.pose.orientation.z))
            mid_pose.pose.orientation.w = cur_pose_copy.pose.orientation.w + (ratio * (self.goal_pose.pose.orientation.w - cur_pose_copy.pose.orientation.w))

            #Filling the rest of the PoseStamped msg
            mid_pose.header.stamp = rospy.Time.now()
            mid_pose.header.frame_id = "odom"

            #Appending the calculated pose to the path
            path.poses.append(mid_pose)
        
        #Publishing the linear path
        self.path_pub.publish(path)



if __name__ == "__main__":
    rospy.init_node("global_planner_node")
    rospy.loginfo("Global Planner initialized")

    gp = GlobalPlanner()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass