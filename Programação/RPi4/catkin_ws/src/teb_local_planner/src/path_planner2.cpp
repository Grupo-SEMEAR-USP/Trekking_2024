#include <teb_local_planner/teb_local_planner_ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/Marker.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>


using namespace teb_local_planner; // it is ok here to import everything for testing purposes

// ============= Global Variables ================
// Ok global variables are bad, but here we only have a simple testing node.
PlannerInterfacePtr planner;
TebVisualizationPtr visual;
std::vector<ObstaclePtr> obst_vector;
ViaPointContainer via_points;
TebConfig config;
boost::shared_ptr< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> > dynamic_recfg;
ros::Subscriber custom_obst_sub;
ros::Subscriber via_points_sub;
ros::Subscriber clicked_points_sub;
std::vector<ros::Subscriber> obst_vel_subs;
unsigned int no_fixed_obstacles;

geometry_msgs::Pose current_pose, goal_pose, init_pose;

// =========== Function declarations =============
void CB_mainCycle(const ros::TimerEvent& e);
void CB_publishCycle(const ros::TimerEvent& e);
void CB_reconfigure(TebLocalPlannerReconfigureConfig& reconfig, uint32_t level);
void CB_clicked_points(const geometry_msgs::PointStampedConstPtr& point_msg);
void CB_via_points(const nav_msgs::Path::ConstPtr& via_points_msg);

void CB_goal(const geometry_msgs::PoseStamped::ConstPtr& msg);
void CB_odom(const nav_msgs::Odometry::ConstPtr& msg);

// =============== Main function =================
int main( int argc, char** argv )
{
  ros::init(argc, argv, "path_planner_node");
  ros::NodeHandle n("~");
 
  
  // load ros parameters from node handle
  config.loadRosParamFromNodeHandle(n);
 
  ros::Timer cycle_timer = n.createTimer(ros::Duration(0.025), CB_mainCycle);
  ros::Timer publish_timer = n.createTimer(ros::Duration(0.1), CB_publishCycle);
  
  // setup dynamic reconfigure
  dynamic_recfg = boost::make_shared< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> >(n);
  dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>::CallbackType cb = boost::bind(CB_reconfigure, _1, _2);
  dynamic_recfg->setCallback(cb);
  
  //--- Publishers and Subscribers ---
  clicked_points_sub = n.subscribe("/clicked_point", 5, CB_clicked_points); // setup callback for clicked points (in rviz) that are considered as via-points
  via_points_sub = n.subscribe("via_points", 1, CB_via_points); // setup callback for via-points (callback overwrites previously set via-points)
  
  ros::Subscriber goal_pose_sub = n.subscribe("goal", 1, CB_goal); //subscribing to the goal topic to get the goal pose 
  ros::Subscriber odom_sub = n.subscribe("/odom", 1, CB_odom); //subscribing to the odom topic to get robot's current pose
  
  // Setup visualization
  visual = TebVisualizationPtr(new TebVisualization(n, config));
  
  // Setup robot shape model
  config.robot_model = TebLocalPlannerROS::getRobotFootprintFromParamServer(n, config);
  
  // Setup planner (homotopy class planning or just the local teb planner)
  if (config.hcp.enable_homotopy_class_planning)
    planner = PlannerInterfacePtr(new HomotopyClassPlanner(config, &obst_vector, visual, &via_points));
  else
    planner = PlannerInterfacePtr(new TebOptimalPlanner(config, &obst_vector, visual, &via_points));
  

  no_fixed_obstacles = obst_vector.size();
  ros::spin();

  return 0;
}

//Updating the robot's current pose
void CB_odom(const nav_msgs::Odometry::ConstPtr& msg){
  current_pose.position.x = msg->pose.pose.position.x;
  current_pose.position.y = msg->pose.pose.position.y;
  current_pose.position.z = msg->pose.pose.position.z;

  current_pose.orientation.x = msg->pose.pose.orientation.x;
  current_pose.orientation.y = msg->pose.pose.orientation.y;
  current_pose.orientation.z = msg->pose.pose.orientation.z;
  current_pose.orientation.w = msg->pose.pose.orientation.w;
}

//Updating the goal pose
void CB_goal(const geometry_msgs::PoseStamped::ConstPtr& msg){
  //Updating the goal_pose
  goal_pose.position.x = msg->pose.position.x;
  goal_pose.position.y = msg->pose.position.y;
  goal_pose.position.z = msg->pose.position.z;

  goal_pose.orientation.x = msg->pose.orientation.x;
  goal_pose.orientation.y = msg->pose.orientation.y;
  goal_pose.orientation.z = msg->pose.orientation.z;
  goal_pose.orientation.w = msg->pose.orientation.w;

  //Updating the init_pose of the trajectory
  init_pose.position.x = current_pose.position.x;
  init_pose.position.y = current_pose.position.y;
  init_pose.position.z = current_pose.position.z;

  init_pose.orientation.x = current_pose.orientation.x;
  init_pose.orientation.y = current_pose.orientation.y;
  init_pose.orientation.z = current_pose.orientation.z;
  init_pose.orientation.w = current_pose.orientation.w;
}

// Planning loop
void CB_mainCycle(const ros::TimerEvent& e)
{
  planner->plan(PoseSE2(init_pose.position.x,init_pose.position.y,init_pose.orientation.z), PoseSE2(goal_pose.position.x,goal_pose.position.y,goal_pose.orientation.z)); // hardcoded start and goal for testing purposes
}

// Visualization loop
void CB_publishCycle(const ros::TimerEvent& e)
{
  planner->visualize();
  visual->publishObstacles(obst_vector);
  visual->publishViaPoints(via_points);
}

void CB_reconfigure(TebLocalPlannerReconfigureConfig& reconfig, uint32_t level)
{
  config.reconfigure(reconfig);
}

void CB_clicked_points(const geometry_msgs::PointStampedConstPtr& point_msg)
{
  // we assume for simplicity that the fixed frame is already the map/planning frame
  // consider clicked points as via-points
  via_points.push_back( Eigen::Vector2d(point_msg->point.x, point_msg->point.y) );
  ROS_INFO_STREAM("Via-point (" << point_msg->point.x << "," << point_msg->point.y << ") added.");
  if (config.optim.weight_viapoint<=0)
    ROS_WARN("Note, via-points are deactivated, since 'weight_via_point' <= 0");
}

void CB_via_points(const nav_msgs::Path::ConstPtr& via_points_msg)
{
  ROS_INFO_ONCE("Via-points received. This message is printed once.");
  via_points.clear();
  for (const geometry_msgs::PoseStamped& pose : via_points_msg->poses)
  {
    via_points.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }
}