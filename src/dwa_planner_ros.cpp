#include <chrono>
#include <pluginlib/class_list_macros.h>
#include "dwa_planner_ros/dwa_planner_ros.h"

// register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(dwa_planner_ros::DWAPlannerROS, nav_core::BaseLocalPlanner)

namespace dwa_planner_ros {

DWAPlannerROS::DWAPlannerROS()
  : initialized_(false), size_x_(0), size_y_(0), goal_reached_(false){}

DWAPlannerROS::~DWAPlannerROS()
{
  freeMemory();

  if (planner_)
    delete planner_;
  if (costmap_model_)
    delete costmap_model_;
}

void DWAPlannerROS::freeMemory()
{
  if (charmap_)
  {
    for (int i = 0; i < size_x_; i++)
    {
      delete[] charmap_[i];
      charmap_[i] = NULL;
    }

    delete[] charmap_;
    charmap_ = NULL;
  }
}

void DWAPlannerROS::allocateMemory()
{
  assert(charmap_ == NULL);

  charmap_ = new unsigned char*[size_x_];
  for (int i = 0; i < size_x_; i++)
    charmap_[i] = new unsigned char[size_y_];
}

void DWAPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  // check if the plugin is already initialized
  if (!initialized_)
  {
    // create Node Handle with name of plugin (as used in move_base for loading)
    ros::NodeHandle private_nh("~/" + name);
    
    odom_topic_ = "/odometry/filtered";
    map_frame_ = "map";
    xy_goal_tolerance_ = 0.2;
    transform_tolerance_ = 0.5;
    max_vel_x_ = 0.55;
    min_vel_x_ = 0.0;
    max_vel_theta_ = 1.2;
    min_vel_theta_ = -1.2;
    acc_lim_x_ = 0.25;
    acc_lim_theta_ = 1.2;
    control_period_ = 0.2;

    // init other variables
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();  // locking should be done in MoveBase.
    size_x = costmap_->getSizeInCellsX();
    size_y = costmap_->getSizeInCellsY();
    resolution = costmap_->getResolution();
    origin_x = costmap_->getOriginX();
    origin_y = costmap_->getOriginY();

    costmap_model_ = new base_local_planner::CostmapModel(*costmap_);

    global_frame_ = costmap_ros_->getGlobalFrameID();
    robot_base_frame_ = costmap_ros_->getBaseFrameID();

    // Get footprint of the robot and minimum and maximum distance from the center of the robot to its footprint
    footprint_spec_ = costmap_ros_->getRobotFootprint();
    costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius_);

    // init the odom helper to receive the robot's velocity from odom messages
    odom_helper_.setOdomTopic(odom_topic_);

    planner_ = new DWAPlanner(costmap_model_, footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius_,
                              private_nh);

    global_plan_pub_ = private_nh.advertise<nav_msgs::Path>("dwa_global_plan", 1);
    planner_util_.initialize(tf_, costmap_, global_frame_);
    // set initialized flag
    initialized_ = true;

    ROS_DEBUG("dwa_local_planner plugin initialized.");
  }
  else
  {
    ROS_WARN("dwa_local_planner has already been initialized, doing nothing.");
  }
}

bool DWAPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_) {
    ROS_ERROR("Pure Pursuit Planner has not been initialized.");
    return false;
  }

  goal_reached_ = false;

  ROS_WARN("start Plan");
  return planner_util_.setPlan(plan);
}

bool DWAPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  // check if plugin is initialized
  if (!initialized_)
  {
    ROS_ERROR("dwa_local_planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  costmap_ros_->getRobotPose(current_pose_);
  double robot_pose_x = current_pose_.pose.position.x;
  double robot_pose_y = current_pose_.pose.position.y; 
  double robot_pose_theta = tf2::getYaw(current_pose_.pose.orientation);

  geometry_msgs::PoseStamped robot_vel_tf;
  odom_helper_.getRobotVel(robot_vel_tf);
  double robot_vel_x = robot_vel_tf.pose.position.x;
  double robot_vel_theta = tf2::getYaw(robot_vel_tf.pose.orientation);

  std::vector<geometry_msgs::PoseStamped> transformed_plan;
  transformed_plan.clear();
  if (!planner_util_.getLocalPlan(current_pose_, transformed_plan)) {
    ROS_ERROR("Could not get local plan");
    return false;
  }
  geometry_msgs::PoseStamped goal_pose = transformed_plan.back();

  const unsigned char* charmap = costmap_->getCharMap();

  if (charmap_ == NULL || size_x_ != size_x || size_y_ != size_y)
  {
    freeMemory();

    size_x_ = size_x;
    size_y_ = size_y;

    allocateMemory();
  }

  for (int j = 0; j < size_y_; j++)
  {
    for (int i = 0; i < size_x_; i++)
      charmap_[i][j] = charmap[i + j * size_x_];
  }

  std::vector<std::vector<double>> reference_path;
  for (const auto& pose : transformed_plan)
  {
    double x = pose.pose.position.x;
    double y = pose.pose.position.y;
    double theta = tf2::getYaw(pose.pose.orientation);
    reference_path.push_back({x, y, theta});
  }
  publishGlobalPlan(transformed_plan);

  double dwa_cmd_vel_x, dwa_cmd_vel_theta;
  bool success = planner_->computeVelocityCommands(robot_vel_x, robot_vel_theta, robot_pose_x, robot_pose_y, robot_pose_theta,
                                                   reference_path, charmap_, size_x_, size_y_,
                                                   resolution, origin_x, origin_y, dwa_cmd_vel_x, dwa_cmd_vel_theta);

  if (!success)
  {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    return false;
  }
  else
  {
    cmd_vel.linear.x = dwa_cmd_vel_x;
    cmd_vel.angular.z = dwa_cmd_vel_theta;
    return true;
  }
  
  geometry_msgs::PoseStamped robot_pose;
  costmap_ros_->getRobotPose(robot_pose);
  geometry_msgs::PoseStamped global_plan_ = transformed_plan.back();

  double dx = robot_pose.pose.position.x - global_plan_.pose.position.x;
  double dy = robot_pose.pose.position.y - global_plan_.pose.position.y;

  if (hypot(dx, dy) < xy_goal_tolerance_) {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    goal_reached_ = true;
    ROS_INFO("Goal reached.");
  }
   return true;
}

bool DWAPlannerROS::isGoalReached()
{
  // check if plugin is initialized
  if (!initialized_)
  {
    ROS_ERROR("dwa_local_planner has not been initialized, please call initialize() before using this planner");
    return false;
  }
  return goal_reached_;
}

void DWAPlannerROS::publishGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan)
{
  nav_msgs::Path gui_path;
  gui_path.header.frame_id = map_frame_;
  gui_path.header.stamp = ros::Time::now();
  gui_path.poses = global_plan;
  global_plan_pub_.publish(gui_path);
}

} // namespace dwa

