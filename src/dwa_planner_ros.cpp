#include <chrono>
#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "dwa_planner_ros/dwa_planner_ros.h"
#include <sensor_msgs/LaserScan.h>

// register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(dwa_planner_ros::DWAPlannerROS, nav_core::BaseLocalPlanner)

namespace dwa_planner_ros {

DWAPlannerROS::DWAPlannerROS()
  : initialized_(false), size_x_(0), size_y_(0), goal_reached_(false)
{
ros::NodeHandle nh;
laser_sub_ = nh.subscribe("/scan", 1, &DWAPlannerROS::laserCallback, this);
}

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
  if(charmap_ != NULL){
     freeMemory();
   }

  charmap_ = new unsigned char*[size_x_];
  for (int i = 0; i < size_x_; i++)
    charmap_[i] = new unsigned char[size_y_];
}

void DWAPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{

  // check if the plugin is already initialized
  if (!initialized_)
  { 
    ros::NodeHandle private_nh("~/" + name);
    private_nh.param("odom_topic", odom_topic_, std::string("/odometry/filtered"));
    private_nh.param("map_frame", map_frame_, std::string("map"));
    private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.2);
    private_nh.param("transform_tolerance", transform_tolerance_, 0.5);
    private_nh.param("max_vel_x", max_vel_x_, 0.55);
    private_nh.param("min_vel_x", min_vel_x_, 0.0);
    private_nh.param("max_vel_theta", max_vel_theta_, 2.5);
    private_nh.param("min_vel_theta", min_vel_theta_, -2.5);
    private_nh.param("acc_lim_x", acc_lim_x_, 0.25);
    private_nh.param("acc_lim_theta", acc_lim_theta_, 1.2);
    private_nh.param("control_period", control_period_, 0.2);

    // init other variables
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();  
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
                              private_nh, costmap_ros_);

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


void DWAPlannerROS::laserCallback(const sensor_msgs::LaserScan& scan)
{
    dynamic_obstacle_detected = false;
    bool front_obstacle_detected = false;
    bool back_obstacle_detected = false;

    std::vector<float> front_x(scan.ranges.begin() + 80, scan.ranges.begin() + 280); // 80~280
    std::vector<float> back_x_1(scan.ranges.begin(), scan.ranges.begin() + 80); // 0~80
    std::vector<float> back_x_2(scan.ranges.begin() + 280, scan.ranges.begin() + 360);  // 280~360

    std::vector<float> combined_ranges;
    combined_ranges.insert(combined_ranges.end(), back_x_1.begin(), back_x_1.end());
    combined_ranges.insert(combined_ranges.end(), back_x_2.begin(), back_x_2.end());

    costmap_ros_->getRobotPose(current_pose_);
    double robot_x = current_pose_.pose.position.x;
    double robot_y = current_pose_.pose.position.y;
    double robot_theta = tf2::getYaw(current_pose_.pose.orientation);

    for(int i = 0; i < front_x.size(); ++i){
     if (front_x[i] <= (scan.range_min + 0.05)) //Obstacle within front: 15 cm
      {
        float scan_angle = scan.angle_min + i * scan.angle_increment;
        front_obstacle_detected = checkObstacle(front_x[i], robot_x, robot_y, robot_theta, scan_angle);
      }
    }

    for(int j = 0; j < combined_ranges.size(); ++j){
     if(combined_ranges[j] <= (scan.range_min + 0.4)) //Obstacle within back: 50cm
      {
        float scan_angle = scan.angle_min + j * scan.angle_increment;
        back_obstacle_detected = checkObstacle(combined_ranges[j], robot_x, robot_y, robot_theta, scan_angle);
      }
    }
  
  if(front_obstacle_detected == true || back_obstacle_detected == true){
    dynamic_obstacle_detected = true;
  }
}

bool DWAPlannerROS::checkObstacle(const double range, const double robot_x, const double robot_y, const double robot_theta, const double scan_angle)
{
    double obstacle_x = robot_x + range * cos(robot_theta + scan_angle);
    double obstacle_y = robot_y + range * sin(robot_theta + scan_angle);
    unsigned int obstacle_mx, obstacle_my;

    if (!costmap_->worldToMap(obstacle_x, obstacle_y, obstacle_mx, obstacle_my)) {
        ROS_WARN("invalid map frame");
        return false;
    }

    unsigned char cost = costmap_->getCost(obstacle_mx, obstacle_my);
    if(cost == costmap_2d::LETHAL_OBSTACLE){
      return false;
    }
    else{
      return true;
    }
}

bool DWAPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_) {
    ROS_ERROR("Pure Pursuit Planner has not been initialized.");
    return false;
  }

  goal_reached_ = false;
  rotate = true;

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

  // Get the current robot pose
  costmap_ros_->getRobotPose(current_pose_);
  double robot_pose_x = current_pose_.pose.position.x;
  double robot_pose_y = current_pose_.pose.position.y; 
  double robot_pose_theta = tf2::getYaw(current_pose_.pose.orientation);

  // Get the current robot velocity
  geometry_msgs::PoseStamped robot_vel_tf;
  odom_helper_.getRobotVel(robot_vel_tf);
  double robot_vel_x = robot_vel_tf.pose.position.x;
  double robot_vel_theta = tf2::getYaw(robot_vel_tf.pose.orientation);

  // Get the transformed global plan
  std::vector<geometry_msgs::PoseStamped> transformed_plan;
  transformed_plan.clear();
  if (!planner_util_.getLocalPlan(current_pose_, transformed_plan)) {
    ROS_ERROR("Could not get local plan");
    return false;
  }

  // If the plan is empty, return false
  if (transformed_plan.empty()) {
    ROS_WARN("Transformed plan is empty");
    return false;
  }

  // Get the first point of the global plan as the initial goal
  geometry_msgs::PoseStamped goal_pose = transformed_plan.back();
  double yaw = getYaw(current_pose_);

  // Calculate the goal direction from the robot's current pose to the goal pose
  double target_yaw = atan2(goal_pose.pose.position.y - robot_pose_y, goal_pose.pose.position.x - robot_pose_x);
  double yaw_error = angles::shortest_angular_distance(yaw,target_yaw);
  
  // If the yaw error is significant, perform a rotational correction
  if (fabs(yaw_error) > 0.1 && rotate == true) {  // Threshold to decide when to rotate in place (0.1 rad)
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.5;  // Rotate proportionally to the yaw error
  }
  rotate = false;
  // Once the robot is oriented correctly, proceed with normal DWA planning
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
                                                   resolution, origin_x, origin_y, dwa_cmd_vel_x, dwa_cmd_vel_theta, transformed_plan);

  if (!success)
  {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    return false;
  }
  else
  {
    if(dynamic_obstacle_detected){
      cmd_vel.linear.x = 0;
      cmd_vel.angular.z = 0;
    }else{
      cmd_vel.linear.x = dwa_cmd_vel_x;
      cmd_vel.angular.z = dwa_cmd_vel_theta;
    

    geometry_msgs::PoseStamped robot_pose;
    costmap_ros_->getRobotPose(robot_pose);
    geometry_msgs::PoseStamped global_plan_ = transformed_plan.back();

    double dx = robot_pose.pose.position.x - global_plan_.pose.position.x;
    double dy = robot_pose.pose.position.y - global_plan_.pose.position.y;

    if (hypot(dx, dy) <= xy_goal_tolerance_) {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
      goal_reached_ = true;
      ROS_INFO("Goal reached.");
    }
      return true;
   }
  }
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

double DWAPlannerROS::getYaw(const geometry_msgs::PoseStamped& pose) {
  tf2::Quaternion q;
  tf2::fromMsg(pose.pose.orientation, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
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
