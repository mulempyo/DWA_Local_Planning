#include "dwa_planner/dwa_planner.h"
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <angles/angles.h>
#include <costmap_2d/costmap_2d.h>
#include "sensor_msgs/LaserScan.h"

PLUGINLIB_EXPORT_CLASS(dwa::DwaPlanner, nav_core::BaseLocalPlanner)

namespace dwa {

DwaPlanner::DwaPlanner()
  : is_initialized_(false), goal_reached_(false) {}

DwaPlanner::DwaPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
  : is_initialized_(false), goal_reached_(false) {
  initialize(name, tf, costmap_ros);
}

DwaPlanner::~DwaPlanner() {}

void DwaPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) {
  if (!is_initialized_) {
    
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    origin_x = costmap_->getOriginX();
    origin_y = costmap_->getOriginY();
    resolution = costmap_->getResolution();
    costmap_ros_->getRobotPose(current_pose_);
    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    planner_util_.initialize(tf_, costmap, costmap_ros_->getGlobalFrameID());

    ros::NodeHandle private_nh("~/" + name);
    laser_sub_ = private_nh.subscribe("scan", 1, &DwaPlanner::laserCallback, this);
    if (!private_nh.getParam("/move_base/global_costmap/inflation_layer/inflation_radius", inflation_radius_)) {
      ROS_WARN("Could not get inflation_radius from costmap_common_params, setting default 0.15");
      inflation_radius_ = 0.15; 
    }
    private_nh.param("lookahead_distance", distance_step_, 0.3);
    private_nh.param("max_velocity", max_velocity_, 0.5);  
    private_nh.param("min_velocity", min_velocity_, 0.01);  
    private_nh.param("max_angular_velocity", max_angular_velocity_, 1.5);  
    private_nh.param("min_angular_velocity", min_angular_velocity_, 0.01);  
    private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.1);
    private_nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.1);
    private_nh.param("sim_time", sim_time_, 1.0);  
    private_nh.param("time_step", time_step_, 0.1);  

    is_initialized_ = true;
    ROS_INFO("Dwa Planner initialized.");
  } else {
    ROS_WARN("Dwa Planner already initialized.");
  }
}

void DwaPlanner::laserCallback(const sensor_msgs::LaserScan& scan){
  scan_ = scan;
  bool dynamic_obstacle_detected = false;

  if(scan_.header.seq == 0){
    ROS_WARN("invalid Lidar data");
    return;
  }

  double left_angle = M_PI/2; //left 90
  double right_angle = -M_PI/2; //right 90
  
  double angle_increment = (scan_.angle_max - scan_.angle_min) / (scan_.ranges.size() - 1);
  int start_index = (right_angle - scan_.angle_min)/angle_increment;
  int end_index = (left_angle - scan_.angle_min)/angle_increment;
  
  for (int i = start_index; i < end_index; ++i) {
    double distance = scan_.ranges[i];

    if (std::isfinite(distance) && distance < 0.15) {  
      dynamic_obstacle_detected = true;
      break;
    }
  }
  dynamic_obstacle_ = dynamic_obstacle_detected;
}

bool DwaPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
  if (!is_initialized_) {
    ROS_ERROR("Dwa Planner has not been initialized.");
    return false;
  }
  goal_reached_ = false;
  return planner_util_.setPlan(plan);
}

double DwaPlanner::evaluateTrajectory(double v, double w, const geometry_msgs::PoseStamped& current_pose) {
  double goal_score = 0.0;
  double obstacle_score = 0.0;
  double velocity_score = 0.0;

  geometry_msgs::PoseStamped lookahead_point = updatePlan(current_pose, global_plan_);
  double distance_to_goal = getDistance(current_pose, lookahead_point);

  goal_score = 1.0 / (distance_to_goal + 0.1);

  for (size_t i = 0; i < scan_.ranges.size(); ++i) {
    if (scan_.ranges[i] < 0.4) {  
      obstacle_score += 1.0 / (scan_.ranges[i] + 0.1);
    }
  }

  velocity_score = v;

  unsigned int mx, my;
  if(!costmap_->worldToMap(current_pose.pose.position.x, current_pose.pose.position.y, mx, my)){
     ROS_WARN("current_pose doesn`t locate in map_boundary");
  }

  int inflation_cells = inflation_radius_ / resolution; 
  double max_cost = 0.0;  

  for (int i = -inflation_cells; i <= inflation_cells; ++i) {
      for (int j = -inflation_cells; j <= inflation_cells; ++j) {
          unsigned int check_mx = mx + i;
          unsigned int check_my = my + j;
          
          if (isInBounds(check_mx, check_my)) {
              double cost = static_cast<double>(costmap_->getCost(check_mx, check_my));
              if (cost > max_cost) {
                  max_cost = cost;
              }
          }
      }
  }

  if (max_cost > costmap_2d::INSCRIBED_INFLATED_OBSTACLE && max_cost != costmap_2d::NO_INFORMATION) {
      costmap_score = -1.0 * max_cost / 255.0;
  } else if (max_cost == costmap_2d::LETHAL_OBSTACLE) {
      costmap_score = -1.0;
  } else {
      costmap_score = 1.0 - max_cost / 255.0;
  }

 return goal_score * 0.4 - obstacle_score * 0.3 + velocity_score * 0.2 + costmap_score * 0.1;
}

bool DwaPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
  if (!is_initialized_) {
    ROS_ERROR("Dwa Planner has not been initialized.");
    return false;
  }

  if (!costmap_ros_->getRobotPose(current_pose_)) {
    ROS_ERROR("Could not get robot pose.");
    return false;
  }

  std::vector<geometry_msgs::PoseStamped> transformed_plan;
  if (!planner_util_.getLocalPlan(current_pose_, transformed_plan)) {
    ROS_ERROR("Could not get local plan");
    return false;
  }

  if (transformed_plan.empty()) {
    ROS_WARN("Received an empty transformed_plan.");
    return false;
  }

  global_plan_.resize(transformed_plan.size());
  for (size_t i = 0; i < transformed_plan.size(); ++i) {
    global_plan_[i] = transformed_plan[i];
  }

  geometry_msgs::PoseStamped lookahead_point = updatePlan(current_pose_, global_plan_);
  double yaw_error = angles::shortest_angular_distance(getYaw(current_pose_), getYaw(lookahead_point));

  if (fabs(yaw_error) > 0.1) {  
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = (yaw_error < 0) ? -min_angular_velocity_ : min_angular_velocity_;
  }else{
/*
  if (dynamic_obstacle_) {
    ROS_WARN("Dynamic obstacle detected, stopping...");
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0; 
  }else{*/
  double best_score = -1e9;
  double best_linear_vel = 0.0;
  double best_angular_vel = 0.0;

  for (double v = min_velocity_; v <= max_velocity_; v += 0.1) {
    for (double w = -max_angular_velocity_; w <= max_angular_velocity_; w += 0.01) {
      double score = evaluateTrajectory(v, w, current_pose_);
      if (score > best_score) {
        best_score = score;
        best_linear_vel = v;
        best_angular_vel = w;
      }
    }
  }
 
    unsigned int mx, my;
    costmap_->worldToMap(current_pose_.pose.position.x, current_pose_.pose.position.y, mx, my);
    int inflation_cells = inflation_radius_ / resolution; 
    double max_cost = 0.0;

    for (int i = -inflation_cells; i <= inflation_cells; ++i) {
      for (int j = -inflation_cells; j <= inflation_cells; ++j) {
        unsigned int check_mx = mx + i;
        unsigned int check_my = my + j;
        
        if (isInBounds(check_mx, check_my)) {
          double cost = static_cast<double>(costmap_->getCost(check_mx, check_my));
          if (cost > max_cost) {
            max_cost = cost;
          }
        }
      }
    }

    if (max_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE && max_cost != costmap_2d::NO_INFORMATION) {
      best_linear_vel *= 0.5;  
    } else if (max_cost == costmap_2d::LETHAL_OBSTACLE) {
      best_linear_vel = 0.0;  
    }

  cmd_vel.linear.x = best_linear_vel;
  cmd_vel.angular.z = best_angular_vel;
   }
  //}
  if (getDistance(current_pose_, global_plan_.back()) < xy_goal_tolerance_ &&
      fabs(angles::shortest_angular_distance(getYaw(current_pose_), getYaw(global_plan_.back()))) < yaw_goal_tolerance_) {
    goal_reached_ = true;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    ROS_INFO("Goal reached.");
    return true;
  }

  return true;
}

bool DwaPlanner::isGoalReached() {
  return goal_reached_;
}

geometry_msgs::PoseStamped DwaPlanner::updatePlan(const geometry_msgs::PoseStamped& current_pose_, std::vector<geometry_msgs::PoseStamped>& global_plan_) {

    geometry_msgs::PoseStamped final_goal = global_plan_.back();
    double angle_to_goal = atan2(final_goal.pose.position.y - current_pose_.pose.position.y, 
                                         final_goal.pose.position.x - current_pose_.pose.position.x);

    geometry_msgs::PoseStamped adjusted_point;
    adjusted_point.pose.position.x = current_pose_.pose.position.x + distance_step_ * cos(angle_to_goal);
    adjusted_point.pose.position.y = current_pose_.pose.position.y + distance_step_ * sin(angle_to_goal);

    if (getDistance(current_pose_, final_goal) <= getDistance(current_pose_, adjusted_point)) {
      return final_goal;
    }else{
      return adjusted_point;
    }
}

bool DwaPlanner::isInBounds(unsigned int mx, unsigned int my){
    if(mx >= 0 && mx < costmap_->getSizeInCellsX() && my >= 0 && my < costmap_->getSizeInCellsY()){
       return true;
    }
    else{
       ROS_WARN("check_mx, check_my don`t exist in map boundary");
       return false;
    }
}

double DwaPlanner::getDistance(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2) {
  return hypot(pose2.pose.position.x - pose1.pose.position.x, pose2.pose.position.y - pose1.pose.position.y);
}

double DwaPlanner::getYaw(const geometry_msgs::PoseStamped& pose) {
  tf2::Quaternion q;
  tf2::fromMsg(pose.pose.orientation, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

}  // namespace dwa

