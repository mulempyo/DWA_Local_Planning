#include "dwa_planner_ros/dwa_planner.h"
#include <Eigen/Core>
#include <cmath>

namespace dwa_planner_ros {

DWAPlanner::DWAPlanner(base_local_planner::CostmapModel* costmap_model,
                       const std::vector<geometry_msgs::Point>& footprint_spec, double inscribed_radius,
                       double circumscribed_radius, ros::NodeHandle& nh)
  : costmap_model_(costmap_model)
  , footprint_spec_(footprint_spec)
  , inscribed_radius_(inscribed_radius)
  , circumscribed_radius_(circumscribed_radius)
{
  candidate_paths_pub_ = nh.advertise<nav_msgs::Path>("dwa_candidate_paths", 1);
  
  map_frame_ = "map";
  max_vel_x_ = 0.55;
  min_vel_x_ = 0.0;
  max_vel_theta_ = 2.5;
  min_vel_theta_ = -2.5;
  acc_lim_x_ = 0.25;
  acc_lim_theta_ = 1.2;
  control_period_ = 0.2;

  sim_time_samples_ = 10;
  vx_samples_ = 10;
  vth_samples_ = 20;
  path_distance_bias_ = 32.0;
  goal_distance_bias_ = 40.0;
  occdist_scale_ = 0.01;
}

DWAPlanner::~DWAPlanner() {}

bool DWAPlanner::computeVelocityCommands(const double& robot_vel_x, const double& robot_vel_theta,
                                         const double& robot_pose_x, const double& robot_pose_y, const double& robot_pose_theta,
                                         const std::vector<std::vector<double>>& global_plan, unsigned char const* const* costmap,
                                         int size_x, int size_y, double resolution, double origin_x, double origin_y,
                                         double& cmd_vel_x, double& cmd_vel_theta)
{
  std::vector<std::pair<double, double>> sample_vels;
  if (!samplePotentialVels(robot_vel_x, robot_vel_theta, sample_vels)){
    return false;
  }

  std::vector<std::vector<double>> pruned_global_plan = cutGlobalPlan(global_plan, size_x, size_y, robot_pose_x, robot_pose_y);
  std::vector<std::pair<double, double>>::iterator it = sample_vels.begin();
  std::vector<std::vector<std::vector<double>>> path_all;
  double min_score = 1e10;

  while (it != sample_vels.end()) {
    std::vector<std::vector<double>> traj;
    generateTrajectory(robot_vel_x, robot_vel_theta, robot_pose_x, robot_pose_y, robot_pose_theta, it->first, it->second, traj);

    if (!isPathFeasible(traj)) {
      ++it;
      continue;
    }

    double score = scoreTrajectory(traj, size_x, size_y, resolution, origin_x, origin_y, pruned_global_plan, costmap);
    if (score < min_score) {
      min_score = score;
      cmd_vel_x = it->first;
      cmd_vel_theta = it->second;
    }

    path_all.push_back(traj);
    ++it;
  }

  publishCandidatePaths(path_all);
  return true;
}

std::vector<std::vector<double>> DWAPlanner::cutGlobalPlan(const std::vector<std::vector<double>>& global_plan,
                                                           const int& size_x, const int& size_y, const double& robot_pose_x, const double& robot_pose_y)
{
  std::vector<std::vector<double>> pruned_global_plan;
  for (auto it = global_plan.begin(); it != global_plan.end(); ++it) {
    if (std::abs(it->at(0) - robot_pose_x) > size_x / 2.0 && std::abs(it->at(1) - robot_pose_y) > size_y / 2.0) {
      break;
    }
    pruned_global_plan.push_back(*it);
  }
  return pruned_global_plan;
}

double DWAPlanner::scoreTrajectory(const std::vector<std::vector<double>>& traj, const int& size_x, const int& size_y,
                                   const double& resolution, const double& origin_x, const double& origin_y,
                                   const std::vector<std::vector<double>>& global_plan, unsigned char const* const* costmap)
{
  int mx, my;
  int occupy = 0;
  for (auto it = traj.begin(); it != traj.end(); ++it) {
    worldToMap(it->at(0), it->at(1), mx, my, resolution, origin_x, origin_y);
    occupy = std::max(costmap[mx][my] - 0, occupy);
  }

  const std::vector<double>& end_pose = traj.back();
  const std::vector<double>& local_end_pose = global_plan.back();
  double dis2end = std::sqrt(std::pow(end_pose[0] - local_end_pose[0], 2) + std::pow(end_pose[1] - local_end_pose[1], 2));

  double dis2path = 1e10;
  for (auto it = global_plan.begin(); it != global_plan.end(); ++it) {
    dis2path = std::min(std::sqrt(std::pow(end_pose[0] - it->at(0), 2) + std::pow(end_pose[1] - it->at(1), 2)), dis2path);
  }

  return occdist_scale_ * occupy + goal_distance_bias_ * dis2end + path_distance_bias_ * dis2path;
}

void DWAPlanner::generateTrajectory(const double& robot_vel_x, const double& robot_vel_theta,
                                    const double& robot_pose_x, const double& robot_pose_y, const double& robot_pose_theta,
                                    const double& sample_vel_x, const double& sample_vel_theta, std::vector<std::vector<double>>& traj)
{
  double pose_x = robot_pose_x;
  double pose_y = robot_pose_y;
  double pose_theta = robot_pose_theta;
  double vel_x = robot_vel_x;
  double vel_theta = robot_vel_theta;

  for (int i = 0; i < sim_time_samples_; ++i) {
    vel_x = computeNewVelocities(sample_vel_x, vel_x, acc_lim_x_);
    vel_theta = computeNewVelocities(sample_vel_theta, vel_theta, acc_lim_theta_);
    computeNewPose(pose_x, pose_y, pose_theta, vel_x, vel_theta);
    traj.push_back({pose_x, pose_y, pose_theta});
  }
}

void DWAPlanner::worldToMap(const double wx, const double wy, int& mx, int& my, const double resolution, const double origin_x, const double origin_y){
  mx = int((wx-origin_x)/resolution);
  my = int((wy-origin_y)/resolution);
}

void DWAPlanner::computeNewPose(double& pose_x, double& pose_y, double& pose_theta,
                                const double& vel_x, const double& vel_theta)
{
  pose_x += vel_x * std::cos(pose_theta) * control_period_;
  pose_y += vel_x * std::sin(pose_theta) * control_period_;
  pose_theta += vel_theta * control_period_;
}

double DWAPlanner::computeNewVelocities(const double& target_vel, const double& current_vel, const double& acc_lim)
{
  if (target_vel < current_vel) {
    return std::max(target_vel, current_vel - acc_lim * control_period_);
  } else {
    return std::min(target_vel, current_vel + acc_lim * control_period_);
  }
}

bool DWAPlanner::samplePotentialVels(const double& robot_vel_x, const double& robot_vel_theta,
                                     std::vector<std::pair<double, double>>& sample_vels)
{
  double min_vel_x = std::max(min_vel_x_, robot_vel_x - acc_lim_x_ * control_period_);
  double max_vel_x = std::min(max_vel_x_, robot_vel_x + acc_lim_x_ * control_period_);
  double min_vel_theta = std::max(min_vel_theta_, robot_vel_theta - acc_lim_theta_ * control_period_);
  double max_vel_theta = std::min(max_vel_theta_, robot_vel_theta + acc_lim_theta_ * control_period_);

  for (double v = min_vel_x; v <= max_vel_x; v += (max_vel_x - min_vel_x) / vx_samples_) {
    for (double w = min_vel_theta; w <= max_vel_theta; w += (max_vel_theta - min_vel_theta) / vth_samples_) {
      sample_vels.push_back(std::make_pair(v, w));
    }
  }

  return !sample_vels.empty();
}

bool DWAPlanner::isPathFeasible(const std::vector<std::vector<double>>& path)
{
  int look_ahead_idx = path.size() - 1;

  for (int i = 0; i <= look_ahead_idx; ++i) {
    if (costmap_model_->footprintCost(path[i][0], path[i][1], path[i][2], footprint_spec_, inscribed_radius_, circumscribed_radius_) == -1) {
      return false;
    }
  }

  return true;
}

void DWAPlanner::publishCandidatePaths(const std::vector<std::vector<std::vector<double>>>& candidate_paths)
{
  nav_msgs::Path gui_path;
  gui_path.header.frame_id = map_frame_;
  gui_path.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped pose;

  for (int i = 0; i < (int)candidate_paths.size(); i++)
  {
    for (int j = 0; j < (int)candidate_paths[i].size(); j++)
    {
      double x = candidate_paths[i][j][0];  // x position
      double y = candidate_paths[i][j][1];  // y position
      double theta = candidate_paths[i][j][2];  // orientation (theta)
      
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
      gui_path.poses.push_back(pose);
    }

    for (int j = (int)candidate_paths[i].size() - 1; j >= 0; j--)
    {
      double x = candidate_paths[i][j][0];  // x position
      double y = candidate_paths[i][j][1];  // y position
      double theta = candidate_paths[i][j][2];  // orientation (theta)

      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
      gui_path.poses.push_back(pose);
    }
  }

  candidate_paths_pub_.publish(gui_path);
}
}
