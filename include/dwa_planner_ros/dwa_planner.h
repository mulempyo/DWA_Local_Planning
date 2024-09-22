#ifndef DWA_PLANNER_H
#define DWA_PLANNER_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <vector>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/costmap_model.h>

namespace dwa_planner_ros{

class DWAPlanner {
public:
  DWAPlanner(base_local_planner::CostmapModel* costmap_model,
             const std::vector<geometry_msgs::Point>& footprint_spec, double inscribed_radius,
             double circumscribed_radius, ros::NodeHandle& nh);

  ~DWAPlanner();

  /**
   * @brief Compute velocity commands for the robot based on the current robot state and global plan.
   *
   * @param robot_vel_x Current linear velocity of the robot.
   * @param robot_vel_theta Current angular velocity of the robot.
   * @param robot_pose_x Current x position of the robot.
   * @param robot_pose_y Current y position of the robot.
   * @param robot_pose_theta Current orientation of the robot.
   * @param global_plan The global plan provided by the planner.
   * @param costmap The local costmap.
   * @param size_x Size of the costmap in x dimension.
   * @param size_y Size of the costmap in y dimension.
   * @param resolution Resolution of the costmap.
   * @param origin_x Origin x coordinate of the costmap.
   * @param origin_y Origin y coordinate of the costmap.
   * @param cmd_vel_x Output computed linear velocity.
   * @param cmd_vel_theta Output computed angular velocity.
   *
   * @return True if a valid command is computed, false otherwise.
   */
  bool computeVelocityCommands(const double& robot_vel_x, const double& robot_vel_theta,
                               const double& robot_pose_x, const double& robot_pose_y, const double& robot_pose_theta,
                               const std::vector<std::vector<double>>& global_plan, unsigned char const* const* costmap,
                               int size_x, int size_y, double resolution, double origin_x, double origin_y,
                               double& cmd_vel_x, double& cmd_vel_theta);

  std::vector<std::pair<int, int>> bresenhamLine(int x0, int y0, int x1, int y1);
  bool obstacleFound(bool found);
  bool obstacleDetected() const;                             

private:
  /**
   * @brief Cuts the global plan to fit within the local costmap.
   */
  std::vector<std::vector<double>> cutGlobalPlan(const std::vector<std::vector<double>>& global_plan,
                                                 const int& size_x, const int& size_y, const double& robot_pose_x, const double& robot_pose_y);

  /**
   * @brief Scores a trajectory based on its distance to obstacles and the global plan.
   */
  double scoreTrajectory(const std::vector<std::vector<double>>& traj, const int& size_x, const int& size_y,
                         const double& resolution, const double& origin_x, const double& origin_y,
                         const std::vector<std::vector<double>>& global_plan, unsigned char const* const* costmap);

  /**
   * @brief Generates a trajectory for the robot.
   */
  void generateTrajectory(const double& robot_vel_x, const double& robot_vel_theta,
                                    const double& robot_pose_x, const double& robot_pose_y, const double& robot_pose_theta,
                                    const double& sample_vel_x, const double& sample_vel_theta, std::vector<std::vector<double>>& traj); 
                                    

  void worldToMap(const double wx, const double wy, int& mx, int& my, const double resolution, const double origin_x, const double origin_y);

  /**
   * @brief Computes the robot's new pose based on its velocity and current pose.
   */
  void computeNewPose(double& pose_x, double& pose_y, double& pose_theta, const double& vel_x, const double& vel_theta);

  /**
   * @brief Computes the new velocity for the robot considering acceleration limits.
   */
  double computeNewLinearVelocities(const double& target_vel, const double& current_vel, const double& acc_lim);

  double computeNewAngularVelocities(const double& target_vel, const double& current_vel, const double& acc_lim);
                                        
  /**
   * @brief Samples potential velocities for the robot to explore.
   */
  bool samplePotentialVels(const double& robot_vel_x, const double& robot_vel_theta, std::vector<std::pair<double, double>>& sample_vels);

  /**
   * @brief Checks if a given trajectory is feasible by ensuring the robot does not collide with obstacles.
   */
  bool isPathFeasible(const std::vector<std::vector<double>>& path);

  /**
   * @brief Publishes candidate paths for visualization.
   */
  void publishCandidatePaths(const std::vector<std::vector<std::vector<double>>>& path_all);
  

  // Parameters
  double max_vel_x_;
  double min_vel_x_;
  double max_vel_theta_;
  double min_vel_theta_;
  double acc_lim_x_;
  double acc_lim_theta_;
  double control_period_;

  int sim_time_samples_;
  int vx_samples_;
  int vth_samples_;
  double path_distance_bias_;
  double goal_distance_bias_;
  double occdist_scale_;
  bool obstacle_found;
  std::string map_frame_;

  base_local_planner::CostmapModel* costmap_model_ = nullptr;
  std::vector<geometry_msgs::Point> footprint_spec_;
  double inscribed_radius_;
  double circumscribed_radius_;

  // ROS
  ros::Publisher candidate_paths_pub_;
};

}  // namespace dwa_planner

#endif  // DWA_PLANNER_H
