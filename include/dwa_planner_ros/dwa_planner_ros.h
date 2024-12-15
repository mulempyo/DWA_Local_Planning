#ifndef DWA_PLANNER_ROS_H
#define DWA_PLANNER_ROS_H

#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/local_planner_util.h>
#include "dwa_planner_ros/dwa_planner.h"
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

namespace dwa_planner_ros{

class DWAPlannerROS: public nav_core::BaseLocalPlanner {
public:

  DWAPlannerROS();
  ~DWAPlannerROS();

   typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
   std::shared_ptr<MoveBaseClient> ac;

  /**
   * @brief Initializes the DWAPlannerROS.
   *
   * @param name The name of the planner.
   * @param tf A pointer to the tf2_ros buffer.
   * @param costmap_ros A pointer to the costmap_2d::Costmap2DROS.
   */
  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
  void safeMode(std_msgs::Float64 safe);
  void personDetect(const std_msgs::Float64::ConstPtr& person);
  void laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan);
  void laserCallback(const sensor_msgs::LaserScan& scan);
 
  bool checkObstacle(const double range, const double robot_x, const double robot_y, const double robot_theta, const double scan_angle);

  /**
   * @brief Set the global plan for the local planner.
   *
   * @param orig_global_plan The global plan from the global planner.
   * @return True if successful, false otherwise.
   */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  /**
   * @brief Compute velocity commands for the robot to follow the global plan.
   *
   * @param cmd_vel The output velocity command for the robot.
   * @return True if a valid command is found, false otherwise.
   */
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  /**
   * @brief Check if the robot has reached its goal.
   *
   * @return True if the goal is reached, false otherwise.
   */
  bool isGoalReached();

  std::vector<geometry_msgs::PoseStamped> global_plan_;
  ros::Subscriber laser_sub_;
  ros::Subscriber sub_;
  ros::Subscriber person_sub_;
  ros::Publisher safe_pub_;
  ros::Subscriber amcl_sub_;
  

private:
  /**
   * @brief Allocates memory for the costmap.
   */
  void allocateMemory();

  /**
   * @brief Frees the memory allocated for the costmap.
   */
  void freeMemory();

  /**
   * @brief Publishes the global plan for visualization.
   *
   * @param global_plan The global plan to publish.
   */
  void publishGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan);

  double getYaw(const geometry_msgs::PoseStamped& pose);

  bool initialized_;            ///< Whether the planner is initialized or not.
  bool rotate;
  bool goal_reached_;           ///< Whether the goal is reached or not.
  bool dynamic_obstacle_detected;
  bool person_detect;
  bool safe_mode;
  int size_x_;                  ///< Size of the costmap in the x direction.
  int size_y_;                  ///< Size of the costmap in the y direction.
  int size_x;
  int size_y;
  int stack;
  unsigned char** charmap_ = nullptr;      ///< The costmap data.
  costmap_2d::Costmap2DROS* costmap_ros_;
  costmap_2d::Costmap2DROS* update_costmap_ros_;  
  costmap_2d::Costmap2D* costmap_; ///< Pointer to the costmap.
  costmap_2d::Costmap2D* costmap;
  base_local_planner::CostmapModel* costmap_model_ = nullptr;  ///< Costmap model used for collision checking.
  DWAPlanner* planner_ = nullptr;         ///< The DWA planner instance.
  ros::Publisher global_plan_pub_;  ///< Publisher for the global plan.
  tf2_ros::Buffer* tf_;
  // Parameters   
  std::string odom_topic_;
  std::string map_frame_;       ///< Map frame name.
  std::string global_frame_;
  std::string robot_base_frame_;
  double xy_goal_tolerance_;    ///< Goal tolerance in the x and y directions.
  double transform_tolerance_;  ///< Tolerance for transforming coordinates.
  double max_vel_x_;            ///< Maximum linear velocity.
  double min_vel_x_;            ///< Minimum linear velocity.
  double max_vel_theta_;        ///< Maximum angular velocity.
  double min_vel_theta_;        ///< Minimum angular velocity.
  double acc_lim_x_;            ///< Maximum linear acceleration.
  double acc_lim_theta_;        ///< Maximum angular acceleration.
  double control_period_;       ///< Control loop period.

  double resolution;            ///< Resolution of the costmap.
  double origin_x;              ///< Origin x coordinate of the costmap.
  double origin_y;              ///< Origin y coordinate of the costmap.
  std::vector<geometry_msgs::Point> footprint_spec_;  ///< Footprint of the robot.
  double robot_inscribed_radius_;  ///< The inscribed radius of the robot.
  double robot_circumscribed_radius_;  ///< The circumscribed radius of the robot.

  std::array<float,7> safe1;
  std::array<float,7> safe2;
  
  geometry_msgs::PoseStamped robot_safe1,robot_safe2;
  geometry_msgs::PoseStamped current_pose_;  ///< The current pose of the robot.

  base_local_planner::OdometryHelperRos odom_helper_;  ///< Helper to get odometry data.
  base_local_planner::LocalPlannerUtil planner_util_;       ///< Utility to assist with planning.

};

}  // namespace dwa

#endif  // DWA_PLANNER_ROS_H
