#ifndef VFH_LOCAL_PLANNER_VFH_PLANNER_ROS_H_
#define VFH_LOCAL_PLANNER_VFH_PLANNER_ROS_H_

#include <vfh_local_planner/vfh_algorithm.h>
#include <nav_core/base_local_planner.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <string>


#define DEG2RAD(a) ((a) * M_PI / 180.0)
#define RAD2DEG(a) ((a) * 180.0 /  M_PI)

std::string scan_topic_;
std::string odom_topic_;

namespace vfh_local_planner {

  class VFHPlannerROS : public nav_core::BaseLocalPlanner {
    public:
     
       /**
       * @brief  Constructor for VFHPlannerROS wrapper
       */
	VFHPlannerROS() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}


      /**
       * @brief  Constructs the ros wrapper
       * @param name The name to give this instance of the trajectory planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */

       void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);

     /**
       * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid trajectory was found, false otherwise
       */
     bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

      /**
       * @brief  Check if the goal pose has been achieved by the local planner
       * @return True if achieved, false otherwise
       */
      bool isGoalReached();

      /**
       * @brief  Set the plan that the local planner is following
       * @param orig_global_plan The plan to pass to the local planner
       * @return True if the plan was updated successfully, false otherwise
       */
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);



     private:

	VFH_Algorithm *m_vfh;

	double m_cell_size;			// 100 mm
	int m_window_diameter;		// cells
	int m_sector_angle;			// in deg
	double m_safety_dist_0ms;
	double m_safety_dist_1ms;
	int m_max_speed;
	int m_max_speed_narrow_opening;
	int m_max_speed_wide_opening;
	int m_max_acceleration;
	int m_min_turnrate;
	int m_max_turnrate_0ms;
	int m_max_turnrate_1ms;
	double m_min_turn_radius_safety_factor;
	double m_free_space_cutoff_0ms;
	double m_obs_cutoff_0ms;
	double m_free_space_cutoff_1ms;
	double m_obs_cutoff_1ms;
	double m_weight_desired_dir;
	double m_weight_current_dir;

	double m_robot_radius;
	double m_robotVel;
    	double m_laser_ranges[361][2];

	int chosen_speed,chosen_turnrate;

	// ros
	ros::NodeHandle nh_;

	ros::Subscriber scan_subscriber_;
	ros::Subscriber odom_subscriber_;

	void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg);
	void odomCallback (const nav_msgs::Odometry::ConstPtr& odom_msg);

      costmap_2d::Costmap2DROS* costmap_ros_;
      tf::TransformListener* tf_;
      double max_vel_th_, min_vel_th_, min_rot_vel_;
      double rot_stopped_vel_, trans_stopped_vel_;
      double yaw_goal_tolerance_, xy_goal_tolerance_;
      bool prune_plan_;
      bool initialized_;
      std::string global_frame_;
      std::string robot_base_frame_;
      double inflation_radius_; 
      ros::Publisher g_plan_pub_, l_plan_pub_;
      boost::mutex odom_mutex_;
      nav_msgs::Odometry base_odom_;
      boost::shared_ptr<VFHPlannerROS> dp_;
      std::vector<geometry_msgs::PoseStamped> global_plan_;
      bool rotating_to_goal_;
      bool latch_xy_goal_tolerance_, xy_tolerance_latch_;
};

};
#endif
