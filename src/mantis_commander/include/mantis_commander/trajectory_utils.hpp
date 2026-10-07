#ifndef TRAJECTORY_UTILS_HPP
#define TRAJECTORY_UTILS_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <map>
#include <string>

class TrajectoryUtils {
public:
    // Cosntructor with move group shared ptr to work on real planner
    TrajectoryUtils(std::shared_ptr<rclcpp::Node> node, 
                    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
                    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group);

    // Declarations
    void load_waypoints();
    geometry_msgs::msg::Pose getWaypoint(const std::string& name);
    void move_cartesian_relative(double z_distance, double x_distance = 0.0, double y_distance = 0.0);
    void move_cartesian_relative_to_waypoint_by_name(const std::string& name, double z_distance, double x_distance = 0.0, double y_distance = 0.0);
    void move_joint_to_waypoint_by_name(const std::string& name);
    void move_joint_relative_to_waypoint_by_name(const std::string& name, double z_distance, double x_distance = 0.0, double y_distance = 0.0);
    void move_gripper(double deg = 45.0);
    void mantis_path_constraints();
    double deg_to_rad(double deg);

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
    std::map<std::string, geometry_msgs::msg::Pose> waypoints_;
};

#endif // TRAJECTORY_UTILS_HPP