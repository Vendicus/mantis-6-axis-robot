#include "mantis_commander/trajectory_utils.hpp"
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <moveit/move_group_interface/move_group_interface.hpp>

//constructor init
TrajectoryUtils::TrajectoryUtils(std::shared_ptr<rclcpp::Node> node, 
                                 std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
                                 std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group)
: node_(node), move_group_(move_group), gripper_group_(gripper_group)  {}


void TrajectoryUtils::load_waypoints() {
    std::string pkg_share = ament_index_cpp::get_package_share_directory("mantis_commander");
    std::string yaml_path = pkg_share + "/../../../../src/mantis_commander/config/waypoints.yaml";

    YAML::Node config;
    try {
        config = YAML::LoadFile(yaml_path);
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "YAML file couldn't be loaded: %s", e.what());
        return;
    }

    for (const auto& wp : config["waypoints"]) {
        std::string name = wp["name"].as<std::string>();
        auto pos = wp["position"].as<std::vector<double>>();
        auto rpy = wp["rpy"].as<std::vector<double>>();

        tf2::Quaternion q;
        q.setRPY(rpy[0], rpy[1], rpy[2]);

        geometry_msgs::msg::Pose pose;
        pose.position.x = pos[0];
        pose.position.y = pos[1];
        pose.position.z = pos[2];
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();

        waypoints_[name] = pose;
    }
    RCLCPP_INFO(node_->get_logger(), "Loaded %zu points from waypoints.yaml.", waypoints_.size());
}

geometry_msgs::msg::Pose TrajectoryUtils::getWaypoint(const std::string& name) {
    if (waypoints_.find(name) != waypoints_.end()) {
        return waypoints_[name];
    }
    RCLCPP_ERROR(node_->get_logger(), "Point '%s' doesn't exist! Returning zero pose.", name.c_str());
    return geometry_msgs::msg::Pose();
}

void TrajectoryUtils::move_cartesian_relative(double z_distance, double x_distance, double y_distance) {
    geometry_msgs::msg::Pose target_pose = move_group_->getCurrentPose().pose;
    std::vector<geometry_msgs::msg::Pose> cartesian_waypoints;
    
    target_pose.position.x += x_distance;
    target_pose.position.y += y_distance;
    target_pose.position.z += z_distance;
    cartesian_waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_->computeCartesianPath(cartesian_waypoints, 0.01, trajectory);

    if (fraction >= 0.95) {
        move_group_->execute(trajectory);
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Cartesian path cannot be planned! (%.2f%%)", fraction * 100.0);
    }
}

void TrajectoryUtils::move_cartesian_relative_to_waypoint_by_name(const std::string& name, double z_distance, double x_distance, double y_distance) {
    geometry_msgs::msg::Pose target_pose = getWaypoint(name);
    std::vector<geometry_msgs::msg::Pose> cartesian_waypoints;

    target_pose.position.x += x_distance;
    target_pose.position.y += y_distance;
    target_pose.position.z += z_distance;
    cartesian_waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_->computeCartesianPath(cartesian_waypoints, 0.01, trajectory);

    if (fraction >= 0.95) {
        move_group_->execute(trajectory);
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Cartesian path cannot be planned! '%s' (%.2f%%)", name.c_str(), fraction * 100.0);
    }
}

void TrajectoryUtils::move_joint_to_waypoint_by_name(const std::string& name) {
    move_group_->setStartStateToCurrentState();
    geometry_msgs::msg::Pose target_pose = getWaypoint(name);
    move_group_->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool planning_result = (move_group_->plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    if (planning_result) {
        move_group_->execute(plan1);
    }
    else {
        RCLCPP_ERROR(node_->get_logger(), "Cannot plan trajectory to specified point '%s'!", name.c_str());
    }
}

void TrajectoryUtils::move_joint_relative_to_waypoint_by_name(const std::string& name, double z_distance, double x_distance, double y_distance) {
    move_group_->setStartStateToCurrentState();
    geometry_msgs::msg::Pose target_pose = getWaypoint(name);

    target_pose.position.x += x_distance;
    target_pose.position.y += y_distance;
    target_pose.position.z += z_distance;

    move_group_->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool planning_result = (move_group_->plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    if (planning_result) {
        move_group_->execute(plan1);
    }
    else {
        RCLCPP_ERROR(node_->get_logger(), "Cannot plan trajectory to specified point '%s' with transalation!", name.c_str());
    }
}

double TrajectoryUtils::deg_to_rad(double deg){
    return deg*3.14159/180.0;
}

void TrajectoryUtils::move_gripper(double deg){

    gripper_group_->setStartStateToCurrentState();
    gripper_group_->setJointValueTarget("joint_7_gripper_sprocket_to_gripper_main", TrajectoryUtils::deg_to_rad(deg));

    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool planning_result = (gripper_group_->plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    if(planning_result){
        gripper_group_->execute(plan1);
    }
    else{
        RCLCPP_ERROR(node_->get_logger(), "Cannot plan gripper move: '%.2f' ", deg);
    }
}

void TrajectoryUtils::mantis_path_constraints(){
    moveit_msgs::msg::Constraints constraints;
    constraints.name = "up_side_of_arm2_link";

    moveit_msgs::msg::JointConstraint jc;
    jc.joint_name = "joint_3_arm2_to_arm1"; // Name of the joint with constraint
    jc.position = -1.2;                     // middle point of acceptance move - rad
    jc.tolerance_above = 1.418;             // Up limit - rad
    jc.tolerance_below = 1.418;             // Min limit - rad
    jc.weight = 0.75;                       // Weight, (1.0 - must have, the lower value the lower constraint)

    constraints.joint_constraints.push_back(jc);

    move_group_->setPathConstraints(constraints);
}


