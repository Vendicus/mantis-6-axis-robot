#include <geometry_msgs/msg/pose.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include "mantis_commander/trajectory_utils.hpp" 

#define OPEN_GRIPPER 45.0 //deg
#define CLOSE_GRIPPER 33.0 //deg

void callService(rclcpp::Node::SharedPtr node, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client) {
    if (!client->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node->get_logger(), "Gripper service unavailable!");
        return;
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result_future = client->async_send_request(request);

    if (result_future.wait_for(std::chrono::seconds(3)) == std::future_status::ready) {
        RCLCPP_INFO(node->get_logger(), "Command to gripper sent successfully.");
    } else {
        RCLCPP_WARN(node->get_logger(), "No answer from service!");
    }
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("main_planner_node");
    rclcpp::executors::SingleThreadedExecutor executor;

    auto  attach_client_box1 = node->create_client<std_srvs::srv::Trigger>("gripper/attach");
    auto  detach_client_box1 = node->create_client<std_srvs::srv::Trigger>("gripper/detach");
    auto  attach_client_box2 = node->create_client<std_srvs::srv::Trigger>("gripper/attach_box1");
    auto  detach_client_box2 = node->create_client<std_srvs::srv::Trigger>("gripper/detach_box1");

    executor.add_node(node);
    auto spinner = std::thread([&executor]() {executor.spin();}); // new thread to not block movegroup executor

    // MoveGroup Init
    auto arm_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "arm");
    auto gripper_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "gripper");
    arm_group->setMaxVelocityScalingFactor(0.5);
    arm_group->setMaxAccelerationScalingFactor(0.5);
    gripper_group->setMaxVelocityScalingFactor(0.5);
    gripper_group->setMaxAccelerationScalingFactor(0.5);

    TrajectoryUtils traj_utils(node, arm_group, gripper_group);

    traj_utils.mantis_path_constraints(); // remember to clear path constrains on the end!
    
    // 1. Load waypoints
    traj_utils.load_waypoints();


    // ---------- MAIN LOOP -----------

    while(rclcpp::ok()) {
        // 2. Calibrate and move to BOX_1
        traj_utils.move_joint_relative_to_waypoint_by_name("HOME", 0.1);
        traj_utils.move_gripper(OPEN_GRIPPER);
        traj_utils.move_joint_to_waypoint_by_name("HOME");
        traj_utils.move_joint_relative_to_waypoint_by_name("BOX_1", 0.1);

        arm_group->setMaxVelocityScalingFactor(0.5);
        arm_group->setMaxAccelerationScalingFactor(0.5);

        traj_utils.move_cartesian_relative(-0.1);
        traj_utils.move_gripper(CLOSE_GRIPPER);
        callService(node, attach_client_box1);
        rclcpp::sleep_for(std::chrono::seconds(1));
        traj_utils.move_cartesian_relative(0.1);

        arm_group->setMaxVelocityScalingFactor(0.5);
        arm_group->setMaxAccelerationScalingFactor(0.5);

        // 3. Move to next point to place BOX_1 
        traj_utils.move_joint_relative_to_waypoint_by_name("BOX_1_PLACED", 0.1);

        arm_group->setMaxVelocityScalingFactor(0.5);
        arm_group->setMaxAccelerationScalingFactor(0.5);

        traj_utils.move_cartesian_relative(-0.1);
        traj_utils.move_gripper(OPEN_GRIPPER); 
        callService(node, detach_client_box1);
        rclcpp::sleep_for(std::chrono::seconds(1));
        traj_utils.move_cartesian_relative(0.1);

        arm_group->setMaxVelocityScalingFactor(0.5);
        arm_group->setMaxAccelerationScalingFactor(0.5);

        // 4. Move to HOME
        traj_utils.move_joint_to_waypoint_by_name("HOME");
        //traj_utils.move_gripper(OPEN_GRIPPER);

        // 5. Move to BOX_2
        traj_utils.move_joint_relative_to_waypoint_by_name("BOX_2",0.11, 0.0, -0.01);

        arm_group->setMaxVelocityScalingFactor(0.5);
        arm_group->setMaxAccelerationScalingFactor(0.5);

        traj_utils.move_cartesian_relative(-0.09);
        traj_utils.move_gripper(CLOSE_GRIPPER); 
        callService(node, attach_client_box2);
        rclcpp::sleep_for(std::chrono::seconds(1));
        traj_utils.move_cartesian_relative(0.1);

        arm_group->setMaxVelocityScalingFactor(0.5);
        arm_group->setMaxAccelerationScalingFactor(0.5);

        // 6. Move to next point to place BOX_2
        traj_utils.move_joint_relative_to_waypoint_by_name("BOX_2_PLACED",0.1);

        arm_group->setMaxVelocityScalingFactor(0.5);
        arm_group->setMaxAccelerationScalingFactor(0.5);

        traj_utils.move_cartesian_relative(-0.09);
        traj_utils.move_gripper(OPEN_GRIPPER); 
        callService(node, detach_client_box2);
        rclcpp::sleep_for(std::chrono::seconds(1));
        traj_utils.move_cartesian_relative(0.1);

        arm_group->setMaxVelocityScalingFactor(0.5);
        arm_group->setMaxAccelerationScalingFactor(0.5);

        // 7. Move to HOME
        traj_utils.move_joint_to_waypoint_by_name("HOME");

        // ----------------> SECOND CYCLE
        rclcpp::sleep_for(std::chrono::seconds(3));

        // 8. Move to BOX 1 to pick it up again
        traj_utils.move_joint_relative_to_waypoint_by_name("BOX_1_PLACED", 0.1);
        
        arm_group->setMaxVelocityScalingFactor(0.5);
        arm_group->setMaxAccelerationScalingFactor(0.5);

        traj_utils.move_cartesian_relative(-0.12);
        traj_utils.move_gripper(CLOSE_GRIPPER);
        callService(node, attach_client_box1);
        rclcpp::sleep_for(std::chrono::seconds(1));
        traj_utils.move_cartesian_relative(0.12);    

        arm_group->setMaxVelocityScalingFactor(0.5);
        arm_group->setMaxAccelerationScalingFactor(0.5);

        // 9. Move to origin point to place BOX_1
        traj_utils.move_joint_relative_to_waypoint_by_name("BOX_1", 0.1);

        arm_group->setMaxVelocityScalingFactor(0.5);
        arm_group->setMaxAccelerationScalingFactor(0.5);

        traj_utils.move_cartesian_relative(-0.1);
        traj_utils.move_gripper(OPEN_GRIPPER);
        callService(node, detach_client_box1);
        rclcpp::sleep_for(std::chrono::seconds(1));
        traj_utils.move_cartesian_relative(0.1);

        arm_group->setMaxVelocityScalingFactor(0.5);
        arm_group->setMaxAccelerationScalingFactor(0.5);

        // 10. Move to HOME 
        traj_utils.move_joint_to_waypoint_by_name("HOME");

        // 11. Move to BOX_2 to pick it up again
        traj_utils.move_joint_relative_to_waypoint_by_name("BOX_2_PLACED", 0.1);
        traj_utils.move_cartesian_relative(-0.09);
        traj_utils.move_gripper(CLOSE_GRIPPER);
        callService(node, attach_client_box2);
        rclcpp::sleep_for(std::chrono::seconds(1));
        traj_utils.move_cartesian_relative(0.1);

        // 12. Move to origin point to place BOX_2
        traj_utils.move_joint_relative_to_waypoint_by_name("BOX_2", 0.11, 0.0, -0.01);
        traj_utils.move_cartesian_relative(-0.09);
        traj_utils.move_gripper(OPEN_GRIPPER);
        callService(node, detach_client_box2);
        rclcpp::sleep_for(std::chrono::seconds(1));
        traj_utils.move_cartesian_relative(0.1);    

        // 13. Move to HOME
        traj_utils.move_joint_to_waypoint_by_name("HOME");
        rclcpp::sleep_for(std::chrono::seconds(3));
    }


    // ---------> clear path constraints <--------
    arm_group->clearPathConstraints();

    rclcpp::shutdown();
    spinner.join();
    return 0;
}