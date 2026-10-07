#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<rclcpp::Node>("example_moveit");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() {executor.spin();});

    auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm"); // second argument is planning group, we defined in moveit setup
    arm.setMaxVelocityScalingFactor(0.5);
    arm.setMaxAccelerationScalingFactor(0.5);

    // named goals
    arm.setStartStateToCurrentState(); //current forward kinematics of robot as start state
    arm.setNamedTarget("box_1"); //name target

    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool planning_result = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    if (planning_result) {
        arm.execute(plan1);

    }

    rclcpp::shutdown();
    spinner.join(); //wait for thread to finish its job
    return 0;
}