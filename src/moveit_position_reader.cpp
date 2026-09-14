/**
 * @file keyboard_input_reader.cpp
 * @brief Reads the current postion of the end-effector from moveit in carthesian coordinates
 * 
 * -------------------------
 * Publishing Topics
 *  /moveit_position_carthesian - geometry_msgs::msg::PoseStamped
 * 
 * -------------------------
 * Subscribed Topics
 *  None
 * 
 * @version 0.1
 * @author Daan krijnen
 * @date September 08, 2026
 */

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <std_msgs/msg/string.hpp>
#include <functional>
#include <chrono>
#include <thread>
#include <rclcpp/executors/multi_threaded_executor.hpp>

class MoveitPositionReader : public rclcpp::Node
{
public:
    MoveitPositionReader(const rclcpp::NodeOptions & node_options) : 
        Node("moveit_position_reader", node_options)
    {
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/moveit_position_carthesian", 10);

        timer_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
    }

    void initialize_move_group()
    {
        arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "ar_manipulator");

        arm_->startStateMonitor();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&MoveitPositionReader::read_current_pose, this),
            timer_callback_group_);
    }

private:
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_;

    void read_current_pose()
    {
        auto current_state = arm_->getCurrentState(1.0);

        if(!current_state) {
            RCLCPP_WARN(get_logger(),
            "No current MoveIt state available yet");
            return;
        }

        auto current_pose = arm_->getCurrentPose("link_6");

        RCLCPP_INFO(
        get_logger(),
        "Pose: frame=%s x=%.3f y=%.3f z=%.3f",
        current_pose.header.frame_id.c_str(),
        current_pose.pose.position.x,
        current_pose.pose.position.y,
        current_pose.pose.position.z);

        current_pose.header.stamp = this->now();

        pose_publisher_->publish(current_pose);
    }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node_options.append_parameter_override("use_sim_time", true);

    auto node = std::make_shared<MoveitPositionReader>(node_options);


    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(),2);

    node->initialize_move_group();

    executor.add_node(node);

    

    std::thread executor_thread([&executor]() {executor.spin(); });

    while (rclcpp::ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    //rclcpp::wait_for_shutdown(); //not included

    executor.cancel();
    executor_thread.join();

    rclcpp::shutdown();
    return 0;
}
