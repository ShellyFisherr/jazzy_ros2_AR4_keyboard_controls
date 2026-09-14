/**
 * @file movement_translator.cpp
 * @brief subscribes to the /keyboard_inputs topic and translates it into MoveIt movement commands
 * 
 * -------------------------
 * Publishing Topics
 *  None
 * 
 * -------------------------
 * Subscribed Topics
 *  /keyboard_inputs - std_msgs::msg::String
 *  /moveit_position_carthesian - geometry_msgs::msg::PoseStamped
 * 
 * @version 0.1
 * @author Daan krijnen
 * @date September 08, 2026
 */

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include "std_msgs/msg/string.hpp"
#include <map>
#include <string>
#include <vector>
#include <functional>
#include <rclcpp/executors/multi_threaded_executor.hpp>

class MovementTranslator : public rclcpp::Node
{
public:
    MovementTranslator(const rclcpp::NodeOptions & node_options) : Node("movement_translator", node_options)
    {
        keyboard_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "/keyboard_inputs", 10,
            std::bind(&MovementTranslator::keyboard_callback, this, std::placeholders::_1));

        pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/moveit_position_carthesian", 10,
            std::bind(&MovementTranslator::pose_callback, this, std::placeholders::_1));
    }

    void initialize_move_group()
    {
        arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "ar_manipulator");

        
        arm_->startStateMonitor();
        arm_->setPlanningTime(0.2);
        arm_->setPlannerId("RRTConnectConfigDefault");

    }

private:

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyboard_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    std::string pending_key_;

    void keyboard_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        pending_key_ = msg->data;

        RCLCPP_INFO(get_logger(),"Recieved key: %s", pending_key_.c_str());
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // if (!arm_->getCurrentState(5.0)) 
        // {
        // RCLCPP_ERROR(
        // get_logger(),
        // "Current robot state is unavailable");
        // return;
        // }

        if (pending_key_.empty())
        {
            return;
        }

        geometry_msgs::msg::PoseStamped target = *msg;
        
        // RCLCPP_INFO(
        //     get_logger(),
        //     "Current pose: x=%.3f, y=%.3f, z=%.3f",
        //     target.pose.position.x,
        //     target.pose.position.y,
        //     target.pose.position.z
        // );

        const double step = 0.01;

        if (pending_key_ == "w" || pending_key_ == "W")
        {
            target.pose.position.x += step;
        }
        else if (pending_key_ == "s" || pending_key_ == "S")
        {
            target.pose.position.x -= step;
        }
        else if (pending_key_ == "a" || pending_key_ == "A")
        {
            target.pose.position.y += step;
        }
        else if (pending_key_ == "d" || pending_key_ == "D")
        {
            target.pose.position.y -= step;
        }
        else if (pending_key_ == "r" || pending_key_ == "R")
        {
            target.pose.position.z += step;
        }
        else if (pending_key_ == "f" || pending_key_ == "F")
        {
            target.pose.position.z -= step;
        }
        else if (pending_key_ == "h" || pending_key_ == "H")
        {
            go_home();
            pending_key_.clear();
            return;
        }
        else
        {
            pending_key_.clear();
            return;
        }

        move_to_pose(target);
        
        pending_key_.clear();
    }

    void move_to_pose(const geometry_msgs::msg::PoseStamped & target)
    {
        arm_->setPoseTarget(target);

        moveit::planning_interface::MoveGroupInterface::Plan plan;

        auto result = arm_->plan(plan);

        if (result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            arm_->execute(plan);
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Could not plan to requested target");
        }

        arm_->clearPoseTargets();

    }

    void go_home()
    {
        std::vector<std::string> joint_names = {
            "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"
        };

        std::vector<double> home = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        std::map<std::string, double> target;
        for (size_t i = 0; i < joint_names.size(); ++i) {
            target[joint_names[i]] = home[i];
        }

        arm_->setJointValueTarget(target);
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        if (arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
        {
            arm_->execute(plan);
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;

    

    node_options.automatically_declare_parameters_from_overrides(true);
    node_options.append_parameter_override("use_sim_time", true);
    

    auto node = std::make_shared<MovementTranslator>(node_options);

    node->initialize_move_group();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}