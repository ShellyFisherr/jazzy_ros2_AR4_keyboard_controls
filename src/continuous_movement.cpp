/**
 * @file coordinate_pos_loop_demo.cpp
 * @brief A more advanced ROS2 and MoveIt 2 framework to move the AR4 robotic arm (based on MK.5)
 * 
 * This program provides a framework on how to use ROS2 and MoveIt 2 to connect to the
 * AR4 robotic arm and move it with continuous movement.
 * It sets up a node, abd uses twist to directly move the robot, bypassing planning
 * 
 * -------------------------
 * Publishing Topics
 *  /servo_node//delta_twist_cmds geometry_msgs::msg::TwistStamped
 * 
 * -------------------------
 * Subscribed Topics
 *  /keyboard_inputs - std_msgs::msg::String
 * 
 * @author Daan Krijnen
 * @date September 09, 2026
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>

class ContinuousMovement : public rclcpp::Node
{
public:
    ContinuousMovement(const rclcpp::NodeOptions & node_options)
    : Node("continuous_movement", node_options)
    {
        command_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/servo_node/delta_twist_cmds", 10);

        keyboard_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "/keyboard_inputs", 10,
        std::bind(&ContinuousMovement::keyboard_callback, this, std::placeholders::_1));

        command_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(40),
            std::bind(&ContinuousMovement::publish_command, this));

        command_type_client_ = this->create_client<moveit_msgs::srv::ServoCommandType>(
            "/servo_node/switch_command_type");
    }

    void initialize_servo()
    {
        while (!command_type_client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                return;
            }
            RCLCPP_INFO(get_logger(),"Waiting for servo command-type service...");
        }

        auto request = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();

        request->command_type = moveit_msgs::srv::ServoCommandType::Request::TWIST;

        command_type_client_->async_send_request(request,[this](rclcpp::Client<moveit_msgs::srv::ServoCommandType>
        ::SharedFuture future)
        {
            auto response = future.get();

            if (response->success)
            {
                RCLCPP_INFO(get_logger(),"Servo switched to Twist mode");
            }
            else
            {
                RCLCPP_ERROR(get_logger(),"Servo could not switch to Twist mode");
            }
        });
    }
private:

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyboard_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_publisher_;
    
    rclcpp::TimerBase::SharedPtr command_timer_;

    rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr command_type_client_;

    std::string latest_key_;
    rclcpp::Time last_input_time_;
    bool received_input_ = false;

    void keyboard_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        latest_key_ = msg->data;

        last_input_time_ = this->now();
        received_input_ = true;
    }

    void publish_command()
    {
        geometry_msgs::msg::TwistStamped command;

        command.header.stamp = this->now();
        command.header.frame_id = "base_link";

        const double speed = 0.005;
        
        if (!received_input_)
        {
            command_publisher_->publish(command);
            RCLCPP_WARN(get_logger(),"no input received yet");
            return;
        }

        if ((this->now() - last_input_time_).seconds() > 0.5)
        {
            latest_key_.clear();
        }

        if (latest_key_ == "w" || latest_key_ == "W")
        {
            command.twist.linear.x += speed;
            RCLCPP_INFO(get_logger(),"W. +x speed altered");
        }
        else if (latest_key_ == "s" || latest_key_ == "S")
        {
            command.twist.linear.x -= speed;
            RCLCPP_INFO(get_logger(),"W. -x speed altered");
        }
        else if (latest_key_ == "a" || latest_key_ == "A")
        {
            command.twist.linear.y += speed;
            RCLCPP_INFO(get_logger(),"W. +y speed altered");
        }
        else if (latest_key_ == "d" || latest_key_ == "D")
        {
            command.twist.linear.y -= speed;
            RCLCPP_INFO(get_logger(),"W. -y speed altered");
        }
        else if (latest_key_ == "r" || latest_key_ == "R")
        {
            command.twist.linear.z += speed;
            RCLCPP_INFO(get_logger(),"W. +z speed altered");
        }
        else if (latest_key_ == "f" || latest_key_ == "F")
        {
            command.twist.linear.z -= speed;
            RCLCPP_INFO(get_logger(),"W. -z speed altered");
        }

        RCLCPP_INFO(
            get_logger(),
            "Publishing command at time %d.%09d",
            command.header.stamp.sec,
            command.header.stamp.nanosec);
        command_publisher_->publish(command);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;

    node_options.automatically_declare_parameters_from_overrides(true);
    node_options.append_parameter_override("use_sim_time", true);

    auto node = std::make_shared<ContinuousMovement>(node_options);

    node->initialize_servo();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}