/**
 * @file DLS_reduced_controller.cpp
 * @brief A more advanced ROS2 and MoveIt 2 framework to move the AR4 robotic arm (based on MK.5)
 * 
 * This program provides a framework on how to use ROS2 and MoveIt 2 to connect to the
 * AR4 robotic arm and move it with continuous movement. It will use DLS and null-space 
 * controll to try and deal with singularities.
 * It sets up a node, and uses twist to directly move the robot, bypassing planning
 * 
 * -------------------------
 * Publishing Topics
 * 
 * -------------------------
 * Subscribed Topics
 * 
 * @author Daan Krijnen
 * @date September 10, 2026
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <vector>
#include <thread>
#include <Eigen/Dense>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <mutex>
#include <unordered_map>
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <Eigen/Geometry>

constexpr double CONTROL_DT = 0.01; // Timer in seconds. (Hz = 1/timer)

class DlsReducedController : public rclcpp::Node
{
public:

    DlsReducedController(const rclcpp::NodeOptions & node_options)
    : Node("dls_reduced_controller", node_options)
    {
        keyboard_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "/keyboard_inputs", 10,
            std::bind(&DlsReducedController::keyboard_callback, this, std::placeholders::_1));

        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::SensorDataQoS(),
            std::bind(&DlsReducedController::joint_state_callback, this, std::placeholders::_1));

        trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10);

        timer_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
    }
    
    void initialize_system()
    {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(),"ar_manipulator");

        const auto robot_model = move_group_->getRobotModel();

        if (!robot_model)
        {
            throw std::runtime_error("Moveit robot model is unavailable");
        }

        joint_model_group_ = robot_model->getJointModelGroup("ar_manipulator");
        if (joint_model_group_ == nullptr)
        {
            throw std::runtime_error("Planning group ar_manipulator was not found");
        }

        robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model);
        robot_state_->setToDefaultValues();

        //move_group_->startStateMonitor();

        control_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(CONTROL_DT),
            std::bind(&DlsReducedController::control_loop, this), timer_callback_group_);
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyboard_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;

    sensor_msgs::msg::JointState latest_joint_state_;
    std::mutex joint_state_mutex_;
    bool received_joint_state_ = false;

    std::string pressed_key_;
    std::mutex input_mutex_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::core::RobotState> robot_state_;
    const moveit::core::JointModelGroup * joint_model_group_;

    std::vector<double> current_positions;
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

    rclcpp::Time last_input_time_;
    bool received_input_ = false;

    
    std::vector<double> commanded_positions;
    bool command_state_initialized_ = false;

    bool orientation_reference_initialized_ = false;
    Eigen::Matrix3d locked_relative_orientation_;


    Eigen::Vector3d previous_ee_position_;
    rclcpp::Time previous_ee_time_;
    bool previous_ee_valid_ = false;

    double command_speed_    = 0.02;
    double rotation_speed_   = 0.2;
    double orientation_gain_ = 1.0;
    double max_pitch_correction_vel_ = 0.4;

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        latest_joint_state_ = *msg;
        received_joint_state_ = true;
    }

    void keyboard_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(input_mutex_);

        pressed_key_ = msg->data;

        // below is for debugging only to check what key is received
        // RCLCPP_INFO(get_logger(),"Recieved key: %s", pressed_key_.c_str());
    }

    // create a basic right-handed coordinate frame
    void make_axis_basis(
        const Eigen::Vector3d & axis,
        Eigen::Vector3d & basis_1,
        Eigen::Vector3d & basis_2)
    {
        Eigen::Vector3d reference;

        if (std::abs(axis.x()) < 0.9)
        {
            reference = Eigen::Vector3d::UnitX();
        }
        else
        {
            reference = Eigen::Vector3d::UnitY();
        }

        basis_1 = axis.cross(reference).normalized();
        basis_2 = axis.cross(basis_1).normalized();
    }

    void control_loop()
    {
        double velocity_x = 0.0;
        double velocity_y = 0.0;
        double velocity_z = 0.0;
        double omega_x = 0.0;
        double omega_y = 0.0;
        double omega_z = 0.0;

        std::string pressed_key;
        rclcpp::Time last_input_time;

        {
            std::lock_guard<std::mutex> lock(input_mutex_);
            pressed_key = pressed_key_;
        }
     
        sensor_msgs::msg::JointState joint_state_copy;
        {
            std::lock_guard<std::mutex> lock(joint_state_mutex_);

            if (!received_joint_state_)
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "Waiting for joint states");

                    return;
            }
            
            joint_state_copy = latest_joint_state_;
        }

        std::unordered_map<std::string, double> position_by_name;

        for (std::size_t index = 0; index < joint_state_copy.name.size(); ++index)
        {
            if (index < joint_state_copy.position.size())
            {
                position_by_name[joint_state_copy.name[index]] = 
                    joint_state_copy.position[index];
            }
        }

        current_positions.clear();

        for (const std::string & joint_name : joint_model_group_->getActiveJointModelNames())
        {
            auto position = position_by_name.find(joint_name);

            if (position == position_by_name.end())
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "Joint state missing joint: %s",
                    joint_name.c_str());

                return;
            }

            current_positions.push_back(position->second);
        }

        if (!command_state_initialized_)
        {
            commanded_positions = current_positions;
            command_state_initialized_ = true;
        }


        robot_state_->setJointGroupPositions(joint_model_group_, current_positions);
        robot_state_->update();

        const auto * end_effector = robot_state_->getLinkModel("ee_link");
        const auto * forearm_link = robot_state_->getLinkModel("link_3");
        

        if (end_effector == nullptr)
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "End-effector link ee_link was not found");

            return;
        }

        if (forearm_link == nullptr)
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Forearm link link_3 was not found");

            return;
        }

        Eigen::Isometry3d ee_transform = robot_state_->getGlobalLinkTransform(end_effector);

        Eigen::Isometry3d forearm_transform = robot_state_->getGlobalLinkTransform(forearm_link);

        Eigen::Matrix3d current_relative_orientation = forearm_transform.rotation().transpose() * ee_transform.rotation();

        if (!orientation_reference_initialized_)
        {
            locked_relative_orientation_ = forearm_transform.rotation().transpose() * ee_transform.rotation();

            orientation_reference_initialized_ = true;
        }

        Eigen::Vector3d ee_position = ee_transform.translation(); 

        if (previous_ee_valid_)
        {
            double dt = (this->now() - previous_ee_time_).seconds();

            if (dt > 1e-6)
            {
                Eigen::Vector3d actual_velocity = (ee_position - previous_ee_position_) / dt;

                RCLCPP_INFO_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    1000,
                    "Actual EE velocity: x= %.5f y=%.5f z=%.5f",
                    actual_velocity.x(), actual_velocity.y(), actual_velocity.z());
            }
        }

        previous_ee_position_ = ee_position;
        previous_ee_time_ = this->now();
        previous_ee_valid_ = true;

        //get jacobians

        Eigen::MatrixXd full_jacobian;
        robot_state_->getJacobian(
            joint_model_group_,
            end_effector,
            Eigen::Vector3d::Zero(),
            full_jacobian);

        if (full_jacobian.rows() !=6 || full_jacobian.cols() != 6)
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Expected a 6x6 Jacobian got %ldx%ld",
                full_jacobian.rows(),
                full_jacobian.cols());

            return;
        }

        Eigen::Matrix<double, 3, 6> linear_jacobian = full_jacobian.topRows<3>();
        Eigen::Matrix<double, 3, 6> angular_jacobian = full_jacobian.bottomRows<3>();

        Eigen::Matrix3d relative_error = locked_relative_orientation_ * current_relative_orientation.transpose();

        Eigen::AngleAxisd error_angle_axis(relative_error);
        Eigen::Vector3d rotation_error = error_angle_axis.angle() * error_angle_axis.axis();

        Eigen::Vector3d pitch_axis = Eigen::Vector3d::UnitY();

        double pitch_error = pitch_axis.dot(rotation_error);

        double pitch_correction = orientation_gain_ * pitch_error;
        pitch_correction = std::clamp(pitch_correction, -max_pitch_correction_vel_, max_pitch_correction_vel_);

        Eigen::MatrixXd forearm_jacobian;
        robot_state_->getJacobian(
            joint_model_group_,
            forearm_link,
            Eigen::Vector3d::Zero(),
            forearm_jacobian);

        if (forearm_jacobian.rows() != 6 || forearm_jacobian.cols() != 6)
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Expected a 6x6 forearm Jacobian");

            return;
        }
        
        Eigen::Matrix<double, 3, 6> forearm_angular_jacobian = forearm_jacobian.bottomRows<3>();

        Eigen::Matrix<double, 3, 6> relative_angular_jacobian = forearm_transform.rotation().transpose() * (angular_jacobian - forearm_angular_jacobian);

        //get inputs

        if (pressed_key == "w" || pressed_key == "W")
        {
            velocity_x = command_speed_;
        }
        else if (pressed_key == "s" || pressed_key == "S")
        {
            velocity_x = -command_speed_;
        }
        else if (pressed_key == "a" || pressed_key == "A")
        {
            velocity_y = command_speed_;
        }
        else if (pressed_key == "d" || pressed_key == "D")
        {
            velocity_y = -command_speed_;
        }
        else if (pressed_key == "r" || pressed_key == "R")
        {
            velocity_z = command_speed_;
        }
        else if (pressed_key == "f" || pressed_key == "F")
        {
            velocity_z = -command_speed_;
        }
        else if (pressed_key == "i" || pressed_key == "I")
        {
            omega_x = rotation_speed_;
        }
        else if (pressed_key == "k" || pressed_key == "K")
        {
            omega_x = -rotation_speed_;
        }
        else if (pressed_key == "j" || pressed_key == "J")
        {
            omega_z = rotation_speed_;
        }
        else if (pressed_key == "l" || pressed_key == "L")
        {
            omega_z = -rotation_speed_;
        }
        else if (pressed_key == "")
        {
            velocity_x = 0.0;
            velocity_y = 0.0;
            velocity_z = 0.0;
            omega_x = 0.0;
            omega_y = 0.0;
            omega_z = 0.0;
        }

        RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(), 1000,
            "Desired velocity: x=%.3f y=%.3f z=%.3f OmegaX=%.3f OmegaY=%.3f OmegaZ=%.3f",
            velocity_x, velocity_y, velocity_z, omega_x, omega_y, omega_z);
        

        //create desired DLS joint velocity
        Eigen::Matrix<double, 6, 1> desired_twist(
            velocity_x,
            velocity_y,
            velocity_z,
            omega_x,
            pitch_correction,
            omega_z);

        Eigen::Matrix<double, 6, 6> task_jacobian;
        task_jacobian.topRows<3>() = linear_jacobian;
        task_jacobian.bottomRows<3>() = relative_angular_jacobian;




        double damping_ = 0.005;
 
        Eigen::Matrix<double, 6, 6> damping_matrix = damping_ * damping_ * Eigen::Matrix<double, 6, 6>::Identity();

        Eigen::Matrix<double, 6, 6> dls_inverse = task_jacobian.transpose() * (task_jacobian * task_jacobian.transpose() + damping_matrix).inverse();

        Eigen::Matrix<double, 6, 1> joint_velocity = dls_inverse * desired_twist;

        Eigen::Matrix<double, 6, 1> achieved_twist = task_jacobian * joint_velocity;

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Desired linear: %.4f %.4f %.4f | Achieved linear: %.4f %.4f %.4f",
            desired_twist(0),
            desired_twist(1),
            desired_twist(2),
            achieved_twist(0),
            achieved_twist(1),
            achieved_twist(2));

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Desired angular: %.4f %.4f %.4f | Achieved angular: %.4f %.4f %.4f",
            desired_twist(3),
            desired_twist(4),
            desired_twist(5),
            achieved_twist(3),
            achieved_twist(4),
            achieved_twist(5));




        RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Joint velocities: %.3f %.3f %.3f %.3f %.3f %.3f",
        joint_velocity(0),
        joint_velocity(1),
        joint_velocity(2),
        joint_velocity(3),
        joint_velocity(4),
        joint_velocity(5));

        //publish
        trajectory_msgs::msg::JointTrajectory command;
        command.header.stamp = this->now();

        for (const std::string & joint_name : joint_model_group_->getActiveJointModelNames())
        {
            command.joint_names.push_back(joint_name);
        }

        trajectory_msgs::msg::JointTrajectoryPoint point;

        point.positions = current_positions;
        point.velocities.resize(joint_velocity.size());

        for (std::size_t idx = 0; idx < current_positions.size(); ++idx)
        {
            point.velocities[idx] = joint_velocity(idx);

            commanded_positions[idx] += joint_velocity(idx) * CONTROL_DT;
            point.positions[idx] = commanded_positions[idx];
        }

        point.time_from_start = rclcpp::Duration::from_seconds(CONTROL_DT);

        command.points.push_back(point);

        trajectory_publisher_->publish(command);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;

    node_options.automatically_declare_parameters_from_overrides(true);
    node_options.append_parameter_override("use_sim_time", true);

    auto node = std::make_shared<DlsReducedController>(node_options);

    node->initialize_system();

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),2);
    executor.add_node(node);
    
    std::thread executor_thread([&executor]() {executor.spin(); });

    while (rclcpp::ok())
    {
        std::this_thread::sleep_for(std::chrono::duration<double>(CONTROL_DT));
    }

    executor.cancel();
    executor_thread.join();

    rclcpp::shutdown();
    return 0;
}