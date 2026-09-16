/**
 * @file keyboard_input_reader.cpp
 * @brief Reads inputs from the keyboard and publishes them to the /keyboard_inputs topic
 * 
 * -------------------------
 * Publishing Topics
 *  /keyboard_inputs
 * 
 * -------------------------
 * Subscribed Topics
 *  None
 * 
 * @version 0.1
 * @author Daan krijnen
 * @date September 08, 2026
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

//Include terminal API readers for keystrokes
#include <termios.h> //puts terminal in 'raw' mode to read keys instantly
#include <unistd.h> // gives access to read() commands

#include <chrono> // timing
#include <iostream>// console output commands
#include <string> // Key storage

//TODO
// get key
// copy key
// publish key
// spin node

using namespace std::chrono_literals;

class KeyboardInputReader : public rclcpp::Node
{
public:
    KeyboardInputReader() : Node("keyboard_input_reader")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>(
            "/keyboard_inputs" , 10
        );

        timer_ = this->create_wall_timer(
            20ms, std::bind(&KeyboardInputReader::poll_keyboard, this));
    }

private:
    void poll_keyboard()
    {
        int key = get_key();

        if (key == -1)
        {
            //return; //nothing pressed
            auto msg = std_msgs::msg::String();
            msg.data = "";

            publisher_->publish(msg);

            return;
        }

        char pressed = static_cast<char>(key);

        //stop on c/C
        if (pressed == 'c' || pressed == 'C')
        {
            std::cout << "Stopping keyboard reader..." << std::endl;
            rclcpp::shutdown();
            return;
        }

        auto msg = std_msgs::msg::String();
        msg.data = std::string(1, pressed);

        publisher_->publish(msg);

        //remove later, debugging only
        // std::cout << "Publish key: " << pressed << std::endl;
    }

    int get_key()
    {
        struct termios oldt, newt;
        tcgetattr( STDIN_FILENO, &oldt); //save terminal settings

        newt = oldt; //change the settings
        newt.c_lflag &= ~(ICANON | ECHO);
        newt.c_cc[VMIN] = 0;
        newt.c_cc[VTIME] = 0; //read instantly and dont display in terminal
        
        tcsetattr( STDIN_FILENO, TCSANOW, &newt); //upload settings to terminal

        char ch = 0;
        ssize_t n = read(STDIN_FILENO, &ch, 1); //read the input

        tcsetattr( STDIN_FILENO, TCSANOW, &oldt); //restore old terminal settings
        
        if (n <= 0)
        {
            return -1; //no key pressed
        }

        return static_cast<unsigned char>(ch); //give pressed key as output
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_; //publisher
    rclcpp::TimerBase::SharedPtr timer_; //timer
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<KeyboardInputReader>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}