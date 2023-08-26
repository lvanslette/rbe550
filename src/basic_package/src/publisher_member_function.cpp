// node dependencies, which are added to package.xml and CMakeLists.txt
#include <chrono>
#include <functional>
#include <memory>
#include <string>

// allows you to use the most common pieces of the ROS2 system
#include "rclcpp/rclcpp.hpp"
// includes built-in message type used to publish data
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node (inherits Node ) and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
// every "this" in the code is referring to the node
{
  public:
    // public constructor initializes count to 0
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      // publisher_ initialized with a String message type, topic named "topic", and max queue size as 10
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      // initialize timer_, which causes timer_callback function to be executed twice a second
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    // function where message data is set and the messages are published
    void timer_callback()
    {
      // creates message variable with type String  
      auto message = std_msgs::msg::String();
      // sets the data of that message to a std::string and adds the number of times
      //    Hello,world has been printed
      message.data = "Hello, world! " + std::to_string(count_++);
      // ensures every published message is printed to the console
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    // declaration of the timer, publisher, and counter fields
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  // initializes ROS2
  rclcpp::init(argc, argv);
  // starts timer, processing data from the node, including callbacks from the timer
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}