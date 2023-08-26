#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    // same underlying inherited class Node as the publisher, creating a new node named "minimal subscriber"
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      // use Node class's create_subscription class to execute the callback  
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
      // NOTE: no timer created because the subscriber responds whenever data is published to the topic "topic"
    }

  private:
    // receives string message data published over "topic" and writes it to the console
    void topic_callback(const std_msgs::msg::String & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }
    // create Subscription object
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  // initializes ROS2
  rclcpp::init(argc, argv);
  // waits for publisher to publish a message
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}