#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "transponder_msgs/msg/transponder.hpp"

class TransponderDebug : public rclcpp::Node
{
  public:
    TransponderDebug()
    : Node("debug_node")
    {

        RCLCPP_INFO(this->get_logger(), "Transponder Debug Node");

        // Parameters
        this->declare_parameter("transponder_in", "/transponder/in");
        std::string param_transponderIn = this->get_parameter("transponder_in").as_string();

        // Publishers
        pub_Transponder_ = this->create_publisher<transponder_msgs::msg::Transponder>(param_transponderIn, 1);

        // Timers
        timer_debug_ = this->create_wall_timer(
            std::chrono::milliseconds(1 * 1000),
            std::bind(&TransponderDebug::callback_debug, this));
    }

  private:
    void callback_debug()
    {
        msg_.header.stamp = this->get_clock()->now();
        msg_.header.frame_id = "map";

        msg_.car_id = 8;
        msg_.lat = msg_.lat + 0.1;
        msg_.lon = msg_.lon - 0.1;
        msg_.v_east = msg_.v_east + 0.2;
        msg_.v_north = msg_.v_north - 0.2;
        msg_.v_up = msg_.v_up + 0.3;
        msg_.state = transponder_msgs::msg::Transponder::STATE_NOMINAL;

        pub_Transponder_->publish(msg_);
    }

    // Publishers / Subscribers / Timers
    rclcpp::TimerBase::SharedPtr timer_debug_;
    rclcpp::Publisher<transponder_msgs::msg::Transponder>::SharedPtr pub_Transponder_;

    // Variables
    transponder_msgs::msg::Transponder msg_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TransponderDebug>());
  rclcpp::shutdown();
  return 0;
}
