#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class DrawNode : public rclcpp::Node
{
public:
    DrawNode() : Node("draw_node")
    {
        timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&DrawNode::loop, this));
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "draw_node <<turtle_race>> is running...");
        step_ = 0;
    }

private:
    void loop()
    {
        auto cmd_msg = geometry_msgs::msg::Twist();

        if (step_ == 0)
        {
            cmd_msg.linear.x = 1.0;
            cmd_msg.angular.z = 0.0;
        }
        else if (step_ == 1)
        {
            cmd_msg.linear.x = 0.0;
            cmd_msg.angular.z = -1.57;
        }
        else if (step_ == 2)
        {
            cmd_msg.linear.x = 1.0;
            cmd_msg.angular.z = 0.0;
        }
        else if (step_ == 3)
        {
            cmd_msg.linear.x = 0.0;
            cmd_msg.angular.z = -1.57;
        }
        else if (step_ == 4)
        {
            cmd_msg.linear.x = 1.0;
            cmd_msg.angular.z = 0.0;
        }
        else if (step_ == 5)
        {
            cmd_msg.linear.x = 0.0;
            cmd_msg.angular.z = -1.57;
        }
        else if (step_ == 6)
        {
            cmd_msg.linear.x = 1.0;
            cmd_msg.angular.z = 0.0;
        }
        else if (step_ == 7)
        {
            cmd_msg.linear.x = 0.0;
            cmd_msg.angular.z = -1.57;
        }
        else
        {
            cmd_msg.linear.x = 0.0;
            cmd_msg.angular.z = 0.0;
        }

        cmd_pub_->publish(cmd_msg);
        step_++;
    }
        


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    int step_ = 0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DrawNode>());
    rclcpp::shutdown();
    return 0;
}

