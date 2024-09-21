/*#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;



class DrawNode : public rclcpp::Node
{
public:
    DrawNode() : Node("draw_node")
    {
        timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&DrawNode::loop, this));
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "draw_node <<turtle_race>> is running...");

    }
private:
    void loop()
    {
        auto cmd_msg = geometry_msgs::msg::Twist();
        if (loop_count_ < 20)
        {
            cmd_msg.linear.x = 1.0;
            cmd_msg.angular.z = 0.0;
        }
        else
        {
            cmd_msg.linear.x = 0.0;
            cmd_msg.angular.z = -1.0;
        }
        cmd_pub_->publish(cmd_msg);
        loop_count_++;
        if (loop_count_ > 50)
        {
            loop_count_ = 0;
        }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    int loop_count_ = 0;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DrawNode>());
    rclcpp::shutdown();
    return 0;
} 
*/
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

        // Inicializáljuk az induló pozíciót
        step_ = 0;
    }

private:
    void loop()
    {
        auto cmd_msg = geometry_msgs::msg::Twist();

        //if else ágak, amikben a step_ et használva kirajozolunk egy négyzetet
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

        // Publikáljuk a parancsot a turtlesimnek
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

