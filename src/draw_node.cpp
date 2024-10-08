#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include <chrono>

using namespace std::chrono_literals;

class DrawNode : public rclcpp::Node
{
private:
    void set_pen(bool enable)
    {
        auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
        request->r = 235; request->g = 237; request->b = 22; request->width = 5; request->off = !enable;
        auto result = set_pen_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Set_pen hiba.");
        }
    }
    void draw_track()
    {
        if (move_index_ != directions_.size())
        {
            auto twist_msg = geometry_msgs::msg::Twist();
            twist_msg.linear.x = directions_[move_index_].first;
            twist_msg.angular.z = directions_[move_index_].second;
            pub_->publish(twist_msg);
            move_index_++;
        }
        else
        {
            auto stop_msg = geometry_msgs::msg::Twist();
            pub_->publish(stop_msg);
            RCLCPP_INFO(this->get_logger(), "A pálya rajzolása befejeződött.");
            /*set_pen(false);
            auto teleport_request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
            teleport_request->x = 7.5;teleport_request->y = 5.5;teleport_request->theta = 0.0; 
            auto teleport_client_ = this->create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute");
            while (!teleport_client_->wait_for_service(5s)) {
                RCLCPP_INFO(this->get_logger(), "Varakozas a turtlesim teleport szolgaltatasra...");
            }
            auto teleport_future = teleport_client_->async_send_request(teleport_request);
            auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(), teleport_future);
            if (result == rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "Sikeres teleportáció.");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Teleportációs hiba.");
            }*/
        
            rclcpp::shutdown();
        }
    }

protected:
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr set_pen_client_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr teleport_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t move_index_;
    std::vector<std::pair<double, double>> directions_;

public:
    DrawNode() : Node("draw_node"), move_index_(0)
    {
        set_pen_client_ = this->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");
        teleport_client_ = this->create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute");
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

        directions_ = {
            {8.0, 0.0}, {0.0, M_PI_2}, {3.0, 0.0}, {0.0, M_PI_2},
            {7.0, 0.0}, {0.0, -M_PI_2}, {2.0, 0.0}, {0.0, -M_PI_2},
            {4.0, 0.0}, {0.0, M_PI_2}, {3.0, 0.0}, {0.0, M_PI_2},
            {4.0, 0.0}, {0.0, -M_PI}, {6.0, 0.0}, {0.0, -M_PI_2},
            {3.0, 0.0}, {0.0, -M_PI_2}, {1.0, 0.0}, {0.0, M_PI_2},
            {1.0, 0.0}, {0.0, M_PI_2}, {2.0, 0.0}, {0.0, M_PI_2},
            {5.0, 0.0}, {0.0, M_PI_2}, {8.0, 0.0}, {0.0, M_PI_2}, 
            {2.0, 0.0}, {0.0, M_PI_2}, {4.0, 0.0}, {0.0, -M_PI_2},
            {1.0, 0.0}, {0.0, -M_PI_2}, {4.0, 0.0}, {0.0, M_PI_2},
            {4.0, 0.0}, {0.0, M_PI_2}, {7.0, 0.0}, {0.0, -M_PI_2},
            {1.0, 0.0}, {0.0, -M_PI_2}, {7.0, 0.0}, {0.0, M_PI_2},
            {1.0, 0.0}, {0.0, 0.0},
        };
        while (!teleport_client_->wait_for_service(5s) || !set_pen_client_->wait_for_service(5s))
        {
            RCLCPP_INFO(this->get_logger(), "Várakozás a turtlesim szolgáltatásokra...\n>> set_pen\n>> teleport_absolute");
        }

        set_pen(false);
        auto teleport_request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
        teleport_request->x = 1.0; teleport_request->y = 1.0; teleport_request->theta = 0.0;
        auto teleport_future = teleport_client_->async_send_request(teleport_request);
        auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(), teleport_future);
        if (result == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Sikeres teleportáció.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Teleportációs hiba.");
        }
        set_pen(true);
        timer_ = this->create_wall_timer(1.85s, std::bind(&DrawNode::draw_track, this));
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DrawNode>());
    rclcpp::shutdown();
    return 0;
}
