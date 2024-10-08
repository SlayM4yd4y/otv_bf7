#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include "turtlesim/msg/pose.hpp"
#include <chrono>
#include <cmath>

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
    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) {current_pose_ = *msg;}
    void draw_track()
    {
        if (move_index_ < directions_.size())
        {
            double target_x = directions_[move_index_].first;
            double target_y = directions_[move_index_].second;
            double final_target_x = current_pose_.x + target_x;
            double final_target_y = current_pose_.y + target_y;

            auto twist_msg = geometry_msgs::msg::Twist();
            twist_msg.linear.x = target_x;  
            twist_msg.linear.y = target_y; 
            pub_->publish(twist_msg);

            rclcpp::sleep_for(1s); 
            double distance_moved = std::hypot(current_pose_.x - final_target_x, current_pose_.y - final_target_y);
            double required_distance = std::hypot(target_x, target_y);
            if (distance_moved < 0.1 && required_distance > 0.1)
            {
                double remaining_distance_x = target_x - (final_target_x - current_pose_.x);
                double remaining_distance_y = target_y - (final_target_y - current_pose_.y);

                if (remaining_distance_x != 0.0 || remaining_distance_y != 0.0)
                {
                    twist_msg.linear.x = remaining_distance_x;
                    twist_msg.linear.y = remaining_distance_y;
                    pub_->publish(twist_msg);
                }
            }
            else {move_index_++; }
        }
        else
        {
            auto stop_msg = geometry_msgs::msg::Twist();
            pub_->publish(stop_msg);
            RCLCPP_INFO(this->get_logger(), "A pálya rajzolása befejeződött.");
            /*set_pen(false);
            auto teleport_request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
            teleport_request->x = 7.5;teleport_request->y = 5.5;teleport_request->theta = 0.0; 
            auto teleport_client = this->create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute");
            while (!teleport_client->wait_for_service(5s)) {
                RCLCPP_INFO(this->get_logger(), "Varakozas a turtlesim teleport szolgaltatasra...");
            }
            auto teleport_future = teleport_client->async_send_request(teleport_request);
            auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(), teleport_future);
            if (result == rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "Sikeres teleportáció.");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Teleportációs hiba.");
            }
            set_pen(true);
            auto pen_request = std::make_shared<turtlesim::srv::SetPen::Request>();
            pen_request->r = 255;pen_request->g = 0;pen_request->b = 0;pen_request->width = 4;pen_request->off = false;
            auto pen_result = set_pen_client_->async_send_request(pen_request);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), pen_result) != rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "Set_pen szin beallitas hiba.");
            }
            */
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
    turtlesim::msg::Pose current_pose_; 

public:
    DrawNode() : Node("draw_node"), move_index_(0)
    {
        set_pen_client_ = this->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");
        teleport_client_ = this->create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute");
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

        directions_ = {
            {8.0, 0.0}, {0.0, 3.0},
            {-7.0, 0.0}, {0.0, 2.0},
            {4.0, 0.0}, {0.0, 3.0},
            {-4.0, 0.0}, {0.0, 0.0},
            {6.0, 0.0}, {0.0, -3.0},
            {-1.0, 0.0}, {0.0, -1.0},
            {2.0, 0.0}, {0.0, 5.0},
            {-8.0, 0.0}, {0.0, -2.0},
            {4.0, 0.0}, {0.0, -1.0},
            {-4.0, 0.0}, {0.0, -4.0},
            {7.0, 0.0}, {0.0, -1.0},
            {-7.0, 0.0}, {0.0, -1.0}
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
        auto pose_subscriber = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&DrawNode::pose_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(2.25s, std::bind(&DrawNode::draw_track, this));
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DrawNode>());
    rclcpp::shutdown();
    return 0;
}
