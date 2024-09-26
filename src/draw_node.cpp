/*#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>  
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath> 

class TurtleSvgDrawer : public rclcpp::Node
{
public:
    TurtleSvgDrawer() : Node("draw_node")
    {
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&TurtleSvgDrawer::pose_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TurtleSvgDrawer::draw, this));
        svg_file_path_ = "/home/ajr/ros2_ws/src/otv_bf7/img/race_track_outline.svg"; 
        waypoints_ = parse_svg_path(svg_file_path_);
        waypoint_index_ = 0;

        if (waypoints_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No waypoints extracted from the SVG file.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Successfully loaded %ld waypoints.", waypoints_.size());
        }

        current_pose_received_ = false;
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string svg_file_path_;
    std::vector<std::pair<double, double>> waypoints_;
    size_t waypoint_index_;

    turtlesim::msg::Pose current_pose_;
    bool current_pose_received_;

    std::vector<std::pair<double, double>> parse_svg_path(const std::string &svg_file)
    {
        std::vector<std::pair<double, double>> coordinates;
        std::ifstream file(svg_file);
        
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open SVG file");
            return coordinates;
        }

        std::string line;
        while (std::getline(file, line))
        {
    
            size_t d_pos = line.find("d=");
            if (d_pos != std::string::npos)
            {
                size_t start = line.find('"', d_pos);
                size_t end = line.find('"', start + 1);
                std::string path_data = line.substr(start + 1, end - start - 1);

                std::stringstream ss(path_data);
                char command;
                double x, y;

                while (ss >> command >> x >> y)
                {
                    if (command == 'M' || command == 'L') 
                    {
                        coordinates.emplace_back(x, y);
                    }
                }

                break; 
            }
        }

        file.close();
        return coordinates;
    }

    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        current_pose_ = *msg;
        current_pose_received_ = true;
    }

    double calculate_distance(double x1, double y1, double x2, double y2)
    {
        return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    }

    void draw()
    {
        if (!current_pose_received_)
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for turtle's current pose...");
            return;  // addig ne rajzoljon, amíg nem kapott pozíciót (remelhetoleg)
        }

        if (waypoint_index_ >= waypoints_.size())
        {
            RCLCPP_INFO(this->get_logger(), "Drawing completed.");
            return;
        }


        auto [target_x, target_y] = waypoints_[waypoint_index_];

    
        double current_x = current_pose_.x;
        double current_y = current_pose_.y;
        double current_theta = current_pose_.theta;

        double dx = target_x - current_x;
        double dy = target_y - current_y;
        double distance = calculate_distance(current_x, current_y, target_x, target_y);
        double target_theta = std::atan2(dy, dx);

        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = distance * 0.5;  


        twist_msg.angular.z = (target_theta - current_theta) * 2.0; 

        cmd_pub_->publish(twist_msg);
        if (distance < 0.1)  waypoint_index_++;
        
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto draw_node = std::make_shared<TurtleSvgDrawer>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(draw_node);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
*/
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>  
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath> 

class TurtleSvgDrawer : public rclcpp::Node
{
public:
    TurtleSvgDrawer() : Node("draw_node")
    {
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&TurtleSvgDrawer::pose_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TurtleSvgDrawer::draw, this));
        svg_file_path_ = "/home/ajr/ros2_ws/src/otv_bf7/img/race_track_outline.svg"; 
        waypoints_ = parse_svg_path(svg_file_path_);
        waypoint_index_ = 0;

        if (waypoints_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No waypoints extracted from the SVG file.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Successfully loaded %ld waypoints.", waypoints_.size());
        }

        current_pose_received_ = false;
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string svg_file_path_;
    std::vector<std::pair<double, double>> waypoints_;
    size_t waypoint_index_;

    turtlesim::msg::Pose current_pose_;
    bool current_pose_received_;

    std::vector<std::pair<double, double>> parse_svg_path(const std::string &svg_file)
    {
        std::vector<std::pair<double, double>> coordinates;
        std::ifstream file(svg_file);
        
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open SVG file");
            return coordinates;
        }

        std::string line;
        while (std::getline(file, line))
        {
            size_t d_pos = line.find("d=");
            if (d_pos != std::string::npos)
            {
                size_t start = line.find('"', d_pos);
                size_t end = line.find('"', start + 1);
                std::string path_data = line.substr(start + 1, end - start - 1);

                std::stringstream ss(path_data);
                char command;
                double x, y;

                while (ss >> command)
                {
                    if (command == 'M' || command == 'L') 
                    {
                        ss >> x >> y;
                        coordinates.emplace_back(x, y);
                    }
                }

                break; 
            }
        }

        file.close();
        return coordinates;
    }

    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        current_pose_ = *msg;
        current_pose_received_ = true;
    }

    double calculate_distance(double x1, double y1, double x2, double y2)
    {
        return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    }

    void draw()
    {
        if (!current_pose_received_)
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for turtle's current pose...");
            return;  
        }

        if (waypoint_index_ >= waypoints_.size())
        {
            RCLCPP_INFO(this->get_logger(), "Drawing completed.");
            return;
        }

        auto [target_x, target_y] = waypoints_[waypoint_index_];
    
        double current_x = current_pose_.x;
        double current_y = current_pose_.y;
        double current_theta = current_pose_.theta;

        double dx = target_x - current_x;
        double dy = target_y - current_y;
        double distance = calculate_distance(current_x, current_y, target_x, target_y);
        double target_theta = std::atan2(dy, dx);

        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = distance * 0.5;  
        twist_msg.angular.z = (target_theta - current_theta) * 2.0; 

        cmd_pub_->publish(twist_msg);
        if (distance < 0.1) waypoint_index_++;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto draw_node = std::make_shared<TurtleSvgDrawer>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(draw_node);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
