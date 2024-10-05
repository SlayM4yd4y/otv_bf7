#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include <termios.h>
#include <unistd.h>

class Iranyitas : public rclcpp::Node
{
private:
    char getKey()
    {
        struct termios oldt, newt;
        char c;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        c = getchar();
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        return c;
    }

protected:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

public:
    Iranyitas() : Node("teleop_turtle")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Iranyitas::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "Használja a 'w', 'a', 's', 'd' billentyűket az irányításhoz. Kilépéshez nyomja meg a 'q' billentyűt.");
    }

    void timer_callback()
    {
        auto twist = geometry_msgs::msg::Twist();
        char c = getKey();

        switch (c)
        {
            case 'w':
                twist.linear.x = 0.8;
                break;
            case 's':
                twist.linear.x = -0.8;
                break;
            case 'a':
                twist.angular.z = 0.8;
                break;
            case 'd':
                twist.angular.z = -0.8;
                break;
            case 'q':
                rclcpp::shutdown();
                break;
            default:
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
        }
        publisher_->publish(twist);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iranyitas>());
    rclcpp::shutdown();
    return 0;
}
