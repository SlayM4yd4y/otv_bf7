#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"

using namespace std::chrono_literals;

class OverlayNode : public rclcpp::Node
{
public:
    OverlayNode() : Node("overlay_node")
    {
        subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&OverlayNode::listener_callback, this, std::placeholders::_1)
        );
        car_image_ = cv::imread("otv_bf7/img/car.png", cv::IMREAD_UNCHANGED);
        if (car_image_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Could not load car.png");
        }

        // Megadjuk a TurtleSim ablak méretét
        window_name_ = "TurtleSim Overlay";
        cv::namedWindow(window_name_);
    }

private:
    void listener_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        int turtle_x = static_cast<int>(msg->x * 10); 
        int turtle_y = static_cast<int>(msg->y * 10); 

        int height = car_image_.rows; // Kép magassága
        int width = car_image_.cols; // Kép szélessége

        // Kép overlay
        int overlay_x = turtle_x - width / 2;  // Kép középre helyezése
        int overlay_y = turtle_y - height;  // Kép a teknős felett

        // Ellenőrizzük, hogy a kép belefér-e az ablakba
        if (overlay_x < 0) overlay_x = 0;
        if (overlay_y < 0) overlay_y = 0;

        // Háttér létrehozása
        cv::Mat background(400, 400, CV_8UC3, cv::Scalar(255, 255, 255));  // Fehér háttér, majd képet is prob.

        // Kép hozzáadása
        if (overlay_x + width <= background.cols && overlay_y + height <= background.rows) {
            car_image_.copyTo(background(cv::Rect(overlay_x, overlay_y, width, height)));
        }

        cv::imshow(window_name_, background);
        cv::waitKey(1);
    }

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    cv::Mat car_image_;
    std::string window_name_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto overlay_node = std::make_shared<OverlayNode>();
    
    rclcpp::spin(overlay_node);
    rclcpp::shutdown();
    return 0;
}
