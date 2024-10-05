#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class OverlayNode : public rclcpp::Node
{
private:
    void display_text()
    {
        cv::Mat background(300, 600, CV_8UC3, cv::Scalar(255, 255, 255));
        std::string text = "1.) lepes:\nros2 run otv_bf7 draw_node\n2.) lepes:\nros2 run otv_bf7 iranyitas\n\nA jatek celja:\nEljutni a startbol a celba.\nJo jatekot!\n\n Ezen ablak bezarasa:\n ESC billentyu";
        int fontFace = cv::FONT_HERSHEY_SIMPLEX;
        double fontScale = 0.6;
        int thickness = 1;

        int baseline = 0; int y_offset = 30;  
        std::istringstream text_stream(text);
        std::string line;
        while (std::getline(text_stream, line)) {
            cv::Size textSize = cv::getTextSize(line, fontFace, fontScale, thickness, &baseline);
            cv::Point textOrg((background.cols - textSize.width) / 2, y_offset);  
            cv::putText(background, line, textOrg, fontFace, fontScale, cv::Scalar(0, 0, 0), thickness, 8);
            y_offset += textSize.height + 10;  
        }

        cv::imshow(window_name_, background);
    }
    
protected:
    std::string window_name_;

public:
    OverlayNode() : Node("overlay_node")
    {
        window_name_ = "Jatekszabalyzat";
        cv::namedWindow(window_name_);
    }

    void run()
    {
        while (rclcpp::ok()) {
            display_text();
            int key = cv::waitKey(1);  
            if (key == 27) {  
                break; 
            }
            if (cv::getWindowProperty(window_name_, cv::WND_PROP_VISIBLE) == 0) {
                break;  
            }
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto overlay_node = std::make_shared<OverlayNode>();
    overlay_node->run(); 
    rclcpp::shutdown();
    return 0;
}
