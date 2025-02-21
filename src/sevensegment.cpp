#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include "math.h"

using namespace std::chrono_literals;

class SevenSegmentDrawer : public rclcpp::Node
{
public:
    SevenSegmentDrawer() : Node("seven_segment_drawer")
    {
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        client_ = this->create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute");
        pen_client_ = this->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");

        draw_seven_segment_display(0.0f);
        draw_seven_segment_display(1.8f);
        draw_seven_segment_display(4.6f);
        draw_seven_segment_display(6.4f);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr client_;
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr pen_client_;

    void move_turtle(float x, float y, float theta)
    {
        auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
        request->x = x;
        request->y = y;
        request->theta = theta;
        auto result = client_->async_send_request(request);
    }

    void set_pen(bool enable)
    {
        auto pen_request = std::make_shared<turtlesim::srv::SetPen::Request>();
        pen_request->r = 0;
        pen_request->g = 0;
        pen_request->b = 0;
        if (!enable) {
            pen_request->width = 0;
            pen_request->off = true;
        } else {
            pen_request->width = 1;
            pen_request->off = false;
        }
        auto result = pen_client_->async_send_request(pen_request);
    }

    void draw_line(float distance, float speed)
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = speed;
        cmd_pub_->publish(msg);
        rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(distance / speed * 1000)));
        msg.linear.x = 0;
        cmd_pub_->publish(msg);
    }

    void draw_seven_segment_display(float offset)
    {
        float start1_x = 0.3f, start1_y = 8.0f;
        float draw_length = 1.5f;
        set_pen(false);
        move_turtle(start1_x + offset, start1_y, 0.0f);
        struct Segment { float x, y, angle; };
        Segment segments[] = {
            { start1_x + offset, start1_y, 0.0f },    // Felső vízszintes
            { start1_x + draw_length + offset, start1_y,  -M_PI/2}, // Jobb felső függőleges
            { start1_x + draw_length + offset, start1_y -draw_length, -M_PI/2 }, // Jobb alsó függőleges
            { start1_x + draw_length + offset, start1_y - 2* draw_length, M_PI }, // Alsó vízszintes
            { start1_x + offset, start1_y - 2* draw_length, M_PI/2 }, // Bal alsó függőleges
            { start1_x + offset, start1_y -draw_length, M_PI/2 },  // Bal felső függőleges
            { start1_x + offset, start1_y - draw_length, 0.0f }   // Középső vízszintes
        };

        for (const auto& segment : segments)
        {
            set_pen(true);
            move_turtle(segment.x, segment.y, segment.angle);
            draw_line(draw_length, 5.0f);
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SevenSegmentDrawer>());
    rclcpp::shutdown();
    return 0;
}
