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

        draw_seven_segment_display(0.0f, 8);
        draw_seven_segment_display(1.8f, 9);
        draw_seven_segment_display(4.6f, 0);
        draw_seven_segment_display(6.4f, 5);
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

    void set_pen(bool enable, bool red)
    {
        auto pen_request = std::make_shared<turtlesim::srv::SetPen::Request>();
        pen_request->r = red ? 255 : 0;
        pen_request->g = 0;
        pen_request->b = 0;
        pen_request->width = enable ? 3 : 0;
        pen_request->off = !enable;
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

    void draw_seven_segment_display(float offset, int digit)
    {
        float start1_x = 0.3f, start1_y = 8.0f;
        float draw_length = 1.5f;
        set_pen(false, false);
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
    
        bool segment_map[10][7] = {
            {1, 1, 1, 1, 1, 1, 0}, // 0
            {0, 1, 1, 0, 0, 0, 0}, // 1
            {1, 1, 0, 1, 1, 0, 1}, // 2
            {1, 1, 1, 1, 0, 0, 1}, // 3
            {0, 1, 1, 0, 0, 1, 1}, // 4
            {1, 0, 1, 1, 0, 1, 1}, // 5
            {1, 0, 1, 1, 1, 1, 1}, // 6
            {1, 1, 1, 0, 0, 0, 0}, // 7
            {1, 1, 1, 1, 1, 1, 1}, // 8
            {1, 1, 1, 1, 0, 1, 1}  // 9
        };
    
        for (int i = 0; i < 7; ++i)
        {
            if (i == 6) {
                set_pen(false, false);
                move_turtle(segments[i].x, segments[i].y, segments[i].angle);
            }
            set_pen(true, segment_map[digit][i]); 
            move_turtle(segments[i].x, segments[i].y, segments[i].angle);
            draw_line(draw_length, 5.0f);
        }
        set_pen(false, false);
        move_turtle(4.05f, 6.5f, M_PI/2);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SevenSegmentDrawer>());
    rclcpp::shutdown();
    return 0;
}
