#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy_feedback.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tota_interfaces/msg/set_speed.hpp>

class JoystickControllerNode : public rclcpp::Node
{
public:
    JoystickControllerNode() : Node("joystick_controller_node")
    {
        this->declare_parameter("max_linear_velocity", 1.5);
        this->declare_parameter("max_angular_velocity", 10.0);
        this->declare_parameter("min_linear_velocity", 1.0);
        this->declare_parameter("min_angular_velocity", 1.0);
        this->declare_parameter("control_mode", "speed");
        this->declare_parameter("controller_type","XBOX");
        this->declare_parameter("enable_acceleration", true);
        this->declare_parameter("enable_deceleration", true);
        this->declare_parameter("accs_vx", 1.0);
        this->declare_parameter("deaccs_vx", 1.5);
        this->declare_parameter("accs_w", 3.0);
        this->declare_parameter("deaccs_w", 4.0);
        this->declare_parameter("wheel_separation", 0.038);
        this->declare_parameter("com_height", 0.053);
        this->declare_parameter("stability_margin", 0.4);

        vibrate_pub_        = this->create_publisher<sensor_msgs::msg::JoyFeedback> ("joy/set_feedback", 10);
        arming_pub_         = this->create_publisher<std_msgs::msg::Bool>           ("arming_status", 10);
        jump_pub_           = this->create_publisher<std_msgs::msg::Int8>           ("jump_status", 10);
        vel_pub_            = this->create_publisher<geometry_msgs::msg::Twist>     ("cmd_vel/joystick", 10);
        wall_align_pub_     = this->create_publisher<std_msgs::msg::Bool>           ("wall_align", 10);
        speed_pub_          = this->create_publisher<tota_interfaces::msg::SetSpeed>("set_speed", 10);
        robot_mode_pub_     = this->create_publisher<std_msgs::msg::String>         ("robot/mode/toggle", 10);
        recovery_latch_pub_ = this->create_publisher<std_msgs::msg::Empty>          ("recovery/latch", 10);
        raw_vel_pub_        = this->create_publisher<geometry_msgs::msg::Twist>     ("cmd_vel/raw", 10);
        vel_limit_pub_      = this->create_publisher<geometry_msgs::msg::Twist>     ("limit_cmd_vel", 10);

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
                this->joy_callback(msg);
            }
        );
        
        RCLCPP_INFO(this->get_logger(), "Joystick Controller Node Initialized");
    }
private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // wall_align_pub_->publish(std_msgs::msg::Bool(msg->buttons[wall_align_idx_]));
        handle_recovery(msg);
        handle_robot_detection(msg);
        handle_arming(msg);
        handle_jump(msg);
        handle_boost(msg);
        // if (!all(msg->axes[0:4] == 1.0))
        //     handle_velocity(msg);
    }

    void handle_recovery(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Recovery button pressed");
    }

    void handle_robot_detection(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Robot detection button pressed");
    }

    void handle_arming(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Arming button pressed");
    }

    void handle_jump(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Jump button pressed");
    }

    void handle_boost(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Boost button pressed");
    }

    void handle_velocity(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Velocity button pressed");
    }

    rclcpp::Publisher<sensor_msgs::msg::JoyFeedback>::SharedPtr vibrate_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arming_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr jump_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr wall_align_pub_;
    rclcpp::Publisher<tota_interfaces::msg::SetSpeed>::SharedPtr speed_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_mode_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr recovery_latch_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr raw_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_limit_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoystickControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
