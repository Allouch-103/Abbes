#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cmath>

class JointBridge : public rclcpp::Node
{
public:
    JointBridge() : Node("joint_bridge")
    {
        auto qos = rclcpp::QoS(1).best_effort();

        leg_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/leg_controller/commands", qos);
        arm_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/arm_controller/commands", qos);
        head_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/head_controller/commands", qos);

        sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "/joint_commands", qos,
            [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                if (msg->data.size() < 18) return;
                const auto& d = msg->data;
                constexpr double D2R = M_PI / 180.0;

                std_msgs::msg::Float64MultiArray leg;
                leg.data = {
                    d[5]*D2R,  d[6]*D2R,  d[7]*D2R,  d[8]*D2R,  d[9]*D2R,
                    d[13]*D2R, d[14]*D2R, d[15]*D2R, d[16]*D2R, d[17]*D2R
                };
                leg_pub_->publish(leg);

                std_msgs::msg::Float64MultiArray arm;
                arm.data = {
                    d[0]*D2R, d[1]*D2R, d[2]*D2R,
                    d[10]*D2R, d[11]*D2R, d[12]*D2R
                };
                arm_pub_->publish(arm);

                std_msgs::msg::Float64MultiArray head;
                head.data = { d[3]*D2R, d[4]*D2R };
                head_pub_->publish(head);
            });

        RCLCPP_INFO(get_logger(), "Joint bridge online — degrees→radians, 18→3 controllers");
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr leg_pub_, arm_pub_, head_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointBridge>());
    rclcpp::shutdown();
    return 0;
}
