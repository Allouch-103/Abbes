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

    leg_pub_  = create_publisher<std_msgs::msg::Float64MultiArray>("/leg_controller/commands",  qos);
    arm_pub_  = create_publisher<std_msgs::msg::Float64MultiArray>("/arm_controller/commands",  qos);
    head_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/head_controller/commands", qos);

    // FIX 1: subscribe to corrected topic, not raw /joint_commands
    sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/joint_commands_corrected", qos,
      [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 18) return;
        const auto& d = msg->data;
        constexpr double D2R = M_PI / 180.0;

        // FIX 2: correct index mapping from URDF ros2_control declaration order:
        // 0=r_hip_roll, 1=r_hip_pitch, 2=r_knee_pitch, 3=r_ankle_pitch, 4=r_ankle_roll
        // 5=l_hip_roll, 6=l_hip_pitch, 7=l_knee_pitch, 8=l_ankle_pitch, 9=l_ankle_roll
        std_msgs::msg::Float64MultiArray leg;
        leg.data = {
          d[0]*D2R, d[1]*D2R, d[2]*D2R, d[3]*D2R, d[4]*D2R,   // right leg
          d[5]*D2R, d[6]*D2R, d[7]*D2R, d[8]*D2R, d[9]*D2R    // left leg
        };
        leg_pub_->publish(leg);

        // 10=r_shoulder_pitch, 11=r_shoulder_roll, 12=r_elbow_roll
        // 13=l_shoulder_pitch, 14=l_shoulder_roll, 15=l_elbow_roll
        std_msgs::msg::Float64MultiArray arm;
        arm.data = {
          d[10]*D2R, d[11]*D2R, d[12]*D2R,   // right arm
          d[13]*D2R, d[14]*D2R, d[15]*D2R    // left arm
        };
        arm_pub_->publish(arm);

        // 16=head_yaw, 17=camera_pitch
        std_msgs::msg::Float64MultiArray head;
        head.data = { d[16]*D2R, d[17]*D2R };
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