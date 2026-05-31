#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cmath>

class JointBridge : public rclcpp::Node
{
public:
  JointBridge() : Node("joint_bridge")
  {
    // Controller command publishers must be RELIABLE: ros2_control's
    // forward_command_controller subscribes its ~/commands topic with the
    // default (reliable) QoS, so a best_effort publisher would NOT match and
    // the commands would never reach Gazebo. (This is why the AI path, which
    // publishes reliable, works.)
    rclcpp::QoS pub_qos(10);   // reliable, keep-last 10 — matches the controllers

    leg_pub_  = create_publisher<std_msgs::msg::Float64MultiArray>("/leg_controller/commands",  pub_qos);
    arm_pub_  = create_publisher<std_msgs::msg::Float64MultiArray>("/arm_controller/commands",  pub_qos);
    head_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/head_controller/commands", pub_qos);

    // Subscribe to the corrected stream from balance_controller (reliable pub).
    rclcpp::QoS sub_qos(10);   // reliable — matches balance_controller's publisher
    sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/joint_commands_corrected", sub_qos,
      [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 18) return;
        const auto& d = msg->data;

        // 90-centred convention: incoming values are servo degrees where 90 =
        // URDF-zero (neutral). Gazebo's controllers want URDF radians where
        // 0 = neutral, so subtract the 90 centre BEFORE converting.
        //   servo 90 deg  -> 0 rad   (straight/neutral)
        constexpr double D2R = M_PI / 180.0;
        auto to_rad = [&](size_t i) { return (static_cast<double>(d[i]) - 90.0) * D2R; };

        // Index mapping = URDF ros2_control declaration order (controller-grouped):
        // 0=r_hip_roll, 1=r_hip_pitch, 2=r_knee_pitch, 3=r_ankle_pitch, 4=r_ankle_roll
        // 5=l_hip_roll, 6=l_hip_pitch, 7=l_knee_pitch, 8=l_ankle_pitch, 9=l_ankle_roll
        std_msgs::msg::Float64MultiArray leg;
        leg.data = {
          to_rad(0), to_rad(1), to_rad(2), to_rad(3), to_rad(4),   // right leg
          to_rad(5), to_rad(6), to_rad(7), to_rad(8), to_rad(9)    // left leg
        };
        leg_pub_->publish(leg);

        // 10=r_shoulder_pitch, 11=r_shoulder_roll, 12=r_elbow_roll
        // 13=l_shoulder_pitch, 14=l_shoulder_roll, 15=l_elbow_roll
        std_msgs::msg::Float64MultiArray arm;
        arm.data = {
          to_rad(10), to_rad(11), to_rad(12),   // right arm
          to_rad(13), to_rad(14), to_rad(15)    // left arm
        };
        arm_pub_->publish(arm);

        // 16=head_yaw, 17=camera_pitch
        std_msgs::msg::Float64MultiArray head;
        head.data = { to_rad(16), to_rad(17) };
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