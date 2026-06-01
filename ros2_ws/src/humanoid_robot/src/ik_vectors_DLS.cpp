// ============================================================================
//  ik_vectors_DLS.cpp
//  ROS2 node — Wampler 1986 DLS-IK. All math lives in wampler_leg.hpp.
//
//  Subscribed:
//    /com_trajectory   (geometry_msgs/PoseStamped)  100 Hz  best_effort
//    /foot_trajectory  (geometry_msgs/PoseArray)    100 Hz  best_effort
//  Published:
//    /joint_commands   (std_msgs/Float32MultiArray[18], degrees)  100 Hz
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include "humanoid_robot/wampler_leg.hpp"
#include "humanoid_robot/joint_table.hpp"

#include <cmath>
#include <array>

class IKVectorsDLS : public rclcpp::Node
{
public:
    IKVectorsDLS() : Node("ik_vectors_dls")
    {
        declare_parameter("L1",          0.1225);
        declare_parameter("L2",          0.1230);
        declare_parameter("hip_width",   0.0800);
        declare_parameter("com_height",  0.2698);   // single source: robot_params.yaml com_height_m
        // The IK places the HIP at the commanded CoM x, but the true body CoM
        // sits ~1.5cm forward of the hip in a knee-bend (bent knees + torso), so
        // the robot tips forward. Shift the hip target back by this offset to put
        // the real CoM over the feet. Calibrated in sim; verify on hardware.
        declare_parameter("com_x_offset", -0.015);
        declare_parameter("enabled",     true);
        declare_parameter("alpha",       0.01);
        declare_parameter("alpha_max",   0.15);
        declare_parameter("w_thresh",    0.005);
        declare_parameter("max_iter",    6);
        declare_parameter("tol",         0.001);

        load_params();

        joint_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
            "/joint_commands", rclcpp::QoS(1).best_effort());

        auto qos = rclcpp::QoS(1).best_effort();
        com_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/com_trajectory", qos,
            [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {
                com_x_ = m->pose.position.x;
                com_y_ = m->pose.position.y;
                com_z_ = m->pose.position.z;
            });
        foot_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
            "/foot_trajectory", qos,
            [this](geometry_msgs::msg::PoseArray::SharedPtr m) {
                if (m->poses.size() >= 2) {
                    fr_ = {m->poses[0].position.x, m->poses[0].position.y, m->poses[0].position.z};
                    fl_ = {m->poses[1].position.x, m->poses[1].position.y, m->poses[1].position.z};
                }
            });

        timer_ = create_wall_timer(std::chrono::milliseconds(10),
            [this]() { tick(); });

        param_cb_ = add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& /*params*/)
                -> rcl_interfaces::msg::SetParametersResult {
                load_params();
                rcl_interfaces::msg::SetParametersResult r;
                r.successful = true;
                return r;
            });

        reset_to_rest();

        RCLCPP_INFO(get_logger(),
            "DLS-IK node ready | L1=%.4f L2=%.4f hip=%.4f α=%.4f α_max=%.3f",
            L1_, L2_, hip_width_, leg_r_.alpha, leg_r_.alpha_max);
    }

private:
    double L1_, L2_, hip_width_, com_height_;

    WamplerLeg leg_r_, leg_l_;

    double com_x_{0}, com_y_{0}, com_z_{0};
    Vec3   fr_{}, fl_{};

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr com_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr  foot_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

    static constexpr double D2R = M_PI / 180.0;
    static constexpr Vec5 Q_LO = {-50*D2R, -80*D2R,  -5*D2R, -90*D2R, -70*D2R};
    static constexpr Vec5 Q_HI = {120*D2R,  90*D2R, 100*D2R,  80*D2R,  60*D2R};

    void load_params()
    {
        L1_         = get_parameter("L1").as_double();
        L2_         = get_parameter("L2").as_double();
        hip_width_  = get_parameter("hip_width").as_double();
        com_height_ = get_parameter("com_height").as_double();
        for (auto* leg : {&leg_r_, &leg_l_}) {
            leg->L1        = L1_;
            leg->L2        = L2_;
            leg->alpha     = get_parameter("alpha").as_double();
            leg->alpha_max = get_parameter("alpha_max").as_double();
            leg->w_thresh  = get_parameter("w_thresh").as_double();
            leg->max_iter  = static_cast<int>(get_parameter("max_iter").as_int());
            leg->tol       = get_parameter("tol").as_double();
        }
    }

    void reset_to_rest()
    {
        for (auto* leg : {&leg_r_, &leg_l_})
            leg->q = {0.0, 10*D2R, 20*D2R, -10*D2R, 0.0};
        com_x_=0; com_y_=0; com_z_=com_height_;
        fr_ = {0,  hip_width_/2.0, 0};
        fl_ = {0, -hip_width_/2.0, 0};
    }

    void apply_right(std::array<float,18>& a) const
    {
        constexpr double R2D = 180.0/M_PI;
        const Vec5& q = leg_r_.q;
        // 90-centred: servo = 90 + sign*q. Signs preserved from the original
        // mapping (verify in Gazebo, flip a sign if a joint turns the wrong way).
        a[J::R_HIP_ROLL]    = clamp_joint(J::R_HIP_ROLL,    90.0 + q[0]*R2D);
        a[J::R_HIP_PITCH]   = clamp_joint(J::R_HIP_PITCH,   90.0 + q[1]*R2D);
        a[J::R_KNEE_PITCH]  = clamp_joint(J::R_KNEE_PITCH,  90.0 + q[2]*R2D);  // +: q_knee>0 = flexion (URDF knee_pitch +)
        a[J::R_ANKLE_PITCH] = clamp_joint(J::R_ANKLE_PITCH, 90.0 + q[3]*R2D);
        a[J::R_ANKLE_ROLL]  = clamp_joint(J::R_ANKLE_ROLL,  90.0 + q[4]*R2D);
    }

    void apply_left(std::array<float,18>& a) const
    {
        constexpr double R2D = 180.0/M_PI;
        const Vec5& q = leg_l_.q;
        // 90-centred, mirror signs preserved from the original mapping.
        // Left leg uses the SAME joint axes as the right in the URDF (no mirror),
        // so the mapping is identical to apply_right — do NOT negate roll, or the
        // legs splay apart on a lateral weight shift instead of leaning together.
        a[J::L_HIP_ROLL]    = clamp_joint(J::L_HIP_ROLL,    90.0 + q[0]*R2D);
        a[J::L_HIP_PITCH]   = clamp_joint(J::L_HIP_PITCH,   90.0 + q[1]*R2D);
        a[J::L_KNEE_PITCH]  = clamp_joint(J::L_KNEE_PITCH,  90.0 + q[2]*R2D);  // +: q_knee>0 = flexion
        a[J::L_ANKLE_PITCH] = clamp_joint(J::L_ANKLE_PITCH, 90.0 + q[3]*R2D);
        a[J::L_ANKLE_ROLL]  = clamp_joint(J::L_ANKLE_ROLL,  90.0 + q[4]*R2D);
    }

    void tick()
    {
        std::array<float,18> angles;
        for (int i = 0; i < 18; ++i) angles[i] = LIMITS[i].rest;

        if (!get_parameter("enabled").as_bool()) { publish(angles); return; }

        // Shift the hip target back by com_x_offset so the true CoM sits over
        // the feet (the hip is not the CoM in a knee-bend).
        const double hip_x = com_x_ + get_parameter("com_x_offset").as_double();

        Vec3 hip_r = {hip_x, com_y_ + hip_width_/2.0, com_z_};
        leg_r_.solve(hip_r, fr_, Q_LO, Q_HI);
        leg_r_.q[3] = std::clamp(-(leg_r_.q[1]+leg_r_.q[2]), Q_LO[3], Q_HI[3]);
        apply_right(angles);

        Vec3 hip_l = {hip_x, com_y_ - hip_width_/2.0, com_z_};
        leg_l_.solve(hip_l, fl_, Q_LO, Q_HI);
        leg_l_.q[3] = std::clamp(-(leg_l_.q[1]+leg_l_.q[2]), Q_LO[3], Q_HI[3]);
        apply_left(angles);

        publish(angles);
    }

    void publish(const std::array<float,18>& a)
    {
        std_msgs::msg::Float32MultiArray msg;
        msg.data.assign(a.begin(), a.end());
        joint_pub_->publish(msg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IKVectorsDLS>());
    rclcpp::shutdown();
    return 0;
}