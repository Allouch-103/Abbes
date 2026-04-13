// ============================================================================
//  inverse_kinematics.cpp
//  Phase 4 — Analytical geometric IK for 18-DOF humanoid robot
//
//  Subscribes:
//    /com_trajectory   (geometry_msgs/PoseStamped)  100 Hz  best_effort
//    /foot_trajectory  (geometry_msgs/PoseArray)    100 Hz  best_effort
//
//  Publishes:
//    /joint_commands   (std_msgs/Float32MultiArray)  100 Hz  reliable
//      array[18]: one float per servo, in degrees, ordered by Table 3.1
//
//  Algorithm — two decoupled 2D problems per leg:
//
//    Sagittal plane (side view):
//      Step A) Knee via law of cosines
//      Step B) Hip via atan2
//      Step C) Ankle = -(hip + knee)  <-- keeps foot horizontal
//
//    Frontal plane (front view):
//      q_hip_roll   = atan2(delta_y, L_leg)
//      q_ankle_roll = -q_hip_roll
//
//  All angles are in radians internally; converted to degrees before publish.
//  Servo offset from Table 3.1 is added before publish.
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <cmath>
#include <array>
#include <algorithm>

// ── Joint index constants (from Table 3.1) ──────────────────────────────────
namespace J {
  constexpr int R_SHOULDER_PITCH = 0;
  constexpr int R_SHOULDER_ROLL  = 1;
  constexpr int R_ELBOW_ROLL     = 2;
  constexpr int HEAD_YAW         = 3;
  constexpr int CAMERA_PITCH     = 4;
  constexpr int R_HIP_ROLL       = 5;
  constexpr int R_HIP_PITCH      = 6;
  constexpr int R_KNEE_PITCH     = 7;
  constexpr int R_ANKLE_PITCH    = 8;
  constexpr int R_ANKLE_ROLL     = 9;
  constexpr int L_SHOULDER_PITCH = 10;
  constexpr int L_SHOULDER_ROLL  = 11;
  constexpr int L_ELBOW_ROLL     = 12;
  constexpr int L_HIP_ROLL       = 13;
  constexpr int L_HIP_PITCH      = 14;
  constexpr int L_KNEE_PITCH     = 15;
  constexpr int L_ANKLE_PITCH    = 16;
  constexpr int L_ANKLE_ROLL     = 17;
}

// ── Joint limits from Table 3.1 [min, max, rest] ────────────────────────────
struct JointLimit { float min, max, rest; };
constexpr std::array<JointLimit, 18> LIMITS = {{
  {  0, 180, 90 },  //  0  r_shoulder_pitch
  {  0, 180, 90 },  //  1  r_shoulder_roll
  {  0, 180, 90 },  //  2  r_elbow_roll
  {  0, 180, 90 },  //  3  head_yaw
  {  0, 180, 90 },  //  4  camera_pitch
  {  0, 170, 50 },  //  5  r_hip_roll
  {  0, 170, 80 },  //  6  r_hip_pitch
  {  0, 170,100 },  //  7  r_knee_pitch
  { 10, 180,100 },  //  8  r_ankle_pitch
  { 20, 150, 90 },  //  9  r_ankle_roll
  {  0, 180, 90 },  // 10  l_shoulder_pitch
  {  0, 180, 90 },  // 11  l_shoulder_roll
  {  0, 180, 90 },  // 12  l_elbow_roll
  {  0, 170, 50 },  // 13  l_hip_roll
  {  0, 170, 80 },  // 14  l_hip_pitch
  {  0, 170, 70 },  // 15  l_knee_pitch  (asymmetric rest — opposite mounting)
  { 10, 180, 90 },  // 16  l_ankle_pitch
  { 20, 150, 90 },  // 17  l_ankle_roll
}};

// ── Small helper ─────────────────────────────────────────────────────────────
inline float clamp_joint(int idx, double deg)
{
  return static_cast<float>(
    std::clamp(deg,
               static_cast<double>(LIMITS[idx].min),
               static_cast<double>(LIMITS[idx].max)));
}

// ── IK result structs ─────────────────────────────────────────────────────────
struct SagAngles {
  double hip   = 0.0;   // q_hip_pitch  [rad]
  double knee  = 0.0;   // q_knee_pitch [rad]
  double ankle = 0.0;   // q_ankle_pitch[rad]
  bool   valid = false;
};

struct FroAngles {
  double hip_roll   = 0.0;   // [rad]
  double ankle_roll = 0.0;   // [rad]
};

// ── Node ─────────────────────────────────────────────────────────────────────
class InverseKinematics : public rclcpp::Node
{
public:
  InverseKinematics() : Node("inverse_kinematics")
  {
    // ── Declare parameters (can be overridden by YAML) ──────────────────────
    declare_parameter("L1",         0.1225);   // thigh length [m]
    declare_parameter("L2",         0.1230);   // shank length [m]
    declare_parameter("hip_width",  0.0800);   // hip separation L–R [m]
    declare_parameter("com_height", 0.1900);   // LIPM z_c [m]
    declare_parameter("enabled",    true);
    // Safety: maximum geometric angle the knee is allowed to reach (deg).
    // Prevents near-singular fully-extended pose.
    declare_parameter("knee_min_deg", 5.0);

    load_params();

    // ── Publisher ────────────────────────────────────────────────────────────
    //  /joint_commands is RELIABLE — matches ESP32 subscriber.
    joint_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      "/joint_commands", rclcpp::QoS(1).reliable());

    // ── Subscribers ──────────────────────────────────────────────────────────
    //  Both trajectories are BEST_EFFORT at 100 Hz.
    //  depth=1: we always want the latest value, never queue stale frames.
    auto qos = rclcpp::QoS(1).best_effort();

    com_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/com_trajectory", qos,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr m){ com_cb(m); });

    foot_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/foot_trajectory", qos,
      [this](const geometry_msgs::msg::PoseArray::SharedPtr m){ foot_cb(m); });

    // ── 100 Hz timer ─────────────────────────────────────────────────────────
    timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      [this](){ compute_and_publish(); });

    // ── Parameter change callback (live tuning without relaunch) ─────────────
    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& params)
        -> rcl_interfaces::msg::SetParametersResult {
          load_params();
          rcl_interfaces::msg::SetParametersResult r;
          r.successful = true;
          return r;
        });

    // ── Initialise state to standing rest pose ───────────────────────────────
    reset_to_rest();

    RCLCPP_INFO(get_logger(),
      "IK node started | L1=%.4fm  L2=%.4fm  hip_w=%.4fm  com_h=%.4fm",
      L1_, L2_, hip_width_, com_height_);
  }

private:
  // ── Parameters ───────────────────────────────────────────────────────────
  double L1_, L2_, hip_width_, com_height_, knee_min_rad_;

  // ── Current world-frame state ─────────────────────────────────────────────
  //   CoM: (x, y, z)   — z = com_height (constant in LIPM)
  //   Foot: world-frame position (x, y, z)  — z=0 when on ground
  double com_x_{0}, com_y_{0}, com_z_{0};
  double fr_x_{0}, fr_y_{0}, fr_z_{0};   // right foot
  double fl_x_{0}, fl_y_{0}, fl_z_{0};   // left  foot

  // ── ROS handles ───────────────────────────────────────────────────────────
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr com_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr foot_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  // ── Helper: load / reload parameters ─────────────────────────────────────
  void load_params()
  {
    L1_          = get_parameter("L1").as_double();
    L2_          = get_parameter("L2").as_double();
    hip_width_   = get_parameter("hip_width").as_double();
    com_height_  = get_parameter("com_height").as_double();
    double kmin  = get_parameter("knee_min_deg").as_double();
    knee_min_rad_= kmin * M_PI / 180.0;
  }

  // ── Helper: set state to the standing rest pose ───────────────────────────
  void reset_to_rest()
  {
    com_x_ = 0.0;  com_y_ = 0.0;  com_z_ = com_height_;
    fr_x_  = 0.0;  fr_y_  = +hip_width_ / 2.0;  fr_z_ = 0.0;
    fl_x_  = 0.0;  fl_y_  = -hip_width_ / 2.0;  fl_z_ = 0.0;
  }

  // ── Callbacks ─────────────────────────────────────────────────────────────
  void com_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    com_x_ = msg->pose.position.x;
    com_y_ = msg->pose.position.y;
    com_z_ = msg->pose.position.z;   // should equal com_height_ in LIPM
  }

  void foot_cb(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    // Convention: poses[0] = right foot, poses[1] = left foot
    if (msg->poses.size() >= 2) {
      fr_x_ = msg->poses[0].position.x;
      fr_y_ = msg->poses[0].position.y;
      fr_z_ = msg->poses[0].position.z;
      fl_x_ = msg->poses[1].position.x;
      fl_y_ = msg->poses[1].position.y;
      fl_z_ = msg->poses[1].position.z;
    }
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  SAGITTAL IK
  //
  //  Input: foot target in hip frame
  //    xf = forward  (+ = in front of hip)
  //    zf = downward (+ = toward ground, always positive for normal walking)
  //
  //  Output: hip_pitch, knee_pitch, ankle_pitch [radians]
  //
  //  Geometry:
  //    The hip, knee, and foot form a triangle.
  //    We know all three side lengths: L1, L2, d=sqrt(xf²+zf²).
  //    Law of cosines → knee angle.
  //    atan2 → hip angle.
  //    Foot-horizontal constraint → ankle angle.
  // ─────────────────────────────────────────────────────────────────────────
  SagAngles solve_sagittal(double xf, double zf)
  {
    SagAngles r;

    // Straight-line distance hip → foot
    double d = std::sqrt(xf * xf + zf * zf);
    const double d_max = (L1_ + L2_) * 0.99;   // 1% margin from singularity
    const double d_min = 0.01;                   // sanity lower bound

    if (d < d_min) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "IK: foot too close to hip (d=%.4fm). Skipping.", d);
      return r;   // valid=false
    }
    if (d > d_max) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "IK: foot unreachable (d=%.4fm > max=%.4fm). Clamping.", d, d_max);
      // Scale (xf, zf) inward so d = d_max
      double scale = d_max / d;
      xf *= scale;
      zf *= scale;
      d = d_max;
    }

    // ── Step A: Knee angle via law of cosines ──────────────────────────────
    //   c_k = (L1² + L2² - d²) / (2·L1·L2)
    //   q_knee = atan2(√(1-c_k²), c_k)
    //
    //   Using atan2(sin,cos) instead of acos:
    //     • works for all quadrants
    //     • more numerically stable near ±1
    //     • always returns q_knee ≥ 0  (knee bent forward = human-like)
    double c_k = (L1_ * L1_ + L2_ * L2_ - d * d) / (2.0 * L1_ * L2_);
    c_k = std::clamp(c_k, -1.0, 1.0);    // guard against tiny float overflow
    double q_knee = std::atan2(std::sqrt(1.0 - c_k * c_k), c_k);

    // Enforce minimum knee bend to avoid near-singular fully-extended pose
    q_knee = std::max(q_knee, knee_min_rad_);

    // ── Step B: Hip pitch ──────────────────────────────────────────────────
    //   γ = angle of foot vector (hip→foot) from vertical downward axis
    //   β = angle inside the hip-knee-foot triangle at the hip vertex
    //   q_hip = γ - β
    double gamma = std::atan2(xf, zf);
    double beta  = std::atan2(L2_ * std::sin(q_knee),
                               L1_ + L2_ * std::cos(q_knee));
    double q_hip = gamma - beta;

    // ── Step C: Ankle pitch — keeps foot horizontal ────────────────────────
    //   The foot segment must stay parallel to the ground.
    //   Sum of all pitch rotations around the Z-axis (sagittal) = 0:
    //       q_hip + q_knee + q_ankle = 0
    //   ∴  q_ankle = -(q_hip + q_knee)
    //
    //   This was MISSING from the original Selmi report.
    //   Without it: foot tilts forward = robot tips on every step.
    double q_ankle = -(q_hip + q_knee);

    r.hip   = q_hip;
    r.knee  = q_knee;
    r.ankle = q_ankle;
    r.valid = true;
    return r;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  FRONTAL IK
  //
  //  Input: delta_y = foot_y - hip_y  (lateral offset, + = outward for right)
  //
  //  Geometry:
  //    The CoM shifts laterally to load a foot.
  //    The leg forms a right triangle in the frontal plane:
  //      adjacent = L_leg (leg length, approximated as constant)
  //      opposite = delta_y
  //    q_hip_roll = atan2(delta_y, L_leg)
  //    q_ankle_roll = -q_hip_roll  (ankle compensates to keep foot flat)
  // ─────────────────────────────────────────────────────────────────────────
  FroAngles solve_frontal(double delta_y)
  {
    FroAngles r;
    double L_leg = L1_ + L2_;
    r.hip_roll   = std::atan2(delta_y, L_leg);
    r.ankle_roll = -r.hip_roll;
    return r;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  FRAME TRANSFORM: world → hip frame
  //
  //  The ZMP controller publishes CoM and foot positions in the world frame
  //  (origin = starting position, z = up).
  //  The IK needs the foot position relative to the hip joint.
  //
  //  Hip world position:
  //    hip_x = com_x
  //    hip_y = com_y ± hip_width/2   (right: +, left: -)
  //    hip_z = com_z                  (hip approximately at CoM height)
  //
  //  Foot in hip frame:
  //    xf = foot_x - hip_x           (forward, sagittal)
  //    yf = foot_y - hip_y           (lateral, frontal)
  //    zf = hip_z  - foot_z          (downward, + = foot below hip — FLIP z)
  // ─────────────────────────────────────────────────────────────────────────

  // ─────────────────────────────────────────────────────────────────────────
  //  SERVO OFFSET CONVERSION
  //
  //  Geometric angles (radians from straight leg = 0°) must be mapped to
  //  the servo's physical range using the rest angles from Table 3.1.
  //
  //  Right leg:
  //    r_hip_roll   [5]:  θ = 50  + q_hip_roll   * (180/π)
  //    r_hip_pitch  [6]:  θ = 80  + q_hip_pitch  * (180/π)
  //    r_knee_pitch [7]:  θ = 100 - q_knee_pitch * (180/π)  ← MINUS (inverted)
  //    r_ankle_pitch[8]:  θ = 100 + q_ankle_pitch* (180/π)
  //    r_ankle_roll [9]:  θ = 90  + q_ankle_roll * (180/π)
  //
  //  Left leg (mirror-mounted → roll signs flipped):
  //    l_hip_roll   [13]: θ = 50  - q_hip_roll   * (180/π)
  //    l_hip_pitch  [14]: θ = 80  + q_hip_pitch  * (180/π)
  //    l_knee_pitch [15]: θ = 70  - q_knee_pitch * (180/π)  ← rest=70, MINUS
  //    l_ankle_pitch[16]: θ = 90  + q_ankle_pitch* (180/π)
  //    l_ankle_roll [17]: θ = 90  - q_ankle_roll * (180/π)
  //
  //  NOTE: Signs must be verified in Gazebo after URDF import.
  //        If a joint moves the wrong way, flip the sign here.
  // ─────────────────────────────────────────────────────────────────────────

  // ── Main 100 Hz loop ──────────────────────────────────────────────────────
  void compute_and_publish()
  {
    const double R2D = 180.0 / M_PI;

    // Start with rest positions for all 18 joints
    std::array<float, 18> angles;
    for (int i = 0; i < 18; ++i) {
      angles[i] = LIMITS[i].rest;
    }

    // If disabled, publish rest pose and return
    if (!get_parameter("enabled").as_bool()) {
      publish(angles);
      return;
    }

    // ═══════════════════════════════
    //  RIGHT LEG
    // ═══════════════════════════════

    // Hip joint position in world frame
    const double hip_r_x = com_x_;
    const double hip_r_y = com_y_ + hip_width_ / 2.0;
    const double hip_r_z = com_z_;   // LIPM: hip ≈ at com height

    // Foot in hip frame (z is flipped: "downward" = +)
    const double xf_r = fr_x_ - hip_r_x;          // forward
    const double yf_r = fr_y_ - hip_r_y;          // lateral
    const double zf_r = hip_r_z - fr_z_;          // downward (hip − foot)

    auto sag_r = solve_sagittal(xf_r, zf_r);
    auto fro_r = solve_frontal(yf_r);

    if (sag_r.valid) {
      angles[J::R_HIP_ROLL]    = clamp_joint(J::R_HIP_ROLL,
                                    50.0 + fro_r.hip_roll   * R2D);
      angles[J::R_HIP_PITCH]   = clamp_joint(J::R_HIP_PITCH,
                                    80.0 + sag_r.hip        * R2D);
      angles[J::R_KNEE_PITCH]  = clamp_joint(J::R_KNEE_PITCH,
                                   100.0 - sag_r.knee       * R2D);  // inverted
      angles[J::R_ANKLE_PITCH] = clamp_joint(J::R_ANKLE_PITCH,
                                   100.0 + sag_r.ankle      * R2D);
      angles[J::R_ANKLE_ROLL]  = clamp_joint(J::R_ANKLE_ROLL,
                                    90.0 + fro_r.ankle_roll * R2D);
    }

    // ═══════════════════════════════
    //  LEFT LEG
    // ═══════════════════════════════

    const double hip_l_x = com_x_;
    const double hip_l_y = com_y_ - hip_width_ / 2.0;
    const double hip_l_z = com_z_;

    const double xf_l = fl_x_ - hip_l_x;
    const double yf_l = fl_y_ - hip_l_y;
    const double zf_l = hip_l_z - fl_z_;

    auto sag_l = solve_sagittal(xf_l, zf_l);
    auto fro_l = solve_frontal(yf_l);

    if (sag_l.valid) {
      // Roll is NEGATED for left leg (mirror mounting)
      angles[J::L_HIP_ROLL]    = clamp_joint(J::L_HIP_ROLL,
                                    50.0 - fro_l.hip_roll   * R2D);
      angles[J::L_HIP_PITCH]   = clamp_joint(J::L_HIP_PITCH,
                                    80.0 + sag_l.hip        * R2D);
      angles[J::L_KNEE_PITCH]  = clamp_joint(J::L_KNEE_PITCH,
                                    70.0 - sag_l.knee       * R2D);  // rest=70, inverted
      angles[J::L_ANKLE_PITCH] = clamp_joint(J::L_ANKLE_PITCH,
                                    90.0 + sag_l.ankle      * R2D);
      angles[J::L_ANKLE_ROLL]  = clamp_joint(J::L_ANKLE_ROLL,
                                    90.0 - fro_l.ankle_roll * R2D);  // negated
    }

    // Arms and head keep rest position (indices 0-4, 10-12 stay at 90°)

    publish(angles);
  }

  void publish(const std::array<float, 18>& a)
  {
    std_msgs::msg::Float32MultiArray msg;
    msg.data.assign(a.begin(), a.end());
    joint_pub_->publish(msg);
  }
};

// ── Entry point ───────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InverseKinematics>());
  rclcpp::shutdown();
  return 0;
}