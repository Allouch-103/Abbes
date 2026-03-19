// ============================================================
// balance_controller.cpp — Standing Balance via PID (C++17)
// ============================================================
//
// WHAT THIS NODE DOES:
//   Reads filtered tilt angles from /tilt_degrees
//   Reads angular velocity from /imu (gyro = clean D term)
//   Runs two independent PID controllers (pitch + roll)
//   Publishes corrected ankle joint angles to /joint_commands
//
// ARCHITECTURE:
//
//   /tilt_degrees ──► pitch error ──► PID ──► ankle pitch correction
//   /imu          ──► gyro (D term)─┘
//
//   /tilt_degrees ──► roll error  ──► PID ──► ankle roll  correction
//   /imu          ──► gyro (D term)─┘
//
//   corrections → add to REST_POSITION → /joint_commands → ESP32
//
// WHY ANKLES?
//   Ankle is the base of the kinematic chain.
//   Correcting there moves the whole body with minimal
//   CoM disturbance. Hip corrections would be much more
//   disruptive to balance.
//
// TOPICS:
//   IN:  /tilt_degrees  (geometry_msgs/Vector3Stamped)
//          x = roll  (deg), y = pitch (deg)
//   IN:  /imu          (sensor_msgs/Imu)
//          angular_velocity.x = roll rate  (rad/s)
//          angular_velocity.y = pitch rate (rad/s)
//   OUT: /joint_commands (std_msgs/Float32MultiArray, 18 floats)
//
// BUILD:
//   cd ~/Abbes/ros2_ws && colcon build --packages-select humanoid_robot
//
// RUN:
//   ros2 run humanoid_robot balance_controller
//
// LIVE TUNING (no recompile):
//   ros2 param set /balance_controller kp_pitch 0.8
//   ros2 param set /balance_controller kd_pitch 0.3
//   ros2 param set /balance_controller enabled false
//
// ============================================================

#include <cmath>
#include <memory>
#include <array>
#include <algorithm>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using std::placeholders::_1;

// ============================================================
// ROBOT CONFIGURATION
// Must match firmware joint_definitions.h exactly.
// ============================================================

static constexpr int NUM_JOINTS = 18;

// Joint index constants — use these instead of raw numbers
enum JointIndex : int {
    R_SHOULDER_PITCH = 0,
    R_SHOULDER_ROLL  = 1,
    R_ELBOW_ROLL     = 2,
    HEAD_YAW         = 3,
    CAMERA_PITCH     = 4,
    R_HIP_ROLL       = 5,
    R_HIP_PITCH      = 6,
    R_KNEE_PITCH     = 7,
    R_ANKLE_PITCH    = 8,   // ← pitch PID corrects this
    R_ANKLE_ROLL     = 9,   // ← roll  PID corrects this
    L_SHOULDER_PITCH = 10,
    L_SHOULDER_ROLL  = 11,
    L_ELBOW_ROLL     = 12,
    L_HIP_ROLL       = 13,
    L_HIP_PITCH      = 14,
    L_KNEE_PITCH     = 15,
    L_ANKLE_PITCH    = 16,  // ← pitch PID corrects this
    L_ANKLE_ROLL     = 17,  // ← roll  PID corrects this
};

// Rest position (degrees) — robot standing upright, no correction
// PID corrections are ADDED to these values
static constexpr std::array<float, NUM_JOINTS> REST_POSITION = {
    90.0f,  // 0:  r_shoulder_pitch
    90.0f,  // 1:  r_shoulder_roll
    90.0f,  // 2:  r_elbow_roll
    90.0f,  // 3:  head_yaw
    90.0f,  // 4:  camera_pitch
    50.0f,  // 5:  r_hip_roll
    80.0f,  // 6:  r_hip_pitch
   100.0f,  // 7:  r_knee_pitch
   100.0f,  // 8:  r_ankle_pitch  ← balance target
    90.0f,  // 9:  r_ankle_roll   ← balance target
    90.0f,  // 10: l_shoulder_pitch
    90.0f,  // 11: l_shoulder_roll
    90.0f,  // 12: l_elbow_roll
    50.0f,  // 13: l_hip_roll
    80.0f,  // 14: l_hip_pitch
    70.0f,  // 15: l_knee_pitch
    90.0f,  // 16: l_ankle_pitch  ← balance target
    90.0f,  // 17: l_ankle_roll   ← balance target
};

// Joint limits (degrees) — hard safety clamp, from Table 3.1
static constexpr std::array<std::pair<float, float>, NUM_JOINTS> JOINT_LIMITS = {{
    {0.0f,  180.0f},  // 0:  r_shoulder_pitch
    {0.0f,  180.0f},  // 1:  r_shoulder_roll
    {0.0f,  180.0f},  // 2:  r_elbow_roll
    {0.0f,  180.0f},  // 3:  head_yaw
    {0.0f,  180.0f},  // 4:  camera_pitch
    {0.0f,  170.0f},  // 5:  r_hip_roll
    {0.0f,  170.0f},  // 6:  r_hip_pitch
    {0.0f,  170.0f},  // 7:  r_knee_pitch
    {10.0f, 180.0f},  // 8:  r_ankle_pitch
    {20.0f, 150.0f},  // 9:  r_ankle_roll
    {0.0f,  180.0f},  // 10: l_shoulder_pitch
    {0.0f,  180.0f},  // 11: l_shoulder_roll
    {0.0f,  180.0f},  // 12: l_elbow_roll
    {0.0f,  170.0f},  // 13: l_hip_roll
    {0.0f,  170.0f},  // 14: l_hip_pitch
    {0.0f,  170.0f},  // 15: l_knee_pitch
    {10.0f, 180.0f},  // 16: l_ankle_pitch
    {20.0f, 150.0f},  // 17: l_ankle_roll
}};

// ============================================================
// PID CONTROLLER CLASS
// ============================================================

class PIDController
{
public:
    PIDController(double kp, double ki, double kd,
                  double integral_limit = 10.0)
        : kp_(kp), ki_(ki), kd_(kd)
        , integral_limit_(integral_limit)
        , integral_(0.0)
    {}

    /**
     * update() — compute PID output for one timestep
     *
     * @param error   angle error in degrees (target - measured)
     *                + = robot leans away from target
     *
     * @param gyro    angular velocity in rad/s from IMU
     *                Used directly as D term — avoids differentiating
     *                noisy angle signal.
     *                Sign convention: positive gyro = tilting in the
     *                direction that INCREASES the error (i.e., falling
     *                further from target). The D term brakes this motion.
     *
     * @param dt      timestep in seconds
     *
     * @return correction in degrees to add to joint angle
     *
     * MATH:
     *   P = Kp × error
     *   I = Ki × ∫error dt    (with anti-windup clamp)
     *   D = -Kd × ω_deg       (gyro in deg/s, negative = damping)
     *   u = P + I + D
     */
    double update(double error, double gyro_rad_s, double dt)
    {
        // ── P term ───────────────────────────────────────
        double p = kp_ * error;

        // ── I term with anti-windup ───────────────────────
        // Accumulate error × dt (trapezoidal would be better
        // but rectangular is fine at 50Hz)
        integral_ += error * dt;

        // Anti-windup: clamp integral to ±integral_limit_
        // Prevents integrator from winding up when robot
        // is held at a tilt for a long time.
        integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);
        double i = ki_ * integral_;

        // ── D term using gyroscope ────────────────────────
        // Convert rad/s → deg/s for consistent units with error
        // Negative sign: if tilting toward error (positive gyro),
        // apply braking force (negative correction)
        double gyro_deg_s = gyro_rad_s * (180.0 / M_PI);
        double d = -kd_ * gyro_deg_s;

        return p + i + d;
    }

    void set_gains(double kp, double ki, double kd)
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    void reset() { integral_ = 0.0; }

private:
    double kp_, ki_, kd_;
    double integral_limit_;
    double integral_;
};

// ============================================================
// BALANCE CONTROLLER NODE
// ============================================================

class BalanceController : public rclcpp::Node
{
public:
    BalanceController()
    : Node("balance_controller")
    , pid_pitch_(0.5, 0.02, 0.2)
    , pid_roll_ (0.5, 0.02, 0.2)
    , gyro_x_(0.0), gyro_y_(0.0)
    , current_pitch_deg_(0.0), current_roll_deg_(0.0)
    , imu_received_(false), tilt_received_(false)
    , last_time_(this->get_clock()->now())
    , initialized_(false)
    {
        // ── Declare parameters ────────────────────────────
        // All tunable gains exposed as ROS2 parameters.
        // Change them live with: ros2 param set /balance_controller kp_pitch 0.8
        //
        // TUNING ORDER (with robot hanging/supported, not free-standing):
        //   1. enabled=false → verify robot publishes rest position
        //   2. enabled=true, all gains=0 → should still be rest position
        //   3. Increase kp_pitch slowly → robot should resist forward push
        //      Stop when it starts oscillating, then back off 20%
        //   4. Add kd_pitch → damps oscillations
        //   5. Add tiny ki_pitch (0.01-0.05) → removes steady-state lean
        //   6. Repeat for roll axis

        this->declare_parameter("kp_pitch",          0.5);
        this->declare_parameter("ki_pitch",          0.02);
        this->declare_parameter("kd_pitch",          0.2);
        this->declare_parameter("kp_roll",           0.5);
        this->declare_parameter("ki_roll",           0.02);
        this->declare_parameter("kd_roll",           0.2);
        this->declare_parameter("max_correction_deg", 5.0);
        this->declare_parameter("target_pitch_deg",   0.0);
        this->declare_parameter("target_roll_deg",    0.0);
        this->declare_parameter("enabled",            true);

        load_params();

        // ── Parameter change callback ─────────────────────
        param_cb_handle_ = this->add_on_set_parameters_callback(
            std::bind(&BalanceController::on_param_change, this, _1)
        );

        // ── QoS — BEST_EFFORT matches ESP32 publisher ────
        // If we use RELIABLE here and ESP32 uses BEST_EFFORT,
        // ROS2 DDS won't match them and we receive nothing.
        rclcpp::QoS sensor_qos(10);
        sensor_qos.best_effort();

        // ── Subscribers ───────────────────────────────────
        tilt_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/tilt_degrees", sensor_qos,
            std::bind(&BalanceController::tilt_callback, this, _1)
        );

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", sensor_qos,
            std::bind(&BalanceController::imu_callback, this, _1)
        );

        // ── Publisher ─────────────────────────────────────
        // RELIABLE QoS for commands — we want these delivered
        cmd_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/joint_commands", 10
        );

        // ── Control loop timer — 50Hz ─────────────────────
        // Same rate as IMU filter output. No benefit running faster.
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&BalanceController::control_loop, this)
        );

        RCLCPP_INFO(this->get_logger(), "Balance Controller started (C++)");
        RCLCPP_INFO(this->get_logger(),
            "Pitch PID: Kp=%.3f Ki=%.3f Kd=%.3f",
            kp_pitch_, ki_pitch_, kd_pitch_);
        RCLCPP_INFO(this->get_logger(),
            "Roll  PID: Kp=%.3f Ki=%.3f Kd=%.3f",
            kp_roll_, ki_roll_, kd_roll_);
        RCLCPP_INFO(this->get_logger(),
            "Live tuning: ros2 param set /balance_controller kp_pitch 0.8");
    }

private:

    // ── PID controllers ───────────────────────────────────
    PIDController pid_pitch_;
    PIDController pid_roll_;

    // ── Sensor state ──────────────────────────────────────
    double gyro_x_;             // rad/s — roll  rate from /imu
    double gyro_y_;             // rad/s — pitch rate from /imu
    double current_pitch_deg_;  // degrees from /tilt_degrees
    double current_roll_deg_;   // degrees from /tilt_degrees
    bool   imu_received_;
    bool   tilt_received_;

    // ── Timing ────────────────────────────────────────────
    rclcpp::Time last_time_;
    bool         initialized_;

    // ── Parameters ────────────────────────────────────────
    double kp_pitch_, ki_pitch_, kd_pitch_;
    double kp_roll_,  ki_roll_,  kd_roll_;
    double max_correction_;
    double target_pitch_;
    double target_roll_;
    bool   enabled_;

    // ── ROS2 objects ──────────────────────────────────────
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr tilt_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr              imu_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr       cmd_pub_;
    rclcpp::TimerBase::SharedPtr                                         timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr    param_cb_handle_;

    // ── load_params() ─────────────────────────────────────
    void load_params()
    {
        kp_pitch_       = this->get_parameter("kp_pitch").as_double();
        ki_pitch_       = this->get_parameter("ki_pitch").as_double();
        kd_pitch_       = this->get_parameter("kd_pitch").as_double();
        kp_roll_        = this->get_parameter("kp_roll").as_double();
        ki_roll_        = this->get_parameter("ki_roll").as_double();
        kd_roll_        = this->get_parameter("kd_roll").as_double();
        max_correction_ = this->get_parameter("max_correction_deg").as_double();
        target_pitch_   = this->get_parameter("target_pitch_deg").as_double();
        target_roll_    = this->get_parameter("target_roll_deg").as_double();
        enabled_        = this->get_parameter("enabled").as_bool();
    }

    // ── on_param_change() ─────────────────────────────────
    // Called automatically when ros2 param set is used.
    // Updates gains live without restarting the node.
    rcl_interfaces::msg::SetParametersResult
    on_param_change(const std::vector<rclcpp::Parameter>& params)
    {
        (void)params;  // we reload all at once
        load_params();
        pid_pitch_.set_gains(kp_pitch_, ki_pitch_, kd_pitch_);
        pid_roll_.set_gains(kp_roll_,  ki_roll_,  kd_roll_);
        RCLCPP_INFO(this->get_logger(),
            "Params updated — Pitch: Kp=%.3f Ki=%.3f Kd=%.3f | "
            "Roll: Kp=%.3f Ki=%.3f Kd=%.3f",
            kp_pitch_, ki_pitch_, kd_pitch_,
            kp_roll_,  ki_roll_,  kd_roll_);
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    // ── tilt_callback() ───────────────────────────────────
    // Receives filtered angles from imu_filter node.
    // Vector3Stamped convention (set in imu_filter.cpp):
    //   x = roll  (left/right tilt, degrees)
    //   y = pitch (forward/backward tilt, degrees)
    void tilt_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
    {
        current_roll_deg_  = msg->vector.x;
        current_pitch_deg_ = msg->vector.y;
        tilt_received_ = true;
    }

    // ── imu_callback() ────────────────────────────────────
    // Receives raw IMU — we only need the gyroscope here.
    // angular_velocity is in rad/s (ROS convention):
    //   x = roll  rate
    //   y = pitch rate
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        gyro_x_ = msg->angular_velocity.x;
        gyro_y_ = msg->angular_velocity.y;
        imu_received_ = true;
    }

    // ── publish_angles() ──────────────────────────────────
    // Clamps each joint to its limits then publishes.
    void publish_angles(std::array<float, NUM_JOINTS> angles)
    {
        for (int i = 0; i < NUM_JOINTS; ++i) {
            angles[i] = std::clamp(
                angles[i],
                JOINT_LIMITS[i].first,
                JOINT_LIMITS[i].second
            );
        }

        std_msgs::msg::Float32MultiArray msg;
        msg.data.assign(angles.begin(), angles.end());
        cmd_pub_->publish(msg);
    }

    // ── control_loop() ────────────────────────────────────
    // Runs every 20ms (50Hz). This is the heart of Phase 3.
    //
    // EACH TICK:
    //   1. Check sensors ready
    //   2. Compute dt
    //   3. Compute pitch and roll errors
    //   4. Run PID for each axis → corrections in degrees
    //   5. Clamp corrections to ±max_correction_
    //   6. Apply corrections to ankle joints only
    //   7. Publish all 18 joint angles
    void control_loop()
    {
        // Wait until we have data from both sensors
        if (!tilt_received_ || !imu_received_) {
            // Still publish rest position so robot doesn't go limp
            publish_angles(REST_POSITION);
            return;
        }

        // ── Compute dt ────────────────────────────────────
        auto now = this->get_clock()->now();
        if (!initialized_) {
            last_time_   = now;
            initialized_ = true;
            publish_angles(REST_POSITION);
            return;
        }

        double dt = (now - last_time_).seconds();
        last_time_ = now;

        // Guard against bad dt values
        // (clock jump, first tick, long pause)
        if (dt <= 0.0 || dt > 0.5) {
            RCLCPP_WARN_THROTTLE(this->get_logger(),
                *this->get_clock(), 2000,
                "Unusual dt=%.4f s, skipping tick", dt);
            return;
        }

        // ── If disabled, hold rest position ───────────────
        // Useful for debugging: verify rest position is correct
        // before enabling active balance.
        if (!enabled_) {
            pid_pitch_.reset();
            pid_roll_.reset();
            publish_angles(REST_POSITION);
            return;
        }

        // ── Compute tilt errors ───────────────────────────
        // error = target - measured
        // Positive pitch error: robot leans backward → need forward correction
        // Negative pitch error: robot leans forward  → need backward correction
        double pitch_error = target_pitch_ - current_pitch_deg_;
        double roll_error  = target_roll_  - current_roll_deg_;

        // ── Run PID controllers ───────────────────────────
        // gyro_y_ = pitch angular velocity (rad/s)
        // gyro_x_ = roll  angular velocity (rad/s)
        // The PID uses gyro as the D term for clean damping.
        double pitch_correction = pid_pitch_.update(pitch_error, gyro_y_, dt);
        double roll_correction  = pid_roll_.update(roll_error,  gyro_x_, dt);

        // ── Clamp corrections ─────────────────────────────
        // Safety: never move ankles more than max_correction_ from rest.
        // Start with 5° — enough to feel the effect but not violent.
        // Increase to 10-15° once robot is tuned and stable.
        pitch_correction = std::clamp(pitch_correction,
                                      -max_correction_, max_correction_);
        roll_correction  = std::clamp(roll_correction,
                                      -max_correction_, max_correction_);

        // ── Build joint command array ─────────────────────
        // Start from rest position, then apply corrections
        // to ankle joints only. All other joints unchanged.
        auto angles = REST_POSITION;

        // Pitch correction: same magnitude to both ankles.
        // When the robot leans forward, both ankles need to
        // push backward equally.
        angles[R_ANKLE_PITCH] += static_cast<float>(pitch_correction);
        angles[L_ANKLE_PITCH] += static_cast<float>(pitch_correction);

        // Roll correction: opposite signs for left and right ankle.
        // When robot leans right, right ankle pushes in, left pushes out.
        // The sign depends on your servo mounting — flip if correction
        // goes the wrong way.
        angles[R_ANKLE_ROLL] += static_cast<float>(roll_correction);
        angles[L_ANKLE_ROLL] -= static_cast<float>(roll_correction);

        // ── Log at reduced rate (once per second) ─────────
        RCLCPP_DEBUG_THROTTLE(this->get_logger(),
            *this->get_clock(), 1000,
            "Pitch: err=%.2f° corr=%.2f° | Roll: err=%.2f° corr=%.2f°",
            pitch_error, pitch_correction,
            roll_error,  roll_correction);

        // ── Publish (with joint limit clamping inside) ────
        publish_angles(angles);
    }
};

// ============================================================
// MAIN
// ============================================================

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BalanceController>());
    rclcpp::shutdown();
    return 0;
}