// ============================================================
// imu_filter.cpp — Complementary Filter for MPU6050
// ============================================================
//
// Subscribes to raw /imu data from ESP32, applies a
// complementary filter, publishes filtered tilt angles.
//
// THE COMPLEMENTARY FILTER:
//
//   angle = α × (angle_prev + gyro × dt) + (1 - α) × accel_angle
//
//   α = 0.98 → trust gyro 98%, accel 2%
//
// WHY THIS WORKS:
//   Gyroscope:     smooth + accurate short-term, BUT drifts
//   Accelerometer: noisy,  BUT correct on average (no drift)
//   Combined:      smooth + drift-free = best of both
//
// TOPICS:
//   IN:  /imu           (sensor_msgs/Imu, 50Hz from ESP32)
//   OUT: /tilt          (geometry_msgs/Vector3Stamped, radians)
//   OUT: /tilt_degrees  (geometry_msgs/Vector3Stamped, degrees)
//
// BUILD & RUN:
//   cd ~/Abbes/ros2_ws
//   colcon build
//   source install/setup.bash
//   ros2 run humanoid_robot imu_filter
//
// ============================================================

#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

using std::placeholders::_1;

class IMUFilter : public rclcpp::Node
{
public:
    IMUFilter() : Node("imu_filter")
    {
        // ── Declare parameters ────────────────────────────
        // Alpha can be changed at launch:
        //   ros2 run humanoid_robot imu_filter --ros-args -p alpha:=0.96
        //
        // Higher alpha = trust gyro more = smoother but slower to correct drift
        // Lower alpha  = trust accel more = noisier but faster to correct drift
        this->declare_parameter("alpha", 0.98);
        alpha_ = this->get_parameter("alpha").as_double();
        RCLCPP_INFO(this->get_logger(), "Complementary filter alpha = %.3f", alpha_);

        // ── QoS: match ESP32's best-effort publisher ──────
        // The ESP32 publishes /imu with BEST_EFFORT QoS.
        // We MUST subscribe with the same QoS, otherwise
        // ROS2's DDS middleware won't deliver messages to us.
        //
        // BEST_EFFORT: no ACK, no retransmission.
        // Perfect for 50Hz sensor data — if one message is
        // lost, the next one arrives in 20ms anyway.
        rclcpp::QoS qos(10);
        qos.best_effort();

        // ── Subscriber: raw IMU from ESP32 ────────────────
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu",
            qos,
            std::bind(&IMUFilter::imu_callback, this, _1)
        );

        // ── Publishers: filtered tilt angles ──────────────
        // Vector3Stamped.vector:
        //   x = roll   (left/right tilt)
        //   y = pitch  (forward/backward tilt)
        //   z = yaw    (unused — can't determine from accel alone)
        tilt_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/tilt", 10
        );
        tilt_deg_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/tilt_degrees", 10
        );

        RCLCPP_INFO(this->get_logger(), "IMU Filter node started");
        RCLCPP_INFO(this->get_logger(), "  Subscribing to: /imu");
        RCLCPP_INFO(this->get_logger(), "  Publishing to:  /tilt (radians)");
        RCLCPP_INFO(this->get_logger(), "  Publishing to:  /tilt_degrees");
    }

private:
    // ── Filter state ──────────────────────────────────────
    double pitch_ = 0.0;          // radians, + = forward tilt
    double roll_  = 0.0;          // radians, + = left tilt
    double alpha_ = 0.98;         // filter coefficient
    bool initialized_ = false;    // have we received the first message?

    // Time tracking for computing dt
    rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};

    // ROS2 objects
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr tilt_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr tilt_deg_pub_;

    // ── Accelerometer angle calculations ──────────────────
    //
    // MATH — When the robot tilts forward by angle θ:
    //
    //   Gravity vector g = (0, 0, -9.81) in world frame
    //
    //   In the tilted IMU frame:
    //     accel_x = g × sin(θ)    ← gravity's forward component
    //     accel_z = g × cos(θ)    ← gravity's upward component
    //
    //   Therefore:
    //     θ = atan2(accel_x, accel_z)
    //
    // WHY atan2 AND NOT atan?
    //   atan(x/z) fails when z = 0 (division by zero!)
    //   atan(x/z) can't distinguish (x=1,z=1) from (x=-1,z=-1)
    //   atan2(x, z) handles ALL cases correctly, returning
    //   angles in the full range: -π to +π
    //
    // SIGN CONVENTION (ROS REP 103):
    //   Positive pitch = leaning forward
    //   Positive roll  = leaning left

    double accel_pitch(const sensor_msgs::msg::Imu& msg)
    {
        return std::atan2(
            msg.linear_acceleration.x,
            msg.linear_acceleration.z
        );
    }

    double accel_roll(const sensor_msgs::msg::Imu& msg)
    {
        return std::atan2(
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        );
    }

    // ── Main callback ─────────────────────────────────────
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Current time from the node's clock
        rclcpp::Time now = this->get_clock()->now();

        // ── First message: initialize from accelerometer ──
        // On the very first message, we have no "previous time"
        // to compute dt from. So we just set our angles to
        // whatever the accelerometer says and save the timestamp.
        if (!initialized_)
        {
            pitch_ = accel_pitch(*msg);
            roll_  = accel_roll(*msg);
            last_time_ = now;
            initialized_ = true;

            RCLCPP_INFO(this->get_logger(),
                "First IMU message. Initial pitch=%.1f deg, roll=%.1f deg",
                pitch_ * 180.0 / M_PI,
                roll_  * 180.0 / M_PI
            );
            return;
        }

        // ── Compute dt (seconds since last message) ───────
        // Expected: ~0.02s at 50Hz
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        // Safety: skip if dt is unreasonable
        // Catches: clock jumps, long pauses, system hiccups
        if (dt <= 0.0 || dt > 0.5)
        {
            RCLCPP_WARN(this->get_logger(),
                "Unusual dt=%.4f s, skipping this sample", dt);
            return;
        }

        // ── Accelerometer angles (noisy but no drift) ─────
        double ap = accel_pitch(*msg);
        double ar = accel_roll(*msg);

        // ── Gyroscope angular velocities (rad/s) ──────────
        //
        // gyro_x = rotation rate around X axis = roll rate
        // gyro_y = rotation rate around Y axis = pitch rate
        //
        // The gyro tells us HOW FAST we're rotating right now.
        // To get angle change, multiply by dt:
        //   Δangle = angular_velocity × dt
        //
        // Example: gyro_y = 0.5 rad/s, dt = 0.02s
        //   Δpitch = 0.5 × 0.02 = 0.01 rad = 0.57°
        double gyro_x = msg->angular_velocity.x;
        double gyro_y = msg->angular_velocity.y;

        // ═══════════════════════��══════════════════════════
        // THE COMPLEMENTARY FILTER — the core equations
        // ══════════════════════════════════════════════════
        //
        // PITCH:
        //   pitch = α × (pitch_prev + gyro_y × dt) + (1-α) × accel_pitch
        //           ─────────────────────────────────   ──────────────────
        //           Gyro integration (98%)              Accel angle (2%)
        //           Smooth, tracks fast changes         Corrects drift
        //
        // ROLL:
        //   roll  = α × (roll_prev  + gyro_x × dt) + (1-α) × accel_roll
        //
        // What happens each tick:
        //   1. Take our previous best angle estimate
        //   2. Add the gyro's measured rotation (very accurate short-term)
        //   3. Weight that at 98% (trust it a lot)
        //   4. Take the accel's angle (correct but noisy)
        //   5. Weight that at 2% (trust it a little)
        //   6. Sum = new best estimate
        //
        // Over time:
        //   - The 2% accel contribution accumulates and cancels gyro drift
        //   - The 98% gyro contribution smooths out accel noise
        //   - Result: smooth AND drift-free

        pitch_ = alpha_ * (pitch_ + gyro_y * dt) + (1.0 - alpha_) * ap;
        roll_  = alpha_ * (roll_  + gyro_x * dt) + (1.0 - alpha_) * ar;

        // ── Publish filtered angles in radians ────────────
        auto tilt_msg = geometry_msgs::msg::Vector3Stamped();
        tilt_msg.header.stamp = now;
        tilt_msg.header.frame_id = "imu_link";
        tilt_msg.vector.x = roll_;
        tilt_msg.vector.y = pitch_;
        tilt_msg.vector.z = 0.0;
        tilt_pub_->publish(tilt_msg);

        // ── Publish in degrees (easier to read in terminal)
        auto deg_msg = geometry_msgs::msg::Vector3Stamped();
        deg_msg.header.stamp = now;
        deg_msg.header.frame_id = "imu_link";
        deg_msg.vector.x = roll_  * 180.0 / M_PI;
        deg_msg.vector.y = pitch_ * 180.0 / M_PI;
        deg_msg.vector.z = 0.0;
        tilt_deg_pub_->publish(deg_msg);
    }
};

// ── main() ────────────────────────────────────────────────
// Standard ROS2 C++ node entry point.
// rclcpp::init()  → starts ROS2 runtime
// rclcpp::spin()  → blocks here, processing callbacks forever
// rclcpp::shutdown() → cleanup when Ctrl+C is pressed
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUFilter>());
    rclcpp::shutdown();
    return 0;
}