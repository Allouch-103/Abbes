// ============================================================
// main.cpp — Humanoid Robot ESP32 Firmware (PlatformIO)
// ============================================================
//
// Architecture overview:
//
//   setup()
//     │
//     ├── init hardware (I2C, PCA9685, MPU6050)
//     ├── move servos to rest position (slow, safe)
//     └── connect WiFi → start micro-ROS task on Core 1
//
//   loop() — runs on Core 0
//     └── state machine: manages agent connection/reconnection
//
//   microROSTask() — runs on Core 1 (separate FreeRTOS task)
//     └── rclc_executor_spin_some() — processes callbacks:
//           ├── joint_cmd_callback  (on new /joint_commands msg)
//           ├── servo_update_timer  (every 10ms → moves servos)
//           └── imu_publish_timer   (every 20ms → sends /imu)
//
// WHY TWO CORES?
// ESP32 has two cores (Core 0 and Core 1).
// micro-ROS is time-sensitive — it needs regular CPU time.
// By pinning it to Core 1, it never competes with Arduino's
// loop() on Core 0. This prevents dropped messages.
//
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// micro-ROS headers
#include <micro_ros_platformio.h>   // ← PlatformIO version (not micro_ros_arduino.h)
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <sensor_msgs/msg/imu.h>

// Our config files
#include "robot_config.h"
#include "joint_definitions.h"

// ============================================================
// ERROR HANDLING
// ============================================================

// RCCHECK: if any micro-ROS call fails, jump to error state.
// We use a softer version than assert() — it sets a flag so
// loop() can handle reconnection gracefully.
bool microros_error = false;

#define RCCHECK(fn) { \
    rcl_ret_t _ret = (fn); \
    if (_ret != RCL_RET_OK) { \
        Serial.printf("[micro-ROS ERROR] %s failed: %d (line %d)\n", \
                      #fn, (int)_ret, __LINE__); \
        microros_error = true; \
        return; \
    } \
}

// RCSOFTCHECK: log error but don't abort (for non-critical calls)
#define RCSOFTCHECK(fn) { \
    rcl_ret_t _ret = (fn); \
    if (_ret != RCL_RET_OK) { \
        Serial.printf("[micro-ROS WARN] %s returned %d\n", #fn, (int)_ret); \
    } \
}

// ============================================================
// GLOBAL HARDWARE OBJECTS
// ============================================================

// Two PCA9685 boards on I2C bus
Adafruit_PWMServoDriver pca[2] = {
    Adafruit_PWMServoDriver(PCA_ADDR_BOARD0, Wire),
    Adafruit_PWMServoDriver(PCA_ADDR_BOARD1, Wire),
};

// MPU6050 IMU
Adafruit_MPU6050 mpu;
bool imu_available = false;

// ============================================================
// SERVO STATE
// ============================================================

// current_deg: where servos ARE right now (updated every tick)
// target_deg:  where servos SHOULD GO (updated by ROS callback)
//
// The interpolation in servo_update_callback() smoothly moves
// current_deg toward target_deg at MAX_DEG_PER_TICK per tick.
float current_deg[NUM_JOINTS];
float target_deg[NUM_JOINTS];

// ============================================================
// micro-ROS OBJECTS
// ============================================================

// The allocator manages memory for micro-ROS objects.
// We use the default which uses malloc/free on the heap.
rcl_allocator_t  allocator;

// Support: holds the ROS context (DDS middleware state)
rclc_support_t   support;

// The node: our ESP32's identity in the ROS graph
// Visible as "humanoid_esp32" in `ros2 node list`
rcl_node_t       node;

// Executor: runs callbacks (subscriptions + timers)
// Think of it as a mini event loop inside micro-ROS
rclc_executor_t  executor;

// Subscriber: receives /joint_commands from PC
rcl_subscription_t        joint_cmd_sub;
std_msgs__msg__Float32MultiArray joint_cmd_msg;

// Publisher: sends /imu to PC
rcl_publisher_t           imu_pub;
sensor_msgs__msg__Imu     imu_msg;

// Timers: fire at fixed intervals
rcl_timer_t  servo_timer;   // 10ms → moves servos
rcl_timer_t  imu_timer;     // 20ms → publishes IMU

// ============================================================
// AGENT STATE MACHINE
// ============================================================
// micro-ROS requires careful connection management.
// We handle: initial connect, normal operation, reconnect.

enum class AgentState : uint8_t {
    WAITING_WIFI   = 0,  // waiting for WiFi to connect
    WAITING_AGENT  = 1,  // WiFi ok, pinging micro-ROS agent
    CONNECTED      = 2,  // fully operational
    DISCONNECTED   = 3,  // was connected, now lost → will reconnect
};

volatile AgentState agent_state = AgentState::WAITING_WIFI;

// FreeRTOS task handle for micro-ROS task on Core 1
TaskHandle_t microros_task_handle = nullptr;

// ============================================================
// SERVO FUNCTIONS
// ============================================================

/**
 * angleToPWM — Convert degrees to PCA9685 PWM count
 *
 * MATH:
 * PCA9685 runs at 50Hz. One period = 20ms.
 * It has a 12-bit counter: 0 to 4095 counts per period.
 * One count = 20ms / 4096 = 4.8828 µs
 *
 * MG996R pulse range:
 *   500µs  = 0°   → 500 / 4.8828 = 102.4 ≈ 102 counts
 *   2500µs = 180° → 2500 / 4.8828 = 512.0 counts
 *
 * Linear interpolation between these:
 *   pwm = MIN + (angle / 180) × (MAX - MIN)
 *
 * If inverted (servo mounted mirrored):
 *   pwm = MAX - (angle / 180) × (MAX - MIN)
 *   This makes 0° in software = 180° mechanically
 */
uint16_t angleToPWM(float angle, bool inverted) {
    float ratio = angle / 180.0f;
    if (inverted) ratio = 1.0f - ratio;
    return (uint16_t)(SERVO_PWM_MIN + ratio * (float)(SERVO_PWM_MAX - SERVO_PWM_MIN));
}

/**
 * setServo — Immediately move a joint to an angle
 *
 * Safety clamps angle to joint limits before moving.
 * Always use this function, never call pca.setPWM() directly.
 */
void setServo(uint8_t joint_idx, float angle_deg) {
    const JointConfig& j = JOINT_CONFIG[joint_idx];

    // Hard clamp to joint limits — protects mechanics
    angle_deg = constrain(angle_deg, j.min_deg, j.max_deg);

    uint16_t pwm = angleToPWM(angle_deg, j.inverted);
    pca[j.pca_board].setPWM(j.pca_channel, 0, pwm);
}

/**
 * moveToRestPositionBlocking — Slowly move all joints to rest
 *
 * This runs BEFORE micro-ROS starts. It's a blocking function
 * (halts code while executing) because we want the robot in a
 * known safe state before any remote control is possible.
 *
 * We interpolate over 1 second in 50 steps (20ms each).
 * Starting assumption: servos are somewhere near 90° (midpoint).
 */
void moveToRestPositionBlocking() {
    Serial.println("Moving to rest position (1 second)...");
    const int STEPS = 50;
    for (int step = 0; step <= STEPS; step++) {
        float t = (float)step / STEPS;  // 0.0 → 1.0
        for (int i = 0; i < NUM_JOINTS; i++) {
            // Linear interpolate: start(90°) → rest
            float angle = 90.0f + t * (JOINT_CONFIG[i].rest_deg - 90.0f);
            setServo(i, angle);
            current_deg[i] = angle;
        }
        delay(20);
    }
    // Lock in final positions
    for (int i = 0; i < NUM_JOINTS; i++) {
        current_deg[i] = JOINT_CONFIG[i].rest_deg;
        target_deg[i]  = JOINT_CONFIG[i].rest_deg;
    }
    Serial.println("Rest position reached.");
}

// ============================================================
// micro-ROS CALLBACKS
// ============================================================

/**
 * joint_cmd_callback — Called when /joint_commands arrives
 *
 * The PC publishes Float32MultiArray with 18 values.
 * We copy them to target_deg[]. Servos move toward these
 * targets in servo_update_callback() (not here — keep callbacks fast).
 *
 * MESSAGE FORMAT:
 *   data[0]  = joint 0 angle in degrees (r_shoulder_pitch)
 *   data[1]  = joint 1 angle in degrees (r_shoulder_roll)
 *   ...
 *   data[17] = joint 17 angle in degrees (l_ankle_roll)
 */
void joint_cmd_callback(const void* msgin) {
    const auto* msg = (const std_msgs__msg__Float32MultiArray*)msgin;

    if (msg->data.size != NUM_JOINTS) {
        Serial.printf("[WARN] Expected %d joints, got %zu\n",
                      NUM_JOINTS, msg->data.size);
        return;
    }

    for (size_t i = 0; i < NUM_JOINTS; i++) {
        target_deg[i] = msg->data.data[i];
    }
}

/**
 * servo_update_callback — Called every 10ms by servo_timer
 *
 * VELOCITY-LIMITED INTERPOLATION:
 *
 * Each tick, we move current_deg[] toward target_deg[] by at
 * most MAX_DEG_PER_TICK degrees.
 *
 * Example with MAX_DEG_PER_TICK = 1.2°:
 *
 *   tick 0: current=80°, target=90°, error=10°  → move to 81.2°
 *   tick 1: current=81.2°, target=90°, error=8.8° → move to 82.4°
 *   ...
 *   tick 7: current=88.4°, target=90°, error=1.6° → move to 89.6°
 *   tick 8: current=89.6°, target=90°, error=0.4° < 1.2° → snap to 90°
 *
 * This creates a linear velocity ramp (constant speed) rather
 * than an exponential approach. Linear is better for gait control
 * because it makes timing predictable.
 *
 * copysignf(magnitude, sign_source):
 *   Returns magnitude with the sign of sign_source.
 *   copysignf(1.2f, -5.0f) = -1.2f
 *   copysignf(1.2f, +5.0f) = +1.2f
 *   This lets us move toward target in the right direction.
 */
void servo_update_callback(rcl_timer_t* timer, int64_t) {
    if (!timer) return;

    for (int i = 0; i < NUM_JOINTS; i++) {
        float error = target_deg[i] - current_deg[i];

        if (fabsf(error) <= MAX_DEG_PER_TICK) {
            current_deg[i] = target_deg[i];   // close enough: snap
        } else {
            current_deg[i] += copysignf(MAX_DEG_PER_TICK, error);
        }

        setServo(i, current_deg[i]);
    }
}

/**
 * imu_publish_callback — Called every 20ms by imu_timer
 *
 * Reads MPU6050 and publishes sensor_msgs/Imu to /imu topic.
 *
 * COORDINATE FRAME:
 * We publish in the sensor's native frame ("imu_link").
 * The PC-side code will transform to the robot's base frame.
 *
 * MPU6050 raw data:
 *   - acceleration in m/s² (includes gravity component)
 *   - angular velocity in rad/s
 *
 * When robot stands still and upright:
 *   - accel Z ≈ +9.81 m/s² (gravity)
 *   - accel X, Y ≈ 0
 *   - gyro X, Y, Z ≈ 0
 *
 * We'll add a complementary filter on the PC side (Phase 2)
 * to compute actual tilt angles from these raw values.
 */
void imu_publish_callback(rcl_timer_t* timer, int64_t) {
    if (!timer || !imu_available) return;

    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    // Timestamp (micro-ROS fills agent-synchronized time)
    imu_msg.header.stamp.sec     = 0;
    imu_msg.header.stamp.nanosec = 0;

    // Linear acceleration (m/s²) — includes gravity
    imu_msg.linear_acceleration.x = accel.acceleration.x;
    imu_msg.linear_acceleration.y = accel.acceleration.y;
    imu_msg.linear_acceleration.z = accel.acceleration.z;

    // Diagonal of covariance matrix: sensor noise variance
    // MPU6050 typical accel noise: 0.004 m/s²
    // We set the diagonal: [0.004, 0, 0, 0, 0.004, 0, 0, 0, 0.004]
    imu_msg.linear_acceleration_covariance[0] = 0.004f;
    imu_msg.linear_acceleration_covariance[4] = 0.004f;
    imu_msg.linear_acceleration_covariance[8] = 0.004f;

    // Angular velocity (rad/s)
    imu_msg.angular_velocity.x = gyro.gyro.x;
    imu_msg.angular_velocity.y = gyro.gyro.y;
    imu_msg.angular_velocity.z = gyro.gyro.z;

    // MPU6050 typical gyro noise: 0.0003 rad/s
    imu_msg.angular_velocity_covariance[0] = 0.0003f;
    imu_msg.angular_velocity_covariance[4] = 0.0003f;
    imu_msg.angular_velocity_covariance[8] = 0.0003f;

    // Orientation unknown — we don't run a filter on ESP32
    // -1 in first element signals "orientation not provided"
    imu_msg.orientation_covariance[0] = -1.0f;

    RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
}

// ============================================================
// micro-ROS LIFECYCLE
// ============================================================

/**
 * microros_init — Create all ROS2 entities
 * Returns true on success.
 *
 * ORDER MATTERS in micro-ROS initialization:
 *   1. allocator  (memory management)
 *   2. support    (DDS context, requires agent connection)
 *   3. node       (requires support)
 *   4. publisher  (requires node)
 *   5. subscriber (requires node)
 *   6. timers     (requires support)
 *   7. executor   (requires support, must add entities after)
 */
bool microros_init() {
    allocator = rcl_get_default_allocator();

    // Try to ping the agent before allocating anything
    // Timeout 2000ms, 3 retries
    if (rmw_uros_ping_agent(2000, 3) != RMW_RET_OK) {
        Serial.printf("[micro-ROS] Agent not reachable at %s:%d\n",
                      AGENT_IP, AGENT_PORT);
        return false;
    }
    Serial.println("[micro-ROS] Agent found! Initializing...");

    // Initialize support (this establishes the DDS session)
    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
        Serial.println("[micro-ROS] support init failed");
        return false;
    }

    // Create node — shows up as /humanoid_esp32 in ros2 node list
    if (rclc_node_init_default(&node, "humanoid_esp32", "", &support) != RCL_RET_OK) {
        Serial.println("[micro-ROS] node init failed");
        return false;
    }

    // ── Subscriber: /joint_commands ───────────────────────
    // Allocate the float array that will hold incoming data
    joint_cmd_msg.data.capacity = NUM_JOINTS;
    joint_cmd_msg.data.size     = 0;
    joint_cmd_msg.data.data     = (float*)malloc(NUM_JOINTS * sizeof(float));
    if (!joint_cmd_msg.data.data) {
        Serial.println("[micro-ROS] malloc for joint_cmd failed");
        return false;
    }

    if (rclc_subscription_init_best_effort(
            &joint_cmd_sub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
            "/joint_commands") != RCL_RET_OK) {
        Serial.println("[micro-ROS] subscriber init failed");
        return false;
    }

    // ── Publisher: /imu ───────────────────────────────────
    // Allocate frame_id string for the IMU message header
    static char frame_id_buf[20] = "imu_link";
    imu_msg.header.frame_id.data     = frame_id_buf;
    imu_msg.header.frame_id.capacity = sizeof(frame_id_buf);
    imu_msg.header.frame_id.size     = strlen(frame_id_buf);

    if (rclc_publisher_init_best_effort(
            &imu_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
            "/imu") != RCL_RET_OK) {
        Serial.println("[micro-ROS] publisher init failed");
        return false;
    }

    // ── Timer: servo update at 100Hz (every 10ms) ─────────
    if (rclc_timer_init_default2(
            &servo_timer, &support,
            RCL_MS_TO_NS(1000 / SERVO_UPDATE_HZ),
            servo_update_callback,
            true) != RCL_RET_OK) {
        Serial.println("[micro-ROS] servo timer init failed");
        return false;
    }

    // ── Timer: IMU publish at 50Hz (every 20ms) ───────────
    if (rclc_timer_init_default2(
            &imu_timer, &support,
            RCL_MS_TO_NS(1000 / IMU_PUBLISH_HZ),
            imu_publish_callback,
            true) != RCL_RET_OK) {
        Serial.println("[micro-ROS] IMU timer init failed");
        return false;
    }

    // ── Executor: handles 1 subscription + 2 timers ───────
    // capacity = number of handles (sub + timers)
    if (rclc_executor_init(&executor, &support.context, 3, &allocator) != RCL_RET_OK) {
        Serial.println("[micro-ROS] executor init failed");
        return false;
    }

    rclc_executor_add_subscription(
        &executor, &joint_cmd_sub,
        &joint_cmd_msg, &joint_cmd_callback, ON_NEW_DATA);

    rclc_executor_add_timer(&executor, &servo_timer);
    rclc_executor_add_timer(&executor, &imu_timer);

    Serial.println("[micro-ROS] Ready! Topics:");
    Serial.printf("  SUB /joint_commands  (Float32MultiArray, %d values)\n", NUM_JOINTS);
    Serial.println("  PUB /imu             (sensor_msgs/Imu, 50Hz)");

    return true;
}

/**
 * microros_destroy — Clean up all ROS2 entities on disconnect
 *
 * Must be called before re-initializing.
 * Sets session timeout to 0 so destroy completes immediately.
 */
void microros_destroy() {
    // Tell rmw not to wait for session teardown confirmation
    rmw_context_t* rmw_ctx = rcl_context_get_rmw_context(&support.context);
    if (rmw_ctx) {
        rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);
    }

    (void)rcl_publisher_fini(&imu_pub, &node);
    (void)rcl_subscription_fini(&joint_cmd_sub, &node);
    (void)rcl_timer_fini(&servo_timer);
    (void)rcl_timer_fini(&imu_timer);
    (void)rclc_executor_fini(&executor);
    (void)rcl_node_fini(&node);
    (void)rclc_support_fini(&support);

    // Free allocated message memory
    if (joint_cmd_msg.data.data) {
        free(joint_cmd_msg.data.data);
        joint_cmd_msg.data.data = nullptr;
    }

    Serial.println("[micro-ROS] Destroyed. Will reconnect...");
}

// ============================================================
// micro-ROS TASK (runs on Core 1)
// ============================================================

/**
 * microROSTask — FreeRTOS task for micro-ROS
 *
 * This runs forever on Core 1, independent of loop() on Core 0.
 *
 * FreeRTOS TASK BASICS:
 * Tasks are like threads. Each has its own stack and runs
 * concurrently. The ESP32 scheduler allocates CPU time.
 *
 * vTaskDelay(pdMS_TO_TICKS(ms)):
 *   Suspends this task for ms milliseconds, yielding CPU
 *   to other tasks. Essential to avoid starving other tasks.
 */
void microROSTask(void* param) {
    // Configure WiFi transport to the agent IP:port
    // This must be called from within the task that will use it
    // set_microros_wifi_transports() for board_microros_transport = wifi
    // The wifi transport requires IPAddress for the agent IP, not a char*.
    // IPAddress::fromString() parses "192.168.x.x" into the internal format.
    IPAddress agent_ip;
    agent_ip.fromString(AGENT_IP);
    set_microros_wifi_transports(
        (char*)WIFI_SSID,
        (char*)WIFI_PASSWORD,
        agent_ip,
        AGENT_PORT
    );

    // Give WiFi time to connect
    Serial.println("[WiFi] Connecting...");
    int wifi_timeout = 0;
    while (WiFi.status() != WL_CONNECTED && wifi_timeout < 30) {
        vTaskDelay(pdMS_TO_TICKS(500));
        Serial.print(".");
        wifi_timeout++;
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\n[WiFi] FAILED to connect. Check credentials!");
        // Fall through — will keep retrying
    } else {
        Serial.printf("\n[WiFi] Connected! IP: %s\n",
                      WiFi.localIP().toString().c_str());
        agent_state = AgentState::WAITING_AGENT;
    }

    // Main micro-ROS state machine loop
    while (true) {
        switch (agent_state) {

        case AgentState::WAITING_WIFI:
            if (WiFi.status() == WL_CONNECTED) {
                Serial.printf("[WiFi] Connected: %s\n",
                              WiFi.localIP().toString().c_str());
                agent_state = AgentState::WAITING_AGENT;
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
            break;

        case AgentState::WAITING_AGENT:
            // Try to connect to micro-ROS agent
            Serial.printf("[micro-ROS] Pinging agent at %s:%d...\n",
                          AGENT_IP, AGENT_PORT);
            if (microros_init()) {
                agent_state = AgentState::CONNECTED;
                digitalWrite(PIN_LED, HIGH);  // LED on = connected
                Serial.println("[micro-ROS] CONNECTED and ready!");
            } else {
                vTaskDelay(pdMS_TO_TICKS(2000));  // wait before retry
            }
            break;

        case AgentState::CONNECTED:
            // Spin the executor for up to 10ms
            // This processes any pending callbacks (messages + timer fires)
            {
                rcl_ret_t ret = rclc_executor_spin_some(
                    &executor, RCL_MS_TO_NS(10));

                if (ret != RCL_RET_OK && ret != RCL_RET_TIMEOUT) {
                    // Something went wrong — agent probably disconnected
                    Serial.printf("[micro-ROS] spin error: %d\n", (int)ret);
                    agent_state = AgentState::DISCONNECTED;
                }
            }
            // Check agent still alive every ~1 second (100 ticks × 10ms)
            {
                static int ping_counter = 0;
                if (++ping_counter >= 500) {
                    ping_counter = 0;
                    if (rmw_uros_ping_agent(500, 3) != RMW_RET_OK) {
                        Serial.println("[micro-ROS] Agent lost!");
                        agent_state = AgentState::DISCONNECTED;
                    }
                }
            }
            vTaskDelay(pdMS_TO_TICKS(1));  // 1ms yield to other tasks
            break;

        case AgentState::DISCONNECTED:
            digitalWrite(PIN_LED, LOW);
            microros_destroy();
            agent_state = AgentState::WAITING_AGENT;
            vTaskDelay(pdMS_TO_TICKS(1000));
            break;
        }
    }
}

// ============================================================
// SETUP & LOOP
// ============================================================

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n================================");
    Serial.println("  Humanoid Robot ESP32 Firmware ");
    Serial.println("  micro-ROS over WiFi (Jazzy)   ");
    Serial.println("================================\n");

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);

    // ── I2C ──────────────────────────────────────────────
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);  // 400kHz fast mode
    Serial.println("[I2C] Bus initialized (SDA=21, SCL=22, 400kHz)");

    // ── PCA9685 boards ───────────────────────────────────
    Serial.println("[PCA9685] Initializing board 0 (0x40)...");
    pca[0].begin();
    pca[0].setPWMFreq(SERVO_FREQ_HZ);
    delay(10);

    Serial.println("[PCA9685] Initializing board 1 (0x41)...");
    pca[1].begin();
    pca[1].setPWMFreq(SERVO_FREQ_HZ);
    delay(10);

    Serial.println("[PCA9685] Both boards ready");

    // ── MPU6050 ──────────────────────────────────────────
    Serial.println("[MPU6050] Initializing...");
    if (mpu.begin()) {
        mpu.setAccelerometerRange(IMU_ACCEL_RANGE);
        mpu.setGyroRange(IMU_GYRO_RANGE);
        mpu.setFilterBandwidth(IMU_FILTER_BW);
        imu_available = true;
        Serial.println("[MPU6050] Ready (±8g accel, ±500°/s gyro, 21Hz LPF)");
    } else {
        Serial.println("[MPU6050] NOT FOUND — check wiring to GPIO 21/22");
        Serial.println("[MPU6050] Continuing without IMU...");
    }

    // ── Servos to rest position ───────────────────────────
    moveToRestPositionBlocking();

    // ── Start micro-ROS task on Core 1 ───────────────────
    // xTaskCreatePinnedToCore arguments:
    //   function, name, stack size, parameter, priority, handle, core
    //
    // Stack: 8192 bytes — micro-ROS needs a generous stack
    // Priority: 1 — low priority, executor handles timing internally
    // Core: 1 — dedicated core for micro-ROS
    xTaskCreatePinnedToCore(
        microROSTask,          // function
        "micro_ros_task",      // debug name
        8192,                  // stack in bytes
        NULL,                  // parameter (unused)
        1,                     // priority
        &microros_task_handle, // handle (for monitoring)
        1                      // core 1
    );

    Serial.println("[Setup] micro-ROS task started on Core 1");
    Serial.println("[Setup] Waiting for WiFi + agent connection...");
    Serial.printf("[Setup] Expected agent at: %s:%d\n", AGENT_IP, AGENT_PORT);
}

void loop() {
    // Loop runs on Core 0.
    // micro-ROS runs on Core 1 — we don't call it here.
    //
    // Core 0 is free for future use:
    //   - Safety watchdog
    //   - Local button input
    //   - Emergency stop logic
    //   - Status display (OLED/LCD)

    // Simple heartbeat: print connection status every 5 seconds
    static uint32_t last_print = 0;
    if (millis() - last_print > 5000) {
        last_print = millis();
        const char* state_names[] = {
            "WAITING_WIFI", "WAITING_AGENT", "CONNECTED", "DISCONNECTED"
        };
        Serial.printf("[Status] Agent: %s | WiFi: %s | Stack free: %d bytes\n",
                      state_names[(int)agent_state],
                      WiFi.status() == WL_CONNECTED ? "OK" : "DOWN",
                      uxTaskGetStackHighWaterMark(microros_task_handle) * 4);
    }

    delay(100);
}