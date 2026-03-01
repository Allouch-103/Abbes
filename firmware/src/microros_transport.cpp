// ============================================================
// microros_transport.cpp — micro-ROS lifecycle implementation
// ============================================================

#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <sensor_msgs/msg/imu.h>

#include "microros_transport.h"
#include "robot_config.h"
#include "joint_definitions.h"
#include "servo_controller.h"
#include "mpu6050_raw.h"

// ── Error macros ───────────────────────────────────────────
#define RCSOFTCHECK(fn) { \
    rcl_ret_t _ret = (fn); \
    if (_ret != RCL_RET_OK) { \
        Serial.printf("[micro-ROS WARN] %s returned %d\n", #fn, (int)_ret); \
    } \
}

// ── Shared state ───────────────────────────────────────────
volatile AgentState agent_state = AgentState::WAITING_WIFI;
TaskHandle_t microros_task_handle = nullptr;

// IMU availability flag
extern bool imu_available;

// ── micro-ROS objects ──────────────────────────────────────
static rcl_allocator_t  allocator;
static rclc_support_t   support;
static rcl_node_t       node;
static rclc_executor_t  executor;

static rcl_subscription_t        joint_cmd_sub;
static std_msgs__msg__Float32MultiArray joint_cmd_msg;

static rcl_publisher_t           imu_pub;
static sensor_msgs__msg__Imu     imu_msg;

static rcl_timer_t  servo_timer;
static rcl_timer_t  imu_timer;

// ── Callbacks ──────────────────────────────────────────────

static void joint_cmd_callback(const void* msgin) {
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

static void servo_update_callback(rcl_timer_t* timer, int64_t) {
    if (!timer) return;
    servos_interpolation_tick();
}

static void imu_publish_callback(rcl_timer_t* timer, int64_t) {
    if (!timer || !imu_available) return;

    IMUData imu;
    if (!mpu6050_read(imu)) return;

    imu_msg.header.stamp.sec     = 0;
    imu_msg.header.stamp.nanosec = 0;

    imu_msg.linear_acceleration.x = imu.accel_x;
    imu_msg.linear_acceleration.y = imu.accel_y;
    imu_msg.linear_acceleration.z = imu.accel_z;

    imu_msg.linear_acceleration_covariance[0] = 0.004f;
    imu_msg.linear_acceleration_covariance[4] = 0.004f;
    imu_msg.linear_acceleration_covariance[8] = 0.004f;

    imu_msg.angular_velocity.x = imu.gyro_x;
    imu_msg.angular_velocity.y = imu.gyro_y;
    imu_msg.angular_velocity.z = imu.gyro_z;

    imu_msg.angular_velocity_covariance[0] = 0.0003f;
    imu_msg.angular_velocity_covariance[4] = 0.0003f;
    imu_msg.angular_velocity_covariance[8] = 0.0003f;

    imu_msg.orientation_covariance[0] = -1.0f;

    RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
}

// ── Lifecycle ──────────────────────────────────────────────

static bool microros_init() {
    allocator = rcl_get_default_allocator();

    if (rmw_uros_ping_agent(2000, 3) != RMW_RET_OK) {
        Serial.printf("[micro-ROS] Agent not reachable at %s:%d\n",
                      AGENT_IP, AGENT_PORT);
        return false;
    }
    Serial.println("[micro-ROS] Agent found! Initializing...");

    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
        Serial.println("[micro-ROS] support init failed");
        return false;
    }

    if (rclc_node_init_default(&node, "humanoid_esp32", "", &support) != RCL_RET_OK) {
        Serial.println("[micro-ROS] node init failed");
        return false;
    }

    // Subscriber: /joint_commands
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

    // Publisher: /imu
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

    // Timer: servo update at 100Hz
    if (rclc_timer_init_default2(
            &servo_timer, &support,
            RCL_MS_TO_NS(1000 / SERVO_UPDATE_HZ),
            servo_update_callback,
            true) != RCL_RET_OK) {
        Serial.println("[micro-ROS] servo timer init failed");
        return false;
    }

    // Timer: IMU publish at 50Hz
    if (rclc_timer_init_default2(
            &imu_timer, &support,
            RCL_MS_TO_NS(1000 / IMU_PUBLISH_HZ),
            imu_publish_callback,
            true) != RCL_RET_OK) {
        Serial.println("[micro-ROS] IMU timer init failed");
        return false;
    }

    // Executor: 1 subscription + 2 timers
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

static void microros_destroy() {
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

    if (joint_cmd_msg.data.data) {
        free(joint_cmd_msg.data.data);
        joint_cmd_msg.data.data = nullptr;
    }

    Serial.println("[micro-ROS] Destroyed. Will reconnect...");
}

// ── FreeRTOS task ──────────────────────────────────────────

static void microROSTask(void* param) {
    Serial.println("[Task] microROSTask started on Core 1"); 
    IPAddress agent_ip;
    agent_ip.fromString(AGENT_IP);
    Serial.println("[WiFi] Calling set_microros_wifi_transports...");
    set_microros_wifi_transports(
        (char*)WIFI_SSID,
        (char*)WIFI_PASSWORD,
        agent_ip,
        AGENT_PORT
    );
    Serial.println("[WiFi] Transport configured, waiting for connection...");

    Serial.println("[WiFi] Connecting...");
    int wifi_timeout = 0;
    while (WiFi.status() != WL_CONNECTED && wifi_timeout < 30) {
        vTaskDelay(pdMS_TO_TICKS(500));
        Serial.printf("  WiFi status: %d (timeout %d/30)\n", WiFi.status(), wifi_timeout);
        Serial.print(".");
        wifi_timeout++;
    }


    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\n[WiFi] FAILED to connect. Check credentials!");
    } else {
        Serial.printf("\n[WiFi] Connected! IP: %s\n",
                      WiFi.localIP().toString().c_str());
        agent_state = AgentState::WAITING_AGENT;
    }

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
            Serial.printf("[micro-ROS] Pinging agent at %s:%d...\n",
                          AGENT_IP, AGENT_PORT);
            if (microros_init()) {
                agent_state = AgentState::CONNECTED;
                digitalWrite(PIN_LED, HIGH);
                Serial.println("[micro-ROS] CONNECTED and ready!");
            } else {
                vTaskDelay(pdMS_TO_TICKS(2000));
            }
            break;

        case AgentState::CONNECTED:
            {
                rcl_ret_t ret = rclc_executor_spin_some(
                    &executor, RCL_MS_TO_NS(10));

                if (ret != RCL_RET_OK && ret != RCL_RET_TIMEOUT) {
                    Serial.printf("[micro-ROS] spin error: %d\n", (int)ret);
                    agent_state = AgentState::DISCONNECTED;
                }
            }
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
            vTaskDelay(pdMS_TO_TICKS(1));
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

void microros_start_task() {
    xTaskCreatePinnedToCore(
        microROSTask,
        "micro_ros_task",
        8192,
        NULL,
        1,
        &microros_task_handle,
        1
    );
    Serial.println("[Setup] micro-ROS task started on Core 1");
}