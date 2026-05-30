# Firmware — ESP32 micro-ROS node (Abbes)

Component context for the on-robot firmware. Loads when I touch files under
`firmware/`. For project-wide context see **`~/Abbes/CLAUDE.md`**; for the ROS2
side see **`~/Abbes/ros2_ws/CLAUDE.md`**.

PlatformIO project. Builds the real-time controller that runs ON the robot's
ESP32 and joins the ROS2 graph as a **micro-ROS node** (`humanoid_esp32`) over
**WiFi/UDP**.

---

## 1. Architecture

- **Framework:** Arduino + PlatformIO (`board = esp32dev`).
- **micro-ROS:** `micro_ros_platformio`, `board_microros_distro = jazzy`,
  `board_microros_transport = wifi_udp`. A micro-ROS **agent** must run on the
  Linux side (at `AGENT_IP:AGENT_PORT`) to bridge the ESP32 into ROS2.
- **Tasking (`main.cpp`):**
    - Core 0: `setup()` + `loop()` — status prints every 5 s (future watchdog).
    - Core 1: `microROSTask` (FreeRTOS) — WiFi, agent lifecycle, executor, callbacks.
- **Modules:**
    - `servo_controller.*` — PCA9685 PWM, per-joint clamp + velocity-limited
      interpolation.
    - `mpu6050_raw.*` — MPU6050 IMU over raw I²C (no Adafruit lib).
    - `microros_transport.cpp` — micro-ROS init/destroy, topics, timers, the
      reconnect state machine (WAITING_WIFI → WAITING_AGENT → CONNECTED →
      DISCONNECTED → …; auto-reconnects if the agent is lost).
    - ⚠️ There are TWO similarly named files: **`microros_transport.cpp`** (the
      real ~10 KB implementation, USE THIS) and **`micro_ros_transport.cpp`**
      (~1.5 KB, looks like a stale older copy — verify/remove before it confuses).

---

## 2. The ROS2 contract (what the node exposes)

- **SUB `/joint_commands`** — `std_msgs/Float32MultiArray`, **best-effort**,
  exactly **`NUM_JOINTS` (18)** values, in **DEGREES**. Copied straight into
  `target_deg[]` (wrong size is dropped with a warning).
- **PUB `/imu`** — `sensor_msgs/Imu`, **best-effort**, **50 Hz**,
  `frame_id="imu_link"`. Carries **accel + gyro only**; orientation is NOT
  fused on-device (`orientation_covariance[0] = -1` flags "no orientation").
  Stamp is left 0 (no time sync). The ROS2-side IMU filter does the fusion.
- **Timers:** servo update **100 Hz** (`SERVO_UPDATE_HZ`), IMU publish **50 Hz**
  (`IMU_PUBLISH_HZ`). Executor = 1 subscription + 2 timers.

**IMPORTANT — this is NOT the same topic the sim uses.** The ROS2
`command_server` publishes `/leg|arm|head_controller/commands` (3 topics,
**radians**, ros2_control). The firmware listens on ONE `/joint_commands`
(**degrees**, different joint ordering). Driving the real robot from the AI/sim
stack would need a **bridge node** that (a) merges the 3 controller arrays into
one 18-vector in THIS file's order and (b) converts radians→degrees. Not built yet.

---

## 3. Joint / servo map (`include/joint_definitions.h`)

`/joint_commands` index order (0-based, **side-grouped**, degrees):

```
 0 r_shoulder_pitch   1 r_shoulder_roll   2 r_elbow_roll
 3 head_yaw           4 camera_pitch
 5 r_hip_roll         6 r_hip_pitch       7 r_knee_pitch
 8 r_ankle_pitch      9 r_ankle_roll
10 l_shoulder_pitch  11 l_shoulder_roll  12 l_elbow_roll
13 l_hip_roll        14 l_hip_pitch      15 l_knee_pitch
16 l_ankle_pitch     17 l_ankle_roll
```

- Indices **0–9 → PCA board 0 (0x40)**, **10–17 → PCA board 1 (0x41)**
  (`pca_channel` = the slot on that board).
- Each joint has `min_deg / max_deg / rest_deg` (from report Table 3.1) and an
  `inverted` flag (flips PWM for mirror-mounted left-side servos; all currently
  `false`). `servo_set()` clamps to [min,max] before driving.
- Note the asymmetry: **l_knee_pitch rest = 70°** vs r_knee_pitch 100° (possible
  mirrored mount — physically verify, may need `inverted=true`).
- `enum JointIndex` gives named constants (`JOINT_R_KNEE_PITCH` = 7, …) — use
  them, not magic numbers.

**This 18-order is the firmware's source of truth and must stay in sync with the
ROS2 side's joint ordering (`humanoid_command_api/pose_definitions.py`) whenever
a sim→hardware bridge is built — they are currently grouped differently.**

---

## 4. Servo + IMU details

- **PWM (`servo_controller.cpp`):** PCA9685 @ `SERVO_FREQ_HZ` (50 Hz). 0–180°
  maps linearly to `SERVO_PWM_MIN..SERVO_PWM_MAX` (MG996R: ~500 µs/0° → ~2500
  µs/180°). `inverted` does `ratio = 1 - ratio`.
- **Motion is velocity-limited:** `servos_interpolation_tick()` (100 Hz) steps
  `current_deg` toward `target_deg` by at most `MAX_DEG_PER_TICK` per tick — so
  commanding a far target ramps smoothly, it doesn't jump. On boot it ramps to
  `rest_deg` over ~1 s (`servos_move_to_rest_blocking`).
- **I²C:** SDA=21, SCL=22, 100 kHz. **IMU:** `mpu6050_init(±8g, ±500°/s, 21 Hz
  DLPF)`; if not found, firmware continues without IMU (sets `imu_available=false`).
- **LED (`PIN_LED`):** ON when agent CONNECTED, OFF otherwise.

⚠️ **CURRENT-STATE GOTCHA:** in `servos_init()` the **board-1 (0x41) init is
commented out** — so right now only board 0 / **joints 0–9 are actually driven**;
left-side joints 10–17 won't move until that block is re-enabled. (Likely a
single-board bench-test leftover.)

---

## 5. Config + secrets (`include/robot_config.h`)

Holds `NUM_JOINTS`, pins (`PIN_LED/SDA/SCL`), PCA addresses, servo PWM/freq
limits, `MAX_DEG_PER_TICK`, `SERVO_UPDATE_HZ`/`IMU_PUBLISH_HZ`, and the WiFi +
agent settings: **`WIFI_SSID`, `WIFI_PASSWORD`, `AGENT_IP`, `AGENT_PORT`**.

🔴 **SECURITY:** `robot_config.h` is **committed to git** (and in history) and
contains the **WiFi password in plaintext**. If this repo is ever shared/public,
rotate the WiFi credentials and consider git-ignoring this file (template +
local override) and purging history. Flag this to the user; do not paste the
secret values anywhere.

---

## 6. Build / flash / run

```bash
cd ~/Abbes/firmware
pio run                       # build
pio run -t upload             # flash over USB (115200)
pio device monitor            # serial monitor (115200, colorized+timestamped)

# On the Linux side, the micro-ROS AGENT must be running for the ESP32 to join:
ros2 run micro_ros_agent micro_ros_agent udp4 --port <AGENT_PORT>
# then on ROS2:  ros2 topic echo /imu   /   ros2 topic pub /joint_commands ...
```
The ESP32 connects to WiFi, pings the agent at `AGENT_IP:AGENT_PORT`, then
advertises `/joint_commands` (sub) and `/imu` (pub). Watch the serial monitor
for the `WAITING_WIFI → WAITING_AGENT → CONNECTED` progression.

`huge_app` partition scheme; micro-ROS stack 16000, task prio 5 (build_flags).

---

## 7. Session handoff (firmware)

**Last updated:** 2026-05-30 (initial scaffold by Claude from reading the code;
NOT yet verified on hardware this session).

**State:** Firmware is written and structured (Phase 3 ✅ in the roadmap). Known
items to confirm / address when firmware work resumes:
1. **Re-enable PCA board 1** in `servos_init()` (only board 0 active now) →
   needed for the left side to move.
2. **Remove the stale `micro_ros_transport.cpp`** if confirmed unused (the live
   one is `microros_transport.cpp`).
3. **WiFi secret in git** (`robot_config.h`) — decide on template + gitignore.
4. **No sim→hardware bridge** yet: `/leg|arm|head_controller/commands` (radians,
   3 topics) → `/joint_commands` (degrees, 18-vector, this file's order).
5. IMU publishes accel+gyro only (no on-device orientation, stamp=0); fusion is
   on the ROS2 side.

**Next concrete action:** confirm with the user what firmware work is intended
before editing (none requested yet — this file was created as part of setting up
the multi-folder CLAUDE.md structure).
