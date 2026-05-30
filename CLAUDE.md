# Humanoid Robot "Abbes" — Project Memory (ROOT)

This is the **project-wide hub**, loaded at the start of every session started
from `~/Abbes`. It holds context shared by the whole project. Component-specific
detail lives in nested CLAUDE.md files, which load automatically when I touch
files in those folders:

- **`ros2_ws/CLAUDE.md`** — ROS2 workspace: packages, conventions, and the
  AI/natural-language phase (Steps 1–9) + its session handoff.
- **`firmware/CLAUDE.md`** — ESP32 micro-ROS firmware: the on-robot real-time
  node, joint/servo map, build/flash, and its session handoff.

> Keep THIS file lean — it costs context every session. Deep, fast-changing
> detail belongs in the nested files, not here.

---

## 1. What this project is

PFE (Projet de Fin d'Études) at INSAT, Tunisia, by Yassine Allouch. A
from-scratch, child-sized **18-DOF humanoid robot** named **Abbes**, built
end-to-end: mechanical CAD → 3D-printed parts → electronics → ESP32 firmware →
kinematics/balance → Gazebo simulation → and an AI/natural-language layer for
high-level control. Designed to walk, balance, gesture, and respond to commands.

---

## 2. Hardware

- **Frame:** 3D-printed, designed in Onshape. ~18 DOF.
- **Joint layout (18 servos):**
    - 2× legs × 5 DOF = 10 (hip_roll, hip_pitch, knee_pitch, ankle_pitch,
      ankle_roll, per leg)
    - 2× arms × 3 DOF = 6 (shoulder_pitch, shoulder_roll, elbow_roll)
    - Head: head_yaw, camera_pitch = 2
- **Actuators:** hobby servos (MG996R-class) driven by **2× PCA9685** I²C PWM
  boards (addresses 0x40 and 0x41).
- **Brain (on-robot):** **ESP32**, runs the real-time control loop as a
  **micro-ROS node over WiFi/UDP** (see `firmware/CLAUDE.md`).
- **Sensors:** **MPU6050 IMU** (accel + gyro, raw I²C); planned head camera.
- **Power:** off-board for now; battery version is post-PFE.

---

## 3. Software stack

- **OS:** Ubuntu 24.04 LTS. **ROS2:** Jazzy Jalisco.
- **Simulator:** Gazebo Harmonic, bridged via `gz_ros2_control`.
- **On-robot:** Arduino/PlatformIO + **micro-ROS** (`micro_ros_platformio`,
  distro jazzy, `wifi_udp` transport). A micro-ROS **agent** runs on the Linux
  side to bridge the ESP32 into the ROS2 graph.
- **Languages:** C++ for real-time (ROS2 control nodes + ESP32 firmware),
  Python for orchestration (command API, LLM dispatcher, web UI).
- **LLM stack:** local **Ollama** (`llama3.2:3b`), tool-use / function-calling
  for command grounding. No cloud dependency.

---

## 4. The big picture — how the pieces connect

```
  You (web UI / voice)  ──►  LLM dispatcher (Ollama tool-use)  ──►  command API
        │                                                              │ actions
        │                                          ┌───────────────────┴────────┐
        ▼                                          ▼                            ▼
  ros2_ws (Python/C++)                      Gazebo (sim)                  ESP32 (real)
                                       /leg|arm|head_controller      micro-ROS node:
                                          /commands (radians)        SUB /joint_commands
                                                                     PUB /imu
```

**KNOWN INTEGRATION GAP (verify before relying on it):** the ROS2 side drives
the *simulator* via three controllers (`/leg|arm|head_controller/commands`, in
**radians**), but the *firmware* listens on a single `/joint_commands`
(Float32MultiArray, 18 values, in **degrees**, side-grouped order). These are
**different topics, units, and orderings** — so there is no automatic sim→real
path yet; a bridge node (reorder + radians→degrees) would be needed. Details in
`firmware/CLAUDE.md`.

---

## 5. Project phases — overall view

```
Mechanical / low-level (NOT the AI phase):
  Phase 1  Mechanical design (Onshape) .................... ✅
  Phase 2  3D printing + assembly ......................... ✅
  Phase 3  Electronics + ESP32 firmware ................... ✅
  Phase 4  IMU + balance controller ....................... ✅
  Phase 5  ZMP walking gait generator ..................... 🔄 in progress
  Phase 6  Gazebo simulation parity ....................... ✅ for non-walking
  Phase 7  Reinforcement learning for gait ................ 📝 post-PFE

AI intelligence phase:
  Steps 1–7 ✅ (vocabulary, command API, Ollama, dispatcher, safety,
                feedback loop, web UI) — see ros2_ws/CLAUDE.md
  Step 8  Voice Input (Whisper) [BONUS] ................... ⏳
  Step 9  Graphic Path Picker [BONUS] .................... ⏳
```

Phase 5 (walking) is independent of the AI phase: the `walk` action currently
stubs to "march in place"; the real ZMP gait swaps in behind the same interface.

---

## 6. Cross-cutting conventions (do not violate)

- **Units boundary.** ROS2-side works in **radians** (URDF zero convention).
  The **ESP32 firmware works in degrees** (0–180 servo range) and expects
  `/joint_commands` already in degrees — so any radians→degrees conversion must
  happen on the ROS2 side, NOT on the ESP32. (NOTE: an older convention note
  said "conversion happens only in firmware" — that is inaccurate for the
  micro-ROS firmware; treat the boundary as described here and verify when a
  real sim→hardware bridge is built.)
- **The LLM never generates motor commands.** It picks from a fixed vocabulary
  and emits JSON; the dispatcher is the only translator.
- **ROS2 interfaces live in `humanoid_msgs`**, never inline one-offs.
- **Actions for goal-driven motions; topics for streaming data** (IMU,
  joint_states).

---

## 7. Critical system constraints (READ before installing/downloading)

- **Root `/` is small (`/dev/nvme0n1p6`, 46 GB) and often near-full.** Do NOT
  download/install anything large to `/`. Check `df -h /` before any op >~100 MB.
- **External USB disk** at **`/mnt/ollama_disk`** (440 GB ext4, label
  `OllamaModels`) holds ALL LLM models / large caches / AI downloads.
  ⚠️ **It has repeatedly become unmounted** (at boot and while idle) — when that
  happens Ollama dies. First check `mount | grep ollama_disk`; recover with
  `sudo mount /mnt/ollama_disk && sudo systemctl restart ollama`. Full war-story
  + the UAS-bridge fix are in `ros2_ws/CLAUDE.md §9`.
- **GPU:** NVIDIA MX150, 4 GB VRAM — fits `llama3.2:3b` (~2 GB), not 8B. Ollama
  GPU-accelerated when the driver matches (history/fix in `ros2_ws/CLAUDE.md §9`).
- **Ollama** redirects `OLLAMA_MODELS` and `$HOME` to `/mnt/ollama_disk` via a
  systemd drop-in (staging copy: `~/Abbes/ollama-override.conf`).

---

## 8. How I want you (Claude Code) to work with me

- **Teach as you go.** I'm learning the AI/LLM and robotics domains. Before a
  non-trivial edit, briefly explain what you'll change and why; after, summarize.
- **One change at a time.** Don't bundle edits — make one, let me read it, then move on.
- **Verify before assuming.** Unsure where a file is or what's implemented? Read
  it / check. Don't guess or bluff; label guesses as guesses.
- **Conservative changes only.** Touch the minimum needed; don't reformat or
  refactor unrelated things.
- **Disk-aware.** Before any download/build >~100 MB, confirm the target is on
  `/mnt/ollama_disk` and has space (§7).
- **Keep the handoffs current.** Each subproject's nested CLAUDE.md has a
  "Session handoff" section — update the relevant one whenever a meaningful chunk
  of work finishes, so a fresh session can resume. Mandatory, not optional.

---

## 9. Current focus / where to look

- **Active work:** AI phase in `ros2_ws/` (Steps 1–7 done & live-verified; Step 8
  Voice or Step 9 Path Picker next). Read **`ros2_ws/CLAUDE.md §10`** for the
  live AI handoff.
- **Firmware:** stable micro-ROS node; read **`firmware/CLAUDE.md`** before
  touching ESP32 code. (Heads-up: WiFi creds are committed in
  `firmware/include/robot_config.h` — see that file's notes.)
