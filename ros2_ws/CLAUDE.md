# Humanoid Robot PFE — Project Memory

This file is read by Claude Code at the start of every session. It is the
durable context for the entire project. Update it whenever a major piece
of work finishes.

---

## 1. What this project is

PFE (Projet de Fin d'Études) at INSAT, Tunisia, by Yassine Allouch.
A from-scratch humanoid robot built end-to-end: mechanical CAD, 3D-printed
parts, electronics, low-level control, kinematics, simulation, and (now) an
AI/natural-language layer for high-level control.

The robot is roughly child-sized, 18 degrees of freedom, designed to walk,
balance, gesture, and respond to spoken commands.

---

## 2. Hardware

- **Frame:** 3D-printed, designed in Onshape. ~18 DOF total.
- **Joint layout (18 servos):**
    - 2× legs × 5 DOF each = 10 (hip_roll, hip_pitch, knee_pitch,
      ankle_pitch, ankle_roll, per leg)
    - 2× arms × 3 DOF each = 6 (shoulder_pitch, shoulder_roll, elbow_roll)
    - Head: head_yaw, camera_pitch = 2
- **Actuators:** standard hobby servos driven by PCA9685 I²C PWM driver.
- **Brain:** ESP32 microcontroller. Runs the real-time control loop on
  hardware, talks to PCA9685, reads IMU.
- **Sensors:** IMU (orientation), planned vision via a head-mounted camera.
- **Power:** off-board for now; battery-powered version is post-PFE.

---

## 3. Software stack

- **OS:** Ubuntu 24.04 LTS
- **ROS2 distro:** Jazzy Jalisco
- **Simulator:** Gazebo Harmonic (the modern Gazebo, not classic Gazebo
  /Ignition naming). Bridged to ROS2 via `gz_ros2_control`.
- **Languages:** C++ for real-time control nodes, Python for
  higher-level orchestration (action servers, dispatcher, etc).
- **LLM stack:** Ollama running local models (target: llama3.1:8b or
  llama3.2:3b depending on RAM). Function-calling / tool-use for
  command grounding. No cloud dependency.

---

## 4. Repository structure (`~/Abbes/ros2_ws/src/`)

- **`my_robot_description`** — URDF, STL meshes, xacro files. URDF
  exported from Onshape via `onshape-to-robot`. URDF zero pose =
  geometric standing (straight legs, arms down).
- **`my_robot_bringup`** — Gazebo launch files, `controllers.yaml`,
  RViz configs.
- **`my_robot_control`** — C++ nodes for real-time work:
    - `imu_filter` — complementary filter on raw IMU
    - `balance_controller` — ankle/hip strategy for standing
    - `inverse_kinematics` — leg IK for given CoM position
    - `joint_fusion` — combines IK output with balance corrections
- **`humanoid_msgs`** — action and service interface definitions.
- **`humanoid_command_api`** — Python action/service server node
  exposing the 7-command vocabulary to higher layers.

There may also be a `firmware/` folder for the ESP32 code (PlatformIO
project) — that lives outside the ros2_ws.

---

## 5. Project phases — overall view

Mechanical and low-level control phases (NOT this AI phase):

  Phase 1  Mechanical design (Onshape) ........................ ✅
  Phase 2  3D printing + assembly ............................. ✅
  Phase 3  Electronics + ESP32 firmware ....................... ✅
  Phase 4  IMU + balance controller ........................... ✅
  Phase 5  ZMP walking gait generator ......................... 🔄 in progress
  Phase 6  Gazebo simulation parity ........................... ✅ for non-walking
  Phase 7  Reinforcement learning for gait .................... 📝 planned, post-PFE

AI intelligence phase (current focus):

  AI Step 1  Define Action Vocabulary ......................... ✅
  AI Step 2  ROS2 Command API (actions/services) .............. ✅
  AI Step 3  LLM Kindergarten (Ollama, tool use) .............. ✅
  AI Step 4  Connect LLM → Robot (dispatcher) ................. ✅
  AI Step 5  Safety Validator ................................. ⏳
  AI Step 6  Feedback Loop .................................... ⏳
  AI Step 7  Web UI ........................................... ⏳
  AI Step 8  Voice Input (Whisper) [BONUS] .................... ⏳
  AI Step 9  Graphic Path Picker [BONUS] ...................... ⏳

Phase 5 (walking) is independent of the AI phase. The `walk` action in
the AI layer currently stubs to "march in place"; when Phase 5 lands,
the stub gets swapped for the real ZMP gait without changing the
interface.

---

## 6. Key conventions (do not violate)

- **Joint angles:** radians, URDF zero convention. Conversion to servo
  degrees happens only in the firmware / bridge layer, not in ROS2.
- **Joint orderings:** must match `controllers.yaml` exactly. The
  authoritative list lives in `humanoid_command_api/pose_definitions.py`
  as LEG_JOINTS, ARM_JOINTS, HEAD_JOINTS.
- **LLM does not generate motor commands.** It picks from a fixed
  vocabulary and emits JSON. The dispatcher is the only translator.
- **Action servers, not topics, for goal-driven motions.** Topics are
  for streaming data (IMU, joint_states); actions are for cancellable,
  long-running, feedback-bearing goals.
- **Interfaces live in `humanoid_msgs`, never inline.** Don't define
  one-off message types in implementation packages.

---

## 7. Critical system constraints

- **Root filesystem `/` is on `/dev/nvme0n1p6` (46 GB) and frequently
  near-full.** Do not install or download anything large to `/`. Check
  `df -h /` before any operation expected to use more than ~100 MB.
- **External USB disk** is mounted at `/mnt/ollama_disk` (440 GB ext4,
  label `OllamaModels`). All LLM models, large caches, and AI-phase
  downloads go here.
- **Ollama configuration:** must redirect both `OLLAMA_MODELS` and
  the `ollama` user's home directory to `/mnt/ollama_disk`, because
  Ollama writes config and SSH keys to `$HOME/.ollama` and `/` cannot
  hold them.
- **GPU:** NVIDIA GPU is present and Ollama detected it. Models run
  GPU-accelerated; expect ~1-2 second response times.

---

## 8. How I want you (Claude Code) to work with me

- **Teach as you go.** I am learning the AI/LLM domain from scratch.
  Before any non-trivial edit, briefly explain what you're going to
  change and why. After the edit, summarize what the new code does.
- **One change at a time.** Don't bundle several edits. Make one
  change, let me read and acknowledge, then move on.
- **Verify before assuming.** If you're unsure where a file is, what
  a function does, what's already implemented, or what the current
  state is — read the file or run a check. Don't guess.
- **Conservative changes only.** Don't reformat, reinstall, or
  refactor unrelated things to "improve" them. Touch the minimum
  needed.
- **Disk-aware.** Before any download/install/build that could exceed
  ~100 MB, confirm the target path is on `/mnt/ollama_disk` and that
  it has space.
- **Honest about uncertainty.** When you don't know, say so. Don't
  bluff. If a fix is a guess, label it as such.
- **Update CLAUDE.md as we complete steps.** When a roadmap item
  finishes, update its checkbox and add a short note about what was
  built.
- **Always leave a session handoff.** Before ending a work session (or
  whenever a meaningful chunk of work finishes), update **section 10
  "Session handoff"** so a fresh session can pick up exactly where we
  left off: what was just done, what's in progress, and the single next
  concrete action. Treat this as mandatory, not optional — it's the
  first thing I rely on at the start of every new session.

---

## 9. Anything else I should know
(Use this section to capture project quirks, dead ends, things tried
and abandoned. Free-form. Update over time.)

- **Ollama install fixed (2026-05-29).** The stock systemd unit ran as
  user `ollama` with `HOME=/usr/share/ollama`, so it wrote `.ollama` to
  `/` and crashed. Fix = systemd drop-in at
  `/etc/systemd/system/ollama.service.d/override.conf` setting
  `HOME=/mnt/ollama_disk/ollama` and
  `OLLAMA_MODELS=/mnt/ollama_disk/ollama/models`. Service now runs,
  detects the GPU, writes only to the big disk. (Staging copy of the
  override lives at `~/Abbes/ollama-override.conf`.)
- **GPU is an MX150 with only 4 GB VRAM.** That's why the model choice is
  `llama3.2:3b` (~2 GB, fits in VRAM) and NOT `llama3.1:8b` (~4.7 GB,
  would spill to CPU and run slow). Revisit only if 3b's tool-calling
  quality proves too weak.
- **External disk: UAS bridge instability (2026-05-29) — FIXED, reboot
  pending.** During `ollama pull` the disk twice fell off the USB bus
  under sustained write load and ext4 aborted (mount flag `shutdown` /
  `emergency_ro`, all writes "Input/output error"). First time looked
  like a physical unplug; the second time `dmesg` showed the true cause:
  `device offline error, dev sd?` then automatic USB re-enumeration of
  `idVendor=2109 idProduct=0715` (VIA Labs VL715 USB-SATA bridge) on the
  `uas` driver — the classic UAS-drops-under-load bug. SMART PASSED
  (~2998 power-on hrs, no bad sectors) so the drive media is fine; it's
  the bridge/driver. **Fix applied:** `/etc/modprobe.d/disable-uas-vli.conf`
  with `options usb-storage quirks=2109:0715:u` (forces stable
  `usb-storage` driver instead of `uas`), then `update-initramfs -u`.
  **REQUIRES A REBOOT to take effect.** After reboot, verify the fix:
  `lsblk` shows the disk, and `dmesg | grep -i uas` should show NO uas
  binding for 2109:0715 (it should attach via usb-storage). Then the
  pull can run without dropping.
  - Notes: device node flips `/dev/sda1`↔`/dev/sdb1` on each
    re-enumeration — harmless, fstab mounts by UUID `dc60e041-…` with
    `nofail`. Recovery when it aborts: `systemctl stop ollama` →
    `umount -l /mnt/ollama_disk` → `fsck -y $(blkid -L OllamaModels)` →
    remount → `systemctl start ollama`. If writes EIO again, check
    `mount | grep ollama_disk` for `shutdown`/`emergency_ro` first.

- **AI Step 3 DONE (2026-05-29).** After the reboot activated the UAS disk
  fix (verified: VL715 bridge `1-2:1.0` bound to `usb-storage`, quirks
  `2109:0715:u` present, NOT `uas`), `ollama pull llama3.2:3b` ran to
  completion WITHOUT the disk dropping — the fix holds under sustained
  write load. Model registered (`a80c4f17acd5`, 2.0 GB). Both learning
  scripts then live-run successfully:
    - `01_chat.py` → clean chat round-trip (first call ~13s incl. ~5s cold
      VRAM load; warm calls faster).
    - `02_weather_tool.py` → full tool-use loop verified: model selected
      `get_weather`, extracted `city="Tunis"`, we executed it, fed the
      result back, model answered naturally. This proves the exact
      function-calling mechanism the dispatcher (Step 4) needs, on the 3B
      model + 4 GB GPU. Step 3 ✅.

---

## 10. Session handoff — where we are right now

**Always keep this section current (see section 8 rule).** This is the
first thing to read at the start of a new session.

**Last updated:** 2026-05-29

**Just done:**
- **Reboot + disk fix VERIFIED WORKING.** Machine rebooted; the UAS-disable
  quirk is now live: VL715 bridge `1-2:1.0` bound to `usb-storage` (not
  `uas`), `/sys/module/usb_storage/parameters/quirks` = `2109:0715:u`. Disk
  mounted at /mnt/ollama_disk (440G, ~417G free).
- **AI Step 3 ✅** — `ollama pull llama3.2:3b` ran to completion with NO disk
  drop (fix holds under sustained write). Model registered. Then live-ran
  both learning scripts successfully: `01_chat.py` (chat round-trip) and
  `02_weather_tool.py` (full tool-use loop — model picked the tool, filled
  args, we executed, model answered). See §9 for details.
- **Step 4 Piece 1 DONE (earlier):** `humanoid_command_api/command_tools.py`
  — the 7 robot commands as Ollama tool-use schemas + `VALID_TOOL_NAMES`.
- **Step 4 Piece 2 DONE + runtime-verified:** wrote
  `humanoid_command_api/llm_dispatcher_node.py` — the ROS2 CLIENT half
  (5 action clients + 2 service clients) mirroring command_server_node.
  Blocking `_run_action`/`_call_service` helpers (synchronous
  spin_until_future_complete, handles goal REJECTION), per-command wrappers,
  and the single `execute_tool(name, args) -> dict` seam Piece 3 plugs into.
  Compile-checked; field names cross-checked against humanoid_msgs (all
  match, incl. the Stop`.success`/GetStatus`.ok` distinction). User ran the
  two-terminal smoke test (command_server + `--test`) — works.
- **Step 4 Piece 4 DONE:** added `llm_dispatcher` console_scripts entry point
  to `humanoid_command_api/setup.py`.
- **Step 4 Piece 3 DONE:** added the Ollama brain into
  `llm_dispatcher_node.py` — `_call_ollama(messages)` (POST with
  `tools=ROBOT_TOOLS`), `dispatch(user_text)` (the 4-phase tool-use loop;
  Phase 3 runs the chosen tool on the robot via `execute_tool()`), a tight
  `ROBOT_SYSTEM_PROMPT`, and a rewritten `main()` = interactive REPL
  (`you>`/`robot>`), with the Piece-2 smoke test kept behind `--test`.
  Compile-checked.
- **AI Step 4 ✅ — LIVE-TESTED END TO END.** Rebuilt with the new entry
  point, ran Ollama + command_server (Terminal A) + llm_dispatcher REPL
  (Terminal B). Plain-English commands → model picks the right tool → real
  ROS2 action/service fires. Confirmed working. (Note: command_server only
  logs for `walk` and `stop`; wave/bow/sit/stand/get_status execute
  silently — the dispatcher's feedback progress in Terminal B is the proof
  the goal ran. Not a bug; an optional one-line log per silent handler was
  offered and deferred.)

**In progress:**
- Nothing actively mid-edit. AI Steps 1–4 ✅. Next focus is AI Step 5.

**Next concrete action (in order):**
1. **AI Step 5 — Safety Validator.** A layer that vets the LLM's chosen
   tool+args BEFORE `execute_tool()` runs them. The command server already
   does range/enum rejection (distance 0.1–3.0, direction/which enums) and
   mutual-exclusion, so decide what the validator adds on top: e.g. tilt/IMU
   gating (don't walk if already tilted — GetStatus exposes tilt), rate
   limiting, an explicit allow/deny policy, or confirmation for "risky"
   commands. Likely lives as a function the dispatcher calls inside
   `dispatch()` between the model's tool choice and `execute_tool()`.
2. Consider a `command_api.launch.py`-style launch file that brings up
   command_server + llm_dispatcher together (currently two manual terminals).
3. **Commit.** A lot is still uncommitted (see below) — good time for a
   clean checkpoint commit of the completed AI Steps 3–4 work.

**Not yet committed:** command_server_node.py, command_tools.py,
llm_dispatcher_node.py, launch/, pose_definitions.py edits, setup.py edits,
urdf edits, CLAUDE.md, ai_experiments/. Commit when at a clean stopping point.
