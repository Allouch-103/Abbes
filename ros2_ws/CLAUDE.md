# Abbes — ROS2 Workspace (component memory)

Component context for the ROS2 workspace. Loads when I touch files under
`ros2_ws/`. **Project-wide context — hardware, software stack, system
constraints, how-to-work-with-me, and the cross-cutting conventions — lives in
`~/Abbes/CLAUDE.md`; read that first.** Firmware context is in
`~/Abbes/firmware/CLAUDE.md`.

This file covers the ROS2 packages, ROS2/AI-specific conventions, the AI
(natural-language) phase roadmap, and its live **session handoff** (§10) — keep
that handoff current whenever a meaningful chunk of work finishes.

---

## 4. Repository structure (`~/Abbes/ros2_ws/src/`)

- **`my_robot_description`** — URDF, STL meshes, xacro files. URDF
  exported from Onshape via `onshape-to-robot`. URDF zero pose =
  geometric standing (straight legs, arms down).
- **`my_robot_bringup`** — Gazebo launch files, `controllers.yaml`,
  RViz configs.
- **`humanoid_robot`** — C++ nodes for real-time work (NOTE: this is the
  actual package name; an older note said `my_robot_control`):
    - `imu_filter` — complementary filter on raw IMU → `/tilt`, `/tilt_degrees`
    - `balance_controller` — ankle PID for standing → publishes `/joint_commands`
    - `inverse_kinematics` — analytical leg IK → publishes `/joint_commands`
    - `ik_vectors_DLS` — alternative DLS (Wampler) IK node
    - `joint_bridge` — `/joint_commands`(deg)→controllers(rad). ⚠️ BROKEN: it
      does a flat `deg*π/180` and does NOT remove the servo rest offsets, so it
      commands garbage poses. Unused (not in any launch). Superseded by the
      reverse bridge below; delete when convenient.
    - ⚠️ `joint_fusion` (combine IK + balance) was PLANNED but **never written**.
      So during "walking" both `inverse_kinematics` and `balance_controller`
      publish to `/joint_commands` and overwrite each other.
- **`humanoid_msgs`** — action and service interface definitions.
- **`humanoid_command_api`** — Python action/service server node
  exposing the 7-command vocabulary to higher layers.

The ESP32 firmware lives in `~/Abbes/firmware/` (a PlatformIO **micro-ROS**
project, outside this workspace) — see `~/Abbes/firmware/CLAUDE.md`. It joins
the ROS2 graph over WiFi: SUB `/joint_commands` (degrees, 18 vals), PUB `/imu`.

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
  AI Step 5  Safety Validator ................................. ✅
  AI Step 6  Feedback Loop .................................... ✅
  AI Step 7  Web UI ........................................... ✅
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

## 7 & 8 — moved to the root hub

Project-wide **system constraints** (small `/`, `/mnt/ollama_disk`, Ollama, GPU)
and **how-to-work-with-me** now live in **`~/Abbes/CLAUDE.md` (§7, §8)**. The
AI-phase operational war-stories (disk drops, GPU/DKMS history, model keep-alive)
remain in §9 below, where the AI work happens.

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

- **GPU fell back to CPU — NVIDIA driver/lib version mismatch (2026-05-30).**
  During the Step 5 live test, the LLM call timed out (120s). Cause: an
  NVIDIA driver package update bumped the USERSPACE libs to `535.309.01`
  while the RUNNING KERNEL module is still the old `535.288.01`
  (`/proc/driver/nvidia/version` vs `libnvidia-ml.so.535.309.01`). NVML
  then fails to init (`nvidia-smi` → "Driver/library version mismatch"),
  so Ollama can't see the GPU and loads the model 100% on CPU
  (`ollama ps` shows `100% CPU`; journald shows `device=CPU`,
  `GPULayers:[]`). On CPU a trivial reply is ~3.5s but the tool-use
  dispatch (big ROBOT_TOOLS prompt) exceeds the 120s client timeout.
  **Fix = REBOOT** (loads the matching 535.309.01 kernel module). After
  reboot verify: `nvidia-smi` works, `ollama ps` shows the GPU not
  `100% CPU`. This is purely environmental — Step 5 code is fine.

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

- **GPU mismatch RESOLVED — reboot done, GPU LIVE (2026-05-30 evening).** The
  DKMS 309 fix below worked: after rebooting onto 6.17.0-23, `nvidia-smi`
  works (Driver 535.309.01, no mismatch), `modinfo nvidia` = 309, and
  `ollama ps` shows `100% GPU`. Steps 4–7 then ran LIVE through Ollama for the
  first time.
- **Model keep-alive / "why is it stuck" (2026-05-30).** On the MX150 the
  model EVICTS from VRAM after ~5 min idle, and a cold RELOAD of the 2 GB model
  FROM THE USB DISK takes ~47 s — so the first command after idle looks frozen.
  Keep the web UI snappy by pinning the model warm with one call (loads it AND
  sets a 30-min TTL, no systemd edit/restart):
  `curl -s localhost:11434/api/generate -d '{"model":"llama3.2:3b","prompt":"hi","keep_alive":"30m"}'`.
- **DISK DROPPED AGAIN — now TWICE in one day (2026-05-30): at boot AND while
  idle.** `/mnt/ollama_disk` was found unmounted at boot, then AGAIN later while
  idle (the 2nd time showed NO dmesg UAS error — device `sda1`/`OllamaModels`
  present, ext4 clean, just not mounted). Downstream symptom: Ollama can't reach
  its model store → `ollama list` errors `mkdir .../ollama: permission denied`
  and `/api/chat` returns `400 {"error":"model is required"}` — the WHOLE LLM
  stack dies. RECOVERY (both times): `sudo mount /mnt/ollama_disk` then
  `sudo systemctl restart ollama`, then re-warm the model. The UAS quirk may not
  be fully holding, or something is unmounting it — NOT yet root-caused. If it
  recurs, investigate fstab `nofail`/automount and whether the VL715 bridge
  still transiently drops.

---

## 10. Session handoff — where we are right now

**Always keep this section current (see section 8 rule).** This is the
first thing to read at the start of a new session.

**Last updated:** 2026-05-31 (later — foundation reconcile complete + validated)

**FOUNDATION RECONCILE COMPLETE & COMMITTED (2026-05-31).** After pulling the
colleague's outer-loop work (commit `de91ebd`: footstep_planner/zmp_reference/
preview_controller/pipeline.py + LQR balance_controller.py + rewritten
joint_bridge — a WIP that did NOT run), reconciled the whole walking stack onto
ONE convention and verified it in Gazebo:
- **P8** untracked `install/`,`log/`,`scripts/install`,`scripts/log` (~84 files) + gitignored.
- **P4/P5** ONE joint order (controller-grouped, `joint_table.hpp`) + **90-centred**
  convention everywhere (ik_vectors_DLS apply_*, joint_table rests, firmware,
  servo_bridge). `joint_bridge` now does `(servo−90)·π/180`. Unit-tested.
- **P1/P2/P3** topic graph fixed: `ik→/joint_commands` (nominal),
  `balance_controller.py→/joint_commands_corrected`, `joint_bridge` reads
  corrected; launch runs the LQR `.py` not the C++ PID; best-effort QoS on
  balance's subs; **joint_bridge controller pubs RELIABLE** (forward_command_controller
  needs it or commands never arrive).
- **P6** one CoM height = **0.2698** everywhere (pipeline/balance/ik/robot_params/gait_params).
- Gazebo launch: `GZ_HEADLESS=1` env → server-only; **`UnsetEnvironmentVariable('SESSION_MANAGER')`**
  fixes the Qt/ICE crash that killed the GUI (`ICE default IO error handler`).
- **balance_controller.py**: robust `_is_enabled()` (string "false" was truthy);
  **default OFF** in launch (`balance_enabled:=false`) until its math is fixed.

**VERIFIED IN GAZEBO (headless, IMU-tilt judged):** bare robot stands (tilt ~0.5°);
**full chain with ik OFF + balance OFF → STANDS** (foundation good end-to-end).
- **ik ON + balance OFF → FALLS** — open-loop gait execution destabilizes (suspect
  a leg joint SIGN flip and/or the trajectory; needs visual check in the GUI).
- **balance ON → FALLS** — rails ankles to ±12°: wrong sign + roll adds the SAME
  Δθ to both ankle_rolls (must be antisymmetric L/R) + gain too high.

**NEXT (the actual "make it walk" work — needs the GUI for sign verification):**
1. Gait: watch `ros2 launch humanoid_robot walking.launch.py` (balance off) in the
   GUI, find which leg joint moves the wrong way, flip its sign in `ik_vectors_DLS`.
2. Balance: fix pitch sign, make roll antisymmetric (left ankle_roll `-=`),
   lower gain/`MAX_ANKLE_CORRECTION_DEG`, then re-enable and tune.
P7 (wire AI `walk` action → ZMP pipeline) still deferred.

**Earlier THIS SESSION (2026-05-31) — deep dive + sim→real bridge:**
- **DEEP DIVE finding — two stacks, two incompatible joint conventions.**
  (A) AI/command path: radians, URDF-zero, published straight to
  `/leg|arm|head_controller/commands` → Gazebo. Consistent → robot moves. ✅
  (B) walking/balance/firmware path: SERVO DEGREES with rest offsets on
  `/joint_commands`. Correct for the ESP32, but never reconciled with (A).
- **Why the robot can't walk (4 independent causes):** (1) there is NO gait —
  `gait_generator`/`zmp_preview_controller` in `gait_params.yaml` are NOT
  implemented (not in CMakeLists, no source); the AI `walk` = `traj_march_in_place`
  = marches in place with ZERO weight shift (its own docstring says it tips over);
  `walk_test.py` is a crude open-loop sinusoid. (2) `joint_bridge` math is wrong
  (see §4). (3) `walking.launch.py` never launches a bridge AND IK+balance both
  publish `/joint_commands` (no `joint_fusion`) → they fight. (4) foot
  friction/contact (`mu1/mu2/kp/kd`) is COMMENTED OUT in the URDF + tiny feet.
  Net: open-loop position streaming on a free biped with no ZMP → must fall.
- **ESP32 gap:** the AI/sim path can't drive the ESP32 as-is (3 topics vs 1,
  Float64 vs Float32, radians-URDF vs servo-deg, different order). Firmware also
  has board-1 (0x41) init commented out (left side dead) and reads
  `imu.orientation` which the ESP32 never fills (tilt always 0 on real HW).
- **BUILT + VERIFIED the real bridge** (the highest-value fix): new node
  `humanoid_command_api/controller_to_servo_bridge.py` (entry point
  `servo_bridge`). Subscribes the 3 controller topics (rad, URDF), republishes
  `/joint_commands` (Float32, 18, servo deg, firmware order). Conversion
  `servo_deg = rest + dir*deg(rad)`, clamp [0,180]. Imports joint orderings from
  `pose_definitions` (DRY). Per-joint `CALIBRATION` table defaults `rest=90`
  (servo center), `dir=+1` — the SAFE placeholder for the un-mounted robot
  (HOME → all servos 90). User will calibrate `rest`/`dir` per joint after
  mounting. VERIFIED with no hardware: idle → all 90.0; `r_hip_pitch=0.5 rad` →
  ONLY firmware index 6 = 118.65 (=90+deg(0.5)), all others 90. Rebuilt OK.
  To drive the ESP32 from the working AI poses: run command_server + this bridge,
  with the micro-ROS agent up.
- **NOT yet done (recommended next, in order):** (1) re-enable foot
  friction/contact in the URDF + verify standing under motion; (2) test the
  bridge → micro-ROS agent → ESP32 with real servos (wave/bow/sit first);
  (3) write `joint_fusion` or make balance emit ankle-deltas only; (4) implement
  a minimal static-walk (full weight-shift) gait before attempting ZMP preview;
  (5) firmware: re-enable board-1, fix the `imu.orientation` tilt read.

**Just done (PREVIOUS SESSION, 2026-05-30 evening):**
- **AI Steps 4–7 LIVE-verified end-to-end through Ollama for the FIRST time.**
  GPU fix activated by the reboot (§9). Browser → dispatcher → Ollama tool pick
  → real ROS2 action all proven: wave/bow/sit/stand fire and report (`Waved with
  left hand.`, `Bow complete.`, `Returned to home pose.`); live status bar tracks
  `idle→walking`; **STOP interrupts an in-flight walk** ("Stopped (was 'walking')
  and snapped to home"). Step 5 rate-limit did NOT trip live (two real motions are
  naturally >1.5 s apart because each action blocks; only fires if the model emits
  both tool calls in one turn — already unit-proven offline). Step 6 tilt-recovery
  not exercised live (no real IMU).
- **Web UI REDESIGNED (frontend-only, `INDEX_HTML` in web_ui_node.py).** Robot
  branded **Abbes**: animated avatar, online/offline + ECHO badges, a live status
  dashboard with a tilt/level DIAL (bubble moves with pitch/roll, ambers >15°),
  "What Abbes can do" cards (the 7 commands w/ descriptions from command_tools.py,
  click-to-send), chat avatars+timestamps, animated "thinking…" with an
  elapsed-seconds counter, Clear button, About blurb. Fully offline (inline
  SVG/emoji, no CDN). Rebuilt; served HTML verified.
- **CONCURRENCY BUG found + FIXED (the important one).** Symptom: after the 1st
  action, EVERY action returned `ok:false` with
  `RCLError: wait set index for cancel client is out of bounds
  (rcl_action/action_client.c:650)`, and the model (per Step-6) confabulated
  "I'm unbalanced, let me stand_still" excuses — looked like "the AI is dumb" but
  was a real ROS bug. ROOT CAUSE (reproduced deterministically): the browser
  polls `/api/status` every 3 s on the SEPARATE `_EmergencyNode`, which SHARED
  the default rclpy context with the dispatcher; two threads each calling
  `spin_until_future_complete` on that ONE context race on its wait set and
  corrupt the dispatcher's action clients. (Earlier curl tests never hit it — no
  continuous polling.) FIX: give `_EmergencyNode` its OWN `rclpy.Context()` +
  `SingleThreadedExecutor` (4 small edits in web_ui_node.py; dispatcher code
  untouched). VERIFIED: shared-context repro crashes on action #2; with separate
  contexts 8/8 pass, and a live run (4 commands under fast concurrent polling) =
  **0** wait-set errors. STOP/status still work on the isolated path. Repro
  scripts: `/tmp/repro_concurrent.py` (fails), `/tmp/repro_fix.py` (passes).
- **System prompt tuned (`ROBOT_SYSTEM_PROMPT` in llm_dispatcher_node.py).** Gave
  the model the identity "Abbes", required ONE short plain-language first-person
  confirmation, forbade JSON/braces/tool-names in the user-facing reply (request
  tools ONLY via the tool channel), kept the Step-6 recovery rule. PARTIALLY
  effective: confirmations are now clean ("Done, I waved my left hand!"), but
  `llama3.2:3b` STILL intermittently (a) types a tool call as TEXT instead of
  using the tool channel (e.g. `{"name":"sit_down", {}}` → action doesn't fire)
  and (b) drops required args (called `walk` with no distance_m/direction →
  rejected, then confabulated). These are 3B tool-calling limits, not fixable by
  prompt alone — see "Next" for the two no-/low-cost mitigations.

**Earlier build (context):**
- **AI Step 7 ✅ — Web UI built + echo-verified (now superseded by the live run above).**
  New node `humanoid_command_api/web_ui_node.py`: a browser control panel that
  reuses the WHOLE Step 4-6 brain. Deliberately built on Python stdlib
  `http.server` (NO Flask/FastAPI — neither was installed and `/` is tight,
  §7; only `uvicorn` was present, useless without a framework).
    - Holds ONE `LLMDispatcherNode`; routes: `GET /` (embedded dark chat page,
      HTML-in-a-string so there are no ament resource-path headaches),
      `GET /health`, `POST /api/command` ({"text":..} -> `dispatch()` ->
      {"reply":..}), `GET /api/status`, `POST /api/stop`. Binds 127.0.0.1:8080.
    - CONCURRENCY (the teaching point): `dispatch()` blocks + spins rclpy, and
      http.server is multi-threaded, so command requests are serialized by a
      single `_DISPATCH_LOCK` (one motion at a time — matches the robot + the
      Step-4 design). BUT the Stop button + status poll use a SEPARATE node
      `_EmergencyNode` (own Stop/GetStatus clients, own `_EMERGENCY_LOCK`) so
      they run CONCURRENTLY with an in-flight dispatch — making the red STOP a
      REAL interrupt (the command server is multi-threaded and its Stop sets a
      flag every action's trajectory loop checks each tick -> `canceled()`).
    - `--echo` debug mode skips Ollama/ROS and echoes (incl. fake status/stop),
      so the browser<->backend loop is testable with no GPU/Ollama/server.
    - Piece 2 = the page's live status bar + STOP button; Piece 3 = extended
      `launch/command_api.launch.py` to bring up command_server + web_ui
      together (NOT llm_dispatcher — web_ui embeds the dispatcher in-process),
      and added the `web_ui` console_scripts entry point to setup.py.
  VERIFIED via `ros2 run humanoid_command_api web_ui --echo` + curl: /health,
  /api/status, /api/stop, /api/command all return correct JSON and the page
  carries the status bar + STOP button. Rebuilt OK. NOT yet driven live
  through Ollama/command_server (needs the GPU reboot). Gotcha learned:
  background `&` test servers ORPHAN across separate Bash calls and keep
  :8080 bound — kill the `web_ui` python by the PID from `ss -ltnp` (do NOT
  `pkill -f web_ui`: it matches and kills the calling shell too).
- **AI Step 6 ✅ — Feedback Loop built + scripted-verified.** Turned
  `dispatch()` in `llm_dispatcher_node.py` from a fixed two-call exchange
  (ask → run tools → ask once for text) into a BOUNDED AGENTIC LOOP: it keeps
  calling the model and running whatever tool the model requests, feeding
  each real robot result back as a `role:"tool"` message, UNTIL the model
  stops requesting tools (= it writes its final reply) or hits the new
  `MAX_AGENT_TURNS=5` ceiling. Because every result — incl. a safety denial
  (`denied_by_safety`) or a rejected goal — now lands in the transcript
  BEFORE the next model call, the model can REACT (recover then retry, pick a
  different tool, or give up and explain) instead of only narrating the
  failure. Same Step-5 safety gate, same `execute_tool` seam, same error
  handling on the first call; added a `RequestException` catch on later
  rounds + a warn-and-return-last-reply on the turn cap. Also added one
  sentence to `ROBOT_SYSTEM_PROMPT` telling the model: on a tool result with
  `"ok": false` / `denied_by_safety`, try to fix the situation (e.g.
  `stand_still` then retry) before apologizing — the loop only helps if the
  model knows it may recover. Both edits compile; rebuilt package OK.
  VERIFIED with a scripted fake-model test (`/tmp/test_step6_loop.py`, NO
  Ollama/GPU needed): model asks to walk while tilted (30°) → safety denies,
  execute_tool NOT called → model stand_still's (robot becomes level) →
  retries walk (now level) → runs → model writes final reply. Asserted: 4
  model rounds, tools executed == [stand_still, walk], non-empty reply →
  PASS. NOT yet live-tested through real Ollama (GPU still CPU-only, see §9).
- **AI Step 5 ✅ — Safety Validator built + wired + unit-verified.** New pure
  (no-rclpy) module `humanoid_command_api/safety_validator.py`: a stateful
  `SafetyValidator.check(name, args, status_provider) -> (allowed, reason)`.
  Three rules (chosen by user): (1) RATE LIMIT — deny a motion if the last
  allowed motion was < `min_interval_sec` (1.5s) ago; (2) TILT GATE (walk
  only) — lazily call `status_provider()` and deny `walk` if |pitch| or
  |roll| > `tilt_limit_deg` (15°); (3) `stop`/`get_status` are EXEMPT (never
  blocked, don't touch the rate-limit clock). Fails OPEN on missing IMU
  (server reports 0.0° tilt with no IMU → walk allowed), matching the
  server's own non-blocking stance. Injectable `clock` for testable rate
  limiting. Wired into `llm_dispatcher_node.py`: `self.validator` built in
  `__init__`; `dispatch()` Phase-3 loop now calls
  `self.validator.check(name, args, self.do_get_status)` BEFORE
  `execute_tool` — on deny it skips the action and feeds
  `{"ok":False,"denied_by_safety":True,"message":reason}` back to the model
  so the robot explains the refusal. Both files compile; ran a 7-case
  controlled-clock test (level-walk OK, immediate 2nd motion rate-limited,
  stop/get_status exempt, tilted-walk denied, tilted-bow allowed, no-status
  walk fails open) — all pass. Rebuilt the package OK. NOT live-tested
  end-to-end with Ollama yet.
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
- Nothing actively mid-edit. AI Steps 1–7 ✅ and now LIVE-verified through Ollama
  on the GPU. Step 8 (Voice/Whisper) and Step 9 (path picker) are the remaining
  BONUS steps and the likely "another one" to start next. A no-download
  reliability fix for the 3B (dispatcher-side guard) was scoped but NOT
  implemented (see Next #2).

**GPU mismatch — RESOLVED 2026-05-30 (reboot done, GPU live; history below).**
Root cause was a half-finished apt upgrade: the 309 driver metapackage
`nvidia-driver-535` *Depends on* `nvidia-dkms-535`, which was NEVER installed
— so the driver sat `iU` (unconfigured) and the running kernel `6.17.0-23`
kept its old prebuilt `535.288.01` module while userspace was bumped to
`535.309.01` → "Driver/library version mismatch", Ollama on CPU.
  KEY CONSTRAINT (user, 2026-05-30): STAY ON KERNEL `6.17.0-23-generic`. Do
  NOT switch to / boot `6.17.0-29` (user explicitly refused). The archive has
  a prebuilt 309 module ONLY for 6.17.0-29, NOT for 23; and 288 userspace is
  gone from the repo (only 309 + old 171.04 remain) — so neither a prebuilt-
  module install nor an apt downgrade could fix 23. DKMS was the right path.
  WHAT WAS DONE: Secure Boot is DISABLED + `linux-headers-6.17.0-23-generic`
  installed, so `sudo apt-get install dkms nvidia-dkms-535` (309) COMPILED
  the module against the running kernel → `dkms status` =
  `nvidia/535.309.01, 6.17.0-23-generic: installed`, module at
  `/lib/modules/6.17.0-23-generic/updates/dkms/nvidia.ko.zst` version
  535.309.01. `modinfo nvidia` now resolves to that 309 DKMS module (updates/
  dir outranks the leftover prebuilt 288 in kernel/). Then removed the two
  broken 6.17.0-29 packages (`linux-modules-nvidia-535-6.17.0-29-generic`,
  `linux-modules-nvidia-535-generic-hwe-24.04`); `dpkg --audit` now clean, no
  nvidia pkg left in iU/iF. Disk fine (~6.1G free on /). Display is on Intel
  i915 so this can't break the screen. Going forward, future kernel upgrades
  will auto-build the nvidia module via DKMS (as long as that kernel's headers
  are installed) — no more prebuilt-module version skew.
  STILL PENDING: the RUNNING module is still 288 (loaded at boot). **REBOOT**
  (stay on 6.17.0-23) to load the 309 DKMS module. After reboot VERIFY:
  `nvidia-smi` works (no mismatch), `modinfo nvidia | grep version` == 309,
  and `ollama ps` shows the GPU (NOT `100% CPU`). THEN the Steps 5+6 live
  Ollama relay tests can finally run.

**Step 5 live test (2026-05-30): PARTIALLY done.** Ran an automated driver
against a live command_server. PROVEN live: `do_get_status()` round-trips a
real status dict; the tilt gate reads that live status (no IMU → 0° → walk
allowed); rate-limiting denies a too-soon 2nd motion; `stop`/`get_status`
are exempt even inside the rate window. NOT yet proven: the natural-language
refusal RELAY through Ollama — the LLM call timed out because the GPU is
currently CPU-only (driver/lib mismatch, see above + §9). Committed as `2867bf2`.

**Next concrete action (in order):**
1. **Start the next step** — user said they want to move to "another one".
   Most likely **AI Step 8 — Voice Input (Whisper) [BONUS]** (Whisper download
   is LARGE — must go to /mnt/ollama_disk, not /, per §7) or **Step 9 — Graphic
   Path Picker [BONUS]**. Confirm which with the user.
2. **(Deferred) Improve 3B tool-call reliability.** Two mitigations were scoped
   but not done; user has LITTLE INTERNET right now so the download option is
   deferred:
   (a) **Model swap to `qwen2.5:3b`** — same ~2 GB / 4 GB-VRAM budget, markedly
       better at function-calling than llama3.2:3b. Needs a ~2 GB pull to
       /mnt/ollama_disk — do when internet allows.
   (b) **Dispatcher-side guard (no download)** in `dispatch()` /`execute_tool`:
       detect a tool-call leaked as TEXT (reply looks like `{"name":...}`),
       parse + execute it; and DEFAULT walk's missing args (e.g. 0.5 m forward)
       instead of rejecting. Directly fixes the two failure modes seen this
       session. (I had just read dispatch()/execute_tool to implement this when
       we paused to commit.)
3. **Watch the disk** (§9): if `/mnt/ollama_disk` unmounts again, remount +
   restart ollama before anything LLM-related.

**Commit state (2026-05-30 evening):** Steps 3–6 were already on `main` (Step 6
`eeaf375`). **This session's commit** adds AI Step 7 + the concurrency fix +
prompt tuning: `web_ui_node.py` (new + redesign + own-context fix), `setup.py`
(web_ui entry point), `launch/command_api.launch.py`, `llm_dispatcher_node.py`
(prompt), and this CLAUDE.md. Being committed AND PUSHED now at the user's
request.
