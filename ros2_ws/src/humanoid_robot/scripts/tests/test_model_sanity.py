#!/usr/bin/env python3
"""test_model_sanity.py — Phase-0 acceptance test (Milestone M0)."""
import argparse, os, subprocess, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
ESTIMATOR = os.path.join(HERE, "..", "..", "..", "my_robot_description", "analysis", "estimate_model.py")
NOMINAL_MASS_G = 1563

def check_model():
    print("-- check 1/3: MODEL (mass + inertial origins) --")
    r = subprocess.run([sys.executable, os.path.abspath(ESTIMATOR)], capture_output=True, text=True)
    print(r.stdout.strip())
    if r.stderr.strip(): print(r.stderr.strip())
    return r.returncode == 0

def check_mass(weighed_g):
    print("-- check 2/3: MASS vs weighed --")
    if weighed_g is None:
        print("SKIP: pass --weighed-g <grams> to compare"); return True
    err = abs(NOMINAL_MASS_G - weighed_g) / weighed_g * 100.0
    ok = err <= 10.0
    print(f"{'PASS' if ok else 'FAIL'}: URDF {NOMINAL_MASS_G} g vs weighed {weighed_g} g -> {err:.1f}% (threshold 10%)")
    return ok

def check_stand(seconds, tilt_deg):
    print("-- check 3/3: STAND under gravity --")
    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Vector3Stamped
    except Exception:
        print("SKIP: rclpy not available"); return True
    rclpy.init(); node = Node("model_sanity_stand"); state = {"max_tilt": 0.0, "n": 0}
    def cb(msg):
        t = (msg.vector.x**2 + msg.vector.y**2) ** 0.5
        state["max_tilt"] = max(state["max_tilt"], t); state["n"] += 1
    node.create_subscription(Vector3Stamped, "/tilt_degrees", cb, 10)
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node(); rclpy.shutdown()
    if state["n"] == 0:
        print("FAIL: no /tilt_degrees messages — is imu_filter running?"); return False
    ok = state["max_tilt"] < tilt_deg
    print(f"{'PASS' if ok else 'FAIL'}: max tilt {state['max_tilt']:.1f} deg over {seconds}s (threshold {tilt_deg}), {state['n']} samples")
    return ok

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weighed-g", type=float, default=None)
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--tilt-deg", type=float, default=5.0)
    a = ap.parse_args()
    results = [check_model(), check_mass(a.weighed_g), check_stand(a.seconds, a.tilt_deg)]
    print("------------------------------")
    ok = all(results); print(f"M0 {'PASS' if ok else 'FAIL'}")
    sys.exit(0 if ok else 1)

if __name__ == "__main__":
    main()
