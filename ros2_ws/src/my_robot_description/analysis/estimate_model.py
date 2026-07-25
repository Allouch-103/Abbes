#!/usr/bin/env python3
"""estimate_model.py — Phase-0 model ground-truth (dependency-free)."""
import re, sys, os
HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_XACRO = os.path.join(HERE, "..", "urdf", "humanoid.urdf.xacro")

def load_props(path):
    text = open(path).read()
    props = {}
    for name, val in re.findall(r'<xacro:property\s+name="([^"]+)"\s+value="([^"]+)"', text):
        try: props[name] = float(val)
        except ValueError: pass
    return props

def build_links(p):
    fh, sh, th = p["foot_height"], p["shank_len"], p["thigh_len"]
    tor_h = p["torso_height"]
    torso_z = fh + sh + th + tor_h / 2.0
    L = []
    L.append(("base_link", 0.001, 0.0, 0.0))
    L.append(("torso", p["torso_mass"], torso_z, torso_z))
    hip_z = torso_z - tor_h / 2.0
    thigh_z = hip_z
    knee_z = thigh_z - th
    ankle_z = knee_z - sh
    for _ in range(2):
        L.append(("hip_bracket", p["bracket_mass"], hip_z, hip_z))
        L.append(("thigh", p["thigh_mass"], thigh_z, thigh_z - th/2.0))
        L.append(("shank", p["shank_mass"], knee_z, knee_z - sh/2.0))
        L.append(("ankle_bracket", p["bracket_mass"], ankle_z, ankle_z))
        L.append(("foot", p["foot_mass"], ankle_z, ankle_z - fh/2.0))
    ua, fa = p["upper_arm_len"], p["forearm_len"]
    sh_z = torso_z + p["shoulder_offset_z"]; ua_z = sh_z
    fa_z = ua_z - ua; hand_z = fa_z - fa
    for _ in range(2):
        L.append(("shoulder_link", 0.001, sh_z, sh_z))
        L.append(("upper_arm", p["upper_arm_m"], ua_z, ua_z - ua/2.0))
        L.append(("forearm", p["forearm_m"], fa_z, fa_z - fa/2.0))
        L.append(("hand", 0.030, hand_z, hand_z))
    neck_z = torso_z + tor_h/2.0 + p["neck_height"]/2.0
    head_z = neck_z + p["neck_height"]/2.0 + p["head_height"]/2.0
    cam_z = head_z + p["head_height"]*0.2
    L.append(("neck", 0.020, neck_z, neck_z))
    L.append(("head", p["head_mass"], head_z, head_z))
    L.append(("camera", 0.020, cam_z, cam_z))
    return L

def main():
    path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_XACRO
    p = load_props(path); links = build_links(p)
    M = sum(m for _, m, _, _ in links)
    com_frame = sum(m*zf for _, m, zf, _ in links) / M
    com_centroid = sum(m*zc for _, m, _, zc in links) / M
    print(f"model: {os.path.relpath(path)}")
    print(f"links: {len(links)}   total mass: {M*1000:.0f} g  ({M:.3f} kg)")
    print(f"CoM height (physically correct, box centroids): {com_centroid*1000:.1f} mm")
    print(f"CoM height (as Gazebo sees it now, frame origins): {com_frame*1000:.1f} mm")
    print(f"  -> inertial-origin modelling error: {abs(com_frame-com_centroid)*1000:.1f} mm")
    text = open(path).read()
    macro = re.search(r'name="box_inertia".*?</xacro:macro>', text, re.S)
    fixed = bool(macro and "<origin" in macro.group(0))
    err_mm = abs(com_frame - com_centroid) * 1000.0
    if fixed:
        print(f"PASS: box_inertia places an inertial <origin> — Gazebo CoM now tracks the {com_centroid*1000:.1f} mm centroid target")
        sys.exit(0)
    else:
        print(f"FAIL: box_inertia has no inertial <origin>; Gazebo CoM is {err_mm:.1f} mm high (> 10 mm).")
        sys.exit(1)

if __name__ == "__main__":
    main()
