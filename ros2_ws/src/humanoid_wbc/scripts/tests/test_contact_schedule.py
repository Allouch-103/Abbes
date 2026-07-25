#!/usr/bin/env python3
"""test_contact_schedule.py — unit test (pure Python, no ROS)."""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
from humanoid_wbc.estimation.contact_schedule import ContactSchedule


def main():
    ds, ss = 0.1, 0.7
    sch = ContactSchedule(t_single=ss, t_double=ds)
    checks = []

    def expect(t, left, right, label):
        c = sch.contacts(t)
        ok = (c.left == left) and (c.right == right)
        checks.append(ok)
        print(f"  [{'ok' if ok else 'XX'}] t={t:5.2f}s  got L={int(c.left)} R={int(c.right)}  "
              f"want L={int(left)} R={int(right)}  ({label})")

    expect(0.00, True,  True,  "start double support")
    expect(0.05, True,  True,  "mid double support")
    expect(ds + ss/2,        True,  False, "right foot swinging")
    expect(ds + ss + ds/2,   True,  True,  "second double support")
    expect(2*ds + ss + ss/2, False, True,  "left foot swinging")
    expect(sch.cycle,        True,  True,  "cycle wrap -> double support")
    both_up = any((not sch.contacts(0.001*i).left) and (not sch.contacts(0.001*i).right)
                  for i in range(int(sch.cycle*1000)+1))
    checks.append(not both_up)
    print(f"  [{'ok' if not both_up else 'XX'}] never both feet airborne over a full cycle")

    ok = all(checks)
    print(f"test_contact_schedule: {'PASS' if ok else 'FAIL'} ({sum(checks)}/{len(checks)})")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
