#!/usr/bin/env python3
"""SlamHub C1: insert ESKF pose covariance diagonal dump into FAST-LIO's
dump_lio_state_to_log(). Run on /catkin_ws/src/FAST_LIO/src/laserMapping.cpp.

Idempotent — exits 0 if the marker is already present.
"""
import sys
from pathlib import Path

path = Path(sys.argv[1]) if len(sys.argv) > 1 else \
    Path("/catkin_ws/src/FAST_LIO/src/laserMapping.cpp")
src = path.read_text()

if "SlamHub C1" in src:
    print("Patch already applied; skipping.")
    sys.exit(0)

marker = (
    'fprintf(fp, "%lf %lf %lf ", state_point.grav[0], '
    'state_point.grav[1], state_point.grav[2]);'
)
idx = src.find(marker)
if idx < 0:
    sys.exit("Marker not found; FAST-LIO source may have changed.")

end_of_line = src.find("\n", idx) + 1
patch = (
    "    // SlamHub C1: ESKF pose covariance diagonal "
    "(pos m^2 x3, rot rad^2 x3)\n"
    "    auto P_cov = kf.get_P();\n"
    "    fprintf(fp, \"%lf %lf %lf \", "
    "P_cov(0,0), P_cov(1,1), P_cov(2,2));\n"
    "    fprintf(fp, \"%lf %lf %lf \", "
    "P_cov(3,3), P_cov(4,4), P_cov(5,5));\n"
)
new_src = src[:end_of_line] + patch + src[end_of_line:]
path.write_text(new_src)
print(f"Patched {path}: inserted {patch.count(chr(10))} lines.")
