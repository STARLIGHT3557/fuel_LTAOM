# Vendored FUEL

Source: https://github.com/HKUST-Aerial-Robotics/FUEL
Revision: `662dd23c7b52b258d3c4a0155ff6632118e8984f`.

All 1538 upstream tracked files are included, including `files/`,
`fuel_planner/`, `uav_simulator/`, README and LICENSE. There is no nested Git
repository or submodule. `UPSTREAM_MANIFEST.txt` records the original paths,
file modes and Git blob IDs. Demo source/assets remain available for reference;
only `fuel_planner/` is linked into this project's catkin `src/` tree.
The original upstream README describes its own standalone demo, not this
project's supported P600 launch. Use the root README for this integration.

Local changes applied to the working source:

- ROS Noetic NLopt discovery, C++14 and generated-message build dependencies.
- MID360 spherical field of view, mounting pitch and translated ray origin.
- Frontier detection ignores unknown neighbors outside the mission box and
  checks voxel indices before address conversion.
- Empty cube/sphere marker lists send DELETE only.
- Empty triggers are rejected; stop/reset allows exploration to restart.
- The trajectory server's automatic startup motion is removed; stop cancels
  streaming, and completed trajectories hold their effective endpoint.
- Default build excludes the unused demo executables `fast_planner_node`,
  `proc_msg`, `proc_msg2`, `obj_generator`, `write_tsp`, `load_obj`, and
  the standalone `lkh_tsp` CLI (the linked solver library is retained).
- Removed the unused `swarmtal_msgs` package dependency from `poly_traj`.

The upstream GPL-3.0 license and other bundled license files are retained.
Make further algorithm edits here, then run `bash compile_fuel.sh` from the
project root. No build-time download or patch application is needed.
