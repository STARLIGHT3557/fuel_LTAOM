# Vendored FUEL

Source: https://github.com/HKUST-Aerial-Robotics/FUEL
Revision: `662dd23c7b52b258d3c4a0155ff6632118e8984f`.

Only the planning tree is linked into the catkin workspace via `src/fuel`.
This branch removes the upstream simulator, sample environments, media and
standalone demo launches. `UPSTREAM_MANIFEST.txt` preserves the original import
paths, modes and blob IDs; it is not a manifest of files retained in this branch.
The GPL-3.0 license and licenses of retained third-party code are preserved.

Local adaptations:

- ROS Noetic NLopt discovery, C++14 and generated-message build dependencies.
- MID360 spherical field of view, full mounting rotation and translated ray origin.
- Shared mounting parameters derived from the onboard Faster-LIO configuration.
- Configurable world frame for trajectories, task status, maps and visualization.
- Frontier detection ignores unknown neighbors outside the mission box and
  checks voxel indices before address conversion.
- Empty cube/sphere marker lists send DELETE only.
- Empty triggers are rejected; stop/reset allows exploration to restart.
- Automatic startup motion is removed; stop cancels streaming.
- Manager-owned task authorization, continuous trajectory handover, exploration
  timeout recovery and persistent obstacle-aware return planning are retained.
- Unused demo executables remain excluded from the build; the linked LKH TSP
  solver library is retained.
- Removed the unused swarmtal_msgs dependency from poly_traj.

Further algorithm edits belong here. Run `bash compile_fuel.sh` from the root;
no build-time download or patch application is required.
