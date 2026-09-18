# Simulation asset sources

P600 meshes/template, cylinder world and Prometheus textures were retained
from this repository at `b950945`.

The MID360 DAE mesh, `scan_mode/mid360.csv`, and GPS model were copied from the
existing `prometheus_px4/Tools/sitl_gazebo/models/` checkout. Its Git revision is
`9566172e7c0760f66681304b963d675c5b120daf`; the parent PX4 revision is
`713814f4eea5990e49dd776a38a36ad53e171f60`. The source checkout contains no
separate license file for these model directories; this refactor does not
assign them a new license. They are now local assets so the P600 model has no
user-specific paths or undeclared model dependencies in another checkout.

PX4 itself and its Gazebo flight-dynamics/GPS plugins remain external runtime
dependencies configured by `PX4_ROOT` and `PX4_BUILD_DIR`.

## XTDrone indoor4

`gazebo_worlds/planning_worlds/indoor4.world` was imported from
https://github.com/robin-shaun/XTDrone/blob/master/sitl_config/worlds/indoor4.world
on 2026-09-18. Original file SHA-256: `68df428917fca7877ae2b86326476112ba3c94d195294f008b0a35d9aae8f528`.
The upstream MIT license is preserved alongside the world as `LICENSE.XTDrone`.

The 150 maze walls and their original poses/collision geometry are retained.
The saved simulation state, saved GUI camera, and decorative arrow were removed.
The ceiling was made static to prevent it falling under gravity. No external
models, meshes, or XTDrone plugins are required; materials are Gazebo built-ins.
