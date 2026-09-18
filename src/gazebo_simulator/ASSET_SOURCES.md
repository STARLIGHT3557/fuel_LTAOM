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
