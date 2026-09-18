#!/usr/bin/env python3
"""Render the P600 SDF to stdout for roslaunch."""
import argparse
from pathlib import Path
import jinja2
parser = argparse.ArgumentParser()
parser.add_argument("filename")
parser.add_argument("--mavlink_id", type=int, default=1)
parser.add_argument("--mavlink_tcp_port", type=int, default=4560)
parser.add_argument("--mavlink_udp_port", type=int, default=14560)
args = parser.parse_args()
model = Path(args.filename)
print(jinja2.Template(model.read_text(), undefined=jinja2.StrictUndefined).render(
    serial_enabled=0, serial_device="/dev/ttyACM0", serial_baudrate=921600, hil_mode=0, **vars(args)))
