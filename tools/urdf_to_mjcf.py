#!/usr/bin/env python3
"""Generate the MuJoCo scene from the xacro so URDF and MJCF never drift apart.

    python3 tools/urdf_to_mjcf.py \
        --xacro config/urdf/robot.urdf.xacro \
        --world config/worlds/flat.xml \
        --out config/mjcf/scene.xml
"""
import argparse
import pathlib


def run_xacro(xacro_path: pathlib.Path) -> str:
    """Expand the xacro with use_ros2_control:=false and return the URDF text."""
    # TODO(M0)
    raise NotImplementedError


def urdf_to_mjcf(urdf_text: str, mesh_dir: pathlib.Path) -> str:
    """Load the URDF with mujoco.MjModel.from_xml_string and dump the MJCF via mj_saveLastXML."""
    # TODO(M0)
    raise NotImplementedError


def add_actuators_and_sensors(mjcf_text: str) -> str:
    """Add free joint on base, wheel velocity actuators, arm motor actuators,
    imu / force-torque sensors, caster spheres with low friction."""
    # TODO(M0)
    raise NotImplementedError


def wrap_in_world(mjcf_text: str, world: pathlib.Path) -> str:
    """Merge the robot into a world file (floor, lights, obstacles)."""
    # TODO(M0)
    raise NotImplementedError


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--xacro", type=pathlib.Path, required=True)
    parser.add_argument("--world", type=pathlib.Path, required=True)
    parser.add_argument("--out", type=pathlib.Path, required=True)
    args = parser.parse_args()

    urdf = run_xacro(args.xacro)
    mjcf = urdf_to_mjcf(urdf, args.xacro.parent.parent / "meshes")
    mjcf = add_actuators_and_sensors(mjcf)
    args.out.write_text(wrap_in_world(mjcf, args.world))


if __name__ == "__main__":
    main()
