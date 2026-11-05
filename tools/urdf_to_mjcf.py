#!/usr/bin/env python3
"""Generate the MuJoCo scene from the xacro so URDF and MJCF never drift apart.

    python3 tools/urdf_to_mjcf.py \
        --xacro config/urdf/robot.urdf.xacro \
        --world config/worlds/flat.xml \
        --out config/mjcf/scene.xml
"""
import argparse
import pathlib
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import mujoco

ARM_JOINTS = [f"arm_joint{i}" for i in range(1, 7)]
WHEEL_JOINTS = ("wheel_left_joint", "wheel_right_joint")
CASTERS = (
    "caster_front_left",
    "caster_front_right",
    "caster_rear_left",
    "caster_rear_right",
)

def run_xacro(xacro_path: pathlib.Path) -> str:
    """Expand the xacro with use_ros2_control:=false and return the URDF text."""
    result = subprocess.run(
        ["xacro", str(xacro_path), "use_ros2_control:=false"],
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout


def urdf_to_mjcf(urdf_text: str, mesh_dir: pathlib.Path) -> str:
    urdf_text = urdf_text.replace(
        "package://wbc/meshes", mesh_dir.resolve().as_posix())
    insert = (
        "<mujoco><compiler"
        ' discardvisual="false"'
        ' fusestatic="false"'
        ' balanceinertia="true"'
        ' boundmass="0.001"'
        ' boundinertia="1e-6"/>'
        "</mujoco>"
    )
    start = urdf_text.find("<robot")
    gt = urdf_text.find(">", start)
    urdf_text = urdf_text[: gt + 1] + insert + urdf_text[gt + 1 :]
    model = mujoco.MjModel.from_xml_string(urdf_text)
    tmp = pathlib.Path(tempfile.mkstemp(suffix=".xml")[1])
    try:
        mujoco.mj_saveLastXML(str(tmp), model)
        return tmp.read_text()
    finally:
        tmp.unlink(missing_ok=True)


def _find(root: ET.Element, tag: str, name: str):
    for el in root.iter(tag):
        if el.get("name") == name:
            return el
    return None


def add_actuators_and_sensors(mjcf_text: str) -> str:
    root = ET.fromstring(mjcf_text)

    base = _find(root, "body", "base_link")
    if base is None:
        raise RuntimeError("URDF compile lost base_link")
    if base.find("freejoint") is None:
        base.insert(0, ET.Element("freejoint", name="base_free"))

    for body in root.iter("body"):
        for joint in list(body):
            if joint.tag == "joint" and joint.get("name") == "chassis_rgbd_joint":
                body.remove(joint)

    for name in CASTERS:
        body = _find(root, "body", name)
        if body is None:
            continue
        for geom in body.iter("geom"):
            geom.set("friction", "0.05 0.005 0.0001")

    # Visual geoms already have group=1. Put collision in group 3 so the viewer
    # can hide the chassis box without also hiding the floor (group 0).
    for geom in root.iter("geom"):
        if geom.get("group") != "1":
            geom.set("group", "3")

    sites = (("imu_link", "imu_site"),
             ("ft_sensor", "ft_site"),
             ("brush_tip", "brush_tip"))
    for body_name, site_name in sites:
        body = _find(root, "body", body_name)
        if body is None:
            raise RuntimeError(f"missing body {body_name}; fusestatic?")
        if all(c.get("name") != site_name for c in body.findall("site")):
            ET.SubElement(body, "site", name=site_name, size="0.01")

    actuator = root.find("actuator")
    if actuator is None:
        actuator = ET.SubElement(root, "actuator")
    for name in ARM_JOINTS:
        ET.SubElement(actuator, "motor", name=name, joint=name)
    for name in WHEEL_JOINTS:
        ET.SubElement(actuator, "velocity", name=name, joint=name, kv="100")

    sensor = root.find("sensor")
    if sensor is None:
        sensor = ET.SubElement(root, "sensor")
    ET.SubElement(sensor, "framequat", name="imu_orientation",
                  objtype="site", objname="imu_site")
    ET.SubElement(sensor, "gyro", name="imu_gyro", site="imu_site")
    ET.SubElement(sensor, "accelerometer", name="imu_accel", site="imu_site")
    ET.SubElement(sensor, "force", name="ft_force", site="ft_site")
    ET.SubElement(sensor, "torque", name="ft_torque", site="ft_site")

    return ET.tostring(root, encoding="unicode")


def wrap_in_world(mjcf_text: str, world: pathlib.Path) -> str:
    robot = ET.fromstring(mjcf_text)
    scene = ET.parse(world).getroot()

    r_asset, s_asset = robot.find("asset"), scene.find("asset")
    if r_asset is not None and s_asset is not None:
        for child in list(r_asset):
            s_asset.append(child)

    r_wb, s_wb = robot.find("worldbody"), scene.find("worldbody")
    if r_wb is not None and s_wb is not None:
        for child in list(r_wb):
            s_wb.append(child)

    for tag in ("compiler", "default", "contact", "actuator", "sensor"):
        src = robot.find(tag)
        if src is None:
            continue
        dst = scene.find(tag)
        if dst is None:
            scene.append(src)
        else:
            for child in list(src):
                dst.append(child)

    ET.indent(scene, space="  ")
    return ET.tostring(scene, encoding="unicode")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--xacro", type=pathlib.Path, required=True)
    parser.add_argument("--world", type=pathlib.Path, required=True)
    parser.add_argument("--out", type=pathlib.Path, required=True)
    args = parser.parse_args()

    urdf = run_xacro(args.xacro)
    mjcf = urdf_to_mjcf(urdf, args.xacro.parent.parent / "meshes")
    mjcf = add_actuators_and_sensors(mjcf)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(wrap_in_world(mjcf, args.world))


if __name__ == "__main__":
    main()
