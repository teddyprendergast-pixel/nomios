"""MuJoCo scene wrapper generator for Nomios."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Union


def generate_scene_xml(
    output_path: Union[str, Path],
    system_xml_rel: str = "system.xml",
    meshdir: str = "../meshes/",
) -> Path:
    """Generate `models/scene.xml` with studio lighting, floor plane, and cameras."""
    out_file = Path(output_path).resolve()
    out_file.parent.mkdir(parents=True, exist_ok=True)

    root = ET.Element("mujoco", model="nomios_scene")
    ET.SubElement(
        root,
        "compiler",
        angle="radian",
        balanceinertia="true",
        discardvisual="false",
        meshdir=meshdir,
        coordinate="local",
        autolimits="true",
    )
    ET.SubElement(root, "statistic", center="0 0 0.30", extent="0.9")

    opt = ET.SubElement(
        root,
        "option",
        timestep="0.002",
        gravity="0 0 -9.81",
        integrator="implicitfast",
    )
    ET.SubElement(opt, "flag", contact="enable", energy="enable")

    visual = ET.SubElement(root, "visual")
    ET.SubElement(
        visual,
        "headlight",
        ambient="0.40 0.40 0.40",
        diffuse="0.80 0.80 0.80",
        specular="0.20 0.20 0.20",
    )
    ET.SubElement(visual, "rgba", haze="0.15 0.25 0.35 1.0")
    ET.SubElement(visual, "global", offwidth="1920", offheight="1080")
    ET.SubElement(visual, "quality", shadowsize="4096")

    asset = ET.SubElement(root, "asset")
    ET.SubElement(
        asset,
        "texture",
        type="skybox",
        builtin="gradient",
        rgb1="0.30 0.50 0.70",
        rgb2="0.05 0.05 0.08",
        width="512",
        height="512",
    )
    ET.SubElement(
        asset,
        "texture",
        name="grid_floor_tex",
        type="2d",
        builtin="checker",
        rgb1="0.22 0.25 0.28",
        rgb2="0.16 0.18 0.21",
        width="512",
        height="512",
        mark="edge",
        markrgb="0.75 0.75 0.75",
    )
    ET.SubElement(
        asset,
        "material",
        name="grid_floor_mat",
        texture="grid_floor_tex",
        texrepeat="10 10",
        reflectance="0.08",
        roughness="0.80",
        specular="0.12",
    )

    wb = ET.SubElement(root, "worldbody")
    ET.SubElement(
        wb,
        "light",
        name="main_light",
        pos="0.6 -1.2 2.8",
        dir="-0.25 0.40 -1.0",
        diffuse="0.88 0.88 0.88",
        specular="0.25 0.25 0.25",
        directional="true",
        castshadow="true",
    )
    ET.SubElement(
        wb,
        "light",
        name="fill_light",
        pos="-1.5 1.5 2.0",
        dir="0.50 -0.50 -1.0",
        diffuse="0.35 0.35 0.35",
        specular="0.05 0.05 0.05",
        directional="true",
    )
    ET.SubElement(
        wb,
        "geom",
        name="floor",
        type="plane",
        size="6.0 6.0 0.05",
        material="grid_floor_mat",
        friction="1.1 0.005 0.0001",
    )
    ET.SubElement(wb, "camera", name="isometric_cam", pos="0.85 -0.85 0.65", xyaxes="0.707 0.707 0 -0.35 0.35 0.87")
    ET.SubElement(wb, "camera", name="side_cam", pos="0 -1.1 0.35", xyaxes="1 0 0 0 0.25 0.97")
    ET.SubElement(wb, "camera", name="front_cam", pos="1.1 0 0.35", xyaxes="0 1 0 -0.25 0 0.97")
    ET.SubElement(wb, "camera", name="top_cam", pos="0 0 1.6", xyaxes="1 0 0 0 1 0")

    ET.SubElement(root, "include", file=system_xml_rel)

    ET.indent(root, space="  ", level=0)
    ET.ElementTree(root).write(out_file, encoding="utf-8", xml_declaration=True)
    return out_file
