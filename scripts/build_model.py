"""
Build Nomios STL meshes, URDF, and MuJoCo MJCF files directly from the SolidWorks
assembly (`parts/full assembly.SLDASM`) and its referenced `.SLDPRT` files.
"""

from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from scripts.scene_gen import generate_scene_xml
from scripts.sld_io import extract_tessellation_tris, read_binary_stl, unpack_sld_streams, write_binary_stl

SERVO_RGBA = (0.18, 0.18, 0.20, 1.0)
SHELL_RGBA = (0.92, 0.92, 0.94, 1.0)
STRUCT_RGBA = (0.90, 0.90, 0.92, 1.0)
CAMERA_PCB_RGBA = (0.10, 0.48, 0.32, 1.0)
BRACKET_RGBA = (0.20, 0.22, 0.25, 1.0)

# Rigid torso/head components attached to `torso_base`: (stl_name, solidworks_label, rgba)
TORSO_COMPONENTS: List[Tuple[str, str, Tuple[float, float, float, float]]] = [
    ("head_1", "head[1]", (0.90, 0.90, 0.90, 1.0)),
    ("arduino_uno_q_housing_1", "arduino_uno_q_housing[1]", (0.85, 0.85, 0.88, 1.0)),
    ("Arduino_UNO_Q_1", "Arduino_UNO_Q[1]", (0.12, 0.45, 0.35, 1.0)),
    ("media_carrier_1", "media carrier[1]", (0.85, 0.85, 0.88, 1.0)),
    ("battery_1", "battery[1]", (0.12, 0.12, 0.14, 1.0)),
    ("battery_2", "battery[2]", (0.12, 0.12, 0.14, 1.0)),
    ("pi_camera_v2_stereo_bracket_80mm_1", "pi_camera_v2_stereo_bracket_80mm[1]", BRACKET_RGBA),
    ("pi_camera_v2_8mp_3", "pi_camera_v2_8mp[3]", CAMERA_PCB_RGBA),
    ("pi_camera_v2_8mp_4", "pi_camera_v2_8mp[4]", CAMERA_PCB_RGBA),
    ("pi_camera_v2_8mp_5", "pi_camera_v2_8mp[5]", CAMERA_PCB_RGBA),
    ("left_hip_yaw_servo", "servo_assembly[10]/servo_STS3215_solid[1]", SERVO_RGBA),
    ("left_hip_yaw_clamshell_base", "servo_assembly[10]/clamshell_base[2]", SHELL_RGBA),
    ("left_hip_yaw_clamshell_cap", "servo_assembly[10]/clamshell_cap[2]", SHELL_RGBA),
    ("right_hip_yaw_servo", "servo_assembly[11]/servo_STS3215_solid[1]", SERVO_RGBA),
    ("right_hip_yaw_clamshell_base", "servo_assembly[11]/clamshell_base[2]", SHELL_RGBA),
    ("right_hip_yaw_clamshell_cap", "servo_assembly[11]/clamshell_cap[2]", SHELL_RGBA),
]

# Link ownership for the 40 leg components (housing stays on parent, U-bracket drives child)
LEG_COMPONENTS: List[Tuple[str, str, str, Tuple[float, float, float, float]]] = [
    # Left leg
    ("left_hip_yaw_u_bracket", "left_hip_yaw_link", "u_bracket_v2[10]", SHELL_RGBA),
    ("left_hip_servo", "left_hip_yaw_link", "servo_assembly[9]/servo_STS3215_solid[1]", SERVO_RGBA),
    ("left_hip_clamshell_base", "left_hip_yaw_link", "servo_assembly[9]/clamshell_base[2]", SHELL_RGBA),
    ("left_hip_clamshell_cap", "left_hip_yaw_link", "servo_assembly[9]/clamshell_cap[2]", SHELL_RGBA),
    ("left_hip_u_bracket", "left_hip_roll_link", "u_bracket_v2[4]", SHELL_RGBA),
    ("left_thigh_u_bracket", "left_hip_roll_link", "u_bracket_v2[5]", SHELL_RGBA),
    ("left_thigh_servo", "left_thigh_link", "servo_assembly[3]/servo_STS3215_solid[1]", SERVO_RGBA),
    ("left_thigh_clamshell_base", "left_thigh_link", "servo_assembly[3]/clamshell_base[2]", SHELL_RGBA),
    ("left_thigh_clamshell_cap", "left_thigh_link", "servo_assembly[3]/clamshell_cap[2]", SHELL_RGBA),
    ("left_upper_shin", "left_thigh_link", "upper_shin_v1[4]", STRUCT_RGBA),
    ("left_knee_u_bracket", "left_thigh_link", "u_bracket_v2[8]", SHELL_RGBA),
    ("left_knee_servo", "left_shin_link", "servo_STS3215_solid[8]", SERVO_RGBA),
    ("left_knee_clamshell_base", "left_shin_link", "clamshell_base[3]", SHELL_RGBA),
    ("left_knee_clamshell_cap", "left_shin_link", "clamshell_cap[3]", SHELL_RGBA),
    ("left_lower_shin", "left_shin_link", "lower_shin_v1[2]", STRUCT_RGBA),
    ("left_ankle_u_bracket", "left_shin_link", "u_bracket_v2[6]", SHELL_RGBA),
    ("left_ankle_servo", "left_foot_link", "servo_STS3215_solid[7]", SERVO_RGBA),
    ("left_ankle_clamshell_base", "left_foot_link", "clamshell_base[4]", SHELL_RGBA),
    ("left_ankle_clamshell_cap", "left_foot_link", "clamshell_cap[4]", SHELL_RGBA),
    ("left_foot", "left_foot_link", "foot_v1[1]", STRUCT_RGBA),
    # Right leg
    ("right_hip_yaw_u_bracket", "right_hip_yaw_link", "u_bracket_v2[9]", SHELL_RGBA),
    ("right_hip_servo", "right_hip_yaw_link", "servo_assembly[8]/servo_STS3215_solid[1]", SERVO_RGBA),
    ("right_hip_clamshell_base", "right_hip_yaw_link", "servo_assembly[8]/clamshell_base[2]", SHELL_RGBA),
    ("right_hip_clamshell_cap", "right_hip_yaw_link", "servo_assembly[8]/clamshell_cap[2]", SHELL_RGBA),
    ("right_hip_u_bracket", "right_hip_roll_link", "u_bracket_v2[2]", SHELL_RGBA),
    ("right_thigh_u_bracket", "right_hip_roll_link", "u_bracket_v2[3]", SHELL_RGBA),
    ("right_thigh_servo", "right_thigh_link", "servo_assembly[4]/servo_STS3215_solid[1]", SERVO_RGBA),
    ("right_thigh_clamshell_base", "right_thigh_link", "servo_assembly[4]/clamshell_base[2]", SHELL_RGBA),
    ("right_thigh_clamshell_cap", "right_thigh_link", "servo_assembly[4]/clamshell_cap[2]", SHELL_RGBA),
    ("right_upper_shin", "right_thigh_link", "upper_shin_v1[3]", STRUCT_RGBA),
    ("right_knee_u_bracket", "right_thigh_link", "u_bracket_v2[7]", SHELL_RGBA),
    ("right_knee_servo", "right_shin_link", "servo_STS3215_solid[10]", SERVO_RGBA),
    ("right_knee_clamshell_base", "right_shin_link", "clamshell_base[1]", SHELL_RGBA),
    ("right_knee_clamshell_cap", "right_shin_link", "clamshell_cap[1]", SHELL_RGBA),
    ("right_lower_shin", "right_shin_link", "lower_shin_v1[1]", STRUCT_RGBA),
    ("right_ankle_u_bracket", "right_shin_link", "u_bracket_v2[1]", SHELL_RGBA),
    ("right_ankle_servo", "right_foot_link", "servo_STS3215_solid[9]", SERVO_RGBA),
    ("right_ankle_clamshell_base", "right_foot_link", "clamshell_base[2]", SHELL_RGBA),
    ("right_ankle_clamshell_cap", "right_foot_link", "clamshell_cap[2]", SHELL_RGBA),
    ("right_foot", "right_foot_link", "foot_v1[2]", STRUCT_RGBA),
]

# 10-DoF kinematic tree:
# (link_name, parent_link, joint_name, range_rad, short_name, sw_u_bracket_label, axis_sign)
JOINT_TREE: List[Tuple[str, str, str, Tuple[float, float], str, str, Tuple[float, float, float]]] = [
    ("left_hip_yaw_link", "torso_base", "joint_left_hip_yaw", (-0.7854, 0.7854), "left_hip_yaw", "u_bracket_v2[10]", (0.0, 0.0, 1.0)),
    ("left_hip_roll_link", "left_hip_yaw_link", "joint_left_hip_roll", (-0.7854, 0.7854), "left_hip_roll", "u_bracket_v2[4]", (1.0, 0.0, 0.0)),
    ("left_thigh_link", "left_hip_roll_link", "joint_left_hip_pitch", (-1.5708, 1.5708), "left_hip_pitch", "u_bracket_v2[5]", (0.0, 1.0, 0.0)),
    ("left_shin_link", "left_thigh_link", "joint_left_knee_pitch", (-2.0944, 2.0944), "left_knee_pitch", "u_bracket_v2[8]", (0.0, 1.0, 0.0)),
    ("left_foot_link", "left_shin_link", "joint_left_ankle_pitch", (-1.2217, 1.2217), "left_ankle_pitch", "u_bracket_v2[6]", (0.0, 1.0, 0.0)),
    ("right_hip_yaw_link", "torso_base", "joint_right_hip_yaw", (-0.7854, 0.7854), "right_hip_yaw", "u_bracket_v2[9]", (0.0, 0.0, 1.0)),
    ("right_hip_roll_link", "right_hip_yaw_link", "joint_right_hip_roll", (-0.7854, 0.7854), "right_hip_roll", "u_bracket_v2[2]", (-1.0, 0.0, 0.0)),
    ("right_thigh_link", "right_hip_roll_link", "joint_right_hip_pitch", (-1.5708, 1.5708), "right_hip_pitch", "u_bracket_v2[3]", (0.0, 1.0, 0.0)),
    ("right_shin_link", "right_thigh_link", "joint_right_knee_pitch", (-2.0944, 2.0944), "right_knee_pitch", "u_bracket_v2[7]", (0.0, 1.0, 0.0)),
    ("right_foot_link", "right_shin_link", "joint_right_ankle_pitch", (-1.2217, 1.2217), "right_ankle_pitch", "u_bracket_v2[1]", (0.0, 1.0, 0.0)),
]

LINK_MASSES: Dict[str, float] = {
    "torso_base": 0.4800,
    "left_hip_yaw_link": 0.0840,
    "left_hip_roll_link": 0.0754,
    "left_thigh_link": 0.0960,
    "left_shin_link": 0.0861,
    "left_foot_link": 0.1224,
    "right_hip_yaw_link": 0.0840,
    "right_hip_roll_link": 0.0754,
    "right_thigh_link": 0.0960,
    "right_shin_link": 0.0861,
    "right_foot_link": 0.1224,
}

CAMERA_SPECS_DEF: List[Tuple[str, str, str, str]] = [
    ("left_stereo_cam", "left_stereo_camera_site", "left_stereo_camera_link", "pi_camera_v2_8mp[3]"),
    ("right_stereo_cam", "right_stereo_camera_site", "right_stereo_camera_link", "pi_camera_v2_8mp[5]"),
    ("top_head_cam", "top_camera_site", "top_camera_link", "pi_camera_v2_8mp[4]"),
]


def _make_mat4(r: np.ndarray, t: np.ndarray) -> np.ndarray:
    m = np.eye(4, dtype=np.float64)
    m[:3, :3] = r
    m[:3, 3] = t
    return m


def _resolve_sldprt_path(stored_path: str, parts_dir: Path) -> Path:
    """Resolve an assembly-referenced SolidWorks part path inside `parts_dir`."""
    cand = Path(stored_path)
    if cand.exists():
        return cand
    direct = parts_dir / cand.name
    if direct.exists():
        return direct
    matches = list(parts_dir.rglob(cand.name))
    if matches:
        return matches[0]
    raise FileNotFoundError(f"Referenced SolidWorks part not found: {stored_path}")


def _parse_sw_assembly(
    sldasm_path: Path,
    parts_dir: Path,
) -> Tuple[Dict[str, np.ndarray], Dict[str, np.ndarray]]:
    """Extract component transforms and part tessellations from `full assembly.SLDASM`."""
    streams = unpack_sld_streams(sldasm_path)
    raw = streams["swXmlContents/COMPINSTANCETREE"]
    troot = ET.fromstring(raw.decode("utf-8").lstrip("\ufeff"))
    ns = {"sw": "http://www.solidworks.com/sw2003/schema"}

    files = {f.attrib["id"]: f.attrib["swPath"] for f in troot.findall(".//sw:swFile", ns)}
    models = {m.attrib["id"]: m for m in troot.findall(".//sw:swModel", ns)}
    full_asm = next(m for m in models.values() if m.attrib.get("swName") == "full assembly")

    b107 = streams.get("FaceTessellations/000-000-107", b"")
    face_sig = b"\x04\x00\x00\x00\x08\x00\x00\x00\x02\x00\x00\x00"

    part_tris: Dict[str, np.ndarray] = {}
    for model in models.values():
        file_ref = model.attrib.get("swFileRef")
        if not file_ref or file_ref not in files:
            continue
        stored = files[file_ref]
        if not stored.lower().endswith(".sldprt"):
            continue
        part_path = _resolve_sldprt_path(stored, parts_dir)
        part_streams = unpack_sld_streams(part_path)
        if "Contents/DisplayLists" not in part_streams:
            raise RuntimeError(f"Missing Contents/DisplayLists in {part_path}")
        dl = part_streams["Contents/DisplayLists"]
        tris = extract_tessellation_tris(dl)
        if len(tris) == 0:
            raise RuntimeError(f"No triangles extracted from {part_path}")

        # The multi-body `head` part caches its rear plate in the assembly tessellation stream.
        if model.attrib["swName"] == "head" and b107:
            first_sig = dl.find(face_sig)
            if first_sig != -1:
                front_start = b107.find(dl[first_sig : first_sig + 64])
                if front_start > 0:
                    back_tris = extract_tessellation_tris(b107[:front_start])
                    if len(back_tris) > 0:
                        tris = np.concatenate([back_tris, tris], axis=0)

        part_tris[model.attrib["swName"]] = tris

    def to_mat4(s: str) -> np.ndarray:
        return np.array([float(x) for x in s.split()], dtype=np.float64).reshape(4, 4).T

    leaf_transforms: Dict[str, np.ndarray] = {}
    for ref in full_asm.findall("sw:swReference", ns):
        if ref.attrib.get("swSuppressed") == "YES":
            continue
        t_ref = to_mat4(ref.attrib["swTransform"])
        sub = models[ref.attrib["swModelRef"]]
        sub_refs = sub.findall("sw:swReference", ns)
        if sub_refs:
            for sref in sub_refs:
                if sref.attrib.get("swSuppressed") == "YES":
                    continue
                t_sub = to_mat4(sref.attrib["swTransform"])
                lbl = (
                    f"{ref.attrib['swName']}[{ref.attrib['swReferenceNumber']}]/"
                    f"{sref.attrib['swName']}[{sref.attrib['swReferenceNumber']}]"
                )
                leaf_transforms[lbl] = t_ref @ t_sub
        else:
            lbl = f"{ref.attrib['swName']}[{ref.attrib['swReferenceNumber']}]"
            leaf_transforms[lbl] = t_ref

    return leaf_transforms, part_tris


def _compute_sw_to_mj_transform(sw_transforms: Dict[str, np.ndarray]) -> np.ndarray:
    """Map SolidWorks assembly coordinates into MuJoCo's Z-up torso frame."""
    t_head = sw_transforms["head[1]"]
    p_axis = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float64)
    r_sw_to_mj = p_axis @ t_head[:3, :3].T
    t_sw_to_mj = -r_sw_to_mj @ t_head[:3, 3]
    return _make_mat4(r_sw_to_mj, t_sw_to_mj)


def _compute_cad_joint_poses(
    t_sw_to_mj: np.ndarray,
    u_bracket_tris: np.ndarray,
    sw_transforms: Dict[str, np.ndarray],
) -> Tuple[Dict[str, np.ndarray], Dict[str, np.ndarray]]:
    """Recover shaft axes and pivot midpoints directly from `u_bracket_v2` geometry."""
    v0, v1, v2 = u_bracket_tris[:, 0], u_bracket_tris[:, 1], u_bracket_tris[:, 2]
    cr = np.cross(v1 - v0, v2 - v0)
    areas = 0.5 * np.linalg.norm(cr, axis=1)
    norms = cr / (2.0 * areas[:, None] + 1e-12)

    z_mask = np.abs(norms[:, 2]) > 0.98
    e_u_sw = (norms[z_mask] * np.sign(norms[z_mask, 2])[:, None] * areas[z_mask, None]).sum(axis=0)
    e_u_sw /= np.linalg.norm(e_u_sw)

    centers = u_bracket_tris.mean(axis=1)
    z_outer = float(centers[norms[:, 2] > 0.98, 2].max())
    hole_tris = u_bracket_tris[
        (np.abs(norms[:, 2]) < 0.15)
        & (centers[:, 2] > z_outer - 0.003)
        & (centers[:, 2] < z_outer)
        & (centers[:, 0] > -0.022)
        & (centers[:, 0] < -0.002)
    ]
    hole_pts = np.unique(hole_tris.reshape(-1, 3), axis=0)
    center_hole = hole_pts[np.linalg.norm(hole_pts[:, :2] - np.array([-0.012, 0.0]), axis=1) < 0.0035]
    if len(center_hole) < 6:
        raise RuntimeError(f"Failed to detect u_bracket center horn hole (got {len(center_hole)} pts)")

    u_pts = np.unique(u_bracket_tris.reshape(-1, 3), axis=0)
    z_mid = 0.5 * float(u_pts[:, 2].min() + u_pts[:, 2].max())
    p_u_sw = np.array([float(center_hole[:, 0].mean()), float(center_hole[:, 1].mean()), z_mid], dtype=np.float64)

    pivots_mj: Dict[str, np.ndarray] = {"torso_base": np.zeros(3, dtype=np.float64)}
    axes_cad: Dict[str, np.ndarray] = {}
    for link_name, _, _, _, short_name, sw_ubrk_lbl, ref_sign in JOINT_TREE:
        t_world = t_sw_to_mj @ sw_transforms[sw_ubrk_lbl]
        pivots_mj[link_name] = (t_world @ np.append(p_u_sw, 1.0))[:3]
        ax = t_world[:3, :3] @ e_u_sw
        ax = ax / np.linalg.norm(ax)
        if float(np.dot(ax, np.asarray(ref_sign, dtype=np.float64))) < 0.0:
            ax = -ax
        axes_cad[short_name] = ax
    return pivots_mj, axes_cad


def _rotmat_to_rpy(r: np.ndarray) -> Tuple[float, float, float]:
    """Convert a 3x3 rotation matrix to extrinsic XYZ (roll, pitch, yaw) radians."""
    sy = float(np.hypot(r[0, 0], r[1, 0]))
    if sy > 1e-6:
        roll = float(np.arctan2(r[2, 1], r[2, 2]))
        pitch = float(np.arctan2(-r[2, 0], sy))
        yaw = float(np.arctan2(r[1, 0], r[0, 0]))
    else:
        roll = float(np.arctan2(-r[1, 2], r[1, 1]))
        pitch = float(np.arctan2(-r[2, 0], sy))
        yaw = 0.0
    return roll, pitch, yaw


def _compute_camera_specs(
    t_sw_to_mj: np.ndarray,
    cam_tris: np.ndarray,
    sw_transforms: Dict[str, np.ndarray],
) -> List[Dict[str, object]]:
    """Compute camera extrinsics from `pi_camera_v2_8mp` lens positions in CAD."""
    cam_pts = np.unique(cam_tris.reshape(-1, 3), axis=0)
    z_lens = float(cam_pts[:, 2].max())
    lens_tip_local = np.array([0.0, 0.0, z_lens], dtype=np.float64)

    specs: List[Dict[str, object]] = []
    for cam_name, site_name, urdf_link, sw_lbl in CAMERA_SPECS_DEF:
        t_mj = t_sw_to_mj @ sw_transforms[sw_lbl]
        pos = (t_mj @ np.append(lens_tip_local, 1.0))[:3]

        look = t_mj[:3, :3] @ np.array([0.0, 0.0, 1.0], dtype=np.float64)
        look /= np.linalg.norm(look)

        # MuJoCo cameras look along -Z_cam with +X_cam right and +Y_cam up
        z_cam = -look
        up_ref = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        x_cam = np.cross(up_ref, z_cam)
        x_cam /= np.linalg.norm(x_cam)
        y_cam = np.cross(z_cam, x_cam)
        y_cam /= np.linalg.norm(y_cam)

        r_urdf = np.column_stack([look, -x_cam, y_cam])
        rpy = _rotmat_to_rpy(r_urdf)

        specs.append(
            {
                "cam_name": cam_name,
                "site_name": site_name,
                "urdf_link": urdf_link,
                "pos": pos,
                "look": look,
                "xyaxes": np.concatenate([x_cam, y_cam]),
                "rpy": rpy,
            }
        )
    return specs


def _add_urdf_sensor_link(
    robot: ET.Element,
    parent_link: str,
    link_name: str,
    joint_name: str,
    xyz: np.ndarray,
    rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    link_el = ET.SubElement(robot, "link", name=link_name)
    in_el = ET.SubElement(link_el, "inertial")
    ET.SubElement(in_el, "origin", xyz="0 0 0", rpy="0 0 0")
    ET.SubElement(in_el, "mass", value="0.001000")
    ET.SubElement(in_el, "inertia", ixx="1.0e-07", ixy="0", ixz="0", iyy="1.0e-07", iyz="0", izz="1.0e-07")

    j_el = ET.SubElement(robot, "joint", name=joint_name, type="fixed")
    ET.SubElement(j_el, "parent", link=parent_link)
    ET.SubElement(j_el, "child", link=link_name)
    ET.SubElement(
        j_el,
        "origin",
        xyz=f"{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}",
        rpy=f"{rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}",
    )


def _write_urdf(
    urdf_path: Path,
    pivots_mj: Dict[str, np.ndarray],
    axes_cad: Dict[str, np.ndarray],
    link_geoms: Dict[str, List[Tuple[str, Tuple[float, float, float, float]]]],
    link_inertials: Dict[str, Tuple[np.ndarray, float, np.ndarray]],
    link_col_boxes: Dict[str, List[Tuple[str, np.ndarray, np.ndarray]]],
    foot_soles: Dict[str, Tuple[np.ndarray, np.ndarray, float]],
    imu_pos: np.ndarray,
    camera_specs: List[Dict[str, object]],
) -> None:
    """Write `urdf/nomios.urdf` with 10-DoF links, colliders, and sensors."""
    urdf_path.parent.mkdir(parents=True, exist_ok=True)
    robot = ET.Element("robot", name="nomios")

    mj_ext = ET.SubElement(robot, "mujoco")
    ET.SubElement(
        mj_ext,
        "compiler",
        angle="radian",
        balanceinertia="true",
        discardvisual="false",
        fusestatic="false",
        meshdir="../meshes/",
    )

    all_links = ["torso_base"] + [link for link, *_ in JOINT_TREE]
    for lname in all_links:
        link_el = ET.SubElement(robot, "link", name=lname)
        com, mass, diag_i = link_inertials[lname]
        in_el = ET.SubElement(link_el, "inertial")
        ET.SubElement(in_el, "origin", xyz=f"{com[0]:.6f} {com[1]:.6f} {com[2]:.6f}", rpy="0 0 0")
        ET.SubElement(in_el, "mass", value=f"{mass:.6f}")
        ET.SubElement(
            in_el,
            "inertia",
            ixx=f"{diag_i[0]:.6e}",
            ixy="0",
            ixz="0",
            iyy=f"{diag_i[1]:.6e}",
            iyz="0",
            izz=f"{diag_i[2]:.6e}",
        )
        for stl_name, rgba in link_geoms[lname]:
            vis = ET.SubElement(link_el, "visual")
            ET.SubElement(vis, "origin", xyz="0 0 0", rpy="0 0 0")
            geom = ET.SubElement(vis, "geometry")
            ET.SubElement(geom, "mesh", filename=f"../meshes/{stl_name}.STL")
            mat = ET.SubElement(vis, "material", name=f"mat_{stl_name}")
            ET.SubElement(mat, "color", rgba=f"{rgba[0]} {rgba[1]} {rgba[2]} {rgba[3]}")

        for _, c_pos, c_half in link_col_boxes.get(lname, []):
            col = ET.SubElement(link_el, "collision")
            ET.SubElement(col, "origin", xyz=f"{c_pos[0]:.6f} {c_pos[1]:.6f} {c_pos[2]:.6f}", rpy="0 0 0")
            cgeom = ET.SubElement(col, "geometry")
            ET.SubElement(cgeom, "box", size=f"{2.0 * c_half[0]:.6f} {2.0 * c_half[1]:.6f} {2.0 * c_half[2]:.6f}")

        if lname in ("left_foot_link", "right_foot_link"):
            side = "left" if lname.startswith("left") else "right"
            c_xy, half_xy, zmin = foot_soles[side]
            hz = 0.004
            sx = max(float(half_xy[0]), 0.025)
            sy = max(float(half_xy[1]), 0.025)
            col = ET.SubElement(link_el, "collision")
            ET.SubElement(col, "origin", xyz=f"{c_xy[0]:.6f} {c_xy[1]:.6f} {zmin + hz:.6f}", rpy="0 0 0")
            cgeom = ET.SubElement(col, "geometry")
            ET.SubElement(cgeom, "box", size=f"{2.0 * sx:.6f} {2.0 * sy:.6f} {2.0 * hz:.6f}")

    for child_link, parent_link, jname, (lo, hi), short_name, _, _ in JOINT_TREE:
        rel = pivots_mj[child_link] - pivots_mj[parent_link]
        ax = axes_cad[short_name]
        j_el = ET.SubElement(robot, "joint", name=jname, type="revolute")
        ET.SubElement(j_el, "parent", link=parent_link)
        ET.SubElement(j_el, "child", link=child_link)
        ET.SubElement(j_el, "origin", xyz=f"{rel[0]:.6f} {rel[1]:.6f} {rel[2]:.6f}", rpy="0 0 0")
        ET.SubElement(j_el, "axis", xyz=f"{ax[0]:.6f} {ax[1]:.6f} {ax[2]:.6f}")
        ET.SubElement(j_el, "limit", lower=f"{lo}", upper=f"{hi}", effort="3.0", velocity="6.0")

    _add_urdf_sensor_link(robot, "torso_base", "imu_link", "joint_imu_fixed", imu_pos)
    for cam in camera_specs:
        _add_urdf_sensor_link(
            robot,
            "torso_base",
            str(cam["urdf_link"]),
            f"joint_{cam['cam_name']}_fixed",
            np.asarray(cam["pos"], dtype=np.float64),
            rpy=cam["rpy"],  # type: ignore[arg-type]
        )
    for side in ("left", "right"):
        c_xy, _, zmin = foot_soles[side]
        _add_urdf_sensor_link(
            robot,
            f"{side}_foot_link",
            f"{side}_foot_contact_link",
            f"joint_{side}_foot_contact_fixed",
            np.array([c_xy[0], c_xy[1], zmin + 0.004], dtype=np.float64),
        )

    gz_imu = ET.SubElement(robot, "gazebo", reference="imu_link")
    imu_sens = ET.SubElement(gz_imu, "sensor", name="bno085_imu", type="imu")
    ET.SubElement(imu_sens, "always_on").text = "true"
    ET.SubElement(imu_sens, "update_rate").text = "200"
    mag_sens = ET.SubElement(gz_imu, "sensor", name="bno085_magnetometer", type="magnetometer")
    ET.SubElement(mag_sens, "always_on").text = "true"
    ET.SubElement(mag_sens, "update_rate").text = "100"

    for cam in camera_specs:
        gz_cam = ET.SubElement(robot, "gazebo", reference=str(cam["urdf_link"]))
        cam_sens = ET.SubElement(gz_cam, "sensor", name=str(cam["cam_name"]), type="camera")
        ET.SubElement(cam_sens, "update_rate").text = "30"
        cam_cfg = ET.SubElement(cam_sens, "camera", name=str(cam["cam_name"]))
        ET.SubElement(cam_cfg, "horizontal_fov").text = "1.0856"
        img_cfg = ET.SubElement(cam_cfg, "image")
        ET.SubElement(img_cfg, "width").text = "1280"
        ET.SubElement(img_cfg, "height").text = "720"

    for side in ("left", "right"):
        gz_foot = ET.SubElement(robot, "gazebo", reference=f"{side}_foot_link")
        ft_sens = ET.SubElement(gz_foot, "sensor", name=f"{side}_foot_contact", type="contact")
        ET.SubElement(ft_sens, "always_on").text = "true"
        ET.SubElement(ft_sens, "update_rate").text = "500"

    ET.indent(robot, space="  ", level=0)
    ET.ElementTree(robot).write(urdf_path, encoding="utf-8", xml_declaration=True)


def _write_system_xml(
    system_xml_path: Path,
    base_z: float,
    pivots_mj: Dict[str, np.ndarray],
    axes_cad: Dict[str, np.ndarray],
    all_stl_names: List[str],
    link_geoms: Dict[str, List[Tuple[str, Tuple[float, float, float, float]]]],
    link_inertials: Dict[str, Tuple[np.ndarray, float, np.ndarray]],
    link_col_boxes: Dict[str, List[Tuple[str, np.ndarray, np.ndarray]]],
    foot_soles: Dict[str, Tuple[np.ndarray, np.ndarray, float]],
    imu_pos: np.ndarray,
    camera_specs: List[Dict[str, object]],
) -> None:
    """Write `models/system.xml` with body frames, colliders, cameras, actuators, and sensors."""
    root = ET.Element("mujoco", model="nomios_system")
    ET.SubElement(
        root,
        "compiler",
        angle="radian",
        balanceinertia="true",
        discardvisual="false",
        meshdir="../meshes/",
        coordinate="local",
        autolimits="true",
    )

    asset = ET.SubElement(root, "asset")
    for sname in all_stl_names:
        ET.SubElement(asset, "mesh", name=sname, file=f"{sname}.STL")

    wb = ET.SubElement(root, "worldbody")
    torso = ET.SubElement(wb, "body", name="torso_base", pos=f"0 0 {base_z:.6f}")
    ET.SubElement(torso, "freejoint", name="root")
    base_link = ET.SubElement(torso, "body", name="base_link", pos="0 0 0")

    t_com, t_mass, t_diag = link_inertials["torso_base"]
    ET.SubElement(
        base_link,
        "inertial",
        pos=f"{t_com[0]:.6f} {t_com[1]:.6f} {t_com[2]:.6f}",
        mass=f"{t_mass:.4f}",
        diaginertia=f"{t_diag[0]:.6f} {t_diag[1]:.6f} {t_diag[2]:.6f}",
    )
    ET.SubElement(
        base_link,
        "site",
        name="imu_site",
        pos=f"{imu_pos[0]:.6f} {imu_pos[1]:.6f} {imu_pos[2]:.6f}",
        size="0.012",
        group="3",
        rgba="0.9 0.2 0.2 1",
    )

    for cam in camera_specs:
        cpos = np.asarray(cam["pos"], dtype=np.float64)
        cxy = np.asarray(cam["xyaxes"], dtype=np.float64)
        ET.SubElement(
            base_link,
            "camera",
            name=str(cam["cam_name"]),
            pos=f"{cpos[0]:.6f} {cpos[1]:.6f} {cpos[2]:.6f}",
            xyaxes=f"{cxy[0]:.6f} {cxy[1]:.6f} {cxy[2]:.6f} {cxy[3]:.6f} {cxy[4]:.6f} {cxy[5]:.6f}",
            fovy="48.8",
        )
        ET.SubElement(
            base_link,
            "site",
            name=str(cam["site_name"]),
            pos=f"{cpos[0]:.6f} {cpos[1]:.6f} {cpos[2]:.6f}",
            size="0.005",
            group="3",
            rgba="0.1 0.8 0.9 0.8",
        )

    for col_name, c_pos, c_size in link_col_boxes["torso_base"]:
        ET.SubElement(
            base_link,
            "geom",
            name=col_name,
            type="box",
            pos=f"{c_pos[0]:.6f} {c_pos[1]:.6f} {c_pos[2]:.6f}",
            size=f"{c_size[0]:.6f} {c_size[1]:.6f} {c_size[2]:.6f}",
            rgba="0.3 0.3 0.35 0.4",
            group="3",
            contype="1",
            conaffinity="1",
            friction="0.8 0.005 0.0001",
        )
    for sname, rgba in link_geoms["torso_base"]:
        ET.SubElement(
            base_link,
            "geom",
            name=f"geom_{sname}",
            type="mesh",
            mesh=sname,
            rgba=f"{rgba[0]} {rgba[1]} {rgba[2]} {rgba[3]}",
            contype="0",
            conaffinity="0",
        )

    body_elements: Dict[str, ET.Element] = {"torso_base": base_link}
    for child_link, parent_link, jname, (lo, hi), short_name, _, _ in JOINT_TREE:
        rel = pivots_mj[child_link] - pivots_mj[parent_link]
        ax = axes_cad[short_name]
        b_el = ET.SubElement(
            body_elements[parent_link],
            "body",
            name=child_link,
            pos=f"{rel[0]:.6f} {rel[1]:.6f} {rel[2]:.6f}",
        )
        ET.SubElement(
            b_el,
            "joint",
            name=jname,
            type="hinge",
            pos="0 0 0",
            axis=f"{ax[0]:.6f} {ax[1]:.6f} {ax[2]:.6f}",
            range=f"{lo:.4f} {hi:.4f}",
            damping="0.15",
            armature="0.005",
        )
        com, mass, diag_i = link_inertials[child_link]
        ET.SubElement(
            b_el,
            "inertial",
            pos=f"{com[0]:.6f} {com[1]:.6f} {com[2]:.6f}",
            mass=f"{mass:.4f}",
            diaginertia=f"{diag_i[0]:.6f} {diag_i[1]:.6f} {diag_i[2]:.6f}",
        )
        for col_name, c_pos, c_size in link_col_boxes.get(child_link, []):
            ET.SubElement(
                b_el,
                "geom",
                name=col_name,
                type="box",
                pos=f"{c_pos[0]:.6f} {c_pos[1]:.6f} {c_pos[2]:.6f}",
                size=f"{c_size[0]:.6f} {c_size[1]:.6f} {c_size[2]:.6f}",
                rgba="0.3 0.3 0.35 0.4",
                group="3",
                contype="1",
                conaffinity="1",
                friction="0.8 0.005 0.0001",
            )
        for sname, rgba in link_geoms[child_link]:
            ET.SubElement(
                b_el,
                "geom",
                name=f"geom_{sname}",
                type="mesh",
                mesh=sname,
                rgba=f"{rgba[0]} {rgba[1]} {rgba[2]} {rgba[3]}",
                contype="0",
                conaffinity="0",
            )
        if child_link in ("left_foot_link", "right_foot_link"):
            side = "left" if child_link.startswith("left") else "right"
            c_xy, half_xy, zmin = foot_soles[side]
            hz = 0.004
            sx = max(float(half_xy[0]), 0.025)
            sy = max(float(half_xy[1]), 0.025)
            ET.SubElement(
                b_el,
                "geom",
                name=f"col_{side}_foot",
                type="box",
                pos=f"{c_xy[0]:.6f} {c_xy[1]:.6f} {zmin + hz:.6f}",
                size=f"{sx:.6f} {sy:.6f} {hz:.6f}",
                rgba="0.3 0.3 0.35 0.5",
                group="3",
                contype="1",
                conaffinity="1",
                friction="1.1 0.005 0.0001",
            )
            ET.SubElement(
                b_el,
                "site",
                name=f"{side}_foot_site",
                type="box",
                pos=f"{c_xy[0]:.6f} {c_xy[1]:.6f} {zmin + hz:.6f}",
                size=f"{sx + 0.002:.6f} {sy + 0.002:.6f} {hz + 0.002:.6f}",
                group="3",
                rgba="0.2 0.8 0.2 0.4" if side == "left" else "0.2 0.4 0.9 0.4",
            )
            if side == "right":
                ET.SubElement(
                    b_el,
                    "site",
                    name="ee_site",
                    type="box",
                    pos=f"{c_xy[0]:.6f} {c_xy[1]:.6f} {zmin + hz:.6f}",
                    size=f"{sx + 0.002:.6f} {sy + 0.002:.6f} {hz + 0.002:.6f}",
                    group="3",
                    rgba="0.9 0.6 0.1 0.4",
                )
        body_elements[child_link] = b_el

    contact = ET.SubElement(root, "contact")
    exclude_pairs = [
        ("base_link", "left_hip_yaw_link"),
        ("base_link", "left_hip_roll_link"),
        ("left_hip_yaw_link", "left_hip_roll_link"),
        ("left_hip_yaw_link", "left_thigh_link"),
        ("left_hip_roll_link", "left_thigh_link"),
        ("left_thigh_link", "left_shin_link"),
        ("left_shin_link", "left_foot_link"),
        ("base_link", "right_hip_yaw_link"),
        ("base_link", "right_hip_roll_link"),
        ("right_hip_yaw_link", "right_hip_roll_link"),
        ("right_hip_yaw_link", "right_thigh_link"),
        ("right_hip_roll_link", "right_thigh_link"),
        ("right_thigh_link", "right_shin_link"),
        ("right_shin_link", "right_foot_link"),
    ]
    for b1, b2 in exclude_pairs:
        ET.SubElement(contact, "exclude", body1=b1, body2=b2)

    act = ET.SubElement(root, "actuator")
    for _, _, jname, (lo, hi), short_name, _, _ in JOINT_TREE:
        ctrl_lo = max(lo, -1.57 if "pitch" in short_name else -0.85)
        ctrl_hi = min(hi, 1.57 if "pitch" in short_name else 0.85)
        ET.SubElement(
            act,
            "position",
            name=f"actuator_{short_name}",
            joint=jname,
            kp="45",
            kv="2.5",
            ctrlrange=f"{ctrl_lo:.2f} {ctrl_hi:.2f}",
            forcerange="-3.0 3.0",
        )

    sens = ET.SubElement(root, "sensor")
    for _, _, jname, _, short_name, _, _ in JOINT_TREE:
        ET.SubElement(sens, "jointpos", name=f"sensor_pos_{short_name}", joint=jname)
        ET.SubElement(sens, "jointvel", name=f"sensor_vel_{short_name}", joint=jname)
        ET.SubElement(sens, "jointactuatorfrc", name=f"sensor_frc_{short_name}", joint=jname)

    # 9-DoF torso IMU on `imu_site`
    ET.SubElement(sens, "accelerometer", name="sensor_imu_accel", site="imu_site")
    ET.SubElement(sens, "gyro", name="sensor_imu_gyro", site="imu_site")
    ET.SubElement(sens, "framequat", name="sensor_imu_quat", objtype="site", objname="imu_site")
    ET.SubElement(sens, "magnetometer", name="sensor_imu_mag", site="imu_site")
    ET.SubElement(sens, "velocimeter", name="sensor_imu_vel", site="imu_site")
    ET.SubElement(sens, "framepos", name="sensor_imu_pos", objtype="site", objname="imu_site")
    ET.SubElement(sens, "framelinvel", name="sensor_imu_linvel", objtype="site", objname="imu_site")
    ET.SubElement(sens, "frameangacc", name="sensor_imu_angacc", objtype="site", objname="imu_site")

    # Onboard camera pose sensors
    ET.SubElement(sens, "framepos", name="sensor_left_cam_pos", objtype="site", objname="left_stereo_camera_site")
    ET.SubElement(sens, "framepos", name="sensor_right_cam_pos", objtype="site", objname="right_stereo_camera_site")
    ET.SubElement(sens, "framepos", name="sensor_top_cam_pos", objtype="site", objname="top_camera_site")

    # Bilateral foot contact and IMU sensors
    ET.SubElement(sens, "touch", name="sensor_left_foot_touch", site="left_foot_site")
    ET.SubElement(sens, "touch", name="sensor_right_foot_touch", site="right_foot_site")
    ET.SubElement(sens, "force", name="sensor_left_foot_force", site="left_foot_site")
    ET.SubElement(sens, "force", name="sensor_right_foot_force", site="right_foot_site")
    ET.SubElement(sens, "torque", name="sensor_left_foot_torque", site="left_foot_site")
    ET.SubElement(sens, "torque", name="sensor_right_foot_torque", site="right_foot_site")
    ET.SubElement(sens, "framepos", name="sensor_left_foot_pos", objtype="site", objname="left_foot_site")
    ET.SubElement(sens, "framelinvel", name="sensor_left_foot_linvel", objtype="site", objname="left_foot_site")
    ET.SubElement(sens, "accelerometer", name="sensor_left_foot_accel", site="left_foot_site")
    ET.SubElement(sens, "gyro", name="sensor_left_foot_gyro", site="left_foot_site")

    ET.SubElement(sens, "framepos", name="sensor_ee_pos", objtype="site", objname="ee_site")
    ET.SubElement(sens, "framelinvel", name="sensor_ee_linvel", objtype="site", objname="ee_site")
    ET.SubElement(sens, "accelerometer", name="sensor_ee_accel", site="ee_site")
    ET.SubElement(sens, "gyro", name="sensor_ee_gyro", site="ee_site")
    ET.SubElement(sens, "touch", name="sensor_ee_touch", site="ee_site")

    ET.SubElement(sens, "subtreecom", name="sensor_robot_com", body="torso_base")
    ET.SubElement(sens, "subtreelinvel", name="sensor_robot_linvel", body="torso_base")
    ET.SubElement(sens, "subtreeangmom", name="sensor_robot_angmom", body="torso_base")

    system_xml_path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space="  ", level=0)
    ET.ElementTree(root).write(system_xml_path, encoding="utf-8", xml_declaration=True)


def build_model(project_root: Optional[Path] = None) -> Path:
    """Regenerate all 56 STL meshes, `urdf/nomios.urdf`, `models/system.xml`, and `models/scene.xml`."""
    root_dir = PROJECT_ROOT if project_root is None else Path(project_root).resolve()
    parts_dir = root_dir / "parts"
    meshes_dir = root_dir / "meshes"
    meshes_dir.mkdir(parents=True, exist_ok=True)

    sw_transforms, part_tris = _parse_sw_assembly(parts_dir / "full assembly.SLDASM", parts_dir)
    t_sw_to_mj = _compute_sw_to_mj_transform(sw_transforms)
    pivots_mj, axes_cad = _compute_cad_joint_poses(t_sw_to_mj, part_tris["u_bracket_v2"], sw_transforms)
    camera_specs = _compute_camera_specs(t_sw_to_mj, part_tris["pi_camera_v2_8mp"], sw_transforms)

    all_stl_names: List[str] = []
    link_geoms: Dict[str, List[Tuple[str, Tuple[float, float, float, float]]]] = {k: [] for k in LINK_MASSES}
    link_vert_accum: Dict[str, List[np.ndarray]] = {k: [] for k in LINK_MASSES}

    def _emit_component(stl_name: str, body_name: str, sw_lbl: str, rgba: Tuple[float, float, float, float]) -> None:
        base_part = sw_lbl.split("/")[-1].split("[")[0]
        t_mj = t_sw_to_mj @ sw_transforms[sw_lbl]
        verts_mj = part_tris[base_part].reshape(-1, 3) @ t_mj[:3, :3].T + t_mj[:3, 3]
        verts_body = verts_mj - pivots_mj[body_name]
        tris_body = verts_body.reshape(-1, 3, 3).astype(np.float32)

        write_binary_stl(meshes_dir / f"{stl_name}.STL", tris_body)
        all_stl_names.append(stl_name)
        link_geoms[body_name].append((stl_name, rgba))
        link_vert_accum[body_name].append(verts_body)

    for stl_name, sw_lbl, rgba in TORSO_COMPONENTS:
        _emit_component(stl_name, "torso_base", sw_lbl, rgba)

    for stl_name, body_name, sw_lbl, rgba in LEG_COMPONENTS:
        _emit_component(stl_name, body_name, sw_lbl, rgba)

    link_inertials: Dict[str, Tuple[np.ndarray, float, np.ndarray]] = {}
    link_col_boxes: Dict[str, List[Tuple[str, np.ndarray, np.ndarray]]] = {k: [] for k in LINK_MASSES}

    def _bbox_center_half(pts_arr: np.ndarray, pad: float = 0.0) -> Tuple[np.ndarray, np.ndarray]:
        pmin = pts_arr.min(axis=0) - pad
        pmax = pts_arr.max(axis=0) + pad
        return 0.5 * (pmin + pmax), np.maximum(0.5 * (pmax - pmin), 0.004)

    for lname, mass in LINK_MASSES.items():
        pts = np.vstack(link_vert_accum[lname])
        com = pts.mean(axis=0)
        span = np.maximum(pts.max(axis=0) - pts.min(axis=0), 0.02)
        diag_i = np.array(
            [
                (mass / 12.0) * (span[1] ** 2 + span[2] ** 2),
                (mass / 12.0) * (span[0] ** 2 + span[2] ** 2),
                (mass / 12.0) * (span[0] ** 2 + span[1] ** 2),
            ],
            dtype=np.float64,
        )
        link_inertials[lname] = (com, mass, np.maximum(diag_i, 2.5e-5))

        if lname == "torso_base":
            by_name = dict(zip([n for n, _ in link_geoms[lname]], link_vert_accum[lname]))
            head_pts = np.vstack([by_name[n] for n, _, _ in TORSO_COMPONENTS[:10]])
            lyaw_pts = np.vstack([by_name[n] for n in by_name if n.startswith("left_hip_yaw_")])
            ryaw_pts = np.vstack([by_name[n] for n in by_name if n.startswith("right_hip_yaw_")])
            link_col_boxes[lname].append(("col_torso_head", *_bbox_center_half(head_pts)))
            link_col_boxes[lname].append(("col_torso_left_hip_yaw", *_bbox_center_half(lyaw_pts)))
            link_col_boxes[lname].append(("col_torso_right_hip_yaw", *_bbox_center_half(ryaw_pts)))
        elif lname in ("left_foot_link", "right_foot_link"):
            by_name = dict(zip([n for n, _ in link_geoms[lname]], link_vert_accum[lname]))
            ankle_pts = np.vstack([v for n, v in by_name.items() if "ankle" in n])
            side = "left" if lname.startswith("left") else "right"
            link_col_boxes[lname].append((f"col_{side}_ankle", *_bbox_center_half(ankle_pts)))
        else:
            link_col_boxes[lname].append((f"col_{lname}", *_bbox_center_half(pts)))

    torso_by_name = dict(zip([n for n, _ in link_geoms["torso_base"]], link_vert_accum["torso_base"]))
    imu_pos, _ = _bbox_center_half(torso_by_name["Arduino_UNO_Q_1"])

    foot_soles: Dict[str, Tuple[np.ndarray, np.ndarray, float]] = {}
    world_zmin = float("inf")
    for side in ("left", "right"):
        fname = f"{side}_foot_link"
        f_pts = read_binary_stl(meshes_dir / f"{side}_foot.STL").reshape(-1, 3)
        zmin = float(f_pts[:, 2].min())
        sole_pts = f_pts[f_pts[:, 2] < zmin + 0.012]
        c_xy = 0.5 * (sole_pts[:, :2].min(axis=0) + sole_pts[:, :2].max(axis=0))
        half_xy = 0.5 * (sole_pts[:, :2].max(axis=0) - sole_pts[:, :2].min(axis=0))
        foot_soles[side] = (c_xy, half_xy, zmin)
        world_zmin = min(world_zmin, float(pivots_mj[fname][2]) + zmin)

    base_z = -world_zmin + 0.002

    _write_urdf(
        urdf_path=root_dir / "urdf" / "nomios.urdf",
        pivots_mj=pivots_mj,
        axes_cad=axes_cad,
        link_geoms=link_geoms,
        link_inertials=link_inertials,
        link_col_boxes=link_col_boxes,
        foot_soles=foot_soles,
        imu_pos=imu_pos,
        camera_specs=camera_specs,
    )
    _write_system_xml(
        system_xml_path=root_dir / "models" / "system.xml",
        base_z=base_z,
        pivots_mj=pivots_mj,
        axes_cad=axes_cad,
        all_stl_names=all_stl_names,
        link_geoms=link_geoms,
        link_inertials=link_inertials,
        link_col_boxes=link_col_boxes,
        foot_soles=foot_soles,
        imu_pos=imu_pos,
        camera_specs=camera_specs,
    )
    return generate_scene_xml(root_dir / "models" / "scene.xml", system_xml_rel="system.xml", meshdir="../meshes/")


if __name__ == "__main__":
    scene_path = build_model()
    print(f"Built Nomios URDF, meshes, and MuJoCo scene: {scene_path}")
