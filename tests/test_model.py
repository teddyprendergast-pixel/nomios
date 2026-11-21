"""Validation tests for the Nomios URDF, meshes, and MuJoCo digital twin."""

from __future__ import annotations

import unittest
from pathlib import Path

import mujoco
import numpy as np

from scripts.sld_io import read_binary_stl

REPO_ROOT = Path(__file__).resolve().parent.parent


def _read_sensor(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> np.ndarray:
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
    if sid < 0:
        raise KeyError(f"Missing sensor: {name}")
    adr = model.sensor_adr[sid]
    dim = model.sensor_dim[sid]
    return data.sensordata[adr : adr + dim].copy()


class TestNomiosModel(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.scene_path = REPO_ROOT / "models" / "scene.xml"
        cls.system_path = REPO_ROOT / "models" / "system.xml"
        cls.urdf_path = REPO_ROOT / "urdf" / "nomios.urdf"
        cls.model = mujoco.MjModel.from_xml_path(str(cls.scene_path))
        cls.data = mujoco.MjData(cls.model)

    def test_scene_and_system_compile(self) -> None:
        for xml_path in (self.scene_path, self.system_path):
            with self.subTest(path=xml_path.name):
                m = mujoco.MjModel.from_xml_path(str(xml_path))
                d = mujoco.MjData(m)
                mujoco.mj_resetData(m, d)
                for _ in range(10):
                    mujoco.mj_step(m, d)
                self.assertFalse(np.isnan(d.qpos).any())
                self.assertFalse(np.isnan(d.qvel).any())

    def test_urdf_compiles(self) -> None:
        m = mujoco.MjModel.from_xml_path(str(self.urdf_path))
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        self.assertEqual(m.nmesh, 56)

    def test_model_structure(self) -> None:
        self.assertEqual(self.model.nmesh, 56)
        self.assertEqual(self.model.nu, 10)
        self.assertEqual(self.model.nv, 16)
        self.assertAlmostEqual(self.model.opt.timestep, 0.002, places=6)
        self.assertAlmostEqual(float(np.sum(self.model.body_mass)), 1.408, places=2)

    def test_cameras_present(self) -> None:
        cam_names = {
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_CAMERA, i)
            for i in range(self.model.ncam)
        }
        expected = {
            "isometric_cam",
            "side_cam",
            "front_cam",
            "top_cam",
            "left_stereo_cam",
            "right_stereo_cam",
            "top_head_cam",
        }
        self.assertTrue(expected.issubset(cam_names), f"Missing cameras: {expected - cam_names}")

    def test_sensors_present(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

        expected_dims = {
            "sensor_pos_left_hip_yaw": 1,
            "sensor_pos_left_hip_roll": 1,
            "sensor_pos_left_hip_pitch": 1,
            "sensor_pos_left_knee_pitch": 1,
            "sensor_pos_left_ankle_pitch": 1,
            "sensor_pos_right_hip_yaw": 1,
            "sensor_pos_right_hip_roll": 1,
            "sensor_pos_right_hip_pitch": 1,
            "sensor_pos_right_knee_pitch": 1,
            "sensor_pos_right_ankle_pitch": 1,
            "sensor_imu_accel": 3,
            "sensor_imu_gyro": 3,
            "sensor_imu_quat": 4,
            "sensor_imu_mag": 3,
            "sensor_left_foot_touch": 1,
            "sensor_right_foot_touch": 1,
            "sensor_left_foot_force": 3,
            "sensor_right_foot_force": 3,
            "sensor_left_foot_torque": 3,
            "sensor_right_foot_torque": 3,
            "sensor_robot_com": 3,
        }
        for name, dim in expected_dims.items():
            val = _read_sensor(self.model, self.data, name)
            self.assertEqual(len(val), dim, f"Unexpected dimension for {name}")
            self.assertFalse(np.isnan(val).any(), f"NaN in sensor {name}")

    def test_head_mesh_integrity(self) -> None:
        head_tris = read_binary_stl(REPO_ROOT / "meshes" / "head_1.STL")
        self.assertEqual(len(head_tris), 1260)


if __name__ == "__main__":
    unittest.main()
