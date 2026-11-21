"""Launch the Nomios digital twin in MuJoCo."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer

DEFAULT_MODEL = Path(__file__).resolve().parent / "models" / "scene.xml"


def main() -> None:
    parser = argparse.ArgumentParser(description="View the Nomios MuJoCo model.")
    parser.add_argument(
        "--model",
        type=Path,
        default=DEFAULT_MODEL,
        help="Path to the MJCF or URDF model file (default: models/scene.xml).",
    )
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="Open the full interactive MuJoCo GUI instead of the passive loop.",
    )
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    data = mujoco.MjData(model)

    if args.interactive:
        mujoco.viewer.launch(model, data)
        return

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            mujoco.mj_step(model, data)
            viewer.sync()

            remaining = model.opt.timestep - (time.time() - step_start)
            if remaining > 0:
                time.sleep(remaining)


if __name__ == "__main__":
    main()
