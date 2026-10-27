"""Put the soles of retargeted Marvin Bot motions on the floor.

ProtoMotions' converter lifts each motion until its lowest body origin is 4 cm above the floor,
which suits G1, whose foot origin sits about that high. Marvin Bot's foot origin is its ankle axis,
6 cm above the sole, so its soles end up below the floor (by up to 4.6 cm with the toe or heel
down). This lifts each clip, as a whole, until the lowest corner of either foot's collision box
over the clip is on the floor. Edits the .motion files in place.

Usage (in the ProtoMotions environment):
  python sim/motion/fix_sole_height.py /path/to/ProtoMotions <folder of .motion files>
"""

import argparse
from itertools import product
from pathlib import Path

import mujoco
import numpy as np
import torch

FEET = ("left_foot_link", "right_foot_link")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("protomotions", type=Path)
    ap.add_argument("motions", type=Path)
    args = ap.parse_args()
    m = mujoco.MjModel.from_xml_path(
        str(args.protomotions / "protomotions/data/assets/mjcf/marvin_bot.xml")
    )
    # each foot's box corners in its body frame; motion body index = MuJoCo body id - 1 (world)
    corners = {}
    for name in FEET:
        b = m.body(name).id
        pts = []
        for g in np.flatnonzero(m.geom_bodyid == b):
            rot = np.zeros(9)
            mujoco.mju_quat2Mat(rot, m.geom_quat[g])
            for s in product((-1, 1), repeat=3):
                pts.append(m.geom_pos[g] + rot.reshape(3, 3) @ (np.array(s) * m.geom_size[g]))
        corners[b - 1] = torch.tensor(np.array(pts), dtype=torch.float32)
    for path in sorted(args.motions.glob("*.motion")):
        motion = torch.load(path, weights_only=False)
        pos, rot = motion["rigid_body_pos"], motion["rigid_body_rot"]  # rot is xyzw
        lowest = []
        for i, pts in corners.items():
            x, y, z, w = rot[:, i].unbind(-1)
            # z row of each frame's rotation matrix
            row = torch.stack(
                [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1
            )
            lowest.append((pos[:, i, 2:3] + row @ pts.T).min())
        lift = -float(torch.stack(lowest).min())
        motion["rigid_body_pos"] = pos + torch.tensor([0.0, 0.0, lift])
        torch.save(motion, path)
        print(f"{path.name}: lifted {lift * 100:+.1f} cm")


if __name__ == "__main__":
    main()
