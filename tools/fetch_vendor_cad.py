"""Download vendor CAD models into cad/vendor/ (git-ignored).

RobStride publishes STEP models of its actuators in a public repository without a licence, so
they are linked here, not committed (see CONTRIBUTING.md). Orbbec publishes a mesh of the Gemini
335L depth camera and its URDF (the lens frames) in its ROS 2 description package under
Apache-2.0; they are fetched the same way to keep large binaries out of the repo. The files are
pinned to a commit so the envelopes measured from them (description/actuators.yaml, the camera
outline in cad/head.py) stay reproducible.

Usage:
  python tools/fetch_vendor_cad.py
"""

import sys
import urllib.parse
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
VENDOR = ROOT / "cad" / "vendor"
SOURCES = [  # (folder, raw-file base URL, commit, {local name: path in the repository})
    (
        "robstride",
        "https://raw.githubusercontent.com/RobStride/Product_Information",
        "384cc561e1ff079c22f68b19908526094430febd",
        {
            "RS00.step": "Product Literature/CAN2.0/RS00/RS00.step",
            "RS03.step": "Product Literature/CAN2.0/RS03/RS03.stp",
            "RS04.step": "Product Literature/CAN2.0/RS04/RS04.stp",
            "RS06.step": "Product Literature/CAN2.0/RS06/RS06.STEP",
        },
    ),
    (
        "orbbec",
        "https://raw.githubusercontent.com/orbbec/OrbbecSDK_ROS2",
        "c153462518ad674650bafa4464fda72c27ab797a",
        {
            "gemini_335l.stl": "orbbec_description/meshes/gemini335L_336L/base_link.STL",
            "gemini_335l.urdf.xacro": "orbbec_description/urdf/gemini_335_L_336_L.urdf.xacro",
        },
    ),
]


def main() -> int:
    for folder, repo, commit, files in SOURCES:
        out = VENDOR / folder
        out.mkdir(parents=True, exist_ok=True)
        for name, path in files.items():
            dest = out / name
            if dest.exists():
                print(f"have {dest.relative_to(ROOT)}")
                continue
            url = f"{repo}/{commit}/{urllib.parse.quote(path)}"
            print(f"fetching {url}")
            urllib.request.urlretrieve(url, dest)
    return 0


if __name__ == "__main__":
    sys.exit(main())
