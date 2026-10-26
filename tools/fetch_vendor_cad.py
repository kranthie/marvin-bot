"""Download vendor CAD models that this repo may not redistribute, into cad/vendor/ (git-ignored).

RobStride publishes STEP models of its actuators in a public repository without a licence, so
they are linked here, not committed (see CONTRIBUTING.md). The files are pinned to a commit so
the envelopes measured from them (description/actuators.yaml) stay reproducible.

Usage:
  python tools/fetch_vendor_cad.py
"""

import sys
import urllib.parse
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "cad" / "vendor" / "robstride"
REPO = "https://raw.githubusercontent.com/RobStride/Product_Information"
COMMIT = "384cc561e1ff079c22f68b19908526094430febd"
FILES = {
    "RS00.step": "Product Literature/CAN2.0/RS00/RS00.step",
    "RS03.step": "Product Literature/CAN2.0/RS03/RS03.stp",
    "RS04.step": "Product Literature/CAN2.0/RS04/RS04.stp",
    "RS06.step": "Product Literature/CAN2.0/RS06/RS06.STEP",
}


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    for name, path in FILES.items():
        dest = OUT / name
        if dest.exists():
            print(f"have {dest.relative_to(ROOT)}")
            continue
        url = f"{REPO}/{COMMIT}/{urllib.parse.quote(path)}"
        print(f"fetching {url}")
        urllib.request.urlretrieve(url, dest)
    return 0


if __name__ == "__main__":
    sys.exit(main())
