#!/usr/bin/env bash
# Every check CI runs, in CI's order. Needs: pip install -e ".[dev,sim,cad]"
# Usage: tools/check_all.sh   (from the repo root)
set -euxo pipefail
ruff check .
ruff format --check .
pytest -q tests
python -m cad.build --check
python -m cad.export --check
python -m cad.strength --check
python -m cad.covers --check
python -m cad.build > /dev/null  # meshes (git-ignored) for the MuJoCo checks
python tools/check_description.py
python tools/bom_summary.py
python tools/ankle_linkage.py --check
python tools/build_urdf.py --check
python tools/build_harness.py --check
python sim/mujoco/check_standing.py
python sim/mujoco/check_standing.py --pose straight
python sim/mujoco/check_step_up.py --heights 0.18
python sim/mujoco/check_symmetry.py
python tools/check_packaging.py
