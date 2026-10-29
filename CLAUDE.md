# Marvin Bot

A ~1.44 m, ~41 kg, 27-DOF open humanoid with 8-DOF hands, on Robstride actuators, with a Jetson AGX Orin 64 GB onboard. Currently in phase 1 (robot in simulation).

## Read first
- `README.md`: overview and layout
- `docs/plan.md`: phases, pass checks and spend per phase
- `docs/decisions.md`: settled decisions, newest first. Don't re-open them without a reason; add new ones here.
- `docs/roadmap.md`: current phase and open items

## Conventions
- `description/robot.yaml` is the single source of truth for joints, ranges, sizes, motors and CAN buses; `actuators.yaml` for actuator specs; `mass_budget.yaml` for non-actuator masses; `skeleton.yaml` for link frames and where masses sit. Run `python tools/build_urdf.py` after editing any of them; the URDF is generated, never hand-edited.
- SI units in code and configs; joint ranges in `robot.yaml` are degrees.
- Stack: Isaac Lab 3.0 (Newton and PhysX) for training, MuJoCo as a cross-check, JetPack 7.2 + Isaac ROS 4.6 + ROS 2 Jazzy on the Orin, GR00T N1.7 for manipulation.

## Rules
- Pushing straight to `main` is fine. Keep `ruff check`, `ruff format --check`, `tools/check_description.py`, `tools/bom/summary.py`, `tools/build_urdf.py --check`, `tools/build_harness.py --check` and `sim/mujoco/check_standing.py` passing (CI runs them).
- Write for future open-source readers: no personal names, local or machine-specific paths, or private links. Keep fabrication tooling generic (no specific printer or mill brands).
- Refer only to open-source projects: no commercial humanoids or other commercial robots as benchmarks, comparisons or style references. Bought parts (actuators, sensors, computer, hand kits) are named as parts. Before drawing on an open-source humanoid or other project, check its licence and terms of use; credit it in `ACKNOWLEDGMENTS.md` with what was used, and respect its licence and attribution terms (for example the AmazingHand's CC-BY-4.0 credit to Pollen Robotics). Asimov v1 is used for ideas only, never its CAD.
- Credit every borrowed paper or post in `ACKNOWLEDGMENTS.md` too.
- Do not commit vendor CAD unless its licence allows redistribution.
- Ask before ordering parts.
- Code marked untested stays marked until it has run.
- Research notes are kept outside the repo; cite sources inline in docs where a figure matters.
