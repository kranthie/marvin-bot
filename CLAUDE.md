# Marvin Bot

A ~1.22 m, ~35 kg, 25-DOF open humanoid on Robstride actuators, with a Jetson AGX Orin 64 GB onboard. Currently in phase 1 (robot in simulation).

## Read first
- `README.md`: overview and layout
- `docs/plan.md`: phases, pass checks and spend per phase
- `docs/decisions.md`: settled decisions, newest first. Don't re-open them without a reason; add new ones here.
- `docs/roadmap.md`: current phase and open items

## Conventions
- `description/robot.yaml` is the single source of truth for joints, ranges, sizes, motors and CAN buses; `actuators.yaml` for actuator specs; `mass_budget.yaml` for non-actuator masses. Everything else is generated from or checked against them.
- SI units in code and configs; joint ranges in `robot.yaml` are degrees.
- Stack: Isaac Lab 3.0 (Newton and PhysX) for training, MuJoCo as a cross-check, JetPack 7.2 + Isaac ROS 4.6 + ROS 2 Jazzy on the Orin, GR00T N1.7 for manipulation.

## Rules
- Pushing straight to `main` is fine. Keep `ruff check`, `ruff format --check`, `tools/check_description.py` and `tools/bom_summary.py` passing (CI runs them).
- Write for future open-source readers: no personal names, local or machine-specific paths, or private links. Keep fabrication tooling generic (no specific printer or mill brands).
- Do not name commercial humanoid makers whose site terms restrict use of their content or marks; only Unitree and Booster Robotics have been checked and may be named.
- Credit every borrowed project, paper or post in `ACKNOWLEDGMENTS.md` and respect its licence. Asimov v1 is used for ideas only, never its CAD.
- Do not commit vendor CAD unless its licence allows redistribution.
- Ask before ordering parts.
- Code marked untested stays marked until it has run.
- Research notes are kept outside the repo; cite sources inline in docs where a figure matters.
