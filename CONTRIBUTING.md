# Contributing

Issues and pull requests are welcome.

## Ground rules

- `description/robot.yaml` is the single source for joint names, ranges, link lengths, actuator assignment and CAN buses. `description/actuators.yaml` holds actuator specifications and `description/mass_budget.yaml` the non-actuator mass estimates. CAD, URDF, simulation and onboard configs must be generated from or checked against them.
- Record design decisions in `docs/decisions.md`, newest first, with the reason and the alternatives considered.
- Credit any project, paper, post or product you borrow from in `ACKNOWLEDGMENTS.md`, and respect its licence.
- Use SI units (m, kg, N·m, rad) in code and configs. Joint ranges in `robot.yaml` are in degrees for readability; tools convert them.
- Mark code that has not run on hardware or in the simulator as untested in its docstring, and keep it marked until it has.
- No personal data, credentials, machine-specific paths or private links in the repo.

## Third-party CAD and data

- Do not commit vendor CAD models (actuators, compute modules, cameras, connectors) unless their licence explicitly allows redistribution. Link to the official download instead, and use simplified envelope models made in this repo.
- Never use models from sites whose terms forbid redistribution or derived designs (for example McMaster-Carr or GrabCAD) in committed designs.
- Openly licensed third-party designs that allow redistribution (for example CC-BY-4.0) may be committed under `description/third_party/`, each folder with its licence, the original source and what was changed, and a credit in `ACKNOWLEDGMENTS.md`. They keep their own licence; they are not relicensed under CERN-OHL-S.
- Specifications quoted in this repo cite their source. Mark figures that come from resellers rather than the manufacturer as unverified.
- Do not include images, logos or trademarks of other companies.

## Licensing of contributions

By contributing, you agree that your contribution is licensed under the licence of the folder it
goes into (see "Licences" in the README): CERN-OHL-S-2.0 for hardware design sources, Apache-2.0
for everything else. Only contribute work you have the right to license this way, and say where
any third-party material comes from. Sign off your commits (`git commit -s`) to certify the
[Developer Certificate of Origin](https://developercertificate.org/).

## Pull requests

- Open an issue first for a design change, so it can be discussed before you spend time on it.
- One topic per pull request.
- Regenerate everything generated (below) in the same pull request, and record design decisions
  in `docs/decisions.md`.
- CI must pass. Report safety or security problems privately ([SECURITY.md](SECURITY.md)), not in
  an issue.

## Development

```bash
python3.12 -m venv .venv && . .venv/bin/activate   # CI uses 3.12; 3.10 or later works
pip install -e ".[dev,cad,sim]"
tools/check_all.sh                                 # every CI check, in CI's order
```

Code style: ruff (line length 100, `ruff format`); every script starts with a docstring that says
what it does and how to run it.

After editing `description/*.yaml` or `cad/`, regenerate in this order:

```bash
python -m cad.build                 # meshes, cad_model.yaml (masses and sizes)
python tools/build_urdf.py          # URDFs
python tools/build_harness.py       # wiring harness
python -m cad.export                # build BOM
MUJOCO_GL=egl python tools/render_robot.py   # README picture, if the look changed
```

For the simulators, rerun `sim/isaaclab/convert_urdf.py` (Isaac Lab) and
`sim/motion/setup_protomotions.py` (ProtoMotions).
