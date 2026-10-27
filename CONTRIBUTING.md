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

## Development

```bash
pip install -e ".[dev]"
ruff check .
ruff format --check .
python tools/check_description.py
python tools/bom_summary.py
```

CI runs the same checks on every push and pull request.
