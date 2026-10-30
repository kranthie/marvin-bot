---
title: Getting started
description: Clone the repository, run the checks, build the robot model and find your way around the documentation.
---

Marvin Bot is in **phase 1: the robot exists only in simulation**. Everything here runs on an ordinary
computer; nothing needs hardware. Training the walking policies needs an NVIDIA GPU and Isaac Lab, but
the checks below do not.

:::caution[Not built or tested on hardware]
The designs, policies and scripts have only run in simulation; the motor driver and bench scripts are
untested. A robot of this size can cause serious injury, fire or damage if built or run wrongly. Read the
[safety notes](/marvin-bot/build/safety/) before powering any part of it.
:::

## 1. Get the code and run the checks

You need Git and Python 3.10 or later (continuous integration uses 3.12).

```bash
git clone https://github.com/kranthie/marvin-bot.git
cd marvin-bot
python3.12 -m venv .venv && . .venv/bin/activate
pip install -e ".[dev,cad,sim]"
tools/check_all.sh          # every check CI runs, in CI's order
```

`check_all.sh` lints the code, runs the unit tests, rebuilds the code-CAD and checks it against the
committed files, checks the strength of the frame, sweeps every joint for collisions, checks the
description, bill of materials, URDF and wiring harness, and stands the robot up in MuJoCo. The
[overview](/marvin-bot/overview/#checks) lists each check on its own.

## 2. Look at the robot

```bash
python -m cad.build                          # part meshes and masses (git-ignored outputs)
MUJOCO_GL=egl python tools/render_robot.py   # the MuJoCo render shown on the overview page
```

Or open the [3-D model](/marvin-bot/guide/00-3d-model/) in your browser: it is generated
from the same files, and you can peel back the shells, follow the assembly order and play simple skills.

## 3. Find your way around

| If you want to… | Read |
|---|---|
| understand the project in ten minutes | the [overview](/marvin-bot/overview/) |
| know what is planned and what is done | the [plan](/marvin-bot/plan/) and the [roadmap](/marvin-bot/roadmap/) |
| know why something is the way it is | the [decision log](/marvin-bot/decisions/), newest first |
| learn humanoid robotics from first principles | the [interactive guide](/marvin-bot/guide/) and the [tutorials page](/marvin-bot/tutorials/) |
| train or check the walking policies | [walking and stairs in Isaac Lab](/marvin-bot/simulation/isaac-lab/) |
| work on motion tracking | the [motion tracking pipeline](/marvin-bot/simulation/motion-tracking/) |
| price or buy parts | the [bill of materials](/marvin-bot/build/bom/) |
| contribute | [contributing](/marvin-bot/project/contributing/) and the [code of conduct](/marvin-bot/project/code-of-conduct/) |

## How the repository is organised

`description/robot.yaml` is the single source of truth for the joints, ranges, sizes, motors and CAN
buses. The code-CAD in `cad/` builds every part from it, `tools/build_urdf.py` generates the robot model
(URDF) that the simulators use, and the checks keep everything consistent. The
[documentation index](/marvin-bot/glossary/) lists every document, with a short glossary.

## Building this site

The site lives in `site/` and is built with [Astro Starlight](https://starlight.astro.build). Its pages
are copied from the repository's Markdown at build time, so edit the Markdown files, not the site.
`site/README.md` explains how to preview it locally.
