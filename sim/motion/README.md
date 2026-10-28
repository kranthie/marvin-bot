# Motion pipeline: human motion to a Marvin Bot tracking policy

Status: the first tracker that walks. Trained on eight 8 s clips from 100STYLE (walking,
turning, jogging, standing) for 1,400 epochs (about an hour), it stays up and covers 87-101% of
the reference's distance on the straight walk, the turning walk and the styled walk, with the
bodies 5-6 cm from the reference; it stays up but falls behind while jogging and falls on the
backward walk, the side step and the start-stop clip. That is a short run: the next is longer.
Trackers 1-4 stood still (ProtoMotions' config leaves out BeyondMimic's global anchor-position
reward, which `setup_protomotions.py` puts back). Nothing has been replayed in MuJoCo yet.

Instead of writing rewards for each skill, train a policy that tracks a reference motion
(BeyondMimic-style motion tracking). A new skill is then a new clip, not new reward code. The
pipeline uses NVIDIA's open tools:

| Step | Tool | Here |
|---|---|---|
| Motions | [100STYLE](https://zenodo.org/records/8127870) motion capture (CC-BY-4.0) | |
| Keypoints | joint positions to the retargeter's keypoints, in ProtoMotions' SOMA convention | `convert_100style.py` |
| Retarget | ProtoMotions' PyRoki retargeter | `retarget_marvin_bot.py` |
| Convert, package | ProtoMotions' `convert_pyroki_retargeted_robot_motions_to_proto.py`, `motion_lib.py` | `fix_sole_height.py` |
| Track | [ProtoMotions](https://github.com/NVlabs/ProtoMotions) on Newton, BeyondMimic-style rewards | `setup_protomotions.py`, `check_tracking.py` |

`setup_protomotions.py` writes Marvin Bot's MJCF, retargeting URDF and robot config into a
ProtoMotions checkout, all generated from `description/`, and the tracker experiment, made from
upstream's BeyondMimic config with the changes below; rerun it after a description change.

What is specific to Marvin Bot:

- **Scale.** Human keypoints are scaled to Marvin Bot's size in place of the retargeter's G1
  scale: the legs uniformly (0.69, the robot's straight leg over the source's standing one), the
  pelvis target put on the hip axis, the shoulders about the hips (0.62 wide, 0.75 high) and
  6.5 cm forward, the arms about each shoulder (0.79, reach to the wrist pitch axis). The knees
  and feet are then moved 1.5 and 3 cm out along the hip axis: scaled down, the human's feet end
  up 12-14 cm apart and Marvin Bot's are 11 cm wide. The solver also starts from and pulls
  towards the standing pose rather than the middle of each joint's range. With the source's
  pelvis and shoulders left high, the solver crouched the robot to reach them (knees at 37-58°
  through a walk).
- **Arms collide with the ground only.** The retargeter's self-collision cost is off upstream,
  and the hand boxes meet the thighs even with the arms hanging straight, so in the tracking
  model the arms do not collide with the body. A real robot would: the retargeted arms still
  need a collision fix before anything runs on hardware.
- **Soles.** ProtoMotions' converter puts the lowest body origin 4 cm above the floor, which
  suits G1's foot frame; `fix_sole_height.py` puts Marvin Bot's soles on the floor instead.
- **Tracker.** `mlp_bm_marvin_bot.py` is upstream's `mlp_bm_l2c2.py` with BeyondMimic's
  global anchor-position reward (0.5, sigma 0.3) and a termination when the torso is 0.5 m off
  the reference's, both left out upstream, which is why trackers 1-4 stood still; the reference
  torso's offset in the policy's observations; softer gains for an action scale of at least
  0.4 rad (the walking task's placeholder gains gave 0.2 rad at the hips and knees); more
  exploration (log std -2.3) and a looser body-velocity term (sigma 0.5).
- **A second check.** ProtoMotions counts a clip as tracked when the torso height stays within
  25 cm of the reference's; `check_tracking.py` also compares the distance covered, the joint
  angles and the body positions, which is how the first tracker's shuffling showed up.

## Environments

Everything lives in one parent folder, outside this repo: the ProtoMotions and PyRoki checkouts,
two Python environments (PyRoki uses JAX) and a work folder. ProtoMotions keeps its checkpoints,
meshes and sample motions in Git LFS (about 6 GB; install `git lfs` first). The torch wheel below
needs an NVIDIA driver for CUDA 12.8 (570 or later). The pipeline was last run on ProtoMotions
`f69fdcaabff31846b4d2247207417b57dc788828` (2026-10-16) and PyRoki
`388e43e1fc0d0ee382968d3dd72970fd62a0450c`, with JAX on the CPU.

```bash
export M=/path/to/marvin-bot          # this repo
cd /path/to/parent && export W=$PWD/work && mkdir -p $W
git lfs install
git clone https://github.com/NVlabs/ProtoMotions
git -C ProtoMotions checkout f69fdcaabff31846b4d2247207417b57dc788828 && git -C ProtoMotions lfs pull
git clone https://github.com/chungmin99/pyroki
git -C pyroki checkout 388e43e1fc0d0ee382968d3dd72970fd62a0450c
uv venv --python 3.12 venvs/proto
VIRTUAL_ENV=venvs/proto uv pip install torch --index-url https://download.pytorch.org/whl/cu128
VIRTUAL_ENV=venvs/proto uv pip install -r ProtoMotions/requirements_newton.txt
VIRTUAL_ENV=venvs/proto uv pip install -e ProtoMotions
uv venv --python 3.12 venvs/pyroki
VIRTUAL_ENV=venvs/pyroki uv pip install -e pyroki
```

`setup_protomotions.py` edits the ProtoMotions checkout (it adds three generated files and
registers `marvin_bot` in `robot_configs/factory.py`), so rerun it after updating ProtoMotions or
the description.

## Steps

Download [100STYLE](https://zenodo.org/records/8127870) (`100STYLE.zip`, about 1.5 GB, BVH)
and unzip it into the work folder. `convert_100style.py` cuts the eight clips the trackers so
far train on (`CLIPS`). Then run from the ProtoMotions checkout
(`cd /path/to/parent/ProtoMotions`); the interpreters are `../venvs/proto` and `../venvs/pyroki`.

```bash
P=../venvs/proto/bin/python; R=../venvs/pyroki/bin/python
$P $M/sim/motion/setup_protomotions.py .
$P $M/sim/motion/convert_100style.py $W/100STYLE $W/keypoints
$R $M/sim/motion/retarget_marvin_bot.py --protomotions . --converted \
    --keypoints-folder-path $W/keypoints --output-dir $W/retargeted --source-type soma --no-visualize
$R $M/sim/motion/retarget_marvin_bot.py --protomotions . --converted \
    --keypoints-folder-path $W/keypoints --output-dir $W/retargeted --source-type soma --no-visualize \
    --save-contacts-only --contacts-dir $W/contacts
$P data/scripts/convert_pyroki_retargeted_robot_motions_to_proto.py \
    --retargeted-motion-dir $W/retargeted --output-dir $W/proto --robot-type marvin_bot \
    --contact-labels-dir $W/contacts --apply-motion-filter --force-remake
$P $M/sim/motion/fix_sole_height.py . $W/proto
$P protomotions/components/motion_lib.py --motion-path $W/proto --output-file $W/marvin_bot_motions.pt
$P protomotions/train_agent.py --robot-name marvin_bot --simulator newton \
    --experiment-path examples/experiments/mimic/mlp_bm_marvin_bot.py --experiment-name marvin_bot_track \
    --motion-file $W/marvin_bot_motions.pt --num-envs 4096 --batch-size 16384 --headless
$P $M/sim/motion/check_tracking.py \
    results/marvin_bot_track/results/predicted_motion_lib_epoch_<N>.pt $W/marvin_bot_motions.pt
```

Retargeting takes about 10 minutes a clip on the CPU (JAX recompiles for each clip length); the
tracker trains at about 2.5 s an epoch with 4096 robots on one RTX-class GPU and writes a
rollout of every clip (`predicted_motion_lib_epoch_<N>.pt`) every 600 epochs, from epoch 200;
score the latest with `check_tracking.py`. Exporting a tracker and replaying it in MuJoCo are
not set up yet.

## Motion sources compared

The same eight motions from three openly licensed sources, retargeted and tracked the same way
(1,400 epochs each):

| | 100STYLE | Kimodo (SOMA-RP model) | CMU |
|---|---|---|---|
| Clips through ProtoMotions' motion filter | 8 of 8 | 8 of 8 | 5 of 8 (the rest jump) |
| Planted foot sliding (median) | 0.07 m/s | 0.06 m/s | 0.43 m/s |
| Tracker stays up | 5 of 8 | 3 of 8; 5 of 8 with gentler prompts | 3 of 5 |

100STYLE is the source: one licence (CC-BY-4.0), 100 styles of one actor, and it tracked well.
Kimodo's text-to-motion is the way to add motions no dataset has, but its text encoder brings in
Llama 3's licence, and the prompt has to ask for a pace Marvin Bot can keep: the first prompts
("a person jogs forward") gave a 2 m/s run, a 400°/s turn and a sidestep that crosses the feet,
and the tracker fell on all three. Asking for slow, short steps (a 0.8 m/s jog, a 75°/s turn)
kept it up on the jog and the turn, both at 97-104% of the reference's distance. CMU's recordings are older and noisier. In all three, the backward and
sideways walks fell, which points at the tracker rather than the data.

## Data licences

Motion data is not committed, and nothing here is derived from data whose licence restricts who
may use it. 100STYLE is CC-BY-4.0: credit "100STYLE, I. Mason, S. Starke and T. Komura,
Real-Time Style Modelling of Human Locomotion via Feature-Wise Transformations and Local Motion
Phases (2022)" with anything you publish from it. ProtoMotions' own sample motions are from the
BONES-SEED dataset, whose licence covers only non-profit academic institutions and companies
under $1M annual revenue; this pipeline does not use them. Kimodo's weights are under the NVIDIA
Open Model License and need Meta's gated Llama 3 8B Instruct; the CMU database
(mocap.cs.cmu.edu) is free to use, with that credit. ProtoMotions' and PyRoki's code is
Apache-2.0 and MIT.
