# Motion pipeline: human motion to a Marvin Bot tracking policy

Status: a tracker walks. Trained on eleven 8 s locomotion clips generated with Kimodo for
2,600 epochs (about two hours on one GPU), it follows nine: walking at two speeds, curving left
and right, starting and stopping, turning in place, jogging, walking backwards and standing,
covering 82-101% of the reference's distance with the bodies 3-5 cm from the reference. It
steps one foot at a time; it still spends more of each stride on both feet than the reference
(26-33% against 10-15%) and does not leave the ground while jogging. It falls on the two
sidesteps. Fourteen more skills (gestures, squatting and picking up, marching, a kick, a dance,
walking with a box) are generated and retargeted, and a tracker for all of them is training.
Nothing has been replayed in MuJoCo yet.

Instead of writing rewards for each skill, train a policy that tracks a reference motion
(BeyondMimic-style motion tracking). A new skill is then a new clip, not new reward code. The
pipeline uses NVIDIA's open tools:

| Step | Tool | Here |
|---|---|---|
| Motions | [Kimodo](https://github.com/nv-tlabs/kimodo) text to motion, with a root path per clip | `generate_kimodo.py` |
| Keypoints | ProtoMotions' `convert_soma23_npz_to_proto.py`, `extract_retargeting_input_keypoints_from_packaged_motionlib.py` | |
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
- **Motions at Marvin Bot's pace.** `generate_kimodo.py` gives every locomotion clip and
  in-place skill a root path, so the speed and turn rate are set rather than worded: Kimodo's
  idea of "jogs forward" was a 2 m/s run, which no tracker followed. Of four samples per clip it
  keeps the one with the least foot sliding and crossing of the feet.
- **Palms in.** The motion pipeline's standing pose turns the wrists 80° so the palms face the
  thighs (`robot.yaml`'s stance, which the walking policies use, leaves them facing forward), and
  the retargeter holds the wrist yaw and pitch there, since no keypoint sets the hand's angle.
- **Soles.** ProtoMotions' converter puts the lowest body origin 4 cm above the floor, which
  suits G1's foot frame; `fix_sole_height.py` lifts each clip until the stance foot (the flatter
  one) is on the floor. Lifting until the clip's lowest corner touched instead left the walks
  3-5 cm in the air, since that corner is a toe tipped down at push-off, and the trackers hopped
  to keep up.
- **Tracker.** `mlp_bm_marvin_bot.py` is upstream's `mlp_bm_l2c2.py` with BeyondMimic's
  global anchor-position reward (0.5, sigma 0.3) and a termination when the torso is 0.5 m off
  the reference's, both left out upstream, which is why trackers 1-4 stood still; the reference
  torso's offset in the policy's observations; softer gains for an action scale of at least
  0.4 rad (the walking task's placeholder gains gave 0.2 rad at the hips and knees); more
  exploration (log std -2.3) and a looser body-velocity term (sigma 0.5); and a penalty (-2.0)
  on each foot whose contact differs from the reference's. Without it the trackers hopped,
  with both feet on the ground 70% of the time and both in the air 20%, where the reference
  steps; upstream's other experiments use -0.1, and -0.5 only halved it.
- **A second check.** ProtoMotions counts a clip as tracked when the torso height stays within
  25 cm of the reference's; `check_tracking.py` also compares the distance covered, the joint
  angles and the body positions, which is how the first tracker's shuffling showed up.

## Environments

Everything lives in one parent folder, outside this repo: the ProtoMotions, PyRoki and Kimodo
checkouts, three Python environments (PyRoki uses JAX; Kimodo its own torch) and a work folder.
ProtoMotions keeps its checkpoints, meshes and sample motions in Git LFS (about 6 GB; install
`git lfs` first). The torch wheels below need an NVIDIA driver for CUDA 12.8 (570 or later);
Kimodo needs about 17 GB of GPU memory (or `TEXT_ENCODER_DEVICE=cpu`, under 3 GB). The pipeline
was last run on ProtoMotions `f69fdcaabff31846b4d2247207417b57dc788828` (2026-10-16), PyRoki
`388e43e1fc0d0ee382968d3dd72970fd62a0450c` and Kimodo `e1c219c99b4b94ba48be1fb6a1cbc64ecee4c954`
(2026-10-15), with JAX on the CPU.

Kimodo encodes prompts with Meta's Llama 3 8B Instruct, which is gated: request access to
`meta-llama/Meta-Llama-3-8B-Instruct` on Hugging Face, accepting the Meta Llama 3 Community
License, and log in with `hf auth login` before the first run. Kimodo downloads its own model
and the text encoder on first use (about 16 GB).

```bash
export M=/path/to/marvin-bot          # this repo
cd /path/to/parent && export W=$PWD/work && mkdir -p $W
git lfs install
git clone https://github.com/NVlabs/ProtoMotions
git -C ProtoMotions checkout f69fdcaabff31846b4d2247207417b57dc788828 && git -C ProtoMotions lfs pull
git clone https://github.com/chungmin99/pyroki
git -C pyroki checkout 388e43e1fc0d0ee382968d3dd72970fd62a0450c
git clone https://github.com/nv-tlabs/kimodo
git -C kimodo checkout e1c219c99b4b94ba48be1fb6a1cbc64ecee4c954
uv venv --python 3.12 venvs/proto
VIRTUAL_ENV=venvs/proto uv pip install torch --index-url https://download.pytorch.org/whl/cu128
VIRTUAL_ENV=venvs/proto uv pip install -r ProtoMotions/requirements_newton.txt
VIRTUAL_ENV=venvs/proto uv pip install -e ProtoMotions
uv venv --python 3.12 venvs/pyroki
VIRTUAL_ENV=venvs/pyroki uv pip install -e pyroki
uv venv --python 3.12 venvs/kimodo
VIRTUAL_ENV=venvs/kimodo uv pip install torch --index-url https://download.pytorch.org/whl/cu128
VIRTUAL_ENV=venvs/kimodo uv pip install -e "kimodo[soma]"
```

`setup_protomotions.py` edits the ProtoMotions checkout (it adds four generated files and
registers `marvin_bot` in `robot_configs/factory.py`), so rerun it after updating ProtoMotions or
the description.

## Steps

Generate the motions (`locomotion`, `skills` or clip names; see `generate_kimodo.py`), then run
the rest from the ProtoMotions checkout (`cd /path/to/parent/ProtoMotions`); the interpreters
are `../venvs/proto` and `../venvs/pyroki`. The retargeter runs one clip per process, all at
once, which takes a few minutes for the eleven locomotion clips on a 32-thread CPU (one after
another it takes about 10 minutes a clip, since JAX recompiles for each).

```bash
../venvs/kimodo/bin/python $M/sim/motion/generate_kimodo.py $W/npz locomotion skills
P=../venvs/proto/bin/python; R=../venvs/pyroki/bin/python
$P $M/sim/motion/setup_protomotions.py .
$P data/scripts/convert_soma23_npz_to_proto.py --input-dir $W/npz --output-dir $W/soma \
    --input-fps 30 --output-fps 30 --ignore-motion-filter
$P protomotions/components/motion_lib.py --motion-path $W/soma --output-file $W/soma.pt
$P data/scripts/extract_retargeting_input_keypoints_from_packaged_motionlib.py $W/soma.pt \
    --output-path $W/keypoints --skeleton-format soma --start-idx 0 --skip-freq 1
for f in $W/keypoints/*.npy; do
  d=$W/keypoints_split/$(basename $f .npy); mkdir -p $d; cp $f $d/
  ( $R $M/sim/motion/retarget_marvin_bot.py --protomotions . --keypoints-folder-path $d \
      --output-dir $W/retargeted --source-type soma --no-visualize &&
    $R $M/sim/motion/retarget_marvin_bot.py --protomotions . --keypoints-folder-path $d \
      --output-dir $W/retargeted --source-type soma --no-visualize \
      --save-contacts-only --contacts-dir $W/contacts ) &
done; wait
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

The tracker trains at about 2.5 s an epoch with 4096 robots on one RTX-class GPU and writes a
rollout of every clip (`predicted_motion_lib_epoch_<N>.pt`) every 600 epochs, from epoch 200;
score the latest with `check_tracking.py`. Exporting a tracker and replaying it in MuJoCo are
not set up yet.

`convert_100style.py` turns [100STYLE](https://zenodo.org/records/8127870) motion capture
(CC-BY-4.0) into keypoints instead (then retarget with `--converted` and skip the Kimodo and
keypoint-extraction steps); it was the source before Kimodo.

## Motion sources compared

The same eight motions from three openly licensed sources, retargeted and tracked the same way
(1,400 epochs each, before the root paths, the contact penalty and the sole fix):

| | 100STYLE | Kimodo (SOMA-RP model) | CMU |
|---|---|---|---|
| Clips through ProtoMotions' motion filter | 8 of 8 | 8 of 8 | 5 of 8 (the rest jump) |
| Planted foot sliding (median) | 0.07 m/s | 0.06 m/s | 0.43 m/s |
| Tracker stays up | 5 of 8 | 3 of 8; 5 of 8 with gentler prompts | 3 of 5 |

Kimodo became the source: its motions tracked as well as 100STYLE's once the prompts asked for
a pace Marvin Bot can keep, a root path now sets that pace exactly, and it makes the skills
beyond walking that no openly licensed dataset has. 100STYLE remains an option for locomotion
(CC-BY-4.0, real capture). CMU's recordings are older and noisier.

## Data licences

Motion data is not committed. Kimodo's models are under the NVIDIA Open Model License (NVIDIA
claims no rights in the motions they generate) and were trained on Bones Studio's motion
capture; its text encoder is Meta's Llama 3 8B Instruct under the Meta Llama 3 Community License
and the LLM2Vec adapters (MIT). Each user accepts those licences when downloading the models;
nothing from them is redistributed here. 100STYLE is CC-BY-4.0: credit "100STYLE, I. Mason,
S. Starke and T. Komura, Real-Time Style Modelling of Human Locomotion via Feature-Wise
Transformations and Local Motion Phases (2022)" with anything you publish from it.
ProtoMotions' own sample motions are from the BONES-SEED dataset, whose licence covers only
non-profit academic institutions and companies under $1M annual revenue; this pipeline does not
use them. The CMU database (mocap.cs.cmu.edu) is free to use, with that credit. ProtoMotions'
and PyRoki's code is Apache-2.0 and MIT.
