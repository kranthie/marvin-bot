# Motion pipeline: human motion to a Marvin Bot tracking policy

Status: runs end to end; no tracker walks yet. Eight walking, jogging and turning clips retarget
within every joint limit. Trackers 1-4 counted the clips as tracked but did not follow them (the
first shuffled on the spot, covering 0-32% of the reference's distance; later ones stood still);
tracker 5, with BeyondMimic's global anchor-position reward restored, is being trained.

> [!IMPORTANT]
> **Motion data licence.** The sample motions used so far come from the BONES-SEED dataset
> (Motion Data by [Bones Studio](https://bones.studio)), whose licence grants rights only to
> non-profit academic institutions and to companies under $1M annual revenue, and requires that
> credit. Check that you qualify before using it, or use openly licensed motion instead. No
> motion data, and no render of it, is published in this repository.

Instead of writing rewards for each skill, train a policy that tracks a reference motion
(BeyondMimic-style motion tracking). A new skill is then a new clip, not new reward code. The
pipeline uses NVIDIA's open tools:

| Step | Tool | Here |
|---|---|---|
| Motions | [Kimodo](https://github.com/nv-tlabs/kimodo) (text to motion) or recorded human motion, on the SOMA skeleton | |
| Keypoints | ProtoMotions' `extract_retargeting_input_keypoints_from_packaged_motionlib.py` | |
| Retarget | ProtoMotions' PyRoki retargeter | `retarget_marvin_bot.py` |
| Convert, package | ProtoMotions' `convert_pyroki_retargeted_robot_motions_to_proto.py`, `motion_lib.py` | `fix_sole_height.py` |
| Track | [ProtoMotions](https://github.com/NVlabs/ProtoMotions) on Newton, BeyondMimic-style rewards | `setup_protomotions.py`, `check_tracking.py` |

`setup_protomotions.py` writes Marvin Bot's MJCF, retargeting URDF and robot config into a
ProtoMotions checkout, all generated from `description/`; rerun it after a description change.

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

Run from the ProtoMotions checkout (`cd /path/to/parent/ProtoMotions`); the interpreters are
`../venvs/proto` and `../venvs/pyroki`. The source motions are ProtoMotions' SOMA sample,
`data/motion_for_trackers/soma23_bones_seed_mini.pt` (61 clips, BONES-SEED: check its licence
first, see below). The trackers so far train on eight walking, jogging and turning clips, which
the copy step selects; retargeting all 61 would take about 10 hours.

```bash
P=../venvs/proto/bin/python; R=../venvs/pyroki/bin/python
$P $M/sim/motion/setup_protomotions.py .
$P data/scripts/extract_retargeting_input_keypoints_from_packaged_motionlib.py \
    data/motion_for_trackers/soma23_bones_seed_mini.pt --output-path $W/keypoints \
    --skeleton-format soma --start-idx 0 --skip-freq 1
mkdir -p $W/keypoints8
for c in child_walk_180_R_max_003__A076 sad_walk_ff_start_315_R_002__A204_M \
    jog_ff_start_180_R_002__A531 walk_ff_start_315_003__A060_M jog_ff_stop_180_R_001__A103_M \
    leaning_walk_arc_cw_stop_003__A166 idle_turn_360_003__A175_M happy_walk_ff_stop_180_R_005__A421; do
  cp $W/keypoints/*"$c"_keypoints.npy $W/keypoints8/
done
$R $M/sim/motion/retarget_marvin_bot.py --protomotions . \
    --keypoints-folder-path $W/keypoints8 --output-dir $W/retargeted --source-type soma --no-visualize
$R $M/sim/motion/retarget_marvin_bot.py --protomotions . \
    --keypoints-folder-path $W/keypoints8 --output-dir $W/retargeted --source-type soma --no-visualize \
    --save-contacts-only --contacts-dir $W/contacts
$P data/scripts/convert_pyroki_retargeted_robot_motions_to_proto.py \
    --retargeted-motion-dir $W/retargeted --output-dir $W/proto --robot-type marvin_bot \
    --contact-labels-dir $W/contacts --apply-motion-filter --force-remake
$P $M/sim/motion/fix_sole_height.py . $W/proto
$P protomotions/components/motion_lib.py --motion-path $W/proto --output-file $W/marvin_bot_motions.pt
$P protomotions/train_agent.py --robot-name marvin_bot --simulator newton \
    --experiment-path examples/experiments/mimic/mlp_bm_l2c2.py --experiment-name marvin_bot_track \
    --motion-file $W/marvin_bot_motions.pt --num-envs 4096 --batch-size 16384 --headless
$P $M/sim/motion/check_tracking.py \
    results/marvin_bot_track/results/predicted_motion_lib_epoch_<N>.pt $W/marvin_bot_motions.pt
```

Retargeting takes about 10 minutes a clip on the CPU (JAX recompiles for each clip length); the
tracker trains at about 2.5 s an epoch with 4096 robots on one RTX-class GPU and writes a
rollout of every clip (`predicted_motion_lib_epoch_<N>.pt`) every 600 epochs, from epoch 200;
score the latest with `check_tracking.py`. The upstream experiment above is what trackers 1-3
used; it does not walk on Marvin Bot (see Status), and the changes being tried for tracker 5
will be added here once one walks. Exporting a tracker and replaying it in MuJoCo are not set up
yet.

## Data licences

Motion data is not committed. Check each source's terms before using it or publishing anything
trained on it: the BONES-SEED motions (ProtoMotions' sample files and Kimodo's SEED models;
Motion Data by [Bones Studio](https://bones.studio), BONES-SEED License) may be used only by
non-profit academic institutions and companies under $1M annual revenue, with that credit; Kimodo's weights are under the NVIDIA Open Model
License, and its text encoder is Meta's gated Llama 3 8B Instruct; ProtoMotions' and PyRoki's
code is Apache-2.0 and MIT.
