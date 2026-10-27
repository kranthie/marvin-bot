# Motion pipeline: human motion to a Marvin Bot tracking policy

Status: in progress, untested end to end. The assets load in ProtoMotions; retargeting and
tracker training have not run yet.

Instead of writing rewards for each skill, train a policy that tracks a reference motion
(BeyondMimic-style motion tracking). A new skill is then a new clip, not new reward code. The
pipeline uses NVIDIA's open tools:

| Step | Tool | Here |
|---|---|---|
| Motions | [Kimodo](https://github.com/nv-tlabs/kimodo) (text to motion) or recorded human motion, on the SOMA skeleton | |
| Keypoints | ProtoMotions' `extract_retargeting_input_keypoints_from_packaged_motionlib.py` | |
| Retarget | ProtoMotions' PyRoki retargeter | `retarget_marvin_bot.py` |
| Convert, package | ProtoMotions' `convert_pyroki_retargeted_robot_motions_to_proto.py`, `motion_lib.py` | `fix_sole_height.py` |
| Track | [ProtoMotions](https://github.com/NVlabs/ProtoMotions) on Newton | `setup_protomotions.py` |

`setup_protomotions.py` writes Marvin Bot's MJCF, retargeting URDF and robot config into a
ProtoMotions checkout, all generated from `description/`; rerun it after a description change.

## Environments

ProtoMotions and PyRoki need separate Python environments (PyRoki uses JAX):

```bash
uv venv --python 3.12 venvs/proto
VIRTUAL_ENV=venvs/proto uv pip install torch --index-url https://download.pytorch.org/whl/cu128
VIRTUAL_ENV=venvs/proto uv pip install -r ProtoMotions/requirements_newton.txt
VIRTUAL_ENV=venvs/proto uv pip install -e ProtoMotions
git clone https://github.com/chungmin99/pyroki
uv venv --python 3.12 venvs/pyroki
VIRTUAL_ENV=venvs/pyroki uv pip install -e pyroki
```

ProtoMotions keeps its checkpoints, meshes and sample motions in Git LFS (about 6 GB).

## Steps

Run from the ProtoMotions checkout, with `$M` the marvin-bot checkout and `$W` a work folder.

```bash
venvs/proto/bin/python $M/sim/motion/setup_protomotions.py .
venvs/proto/bin/python data/scripts/extract_retargeting_input_keypoints_from_packaged_motionlib.py \
    <soma motions .pt> --output-path $W/keypoints --skeleton-format soma --start-idx 0 --skip-freq 1
venvs/pyroki/bin/python $M/sim/motion/retarget_marvin_bot.py --protomotions . \
    --keypoints-folder-path $W/keypoints --output-dir $W/retargeted --source-type soma --no-visualize
venvs/pyroki/bin/python $M/sim/motion/retarget_marvin_bot.py --protomotions . \
    --keypoints-folder-path $W/keypoints --output-dir $W/retargeted --source-type soma --no-visualize \
    --save-contacts-only --contacts-dir $W/contacts
venvs/proto/bin/python data/scripts/convert_pyroki_retargeted_robot_motions_to_proto.py \
    --retargeted-motion-dir $W/retargeted --output-dir $W/proto --robot-type marvin_bot \
    --contact-labels-dir $W/contacts --apply-motion-filter --force-remake
venvs/proto/bin/python $M/sim/motion/fix_sole_height.py . $W/proto
venvs/proto/bin/python protomotions/components/motion_lib.py --motion-path $W/proto \
    --output-file $W/marvin_bot_motions.pt
venvs/proto/bin/python protomotions/train_agent.py --robot-name marvin_bot --simulator newton \
    --experiment-path examples/experiments/mimic/mlp.py --experiment-name marvin_bot_track \
    --motion-file $W/marvin_bot_motions.pt --num-envs 4096 --batch-size 16384 --headless
```

## Data licences

Motion data is not committed. Check each source's terms before publishing anything trained on
it: the BONES-SEED motions (ProtoMotions' sample files and Kimodo's SEED models) are gated,
for academic use and qualifying startups; Kimodo's weights are under the NVIDIA Open Model
License, and its text encoder is Meta's gated Llama 3 8B Instruct; ProtoMotions' and PyRoki's
code is Apache-2.0 and MIT.
