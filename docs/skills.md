# Skills in simulation

Every skill the motion tracker has learned, on the current 1.41 m model (RS10P arm motors, the
rounded head). The solid robot is the trained policy; the translucent blue robot is the reference
motion it follows (a human motion generated with Kimodo and retargeted to Marvin Bot). One policy
plays all 25 clips: tracker v6, warm-started from the previous tracker (about 5,000 epochs on the
1.44 m model) and trained 1,800 more epochs on the current model, on Newton with 16,384 robots at
once (about 3 hours on one GPU); every clip stays upright. Under each clip: the hands' and feet's
mean distance from where the reference puts them, measured relative to the pelvis.

The clips are rendered with `sim/motion/render_tracking.py`; how they are made, trained and scored
is in [sim/motion/README.md](../sim/motion/README.md). This is simulation only, though in two
simulators: replayed in MuJoCo (`sim/motion/replay_mujoco.py`) with sensor noise and the motors'
torque-speed limits, every clip stays upright and the hands, feet and joints track as closely as
in Newton, where it was trained. It has not run on hardware. The weakest skills are the
arm-heavy ones (the two-hand wave, reach forward, clap, arms raise: hands 9–13 cm off), and the
squat and pick-up stop 4.5 and 9 cm short of the reference's lowest point. The arm motors are
closest to their limits in the two-hand wave (the left shoulder yaw touches the RS10P's 42 N·m
peak) and the clap (the left shoulder roll at 78% of its rated torque, RMS).

## Walking and turning

| Walk | Walk slow | Walk back |
|---|---|---|
| ![walk](media/skills/walk.webp) | ![walk slow](media/skills/walk_slow.webp) | ![walk back](media/skills/walk_back.webp) |
| hands 4.2 cm, feet 5.5 cm | hands 4.7 cm, feet 4.4 cm | hands 5.4 cm, feet 4.2 cm |

| Jog | March | Start stop |
|---|---|---|
| ![jog](media/skills/jog.webp) | ![march](media/skills/march.webp) | ![start stop](media/skills/start_stop.webp) |
| hands 4.1 cm, feet 6.1 cm | hands 4.9 cm, feet 6.1 cm | hands 3.6 cm, feet 4.8 cm |

| Carry walk | Side left | Side right |
|---|---|---|
| ![carry walk](media/skills/carry_walk.webp) | ![side left](media/skills/side_left.webp) | ![side right](media/skills/side_right.webp) |
| hands 6.7 cm, feet 5.5 cm | hands 6.2 cm, feet 5.3 cm | hands 7.2 cm, feet 5.2 cm |

| Turn left | Turn right | Turn in place |
|---|---|---|
| ![turn left](media/skills/turn_left.webp) | ![turn right](media/skills/turn_right.webp) | ![turn in place](media/skills/turn_in_place.webp) |
| hands 4.7 cm, feet 4.6 cm | hands 5.2 cm, feet 5.5 cm | hands 2.6 cm, feet 6.4 cm |

## Whole body

| Kick | Squat | Pick up |
|---|---|---|
| ![kick](media/skills/kick.webp) | ![squat](media/skills/squat.webp) | ![pick up](media/skills/pick_up.webp) |
| hands 3.9 cm, feet 4.4 cm | hands 6.8 cm, feet 5.6 cm | hands 3.2 cm, feet 3.8 cm |

| Bow | Dance sway | Idle |
|---|---|---|
| ![bow](media/skills/bow.webp) | ![dance sway](media/skills/dance_sway.webp) | ![idle](media/skills/idle.webp) |
| hands 3.6 cm, feet 4.3 cm | hands 7.3 cm, feet 3.5 cm | hands 2.9 cm, feet 4.4 cm |

| Look around |
|---|
| ![look around](media/skills/look_around.webp) |
| hands 3.4 cm, feet 3.6 cm |

## Arms and gestures

| Clap | Wave | Wave both |
|---|---|---|
| ![clap](media/skills/clap.webp) | ![wave](media/skills/wave.webp) | ![wave both](media/skills/wave_both.webp) |
| hands 8.8 cm, feet 3.8 cm | hands 5.1 cm, feet 2.7 cm | hands 12.8 cm, feet 5.2 cm |

| Point | Reach forward | Arms raise |
|---|---|---|
| ![point](media/skills/point.webp) | ![reach forward](media/skills/reach_forward.webp) | ![arms raise](media/skills/arms_raise.webp) |
| hands 3.4 cm, feet 2.9 cm | hands 9.7 cm, feet 4.8 cm | hands 8.8 cm, feet 1.8 cm |
