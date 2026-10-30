# Skills in simulation

Every skill the motion tracker has learned, on the current 1.44 m model. The solid robot is the
trained policy; the translucent blue robot is the reference motion it follows (a human motion
generated with Kimodo and retargeted to Marvin Bot). One policy plays all 25 clips: it was trained
for about 5,000 epochs on Newton with 16,384 robots at once (about 9 hours on one GPU) and every
clip stays upright. Under each clip: the hands' and feet's mean distance from where the reference
puts them, measured relative to the pelvis.

The clips are rendered with `sim/motion/render_tracking.py`; how they are made, trained and scored
is in [sim/motion/README.md](../sim/motion/README.md). This is simulation only: the policy has not
been replayed in MuJoCo or run on hardware. The weakest skills are the two-handed ones (clap, the
two-hand wave, arms raise: hands 12–16 cm off), and the squat and pick-up stop 7–11 cm short of
the reference's lowest point.

## Walking and turning

| Walk | Walk slow | Walk back |
|---|---|---|
| ![walk](media/skills/walk.webp) | ![walk slow](media/skills/walk_slow.webp) | ![walk back](media/skills/walk_back.webp) |
| hands 4.6 cm, feet 5.1 cm | hands 4.9 cm, feet 5.4 cm | hands 6.2 cm, feet 4.8 cm |

| Jog | March | Start stop |
|---|---|---|
| ![jog](media/skills/jog.webp) | ![march](media/skills/march.webp) | ![start stop](media/skills/start_stop.webp) |
| hands 4.7 cm, feet 6.5 cm | hands 5.0 cm, feet 6.6 cm | hands 4.3 cm, feet 5.5 cm |

| Carry walk | Side left | Side right |
|---|---|---|
| ![carry walk](media/skills/carry_walk.webp) | ![side left](media/skills/side_left.webp) | ![side right](media/skills/side_right.webp) |
| hands 6.9 cm, feet 6.2 cm | hands 8.2 cm, feet 5.9 cm | hands 8.9 cm, feet 6.7 cm |

| Turn left | Turn right | Turn in place |
|---|---|---|
| ![turn left](media/skills/turn_left.webp) | ![turn right](media/skills/turn_right.webp) | ![turn in place](media/skills/turn_in_place.webp) |
| hands 5.3 cm, feet 3.7 cm | hands 5.5 cm, feet 5.6 cm | hands 3.7 cm, feet 6.5 cm |

## Whole body

| Kick | Squat | Pick up |
|---|---|---|
| ![kick](media/skills/kick.webp) | ![squat](media/skills/squat.webp) | ![pick up](media/skills/pick_up.webp) |
| hands 4.9 cm, feet 4.5 cm | hands 8.6 cm, feet 6.7 cm | hands 3.6 cm, feet 4.7 cm |

| Bow | Dance sway | Idle |
|---|---|---|
| ![bow](media/skills/bow.webp) | ![dance sway](media/skills/dance_sway.webp) | ![idle](media/skills/idle.webp) |
| hands 5.2 cm, feet 4.0 cm | hands 7.9 cm, feet 3.6 cm | hands 3.4 cm, feet 4.1 cm |

| Look around |
|---|
| ![look around](media/skills/look_around.webp) |
| hands 3.1 cm, feet 3.1 cm |

## Arms and gestures

| Clap | Wave | Wave both |
|---|---|---|
| ![clap](media/skills/clap.webp) | ![wave](media/skills/wave.webp) | ![wave both](media/skills/wave_both.webp) |
| hands 15.2 cm, feet 2.5 cm | hands 5.9 cm, feet 2.5 cm | hands 15.8 cm, feet 4.3 cm |

| Point | Reach forward | Arms raise |
|---|---|---|
| ![point](media/skills/point.webp) | ![reach forward](media/skills/reach_forward.webp) | ![arms raise](media/skills/arms_raise.webp) |
| hands 5.1 cm, feet 3.2 cm | hands 10.7 cm, feet 5.6 cm | hands 12.3 cm, feet 3.0 cm |
