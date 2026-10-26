# Concept v0.1

![Concept layout v0.1](concept_v0.svg)

The drawing is to scale and shows every powered joint, coloured by actuator model. It is drawn by `tools/sketch_concept.py` from the sizes in `description/robot.yaml`.

## Proportions

| | |
|---|---|
| Height | 1.255 m |
| Hip axis to sole | 0.69 m (55% of height; sized for 17–18 cm stairs) |
| Thigh, shin | 0.32 m, 0.30 m |
| Ankle axis height | 0.07 m |
| Shoulder width | ~0.49 m |
| Upper arm, forearm to gripper | 0.20 m, 0.215 m |
| Head | ~0.16 m wide, ~0.17 m tall |

## Design language

The goal is a robot that looks finished without a separate cover, because the visible parts are the structure.

- **Limbs:** graphite structural links that taper from joint to joint, with brushed-aluminium actuator caps showing at every joint. The caps are the actuators' own housings or thin machined covers that also act as heat spreaders.
- **Torso:** a light-coloured structural chest piece over an aluminium spine, holding the computer, with the battery behind and below it in a quick-release cradle.
- **Head:** a smooth, glossy dark egg shape with no face. The depth camera sits behind an open slot, not behind glass, because a cover window reflects the camera's IR pattern.
- **Colour:** two-tone, light structural pieces against dark links. The exact scheme is open.
- **Optional covers:** soft covers or padding can be added later at impact points without changing the structure.

## Structure

- **Load path:** machined 6061 cores through each leg and the pelvis. Printed carbon-fibre-nylon shells bolt through to metal inserts and add stiffness, never carry the primary load alone.
- **Part size:** parts are sized for a desktop CNC mill with about 200 × 150 mm usable area. Long thigh and shin cores are split into pinned segments or ordered from an online machining service.
- **Bearings:** far-side bearings at hip roll, hip yaw and knee; a crossed-roller bearing at the waist.
- **Ankles:** two RS03 motors in the calf drive pitch and roll through push rods with spherical rod ends.
- **Feet:** flat aluminium plates with replaceable rubber or TPU soles; no toe joint in v1.
- **Cables:** each limb's CAN and power run through the joints with service loops; connectors at each limb root so a limb can be removed as a unit.

## Open design questions for phase 1

- Knee motor at the joint (v1) or near the hip with a linkage: decided by simulated leg inertia and torque.
- RS06 instead of RS03 at the ankles to save ~1.1 kg: decided by simulated stair and push-recovery torques.
- Exact ankle linkage geometry, push-rod loads and rod-end size.
- Colour scheme and finish of the printed shells.
