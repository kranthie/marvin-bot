const STAGES=[
 {n:0,name:'Start here',c:'var(--sl-color-white)',why:'The whole robot in 3-D: what it is made of, how it is assembled, and how its joints move.'},
 {n:1,name:'Foundations',c:'var(--s1)',why:'What a humanoid is made of, how its joints add up, and why it falls over.'},
 {n:2,name:'Mechanics',c:'var(--s2)',why:'Motors, loads, the ankle linkage, mass and the frame: what sets how strong and heavy the robot is.'},
 {n:3,name:'Electrical, sensing and safety',c:'var(--s3)',why:'Battery, power, the CAN buses, the sensors and computer, and how to keep people safe.'},
 {n:4,name:'Model and simulation',c:'var(--s4)',why:'One description file becomes the models every simulator uses; how simulators and joint controllers work.'},
 {n:5,name:'Learning',c:'var(--s5)',why:'How the robot learns to walk, to copy human motion, and later to use its hands.'},
 {n:6,name:'Toward hardware',c:'var(--s6)',why:'Closing the gap between simulation and the real robot, what it costs, and the order it gets built in.'}];
const PAGES=[
 ['00-3d-model',0,'Marvin Bot in 3-D','Turn the real model around, X-ray the motors, electronics and battery, step through the assembly order, and play skills with live motor loads.'],
 ['01-dof-kinematics',1,'Degrees of freedom and kinematic chains','Joints, ranges and how 27 of them add up; bend a leg and see where the foot goes.'],
 ['02-balance',1,'Balance: centre of mass, support polygon, ZMP and capture point','Why a robot stays up, why it falls, and where to step to recover from a push.'],
 ['03-architecture',1,'System architecture: from battery to policy','Every block from battery to neural network, and one 20 ms control tick traced end to end.'],
 ['04-actuators',2,'Actuators: quasi-direct drive, torque, speed and heat','Gear ratios, torque–speed curves, rated vs peak torque, motor heating, and why each RobStride model sits where it does.'],
 ['05-leg-loads-stairs',2,'Leg loads and stairs','Bend a leg on a stair and watch the knee torque climb past its rating.'],
 ['06-ankle-linkage',2,'The parallel-linkage ankle','Two motors on the shin push rods that tilt the foot; see how they share pitch and roll.'],
 ['07-mass-proportions',2,'Mass budget and proportions','Where the 41 kg goes, how scaling changes torque, and the 1.44 m redesign.'],
 ['08-structure-cad',2,'Structure and code-CAD','Aluminium or plastic, safety factors, pocketed plates, and sweeping joints for collisions.'],
 ['09-power-battery',3,'Power and battery','Runtime calculator, regenerative-braking voltage margin, and the power path from pack to motors.'],
 ['10-can-bus',3,'CAN buses and bandwidth','How many motors fit on one bus at 500 Hz, worked out frame by frame.'],
 ['11-sensors-compute',3,'Sensors and compute','IMU noise and delay, stereo depth error, and the Jetson AGX Orin compute budget.'],
 ['12-safety',3,'Safety','Fall energy, pinch hazards, the e-stop chain and what catches each fault.'],
 ['13-description',4,'The robot description: one source of truth','robot.yaml becomes URDF and MJCF; explore the coupled shoulder limit.'],
 ['14-simulation',4,'Simulation: MuJoCo, Isaac Lab, Newton and PhysX','Timesteps, contacts and integrators, and why a second simulator cross-checks the first.'],
 ['15-pd-control',4,'PD control, gains and armature','Tune a joint’s stiffness and damping and watch it overshoot, settle or go unstable.'],
 ['16-rl-walking',5,'Reinforcement learning for walking','PPO, observations, actions and a reward-shaping sandbox where you can make the robot cheat.'],
 ['17-motion-tracking',5,'Motion tracking: Kimodo, retargeting and BeyondMimic','From a text prompt to a tracked skill: scale a human onto the robot and tune reward kernels.'],
 ['18-wbc-vla',5,'Whole-body control and VLA models','Decoupled control versus one general tracker, and how vision-language-action models drive the arms.'],
 ['19-hands',5,'Hands','The four-finger hands, the wrist, and how much the arm can actually hold.'],
 ['20-sim-to-real',6,'Sim-to-real and system identification','Fit a motor model to bench data and see why randomisation ranges matter.'],
 ['21-bom-costs',6,'Bill of materials and costs','Explore the ≈$17.5k bill of materials by group, phase and cost per newton-metre.'],
 ['22-roadmap',6,'Build phases and roadmap','Seven phases from foundations to perception, what each must prove, and which risks it retires.']];
const INFO={
 head:['Head and neck','A printed helmet with a dark visor holds the Orbbec Gemini 335L depth camera at eye height, on a 2-DOF neck driven by two small serial-bus servos.',['00-3d-model#view=xelec','11-sensors-compute','13-description']],
 torso:['Torso','Carries the 48 V battery high under the shoulder motors, the Jetson AGX Orin stood on its edge, and the electronics bay: contactor, precharge, e-stop relay and the USB-CAN adapters.',['00-3d-model#view=xelec','03-architecture','09-power-battery','11-sensors-compute','12-safety']],
 waist:['Waist yaw','One RS03 turns the upper body ±150°. A waist that could also tilt was considered (+2.5 kg, +$630) and left for a simulation experiment first.',['00-3d-model#tab=skills&skill=look','01-dof-kinematics','07-mass-proportions']],
 pelvis:['Pelvis and IMU','The pelvis holds the hip-roll motors and the IMU, the sensor that tells the policy which way is down and how fast the body is turning.',['00-3d-model#view=xelec','02-balance','11-sensors-compute']],
 arms:['Arms','Six joints per arm: shoulder pitch, roll and yaw, elbow, wrist yaw and wrist pitch. The shoulder roll may only go below 0° when the arm is swung forward (a coupled limit).',['00-3d-model#tab=skills&skill=clap','01-dof-kinematics','04-actuators','13-description','19-hands']],
 hands:['Hands','Bought AmazingHand kits: four fingers, eight small servos each, about 0.40 kg, carried on a servo-driven wrist pitch joint.',['00-3d-model#tab=skills&skill=wave','19-hands','18-wbc-vla']],
 hips:['Hips','Roll, then pitch, then yaw. The two RS04 hip-roll motors sit fixed in the pelvis, so they don’t swing with the leg.',['00-3d-model#view=xact','01-dof-kinematics','04-actuators','05-leg-loads-stairs']],
 legs:['Thigh, knee and shin','Thigh 0.325 m, shin 0.315 m. The knee RS04 is the joint most at risk on stairs: about 118% of its continuous rating in the static 18 cm step check.',['00-3d-model#tab=skills&skill=stair','05-leg-loads-stairs','04-actuators','08-structure-cad']],
 ankles:['Ankle motors','Two RS03 per ankle, staggered on the shin, push rods that tilt the foot in pitch and roll together: a parallel linkage.',['00-3d-model#view=xact','06-ankle-linkage','05-leg-loads-stairs']],
 feet:['Feet','Shoe-shaped plates, 0.24 × 0.11 m, with a rubbery (TPU) sole. The foot outline defines the support polygon the robot balances over.',['00-3d-model#tab=skills&skill=walk','02-balance','08-structure-cad']],
 yaml:['robot.yaml: the single source of truth','Joints, ranges, sizes, motors and CAN buses are written once in description/robot.yaml; everything else is generated or checked from it.',['13-description']],
 cad:['Code-CAD','Python (manifold3d) generates every part, computes masses and inertias for the models, checks strength, and sweeps parts through every joint range looking for collisions.',['00-3d-model#view=made','08-structure-cad','07-mass-proportions']],
 bom:['Bill of materials','One workbook lists every bought part, with price, source and spares: about $17.5k to replicate the robot.',['21-bom-costs']],
 urdf:['Generated models','The URDF (robot model format) is generated from the description; Isaac Lab converts it to USD, and MuJoCo builds its model from the URDF in memory (an MJCF file is written only for the motion-tracking tools).',['00-3d-model','13-description','14-simulation']],
 isaac:['Isaac Lab training','Thousands of simulated robots learn to walk and climb stairs with PPO, on the Newton or PhysX physics engines.',['16-rl-walking','14-simulation','15-pd-control']],
 motion:['Motion tracking','Human motions generated from text (Kimodo) are retargeted onto Marvin Bot (PyRoki) and tracked by one policy (BeyondMimic-style, in ProtoMotions); its results are provisional while a new version trains.',['17-motion-tracking']],
 mujoco:['MuJoCo cross-check','Every trained policy is replayed in a second simulator, with sensor noise, to catch tricks that only work in the first one.',['14-simulation','20-sim-to-real']],
 vla:['Vision-language-action model','Planned: GR00T N1.7, fine-tuned on teleoperated demonstrations, sends arm and hand commands on top of the walking controller.',['18-wbc-vla','19-hands']],
 bench:['Bench and system identification','Phase 2 buys one RS04 and one RS03 and measures torque, speed, heating and CAN timing, then corrects the simulation to match.',['20-sim-to-real','04-actuators','22-roadmap']],
 model3d:['Interactive 3-D model','The generated model, drawn in your browser: peel back the shells, X-ray the motors and electronics, follow the assembly order and watch every motor’s load as the robot squats, walks and steps up a stair.',['00-3d-model','00-3d-model#tab=skills&skill=stair','13-description']],
 robot:['The real robot','Built leg first, then lower body, then the rest; a real-time process on the Orin runs the 50 Hz policy and talks to the motors over CAN, behind a hardware e-stop.',['00-3d-model#tab=assembly&step=1','03-architecture','10-can-bus','12-safety','22-roadmap']]};
const byId=Object.fromEntries(PAGES.map(p=>[p[0],p]));
const stColor=n=>STAGES.find(s=>s.n===n).c;
document.getElementById('legend').innerHTML=STAGES.map(s=>`<span><i style="background:${s.c}"></i>${s.n?`Stage ${s.n} · `:''}${s.name}</span>`).join('');
document.getElementById('stages').innerHTML=STAGES.map(s=>`<section class="stage" style="--c:${s.c}"><h3>${s.n?`Stage ${s.n} · `:''}${s.name}</h3><p class="why">${s.why}</p><div class="cards">`+
  PAGES.filter(p=>p[1]===s.n).map(p=>`<a class="cardlink" style="--c:${s.c}" href="${p[0]}/"><span class="n">${p[0].slice(0,2)}</span><span class="t">${p[2]}</span><span class="d">${p[3]}</span></a>`).join('')+`</div></section>`).join('');
const info=document.getElementById('info');
function show(k,el){
  document.querySelectorAll('.hot.sel').forEach(e=>e.classList.remove('sel')); el.classList.add('sel');
  const [t,d,links]=INFO[k];
  info.innerHTML=`<h3>${t}</h3><p style="margin:.3em 0">${d}</p><div class="links">`+links.map(l=>{const [id,h]=l.split('#'),p=byId[id];return `<a href="${id}/${h?'#'+h:''}" style="color:${stColor(p[1])}">${id.slice(0,2)} · ${h?'see it in 3-D':p[2]} →</a>`}).join('')+`</div>`;
}
document.querySelectorAll('.hot').forEach(el=>{
  el.setAttribute('role','button');
  el.addEventListener('click',()=>show(el.dataset.k,el));
  el.addEventListener('keydown',e=>{if(e.key==='Enter'||e.key===' '){e.preventDefault();show(el.dataset.k,el);}});
});
