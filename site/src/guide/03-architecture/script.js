const $ = id => document.getElementById(id);
const NS='http://www.w3.org/2000/svg';
function el(tag, attrs, parent, text){ const e=document.createElementNS(NS,tag); for(const k in attrs) e.setAttribute(k,attrs[k]); if(text!=null) e.textContent=text; if(parent) parent.appendChild(e); return e; }

/* ---------- Playground 1: architecture ---------- */
const ROWS=[
 {y:8,  lab:'Training (desktop)'},
 {y:88, lab:'High level, 1–30 Hz'},
 {y:168,lab:'Real-time, 50 Hz'},
 {y:248,lab:'Buses and links'},
 {y:328,lab:'Joint level, kHz'},
 {y:408,lab:'Sensors'},
 {y:488,lab:'Power'},
 {y:568,lab:'Mechanical'}];
const B={
 isaac:{r:0,t:'Isaac Lab 3.0 + rsl_rl',s:'walking, stairs (PPO)',what:'Trains the walking and stair policies with PPO, thousands of simulated robots in parallel on Newton or PhysX, with domain randomisation and the 0–20 ms / 0–10 ms delays.',rate:'offline; one policy in hours',lat:'n/a (not on the robot)',repo:'sim/isaaclab/ (robot_cfg.py, tasks/)',page:['16-rl-walking','16 Reinforcement learning for walking']},
 proto:{r:0,t:'ProtoMotions',s:'+ Kimodo, PyRoki',what:'Trains one motion-tracking policy on 25 Kimodo clips retargeted with PyRoki (BeyondMimic-style), on Newton.',rate:'offline; tracker v4 still training (results provisional)',lat:'n/a',repo:'sim/motion/ (README.md, retarget_marvin_bot.py)',page:['17-motion-tracking','17 Motion tracking']},
 mujoco:{r:0,t:'MuJoCo cross-check',s:'a second physics engine',what:'Replays every exported policy with sensor noise: walk, turn, stop, stairs; prints posture, sway, heat and gait figures.',rate:'offline',lat:'n/a',repo:'sim/mujoco/check_policy.py',page:['14-simulation','14 Simulation']},
 onnx:{r:0,t:'ONNX export',s:'policy.onnx + policy.json',what:'Freezes the trained network into a portable file plus a JSON describing joint order, default pose, action scale and observation layout.',rate:'once per trained policy',lat:'n/a',repo:'sim/isaaclab/export_policy.py',page:['20-sim-to-real','20 Sim-to-real']},
 vla:{r:1,t:'VLA: GR00T N1.7',s:'planned; desktop GPU first',what:'Vision-language-action model for manipulation: camera images and an instruction in, short chunks of arm and hand motion out. Fine-tuned on teleoperation data.',rate:'~1–4 Hz (one 16-step chunk ≈0.28 s on AGX Orin with TensorRT, one report)',lat:'hundreds of ms',repo:'docs/decisions.md 2026-10-25; roadmap open item 10',page:['18-wbc-vla','18 Whole-body control and VLA models']},
 teleop:{r:1,t:'Teleoperation',s:'gamepad, later VR',what:'A person gives velocity commands or arm poses; the same path records demonstrations for the VLA.',rate:'10–100 Hz typical',lat:'tens of ms over Wi-Fi',repo:'docs/plan.md, phase 6',page:['18-wbc-vla','18 Whole-body control and VLA models']},
 ros:{r:1,t:'ROS 2 Jazzy + Isaac ROS 4.6',s:'camera, logging, teleop',what:'Middleware for the jitter-tolerant work: camera driver, recording, teleoperation bridge, later the terrain map. Not in the hard-timed path.',rate:'per topic (camera 30 Hz)',lat:'ms, with jitter',repo:'docs/decisions.md 2026-10-25',page:['11-sensors-compute','11 Sensors and compute']},
 policy:{r:2,t:'Policy (ONNX)',s:'MLP, 12 leg joints',what:'Maps the observation (gyro, gravity, command, joint positions and velocities, last action, height map on rough ground) to joint position targets.',rate:'50 Hz (20 ms tick)',lat:'well under 1 ms inference (MLP)',repo:'sim/isaaclab/marvin_bot/tasks/agents.py (256-128-128 / 512-256-128)',page:['16-rl-walking','16 Reinforcement learning for walking']},
 safety:{r:2,t:'Safety layer',s:'clamps, watchdog, falls',what:'Clamps every command to joint, velocity and torque limits; holds the pose if commands stop; switches to damping on a fall; a watchdog relay opens the contactor if the loop dies.',rate:'every tick',lat:'µs',repo:'docs/safety.md (software safety layers)',page:['12-safety','12 Safety']},
 rtio:{r:2,t:'CAN I/O + IMU read',s:'real-time process',what:'Owns the buses: sends each motor a command and reads its reply every cycle, reads the IMU, timestamps everything. Runs outside ROS.',rate:'500 Hz legs, 250 Hz arms',lat:'≈1 ms per cycle (frames)',repo:'robot/drivers/robstride.py (untested on hardware)',page:['10-can-bus','10 CAN buses and bandwidth']},
 orin:{r:2,t:'Jetson AGX Orin 64 GB',s:'JetPack 7.2',what:'Runs everything onboard: the real-time process on reserved CPU cores, ROS 2 and the GPU work. 12 Arm cores, 2048-core GPU, 64 GB shared memory, 15–60 W.',rate:'n/a',lat:'n/a',repo:'docs/requirements.md R40; bom/bom.xlsx ($3,499; 0.877 kg weighed)',page:['11-sensors-compute','11 Sensors and compute']},
 can:{r:3,t:'6 motor CAN buses',s:'1 Mbit/s, ≤48% load',what:'Left/right hip and shank (3 motors each, 500 Hz), left arm (5) and right arm + waist (6) at 250 Hz. Twisted pair, 120 Ω at both ends.',rate:'500 / 250 Hz',lat:'160 µs per frame',repo:'description/robot.yaml (buses)',page:['10-can-bus','10 CAN buses and bandwidth']},
 bms:{r:3,t:'Battery CAN',s:'BMS telemetry',what:'The battery management system reports charge, cell voltages and temperatures; the robot sits down at about 15%.',rate:'10 Hz',lat:'n/a',repo:'description/robot.yaml (battery bus)',page:['09-power-battery','09 Power and battery']},
 serial:{r:3,t:'Feetech serial buses',s:'STS: neck, wrist; SCS: hands',what:'Half-duplex serial buses for the hobby-class servos: one STS bus for the neck and wrist pitch servos, one SCS bus per hand. One USB adapter each.',rate:'tens to hundreds of Hz',lat:'ms',repo:'docs/design/assembly_wiring.md',page:['19-hands','19 Hands']},
 usb:{r:3,t:'USB',s:'camera, IMU, adapters',what:'USB-C to the depth camera; USB (or a microcontroller link) to the IMU; the USB-CAN and servo adapters through a hub.',rate:'camera 30 fps',lat:'≈1 ms polling and up',repo:'bom/bom.xlsx (usb-hub line)',page:['11-sensors-compute','11 Sensors and compute']},
 drv:{r:4,t:'RobStride driver: joint PD',s:'inside each actuator',what:'Computes τ = kp(q* − q) + kd(q̇* − q̇) + τff from the latest command and its own encoder. This is the loop that actually holds the joint.',rate:'kHz (rate not published; bench test)',lat:'<1 ms',repo:'robot/drivers/robstride.py (MIT-style command: position, velocity, kp, kd, torque)',page:['15-pd-control','15 PD control, gains and armature']},
 cur:{r:4,t:'Current loop (FOC)',s:'τ = Kt · I · ratio',what:'Field-oriented control switches the driver transistors to hold the current that gives the requested torque.',rate:'tens of kHz (typical)',lat:'≈0.1–0.5 ms rise',repo:'description/actuators.yaml (torque constants)',page:['04-actuators','04 Actuators']},
 servo:{r:4,t:'Serial servos',s:'own position loop',what:'STS3250 (neck, wrist pitch) and SCS0009 (fingers) hold position with their own internal loop.',rate:'internal',lat:'ms',repo:'description/actuators.yaml (STS3250)',page:['19-hands','19 Hands']},
 imu:{r:5,t:'Pelvis IMU',s:'≥500 Hz gyro + accel',what:'Gives the policy angular velocity and the gravity direction. Industrial AHRS or a BMI088 on a microcontroller, chosen after a bench test.',rate:'≥500 Hz',lat:'a few ms target',repo:'docs/requirements.md R42; sim/isaaclab/marvin_bot/robot_cfg.py (IMU on pelvis, +0.06 m)',page:['11-sensors-compute','11 Sensors and compute']},
 enc:{r:5,t:'Joint encoders',s:'in each actuator',what:'Absolute magnetic encoders give joint position; velocity comes from differencing. Reported back in every reply frame.',rate:'with each CAN reply',lat:'one CAN cycle',repo:'RobStride manuals (14-bit on RS04)',page:['11-sensors-compute','11 Sensors and compute']},
 cam:{r:5,t:'Orbbec Gemini 335L',s:'active stereo, head',what:'Depth computed on the camera from two IR cameras and a projected pattern, about 90 × 65°. For the terrain map on stairs and for the VLA.',rate:'up to 30 fps at 1280 × 800',lat:'tens of ms',repo:'docs/decisions.md 2026-10-25 and 2026-10-28',page:['11-sensors-compute','11 Sensors and compute']},
 bat:{r:6,t:'Battery 13S3P',s:'48 V, ~630 Wh, 3.4 kg',what:'21700 Li-ion cells, 54.6 V full, charged to 53.3 V daily to keep regen spikes under the 60 V actuator limit. Quick-release.',rate:'n/a',lat:'n/a',repo:'docs/decisions.md 2026-10-25 (power)',page:['09-power-battery','09 Power and battery']},
 prot:{r:6,t:'Fuse, precharge, contactor',s:'e-stop opens it',what:'Class-T fuse, precharge then contactor for the actuator power. Wired and wireless e-stops in series and the watchdog relay open the contactor.',rate:'n/a',lat:'contactor opens in tens of ms',repo:'docs/safety.md; docs/design/assembly_wiring.md',page:['12-safety','12 Safety']},
 dcdc:{r:6,t:'DC-DCs + keep-alive',s:'19–20 V, 12 V, 5 V',what:'Computer supply taken before the contactor (an e-stop never reboots it); 12 V for the serial servos; 5 V at each wrist for the hands; keep-alive pack through an ideal-diode OR.',rate:'n/a',lat:'n/a',repo:'docs/design/assembly_wiring.md',page:['09-power-battery','09 Power and battery']},
 frame:{r:7,t:'Frame and shells',s:'6061/7075 + printed',what:'Machined aluminium cores carry the loads; printed shells add stiffness and looks. Strength-checked at SF 2 for a 3 g landing.',rate:'n/a',lat:'n/a',repo:'cad/ (strength.py)',page:['08-structure-cad','08 Structure and code-CAD']},
 joints:{r:7,t:'27 joints',s:'RS04/RS03/RS06/RS00 + servos',what:'6 per leg, 6 per arm, waist yaw, 2 neck. Ranges and actuators in the single source of truth.',rate:'n/a',lat:'n/a',repo:'description/robot.yaml',page:['01-dof-kinematics','01 Degrees of freedom']},
 hands:{r:7,t:'Two AmazingHands',s:'8 servos each',what:'Open-source four-finger hands, bought as kits, on a wrist pitch servo.',rate:'n/a',lat:'n/a',repo:'description/third_party/amazing_hand/',page:['19-hands','19 Hands']}
};
const ARROWS=[['onnx','policy'],['vla','policy'],['teleop','policy'],['policy','safety'],['safety','rtio'],['rtio','can'],['can','drv'],['drv','cur'],['enc','can'],['imu','usb'],['usb','rtio'],['cam','usb'],['usb','ros'],['ros','vla'],['rtio','serial'],['serial','servo'],['bat','prot'],['prot','dcdc'],['isaac','onnx'],['proto','onnx'],['mujoco','onnx'],['bms','rtio']];
const PATHS={
 tick:{ids:['imu','usb','rtio','policy','safety','can','drv','cur','enc','joints'],hint:'One control tick: the IMU and encoders report, the controller runs the policy and the safety layer, CAN carries the targets, the driver closes PD and current loops.'},
 vision:{ids:['cam','usb','ros','vla','policy','orin'],hint:'Perception: the camera computes depth itself; ROS 2 carries it to the terrain map and the VLA, which set goals for the policy.'},
 power:{ids:['bat','prot','dcdc','drv','orin','servo'],hint:'Power: the pack feeds the actuators through the fuse, precharge and contactor; the computer is fed before the contactor so an e-stop never reboots it.'},
 train:{ids:['isaac','proto','mujoco','onnx','policy'],hint:'Training to robot: policies are trained on the desktop, cross-checked in MuJoCo, exported to ONNX and run by the controller.'},
 none:{ids:[],hint:'Click any block for details.'}};
const arch=$('arch'); const pos={};
(function drawArch(){
  const defs=el('defs',{},arch); const m=el('marker',{id:'ah',viewBox:'0 0 10 10',refX:9,refY:5,markerWidth:7,markerHeight:7,orient:'auto-start-reverse'},defs); el('path',{d:'M0,0 L10,5 L0,10 z',fill:'#7d8b9a'},m);
  ROWS.forEach((r,i)=>{ el('rect',{x:0,y:r.y-4,width:900,height:72,rx:6,class:'rowband'},arch); el('text',{x:8,y:r.y+38,class:'rowlab'},arch,r.lab); });
  const X0=140, W=900-X0-8, GAP=12;
  ROWS.forEach((r,ri)=>{ const ks=Object.keys(B).filter(k=>B[k].r===ri); const w=(W-GAP*(ks.length-1))/ks.length;
    ks.forEach((k,i)=>{ const x=X0+i*(w+GAP), y=r.y+6; pos[k]={x,y,w,h:52};
      const g=el('g',{class:'blk',id:'b_'+k,tabindex:0,role:'button'},arch);
      el('rect',{x,y,width:w,height:52,rx:7},g);
      el('text',{x:x+10,y:y+22},g,B[k].t); el('text',{x:x+10,y:y+40,class:'sub'},g,B[k].s);
      g.addEventListener('click',()=>select(k)); g.addEventListener('keydown',e=>{if(e.key==='Enter'||e.key===' '){e.preventDefault();select(k);}});
    });});
  const ag=el('g',{id:'arrows'},arch); arch.insertBefore(ag, arch.querySelector('.blk'));
  ARROWS.forEach(([a,b])=>{ const A=pos[a],Bp=pos[b]; let x1=A.x+A.w/2,y1,x2=Bp.x+Bp.w/2,y2;
    if(Math.abs(A.y-Bp.y)<5){ y1=y2=A.y+26; if(A.x<Bp.x){x1=A.x+A.w;x2=Bp.x;} else {x1=A.x;x2=Bp.x+Bp.w;} }
    else if(A.y<Bp.y){ y1=A.y+52; y2=Bp.y; } else { y1=A.y; y2=Bp.y+52; }
    const my=(y1+y2)/2; el('path',{d:`M${x1},${y1} C${x1},${my} ${x2},${my} ${x2},${y2}`,class:'arr','data-a':a,'data-b':b},ag); });
})();
let curPath='tick', selKey='policy';
function setPath(p){ curPath=p; document.querySelectorAll('#paths button').forEach(b=>b.classList.toggle('on',b.dataset.path===p));
  const ids=PATHS[p].ids; $('pathhint').textContent=PATHS[p].hint;
  for(const k in B){ const g=$('b_'+k); g.classList.toggle('hl',ids.includes(k)); g.classList.toggle('dim',ids.length>0&&!ids.includes(k)&&k!==selKey); }
  document.querySelectorAll('#arch .arr').forEach(a=>a.classList.toggle('hl',ids.includes(a.dataset.a)&&ids.includes(a.dataset.b)));
}
function select(k){ selKey=k; document.querySelectorAll('#arch .blk').forEach(g=>g.classList.remove('sel')); $('b_'+k).classList.add('sel');
  const b=B[k]; $('info').innerHTML=`<h4>${b.t}</h4><div>${b.what}</div><dl><dt>Rate</dt><dd>${b.rate}</dd><dt>Latency</dt><dd>${b.lat}</dd><dt>In the repo</dt><dd><code>${b.repo}</code></dd><dt>In depth</dt><dd><a href=${JSON.stringify(b.page[0]+".html")}>${b.page[1]}</a></dd></dl>`;
  setPath(curPath); updateNote(); }
document.querySelectorAll('#paths button').forEach(b=>b.addEventListener('click',()=>{ setPath(b.dataset.path); updateNote(); }));

/* ---------- Playground 2: control tick ---------- */
const DEFAULTS={prate:50,crate:500,nmot:3,imu:500,sdel:2,inf:0.3,drv:10,mode:'avg'};
const state={...DEFAULTS};
const PRESETS={
 today:{...DEFAULTS},
 usb:{...DEFAULTS,imu:200,sdel:12},
 arm:{...DEFAULTS,crate:250,nmot:6},
 cpu:{...DEFAULTS,inf:8},
 fast:{...DEFAULTS,prate:100}};
const FMT={prate:v=>v+' Hz',crate:v=>v+' Hz',nmot:v=>v,imu:v=>v+' Hz',sdel:v=>(+v).toFixed(1)+' ms',inf:v=>(+v).toFixed(2)+' ms',drv:v=>v+' kHz'};
function bind(id){ const e=$(id), o=$('o_'+id); const upd=()=>{ state[id]=parseFloat(e.value); if(o) o.textContent=FMT[id](state[id]); render(); };
  e.addEventListener('input',upd); if(o) o.textContent=FMT[id](e.value); }
Object.keys(FMT).forEach(bind);
$('mode').addEventListener('change',()=>{ state.mode=$('mode').value; render(); });
function setState(s){ Object.assign(state,s); for(const k in FMT){ $(k).value=state[k]; $('o_'+k).textContent=FMT[k](state[k]); } $('mode').value=state.mode; render(); }
document.querySelectorAll('[data-preset]').forEach(b=>b.addEventListener('click',()=>setState(PRESETS[b.dataset.preset])));

const FRAME_US=160, OBS_MS=0.15, CUR_RISE=0.3;
let segs=[], model={};
function compute(){
  const k = state.mode==='avg'?0.5:1;
  const T=1000/state.prate, imuP=1000/state.imu, canP=1000/state.crate;
  const hold=k*imuP, wait=k*T, canWait=k*canP;
  const frames=state.nmot*2*FRAME_US/1000; // ms per cycle, all motors
  const frameOne = state.mode==='avg' ? frames/2 : frames; // our motor's command lands somewhere in the burst
  const drv=k*(1/state.drv)+CUR_RISE;
  const comp=OBS_MS+state.inf;
  segs=[
   {k:'IMU sample hold',v:hold,c:'#4fb3ff',side:'s'},
   {k:'Sensor transport',v:state.sdel,c:'#7fc8ff',side:'s'},
   {k:'Wait for policy tick',v:wait,c:'#56606c',side:'w'},
   {k:'Observation + inference',v:comp,c:'#ffb454',side:'a'},
   {k:'Wait for CAN slot',v:canWait,c:'#b48cff',side:'a'},
   {k:'Frames on the bus',v:frameOne,c:'#d2b8ff',side:'a'},
   {k:'Driver PD + current rise',v:drv,c:'#4cc38a',side:'a'}];
  const sens=hold+state.sdel, act=comp+canWait+frameOne+drv, total=sens+wait+act;
  const load=state.nmot*2*FRAME_US*state.crate/1e6;
  model={T,sens,act,total,load,frames,canP,comp,fromTick:act};
}
const tick=$('tick');
function drawTick(){
  tick.innerHTML='';
  const L=20,R=880,scaleMax=Math.max(model.total, 2*model.T, 25)*1.04; const sx=v=>L+(R-L)*v/scaleMax;
  // tick boundaries: ticks happen at times when the wait ends
  const tickStart=model.sens+segs[2].v; // the policy tick that uses this observation
  el('text',{x:L,y:18},tick,'Event (pelvis starts to tip)');
  el('text',{x:R,y:18,'text-anchor':'end'},tick,'Torque at the joint');
  // ruler
  el('line',{x1:L,y1:150,x2:R,y2:150,stroke:'#2a3340'},tick);
  const step = scaleMax>60?10:5;
  for(let t=0;t<=scaleMax;t+=step){ el('line',{x1:sx(t),y1:150,x2:sx(t),y2:156,stroke:'#5d6b7a'},tick); el('text',{x:sx(t),y:170,'text-anchor':'middle',class:'small'},tick,t+' ms'); }
  // policy tick windows
  for(let n=-3;n<6;n++){ const t0=tickStart+n*model.T; if(t0>scaleMax) break; if(t0+model.T<0) continue;
    const a=Math.max(0,t0), b=Math.min(scaleMax,t0+model.T);
    el('rect',{x:sx(a),y:182,width:Math.max(0,sx(b)-sx(a)),height:16,fill:n===0?'#18324a':'#141a21',stroke:'#2a3340'},tick);
    if(n===0) el('text',{x:sx(a)+4,y:194,class:'small',fill:'#cfe3f5'},tick,'the tick that acts ('+model.T.toFixed(1)+' ms)');
    if(t0>=0) el('line',{x1:sx(t0),y1:40,x2:sx(t0),y2:200,stroke:'#3b4756','stroke-dasharray':'3 3'},tick);
  }
  el('text',{x:L,y:218,class:'small'},tick,'Policy ticks (dashed lines): a new observation is used only at the next tick.');
  // stacked bar
  let t=0; segs.forEach((s,i)=>{ const x=sx(t), w=Math.max(0.5,sx(t+s.v)-sx(t));
    el('rect',{x,y:60,width:w,height:46,fill:s.c,opacity:s.side==='w'?0.55:0.9},tick);
    if(w>46) el('text',{x:x+w/2,y:88,'text-anchor':'middle',fill:'#0b0e12','font-weight':600},tick,s.v.toFixed(1));
    t+=s.v; });
  // braces for sensor / action sides
  const brace=(a,b,y,txt,col)=>{ el('path',{d:`M${sx(a)},${y} v6 H${sx(b)} v-6`,stroke:col,fill:'none'},tick); el('text',{x:(sx(a)+sx(b))/2,y:y+20,'text-anchor':'middle',class:'small'},tick,txt); };
  brace(0,model.sens,110,'sensor side '+model.sens.toFixed(1)+' ms','#4fb3ff');
  brace(model.sens+segs[2].v,model.total,110,'action side '+model.act.toFixed(1)+' ms','#ffb454');
  // trained ranges overlay
  el('rect',{x:sx(0),y:44,width:sx(20)-sx(0),height:8,fill:'#4fb3ff',opacity:.25},tick);
  el('text',{x:sx(0)+2,y:40,class:'small'},tick,'trained sensor delay 0–20 ms');
  const a0=model.sens+segs[2].v; el('rect',{x:sx(a0),y:108-56,width:sx(a0+10)-sx(a0),height:8,fill:'#ffb454',opacity:.25},tick);
  el('text',{x:sx(a0)+2,y:40,class:'small'},tick,'trained command delay 0–10 ms');
  dot=el('circle',{cx:sx(0),cy:83,r:7,fill:'#fff',stroke:'#0b0e12','stroke-width':2},tick);
  sxf=sx;
}
let dot=null, sxf=null, playing=true, t0=performance.now();
function anim(now){ if(playing&&dot&&model.total){ const slow=120; // 1 ms of robot time = 120 ms on screen
    const tt=((now-t0)/slow)%(model.total+6); dot.setAttribute('cx',sxf(Math.min(tt,model.total))); }
  requestAnimationFrame(anim); }
requestAnimationFrame(anim);
$('play').addEventListener('click',()=>{ playing=!playing; $('play').textContent=playing?'Pause animation':'Play animation'; });

function ro(id,val,cls){ const r=$(id); r.querySelector('.v').textContent=val; r.className='ro '+(cls||''); }
function render(){
  compute(); drawTick();
  $('seglegend').innerHTML=segs.map(s=>`<span><i style="background:${s.c}"></i>${s.k}: <b>${s.v.toFixed(2)} ms</b></span>`).join('');
  ro('r_total',model.total.toFixed(1)+' ms','');
  const fit=model.fromTick/model.T; ro('r_fit',model.fromTick.toFixed(1)+' / '+model.T.toFixed(0)+' ms', fit<=0.5?'ok':fit<=1?'warn':'bad');
  ro('r_sens',model.sens.toFixed(1)+' ms', model.sens<=15?'ok':model.sens<=20?'warn':'bad');
  ro('r_act',model.act.toFixed(1)+' ms', model.act<=7.5?'ok':model.act<=10?'warn':'bad');
  ro('r_load',model.load.toFixed(2)+(model.frames>model.canP?' (frames > cycle!)':''), model.load<=0.6?'ok':model.load<=0.85?'warn':'bad');
  updateNote();
}
function updateNote(){
  const parts=[]; const D=DEFAULTS;
  if(state.prate!==D.prate) parts.push(`policy at ${state.prate} Hz (tick ${(1000/state.prate).toFixed(1)} ms; the policy must be retrained at this rate)`);
  if(state.crate!==D.crate) parts.push(`CAN commands at ${state.crate} Hz`);
  if(state.nmot!==D.nmot) parts.push(`${state.nmot} motors on the bus`);
  if(state.imu!==D.imu) parts.push(`IMU at ${state.imu} Hz`);
  if(state.sdel!==D.sdel) parts.push(`${state.sdel} ms sensor transport delay`);
  if(state.inf!==D.inf) parts.push(`${state.inf} ms inference`);
  if(state.drv!==D.drv) parts.push(`a ${state.drv} kHz driver loop`);
  if(state.mode!==D.mode) parts.push('worst-case timing');
  const res=`Result (${state.mode==='avg'?'average':'worst case'}): event to torque ${model.total.toFixed(1)} ms; tick start to torque ${model.fromTick.toFixed(1)} ms of a ${model.T.toFixed(1)} ms tick; sensor side ${model.sens.toFixed(1)} ms vs the 0–20 ms trained range; action side ${model.act.toFixed(1)} ms vs the 0–10 ms trained range; CAN load ${model.load.toFixed(2)} (limit 0.6).`;
  const flags=[]; if(model.sens>20) flags.push('sensor side is outside the trained delay range: widen the randomisation and retrain, or cut IMU latency');
  if(model.act>10) flags.push('action side is outside the trained motor-delay range: speed up inference or retrain with a wider range');
  if(model.load>0.6) flags.push('CAN load above 0.6: split the bus or lower the rate');
  if(model.fromTick>model.T) flags.push('the torque lands after the next tick has started');
  const sel = selKey? ` Block in focus in the architecture diagram: ${B[selKey].t} (${B[selKey].repo}).`:'';
  $('note').textContent = parts.length
   ? `For Marvin Bot (1.44 m, 41 kg humanoid; Jetson AGX Orin, real-time control process outside ROS, policy trained with 0–20 ms observation and 0–10 ms command delays), I explored a control tick with ${parts.join(', ')}. ${res}${flags.length?' Issues: '+flags.join('; ')+'.':' Everything stays inside the trained ranges.'}${sel} Next step: confirm with the bench latency test (bench/actuator/bench_robstride.py latency) on the Orin.`
   : `Change a control in "Follow one control tick" to build a note. (Marvin Bot today: ${res})${sel}`;
}
$('copy').addEventListener('click',()=>{ navigator.clipboard.writeText($('note').textContent).then(()=>{ const b=$('copy'); b.textContent='Copied!'; setTimeout(()=>b.textContent='Copy',1200); }); });
render(); select('policy'); setPath('tick');
