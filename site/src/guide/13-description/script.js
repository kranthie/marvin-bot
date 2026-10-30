const $ = id => document.getElementById(id);
const D2R = Math.PI/180;
/* ---------------- Playground A: coupled limit ---------------- */
const PTS = [[20,0],[45,-10],[75,-10],[90,-20],[135,-20],[150,0]];
function lb(p){ if(p<=PTS[0][0]||p>=PTS[PTS.length-1][0]) return 0;
  for(let i=0;i<PTS.length-1;i++){const [a,qa]=PTS[i],[b,qb]=PTS[i+1]; if(p>=a&&p<=b) return qa+(p-a)/(b-a)*(qb-qa);} return 0; }
const DEFA = {pitch:0, roll:8};
const A = {...DEFA, zoom:true, showsim:true};
const PRESETS = {stance:{pitch:0,roll:8}, clap:{pitch:90,roll:-20}, reach:{pitch:60,roll:-10}, hang:{pitch:0,roll:-10}, over:{pitch:140,roll:-15}};
const SH_Y = 0.158, SH_Z = 1.165, ARM = 0.569;
function classify(p,r){
  if(p<-60||p>180||r<-20||r>120) return 'out';
  if(r<lb(p)-1e-9) return 'sim';
  return 'ok';
}
function armDir(p,r){ // robot frame x fwd, y left, z up, left arm
  const P=p*D2R, R=r*D2R; return [Math.cos(R)*Math.sin(P), Math.sin(R), -Math.cos(R)*Math.cos(P)]; }
let plotGeom=null;
function drawA(){
  const svg=$('cl'); const W=640;
  const px0=56, px1=620, py0=20, py1=300; // plot area
  const ymin = A.zoom?-25:-25, ymax = A.zoom?40:120;
  const X=p=>px0+(p+60)/240*(px1-px0), Y=r=>py1-(r-ymin)/(ymax-ymin)*(py1-py0);
  plotGeom={X,Y,px0,px1,py0,py1,ymin,ymax};
  let s='';
  s+=`<rect x="${px0}" y="${py0}" width="${px1-px0}" height="${py1-py0}" fill="#2a3340"/>`;
  // allowed region polygon: from lb(p) to 120 for p in [-60,180]
  let top=Math.min(120,ymax);
  let poly=[];
  for(let p=-60;p<=180;p+=1) poly.push([X(p),Y(Math.max(lb(p),ymin))]);
  poly.push([X(180),Y(top)],[X(-60),Y(top)]);
  s+=`<polygon points="${poly.map(q=>q.join(',')).join(' ')}" fill="#1f4d3a"/>`;
  if(A.showsim){ let sp=[]; for(let p=-60;p<=180;p+=1) sp.push([X(p),Y(lb(p))]); sp.push([X(180),Y(-20)],[X(-60),Y(-20)]);
    s+=`<polygon points="${sp.map(q=>q.join(',')).join(' ')}" fill="#5a2330"/>`; }
  // grid
  for(let p=-60;p<=180;p+=30){ s+=`<line x1="${X(p)}" x2="${X(p)}" y1="${py0}" y2="${py1}" stroke="#0f1216" stroke-width="1"/><text x="${X(p)}" y="${py1+14}" fill="#9aa7b4" font-size="11" text-anchor="middle">${p}°</text>`; }
  const step = A.zoom?10:20;
  for(let r=Math.ceil(ymin/step)*step;r<=ymax;r+=step){ s+=`<line x1="${px0}" x2="${px1}" y1="${Y(r)}" y2="${Y(r)}" stroke="#0f1216"/><text x="${px0-6}" y="${Y(r)+4}" fill="#9aa7b4" font-size="11" text-anchor="end">${r}°</text>`; }
  s+=`<line x1="${px0}" x2="${px1}" y1="${Y(-20)}" y2="${Y(-20)}" stroke="#ff6b6b" stroke-dasharray="5 4"/><text x="${px1-4}" y="${Y(-20)+13}" fill="#ff6b6b" font-size="11" text-anchor="end">plain −20° limit (simulation)</text>`;
  // bound polyline
  const bl=[[ -60,0],[20,0],...PTS.slice(1,-1),[150,0],[180,0]];
  s+=`<polyline points="${bl.map(([p,r])=>X(p)+','+Y(r)).join(' ')}" fill="none" stroke="#ffb454" stroke-width="2.5"/>`;
  PTS.forEach(([p,r])=>{s+=`<circle cx="${X(p)}" cy="${Y(r)}" r="3.5" fill="#ffb454"/>`;});
  s+=`<text x="${(px0+px1)/2}" y="${py1+30}" fill="#9aa7b4" font-size="12" text-anchor="middle">shoulder pitch (+ = arm forward)</text>`;
  s+=`<text transform="translate(14 ${(py0+py1)/2}) rotate(-90)" fill="#9aa7b4" font-size="12" text-anchor="middle">shoulder roll (+ = out)</text>`;
  if(A.zoom) s+=`<text x="${px0+6}" y="${py0+14}" fill="#cfe3f5" font-size="11">all roll up to 120° is allowed above this view</text>`;
  // the point
  const c=classify(A.pitch,A.roll);
  const col=c==='ok'?'#4cc38a':(c==='sim'?'#ff6b6b':'#9aa7b4');
  const cy=Math.min(Math.max(Y(A.roll),py0),py1);
  s+=`<circle id="dot" cx="${X(A.pitch)}" cy="${cy}" r="9" fill="${col}" stroke="#fff" stroke-width="2" style="cursor:grab"/>`;
  // ------- front view and side view
  const fy0=340, fh=170;
  s+=`<rect x="0" y="${fy0-8}" width="${W}" height="${fh+16}" fill="#0c1015"/>`;
  s+=`<text x="10" y="${fy0+8}" fill="#9aa7b4" font-size="11">front view (facing the robot)</text><text x="440" y="${fy0+8}" fill="#9aa7b4" font-size="11">side view (left arm)</text>`;
  const sc=170, cx=230, zc=fy0+40; // world z=SH_Z at zc
  const FX=y=>cx - y*sc, FZ=z=>zc-(z-SH_Z)*sc; // facing robot: robot's left on the viewer's right -> x=cx - y? viewer right = robot left -> +y to the right
  const FX2=y=>cx + y*sc;
  // torso box (collision ~0.20 wide, from z 0.81 to 1.06 + head)
  s+=`<rect x="${FX2(-0.10)}" y="${FZ(1.25)}" width="${0.20*sc}" height="${(1.25-0.72)*sc}" fill="#232c37" stroke="#3a4757"/>`;
  s+=`<rect x="${FX2(-0.075)}" y="${FZ(1.40)}" width="${0.15*sc}" height="${0.14*sc}" rx="10" fill="#232c37" stroke="#3a4757"/>`;
  s+=`<line x1="${FX2(-0.2)}" x2="${FX2(0.2)}" y1="${FZ(SH_Z)}" y2="${FZ(SH_Z)}" stroke="#3a4757" stroke-dasharray="3 3"/>`;
  const d=armDir(A.pitch,A.roll);
  for(const side of [1,-1]){
    const sy=side*SH_Y, hy=sy+side*ARM*d[1], hz=SH_Z+ARM*d[2];
    const ey=sy+side*0.2*d[1], ez=SH_Z+0.2*d[2];
    const depth=d[0]; // forward towards viewer
    const w=6+4*Math.max(0,depth);
    s+=`<line x1="${FX2(sy)}" y1="${FZ(SH_Z)}" x2="${FX2(hy)}" y2="${FZ(hz)}" stroke="${col}" stroke-width="${w}" stroke-linecap="round" opacity="0.9"/>`;
    s+=`<circle cx="${FX2(sy)}" cy="${FZ(SH_Z)}" r="7" fill="#4fb3ff"/><circle cx="${FX2(ey)}" cy="${FZ(ez)}" r="4" fill="#0c1015"/><circle cx="${FX2(hy)}" cy="${FZ(hz)}" r="6" fill="#e6edf3"/>`;
  }
  s+=`<text x="${FX2(0.30)}" y="${fy0+24}" fill="#9aa7b4" font-size="10">robot's left →</text>`;
  // side view
  const sx0=520, SX=x=>sx0+x*sc*0.75, SZ=z=>zc-(z-SH_Z)*sc*0.75;
  s+=`<rect x="${SX(-0.10)}" y="${SZ(1.25)}" width="${0.16*sc*0.75}" height="${0.53*sc*0.75}" fill="#232c37" stroke="#3a4757"/>`;
  s+=`<line x1="${SX(0)}" y1="${SZ(SH_Z)}" x2="${SX(ARM*d[0])}" y2="${SZ(SH_Z+ARM*d[2])}" stroke="${col}" stroke-width="7" stroke-linecap="round"/><circle cx="${SX(0)}" cy="${SZ(SH_Z)}" r="6" fill="#4fb3ff"/>`;
  s+=`<text x="${SX(0.25)}" y="${fy0+24}" fill="#9aa7b4" font-size="10">forward →</text>`;
  svg.innerHTML=s;
  // readouts
  const bound=lb(A.pitch);
  ro('r_lb', bound.toFixed(1)+'°', '');
  ro('r_st', c==='ok'?'allowed':(c==='sim'?'illegal on robot':'out of range'), c==='ok'?'ok':(c==='sim'?'bad':'warn'));
  const gap=2*(SH_Y+ARM*d[1]);
  ro('r_gap', (gap*100).toFixed(1)+' cm', gap<0?'warn':'');
  ro('r_reach', (ARM*d[0]*100).toFixed(0)+' cm / '+((SH_Z+ARM*d[2])*100).toFixed(0)+' cm','');
  $('o_pitch').textContent=A.pitch+'°'; $('o_roll').textContent=A.roll+'°';
  $('pitch').value=A.pitch; $('roll').value=A.roll;
  note();
}
function ro(id,v,cls){ const e=$(id); e.className='ro '+(cls||''); e.querySelector('.v').textContent=v; }
['pitch','roll'].forEach(k=>$(k).addEventListener('input',()=>{A[k]=parseFloat($(k).value); drawA();}));
$('zoom').addEventListener('change',()=>{A.zoom=$('zoom').checked; drawA();});
$('showsim').addEventListener('change',()=>{A.showsim=$('showsim').checked; drawA();});
$('clamp').addEventListener('click',()=>{ A.roll=Math.round(Math.min(120,Math.max(A.roll, lb(A.pitch), -20))); A.pitch=Math.min(180,Math.max(-60,A.pitch)); drawA(); });
document.querySelectorAll('[data-preset]').forEach(b=>b.addEventListener('click',()=>{Object.assign(A,PRESETS[b.dataset.preset]); drawA();}));
(function(){ const svg=$('cl'); let drag=false;
  function setFrom(ev){ const r=svg.getBoundingClientRect(); const x=(ev.clientX-r.left)/r.width*640, y=(ev.clientY-r.top)/r.height*520;
    const g=plotGeom; if(!g) return; let p=(x-g.px0)/(g.px1-g.px0)*240-60; let q=g.ymin+(g.py1-y)/(g.py1-g.py0)*(g.ymax-g.ymin);
    A.pitch=Math.round(Math.min(180,Math.max(-60,p))); A.roll=Math.round(Math.min(120,Math.max(-25,q))); drawA(); }
  svg.addEventListener('pointerdown',ev=>{ const r=svg.getBoundingClientRect(); const y=(ev.clientY-r.top)/r.height*520; if(y>310) return; drag=true; svg.setPointerCapture(ev.pointerId); setFrom(ev); });
  svg.addEventListener('pointermove',ev=>{ if(drag) setFrom(ev); });
  svg.addEventListener('pointerup',()=>drag=false);
})();

/* ---------------- Playground B: YAML -> URDF -> MJCF ---------------- */
const ACTS = { // description/actuators.yaml
  RS04:{peak:120,rated:40,rpm:200,iface:'can'}, RS03:{peak:60,rated:20,rpm:195,iface:'can'},
  RS06:{peak:36,rated:11,rpm:480,iface:'can'}, RS00:{peak:14,rated:5,rpm:315,iface:'can'},
  STS3250:{peak:4.9,rated:1.57,rpm:75,iface:'serial'} };
const BUSES = {'{side}_hip':{type:'can',rate:500,n:3}, '{side}_shank':{type:'can',rate:500,n:3}, '{side}_arm':{type:'can',rate:250,n:5}, 'servo':{type:'serial'}};
const JP = { // from robot.yaml, skeleton.yaml and robot_cfg.py
  knee:{name:'knee',axis:'pitch',lo:0,hi:130,model:'RS04',bus:'{side}_shank',parent:'left_thigh_link',child:'left_shin_link',xyz:[0,0,-0.155],vec:'[0, 1, 0]',kp:150,kd:5,arm:0.03,fr:0.3,cad:{hi:130,why:'beyond 130° the upper ankle motor meets the thigh shell'}},
  hip_pitch:{name:'hip_pitch',axis:'pitch',lo:-30,hi:120,model:'RS04',bus:'{side}_hip',parent:'left_hip_roll_link',child:'left_hip_pitch_link',xyz:[0,-0.025,0],vec:'[0, -1, 0]',kp:150,kd:5,arm:0.03,fr:0.3,cad:{lo:-30,why:'beyond −30° the hip pitch yoke meets the hip roll motor'}},
  shoulder_roll:{name:'shoulder_roll',axis:'roll',lo:-20,hi:120,model:'RS03',bus:'{side}_arm',parent:'left_shoulder_pitch_link',child:'left_shoulder_roll_link',xyz:[0,0.048,-0.025],vec:'[1, 0, 0]',kp:40,kd:2,arm:0.02,fr:0.2,cad:{lo:-20,why:'below −20° the shoulder yaw motor meets the chest shell'}},
  elbow:{name:'elbow',axis:'pitch',lo:0,hi:135,model:'RS06',bus:'{side}_arm',parent:'left_upper_arm_link',child:'left_forearm_link',xyz:[0,0,-0.115],vec:'[0, -1, 0]',kp:30,kd:1.5,arm:0.01,fr:0.1},
  wrist_yaw:{name:'wrist_yaw',axis:'yaw',lo:-90,hi:90,model:'RS00',bus:'{side}_arm',parent:'left_forearm_link',child:'left_wrist_link',xyz:[0,0,-0.125],vec:'[0, 0, -1]',kp:15,kd:0.8,arm:0.005,fr:0.05},
};
JP.broken = {...JP.knee, lo:130, hi:0, model:'STS3250', vec:'[1, 0, 0]', kp:150, eff:150, arm:-0.01};
let B = {...JP.knee, eff:'', mir:'mirror', base:'knee'};
let tab='urdf';
function vecOf(s){ return JSON.parse(s); }
function f6(x){ return Number(x.toPrecision(6)).toString(); }
function fv(v){ return v.map(x=>f6(x===0?0:x)).join(' '); }
function mirrorAxis(a){ return [-a[0]+0, a[1], -a[2]+0]; }
function esc(s){ return s.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;'); }
function hl(xml){ // light syntax colouring
  return esc(xml).replace(/([\w:]+)=("[^"]*")/g,'<span class="attr">$1</span>=<span class="val">$2</span>').replace(/(&lt;\/?)([\w:]+)/g,'$1<span class="tag">$2</span>');
}
function readB(){
  B.name=$('y_name').value.trim(); B.axis=$('y_axis').value; B.lo=parseFloat($('y_lo').value); B.hi=parseFloat($('y_hi').value);
  B.model=$('y_model').value; B.bus=$('y_bus').value; B.vec=$('y_vec').value;
  B.kp=parseFloat($('y_kp').value); B.kd=parseFloat($('y_kd').value); B.arm=parseFloat($('y_arm').value); B.fr=parseFloat($('y_fr').value);
  B.eff=$('y_eff').value; B.mir=$('y_mir').value;
}
function writeB(){
  $('y_name').value=B.name; $('y_axis').value=B.axis; $('y_lo').value=B.lo; $('y_hi').value=B.hi; $('y_model').value=B.model; $('y_bus').value=B.bus;
  $('y_vec').value=B.vec; $('y_kp').value=B.kp; $('y_kd').value=B.kd; $('y_arm').value=B.arm; $('y_fr').value=B.fr; $('y_eff').value=(B.eff===undefined?'':B.eff); $('y_mir').value=B.mir||'mirror';
  $('y_parent').textContent=B.parent; $('y_xyz').textContent='['+B.xyz.join(', ')+']';
}
function drawB(){
  const a=ACTS[B.model];
  const genEff = a?a.peak:NaN, vel = a? a.rpm*2*Math.PI/60 : NaN;
  const effOv = B.eff!=='' && !isNaN(parseFloat(B.eff)) ? parseFloat(B.eff) : null;
  const eff = effOv!==null?effOv:genEff;
  const lo=B.lo*D2R, hi=B.hi*D2R;
  const ax=vecOf(B.vec);
  const rax = B.mir==='mirror'?mirrorAxis(ax):ax.slice();
  const nameL='left_'+B.name, nameR='right_'+B.name;
  const childL=B.child, childR=B.child.replace('left_','right_'), parL=B.parent, parR=B.parent.replace('left_','right_');
  const xyzR=[B.xyz[0],-B.xyz[1]+0,B.xyz[2]];
  const effS=isNaN(eff)?'?':f6(eff), velS=isNaN(vel)?'?':f6(vel);
  const urdf =
`<!-- Generated by tools/build_urdf.py from description/*.yaml. Do not edit. -->
<joint name="${nameL}" type="revolute">
  <parent link="${parL}"/><child link="${childL}"/>
  <origin xyz="${fv(B.xyz)}"/><axis xyz="${fv(ax)}"/>
  <limit lower="${f6(lo)}" upper="${f6(hi)}" effort="${effS}" velocity="${velS}"/>
</joint>
<joint name="${nameR}" type="revolute">
  <parent link="${parR}"/><child link="${childR}"/>
  <origin xyz="${fv(xyzR)}"/><axis xyz="${fv(rax)}"/>
  <limit lower="${f6(lo)}" upper="${f6(hi)}" effort="${effS}" velocity="${velS}"/>
</joint>
<!-- URDF has no armature, gains or friction here: those are added per simulator -->`;
  const mj =
`<!-- What MjSpec.from_string(urdf) + check_policy.py build, written out as MJCF -->
<compiler angle="radian" autolimits="true"/>
<option timestep="0.002" integrator="implicitfast"/>
<body name="${childL}" pos="${fv(B.xyz)}">
  <!-- inertial from the URDF link: mass, pos, fullinertia -->
  <joint name="${nameL}" type="hinge" axis="${fv(ax)}" range="${f6(lo)} ${f6(hi)}"
         actuatorfrcrange="${isNaN(eff)?'?':f6(-eff)} ${effS}" armature="${f6(B.arm)}" frictionloss="${f6(B.fr)}"/>
  <geom type="box" ... />  <!-- collision boxes; contype/conaffinity default 1 -->
</body>
<actuator>
  <!-- a torque motor; the PD and the torque-speed line run in Python each 2 ms step:
       tau = kp (q* - q) - kd qd, kp ${f6(B.kp)}, kd ${f6(B.kd)},
       clipped to ${effS} (1 - qd / ${velS}) -->
  <motor name="${nameL}" joint="${nameL}" ctrlrange="${isNaN(eff)?'?':f6(-eff)} ${effS}"/>
</actuator>
<!-- MuJoCo has no joint velocity limit: the ${velS} rad/s from the URDF is used only in the motor model -->`;
  const sg = {RS04:[400,8,0.03],RS03:[250,5,0.02],RS06:[40,1.5,0.01],RS00:[20,1,0.005],STS3250:[10,0.5,0.002]}[B.model]||[NaN,NaN,NaN];
  const mjpos =
`<!-- check_standing.py: a position servo per joint, stiff holding gains by motor model -->
<joint name="${nameL}" type="hinge" axis="${fv(ax)}" range="${f6(lo)} ${f6(hi)}" armature="${f6(sg[2])}"/>
<actuator>
  <position name="${nameL}" joint="${nameL}" kp="${f6(sg[0])}" kv="${f6(sg[1])}"
            ctrlrange="${f6(lo)} ${f6(hi)}" forcelimited="true" forcerange="${isNaN(eff)?'?':f6(-eff)} ${effS}"/>
</actuator>`;
  const pat = B.name.startsWith('neck')||B.name==='waist_yaw'?B.name:'.*_'+B.name;
  const isaac =
`# sim/isaaclab/marvin_bot/robot_cfg.py (generated from the same table)
GAINS["${pat}"] = (${f6(B.kp)}, ${f6(B.kd)}, ${f6(B.arm)}, ${f6(B.fr)})   # kp, kd, armature, friction
DelayedDCMotorCfg(
    joint_names_expr=["${pat}"],
    saturation_effort=${effS}, actuator_effort_limit=${effS}, joint_effort_limit=${effS},
    actuator_velocity_limit=${velS}, joint_velocity_limit=${velS},
    stiffness=${f6(B.kp)}, damping=${f6(B.kd)}, armature=${f6(B.arm)}, friction=${f6(B.fr)},
    min_delay=0, max_delay=2,   # 0-10 ms at the 5 ms physics step
)`;
  const code={urdf,mjcf:mj,mjpos,isaac}[tab];
  $('code').innerHTML=hl(code);
  // checks
  const C=[];
  const add=(st,tool,msg)=>C.push({st,tool,msg});
  if(!/^[a-z][a-z0-9_]*$/.test(B.name)) add('fail','build_urdf.py',`joint name "${B.name}" is not a plain lower-case identifier; and "joints differ" between robot.yaml and skeleton.yaml`);
  else if(B.name!==JP[B.base==='broken'?'knee':B.base].name) add('fail','build_urdf.py',`joints differ: robot.yaml has ${nameL} but skeleton.yaml still names the joint ${'left_'+JP[B.base==='broken'?'knee':B.base].name}`);
  else add('ok','build_urdf.py','robot.yaml and skeleton.yaml name the same joints');
  if(!(B.lo<B.hi)) add('fail','check_description.py',`range [${B.lo}, ${B.hi}] is not increasing`); else add('ok','check_description.py',`range [${B.lo}, ${B.hi}]° increasing`);
  if(!a) add('fail','check_description.py',`unknown model ${B.model} (not in actuators.yaml)`);
  else {
    const bus=BUSES[B.bus];
    if(a.iface!==bus.type) add('fail','check_description.py',`${B.model} (${a.iface}) cannot run on bus ${B.bus} (${bus.type})`);
    else add('ok','check_description.py',`${B.model} speaks ${a.iface}, bus ${B.bus} is ${bus.type}`);
    if(bus.type==='can'){ const n=bus.n + (B.bus!==JP[B.base==='broken'?'knee':B.base].bus?1:0);
      const load=n*bus.rate*2*160/1e6;
      add(load>0.6?'fail':'ok','check_description.py',`bus ${B.bus.replace('{side}','left')}: ${n} motors × ${bus.rate} Hz × 2 frames × 160 bits = ${(load*100).toFixed(0)}% load (max 60%)`);
    }
  }
  if(effOv!==null && a){
    add('fail','build_urdf.py --check',`URDF effort ${effOv} N·m differs from a fresh build (${genEff} N·m): "marvin_bot.urdf is out of date"`);
    if(effOv>genEff) add('fail','check_robot.py',`effort limit ${effOv} N·m vs the description's ${genEff}: the simulator would let the joint exceed the ${B.model}'s peak by ${((effOv/genEff-1)*100).toFixed(0)}%`);
  } else if(a) add('ok','build_urdf.py --check',`effort = ${B.model} peak ${genEff} N·m, velocity = ${a.rpm} rpm = ${vel.toFixed(2)} rad/s`);
  const want={pitch:1,roll:0,yaw:2}[B.axis]; const along=ax.findIndex(v=>v!==0);
  if(along!==want) add('warn','(no automatic check)',`a ${B.axis} joint should turn about the ${'xyz'[want]} axis, the skeleton gives ${'xyz'[along]}: only visible in renders or as a policy that cannot stand`);
  else if(B.base!=='broken' && B.vec!==JP[B.base].vec && B.name===JP[B.base].name) add('warn','(no automatic check)',`axis sign flipped: + would no longer mean ${B.axis==='pitch'?'flexion':B.axis==='roll'?'abduction':'turning in'}; stance and policies would bend the wrong way`);
  else add('ok','convention',`+ ${B.axis} = ${B.axis==='pitch'?'flexion':B.axis==='roll'?'abduction':'towards the midline'} on both sides`);
  if(B.mir==='copy' && B.axis!=='pitch') add('fail','check_symmetry.py',`right ${B.axis} axis not negated: mirrored poses land up to tens of cm off their mirror image (tolerance 1 µm)`);
  else if(B.mir==='copy') add('ok','check_symmetry.py','pitch axes are unchanged by the mirror, so copying happens to be right here');
  else add('ok','check_symmetry.py',`right axis ${fv(rax)} = mirror of the left`);
  const cad=(JP[B.base==='broken'?'knee':B.base]||{}).cad;
  if(cad){ if((cad.hi!==undefined&&B.hi>cad.hi)||(cad.lo!==undefined&&B.lo<cad.lo)) add('fail','cad.covers --check',cad.why); else add('ok','cad.covers --check','parts clear over the range'); }
  if(B.arm<0||B.fr<0||B.kp<0||B.kd<0) add('fail','MuJoCo / Isaac Lab',`negative armature, friction or gain: MuJoCo refuses to compile a negative armature, and a negative gain pushes the joint away from its target`);
  if(B.kp>0 && a){ const scale=0.25*eff/B.kp; add(scale<0.4?'warn':'ok','setup_protomotions.py',`BeyondMimic action scale 0.25 × ${f6(eff)}/${f6(B.kp)} = ${scale.toFixed(2)} rad${scale<0.4?' (< 0.4: the motion tracker softens kp)':''}`); }
  $('checks').innerHTML=C.map(c=>`<li class="${c.st==='ok'?'':c.st}"><b>${c.st==='ok'?'PASS':c.st==='fail'?'FAIL':'NOTE'}</b><code>${c.tool}</code> ${esc(c.msg)}</li>`).join('');
  const nf=C.filter(c=>c.st==='fail').length;
  ro('r_eff', isNaN(eff)?'?':f6(eff)+' N·m', effOv!==null?'bad':'');
  ro('r_vel', isNaN(vel)?'?':vel.toFixed(2)+' rad/s','');
  ro('r_rng', f6(lo)+' / '+f6(hi), B.lo<B.hi?'':'bad');
  ro('r_n', String(nf), nf?'bad':'ok');
  B.nfail=nf; B.fails=C.filter(c=>c.st==='fail').map(c=>c.tool+': '+c.msg);
  note();
}
document.querySelectorAll('#yaml input,#yaml select').forEach(el=>el.addEventListener('input',()=>{readB(); drawB();}));
document.querySelectorAll('#jpresets [data-j]').forEach(b=>b.addEventListener('click',()=>{ const k=b.dataset.j; B={...JP[k], eff:JP[k].eff===undefined?'':JP[k].eff, mir:'mirror', base:k}; writeB(); drawB(); }));
document.querySelectorAll('[data-tab]').forEach(b=>b.addEventListener('click',()=>{ tab=b.dataset.tab; document.querySelectorAll('[data-tab]').forEach(x=>x.classList.toggle('on',x===b)); drawB(); }));

function note(){
  const parts=[];
  if(A.pitch!==DEFA.pitch||A.roll!==DEFA.roll){
    const c=classify(A.pitch,A.roll);
    parts.push(`Shoulder pose explored: pitch ${A.pitch}°, roll ${A.roll}°. The coupled limit in description/robot.yaml allows roll down to ${lb(A.pitch).toFixed(1)}° at this pitch, so the pose is ${c==='ok'?'allowed':c==='sim'?'ILLEGAL on the robot although the plain −20° limit used in simulation allows it (the controller must clamp roll to the bound; in simulation only the collision boxes stop it)':'outside the joint ranges'}. Gap between the fingertips with both arms mirrored and elbows straight: ${(2*(SH_Y+ARM*armDir(A.pitch,A.roll)[1])*100).toFixed(1)} cm.`);
  }
  const base=JP[B.base]||JP.knee;
  const diffs=[];
  for(const k of ['name','axis','lo','hi','model','bus','vec','kp','kd','arm','fr']) if(B[k]!==base[k]) diffs.push(`${k} ${base[k]} → ${B[k]}`);
  if(B.eff!==''&&B.eff!==undefined) diffs.push(`URDF effort hand-edited to ${B.eff} N·m`);
  if(B.mir==='copy') diffs.push('right-side axis copied instead of mirrored');
  if(diffs.length||B.base!=='knee'){
    parts.push(`Description edit for the ${base.name} joint (starting from the ${B.base==='broken'?'deliberately broken example':'repo values'}): ${diffs.length?diffs.join('; '):'no changes'}. ${B.nfail?`${B.nfail} check(s) would fail: ${B.fails.join(' | ')}.`:'All modelled checks pass (check_description.py, build_urdf.py --check, check_symmetry.py, cad.covers --check).'} Remember the URDF, MuJoCo model and USD are generated: edit description/*.yaml (and robot_cfg.py GAINS for kp, kd, armature, friction), then rerun tools/build_urdf.py and sim/isaaclab/convert_urdf.py.`);
  }
  $('note').textContent = parts.length? 'Marvin Bot (1.44 m, 41 kg, 27-DOF humanoid in simulation), robot description. '+parts.join('\n\n') : 'Change a control above to build a note.';
}
$('copy').addEventListener('click',()=>{ navigator.clipboard.writeText($('note').textContent).then(()=>{ const b=$('copy'); b.textContent='Copied!'; setTimeout(()=>b.textContent='Copy',1200); }); });
writeB(); drawA(); drawB();
