const $ = id => document.getElementById(id);
const D2R = Math.PI/180;
const L1=0.325, L2=0.315, HA=0.060, TOE=0.15, HEEL=0.09;
// ---------- Playground A ----------
const DEFAULTS = {hip:6, knee:15, ankle:9, cloud:true, flat:false};
const state = {...DEFAULTS};
const PRESETS = {
  stance:{hip:6,knee:15,ankle:9,flat:false},
  straight:{hip:0,knee:0,ankle:0,flat:false},
  stair:{hip:52,knee:66,ankle:14,flat:false},
  sit:{hip:90,knee:90,ankle:0,flat:false},
  kick:{hip:-25,knee:100,ankle:-30,flat:false}
};
function fk(h,k,a){
  const t=h*D2R, s=(h-k)*D2R, f=(h-k+a)*D2R;
  const knee=[L1*Math.sin(t), -L1*Math.cos(t)];
  const ank=[knee[0]+L2*Math.sin(s), knee[1]-L2*Math.cos(s)];
  const R=(p)=>[ank[0]+p[0]*Math.cos(f)-p[1]*Math.sin(f), ank[1]+p[0]*Math.sin(f)+p[1]*Math.cos(f)];
  return {knee, ank, sole:R([0,-HA]), toe:R([TOE,-HA]), heel:R([-HEEL,-HA]), f:h-k+a};
}
const SC=300, OX=300, OY=200;
const X=x=>OX+x*SC, Y=z=>OY-z*SC;
let cloudSvg='';
(function(){ const pts=[];
  for(let h=-30;h<=120;h+=3) for(let k=0;k<=130;k+=3){ const p=fk(h,k,0).ank; const ok=Math.abs(k-h)<=40;
    pts.push(`<circle cx="${X(p[0]).toFixed(1)}" cy="${Y(p[1]).toFixed(1)}" r="1.6" fill="${ok?'#4cc38a':'#56606c'}" opacity="${ok?0.55:0.5}"/>`); }
  cloudSvg=pts.join(''); })();
function renderLeg(){
  if(state.flat){ const a=Math.max(-40,Math.min(40,state.knee-state.hip)); state.ankle=a; $('ankle').value=a; $('o_ankle').textContent=a+'°'; }
  const r=fk(state.hip,state.knee,state.ankle);
  let s='';
  // grid
  for(let gx=-0.6;gx<=0.61;gx+=0.2) s+=`<line x1="${X(gx)}" y1="20" x2="${X(gx)}" y2="460" stroke="#1d252f"/>`;
  for(let gz=-0.8;gz<=0.61;gz+=0.2) s+=`<line x1="10" y1="${Y(gz)}" x2="590" y2="${Y(gz)}" stroke="#1d252f"/>`;
  s+=`<text x="${X(0.6)-4}" y="${Y(0)-4}" fill="#6b7785" font-size="11" text-anchor="end">x forward →</text><text x="${X(0)+4}" y="30" fill="#6b7785" font-size="11">z up</text>`;
  // standing floor (stance sole)
  s+=`<line x1="10" y1="${Y(-0.694)}" x2="590" y2="${Y(-0.694)}" stroke="#3a4552" stroke-dasharray="6 5"/><text x="14" y="${Y(-0.694)-5}" fill="#6b7785" font-size="11">floor in the stance (0.694 m below the hip)</text>`;
  if(state.cloud) s+=cloudSvg;
  // pelvis
  s+=`<rect x="${X(-0.07)}" y="${Y(0.09)}" width="${0.14*SC}" height="${0.12*SC}" rx="8" fill="#232c37" stroke="#3a4552"/>`;
  const seg=(a,b,c,w)=>`<line x1="${X(a[0])}" y1="${Y(a[1])}" x2="${X(b[0])}" y2="${Y(b[1])}" stroke="${c}" stroke-width="${w}" stroke-linecap="round"/>`;
  s+=seg([0,0],r.knee,'#4fb3ff',10)+seg(r.knee,r.ank,'#b48cff',9);
  s+=`<polygon points="${[r.ank,r.toe,r.heel].map(p=>X(p[0])+','+Y(p[1])).join(' ')}" fill="#ffb45433" stroke="#ffb454" stroke-width="2"/>`;
  s+=seg(r.heel,r.toe,'#ffb454',5);
  for(const [p,n] of [[[0,0],'hip'],[r.knee,'knee'],[r.ank,'ankle']]) s+=`<circle cx="${X(p[0])}" cy="${Y(p[1])}" r="6" fill="#0c1015" stroke="#e6edf3" stroke-width="2"/><text x="${X(p[0])+10}" y="${Y(p[1])-8}" fill="#e6edf3" font-size="12">${n}</text>`;
  s+=`<circle cx="${X(r.sole[0])}" cy="${Y(r.sole[1])}" r="3" fill="#ffb454"/>`;
  $('legview').innerHTML=s;
  $('r_ank').querySelector('.v').textContent=`${r.ank[0].toFixed(3)}, ${r.ank[1].toFixed(3)} m`;
  $('r_h').querySelector('.v').textContent=`${(-r.sole[1]).toFixed(3)} m`;
  const pr=$('r_pitch'); pr.querySelector('.v').textContent=`${r.f.toFixed(0)}°`; pr.className='ro '+(Math.abs(r.f)<1?'ok':Math.abs(r.f)<10?'warn':'');
  $('r_reach').querySelector('.v').textContent=`${Math.hypot(r.ank[0],r.ank[1]).toFixed(3)} m`;
}
function bind(id){ const el=$(id), out=$('o_'+id);
  el.addEventListener('input',()=>{ state[id]=parseFloat(el.value); if(id==='ankle') {state.flat=false;$('flat').checked=false;} if(out) out.textContent=el.value+'°'; renderAll(); });
  if(out) out.textContent=el.value+'°'; }
['hip','knee','ankle'].forEach(bind);
$('cloud').addEventListener('change',e=>{state.cloud=e.target.checked;renderAll();});
$('flat').addEventListener('change',e=>{state.flat=e.target.checked;renderAll();});
document.querySelectorAll('[data-preset]').forEach(b=>b.addEventListener('click',()=>{ const p=PRESETS[b.dataset.preset]; Object.assign(state,p);
  for(const k of ['hip','knee','ankle']){ $(k).value=p[k]; $('o_'+k).textContent=p[k]+'°'; } $('flat').checked=p.flat; renderAll(); }));

// ---------- Playground B ----------
const BUS_NOTE={hip:'CAN, 500 Hz', shank:'CAN, 500 Hz', arm:'CAN, 250 Hz', servo:'Feetech serial bus, 12 V'};
const J=[]; // name, group, axis, range, actuator, bus, pos (front view x,z)
function addLeg(side,sx){ const S=side[0].toUpperCase()+side.slice(1);
  J.push({n:`${side}_hip_roll`,g:`${S} leg`,ax:'roll (x)',r:'−30° … 45°',act:'RS04',bus:`${side}_hip`,p:[sx*0.1,0.70],d:'First hip joint; motor fixed in the pelvis behind the hip.'});
  J.push({n:`${side}_hip_pitch`,g:`${S} leg`,ax:'pitch (y)',r:'−30° … 120°',act:'RS04',bus:`${side}_hip`,p:[sx*0.1,0.70],d:'Carried by the roll bracket; the thigh hangs from it.'});
  J.push({n:`${side}_hip_yaw`,g:`${S} leg`,ax:'yaw (along thigh)',r:'−90° … 90°',act:'RS03',bus:`${side}_hip`,p:[sx*0.1,0.53],d:'Motor at the top of the thigh; turns the leg about its long axis.'});
  J.push({n:`${side}_knee`,g:`${S} leg`,ax:'pitch (y)',r:'0° … 130°',act:'RS04',bus:`${side}_shank`,p:[sx*0.1,0.375],d:'Motor at the knee. The joint that heats up on slow stairs (page 05).'});
  J.push({n:`${side}_ankle_pitch`,g:`${S} leg`,ax:'pitch (y)',r:'−40° … 40°',act:'2 × RS03 via push rods (shared with ankle roll)',bus:`${side}_shank`,p:[sx*0.1,0.06],d:'Parallel linkage: both calf motors move together for pitch.'});
  J.push({n:`${side}_ankle_roll`,g:`${S} leg`,ax:'roll (x)',r:'−20° … 20°',act:'2 × RS03 via push rods (shared with ankle pitch)',bus:`${side}_shank`,p:[sx*0.1,0.06],d:'Parallel linkage: the two calf motors move in opposite directions for roll.'});
}
function addArm(side,sx){ const S=side[0].toUpperCase()+side.slice(1);
  J.push({n:`${side}_shoulder_pitch`,g:`${S} arm`,ax:'pitch (y)',r:'−60° … 180°',act:'RS03',bus:`${side}_arm`,p:[sx*0.11,1.165],d:'Swings the arm forward and back.'});
  J.push({n:`${side}_shoulder_roll`,g:`${S} arm`,ax:'roll (x)',r:'−20° … 120° (below 0° only with the arm forward)',act:'RS03',bus:`${side}_arm`,p:[sx*0.158,1.14],d:'Coupled limit: its minimum depends on shoulder pitch.'});
  J.push({n:`${side}_shoulder_yaw`,g:`${S} arm`,ax:'yaw (along upper arm)',r:'−150° … 150°',act:'RS06',bus:`${side}_arm`,p:[sx*0.158,1.055],d:''});
  J.push({n:`${side}_elbow`,g:`${S} arm`,ax:'pitch (y)',r:'0° … 135°',act:'RS06',bus:`${side}_arm`,p:[sx*0.158,0.94],d:''});
  J.push({n:`${side}_wrist_yaw`,g:`${S} arm`,ax:'yaw (along forearm)',r:'−90° … 90°',act:'RS00',bus:`${side}_arm`,p:[sx*0.158,0.815],d:''});
  J.push({n:`${side}_wrist_pitch`,g:`${S} arm`,ax:'pitch (y)',r:'−60° … 60°',act:'STS3250 servo',bus:'servo',p:[sx*0.158,0.752],d:'Carries the bought four-finger hand (its 8 finger servos are not counted in the 27).'});
}
addLeg('left',1); addLeg('right',-1);
J.push({n:'waist_yaw',g:'Torso and head',ax:'yaw (z)',r:'−150° … 150°',act:'RS03',bus:'right_arm',p:[0,0.815],d:'Shares the right arm bus (5 + 1 motors, 48% load in the repo’s estimate).'});
J.push({n:'neck_yaw',g:'Torso and head',ax:'yaw (z)',r:'−60° … 60°',act:'STS3250 servo',bus:'servo',p:[0,1.225],d:''});
J.push({n:'neck_pitch',g:'Torso and head',ax:'pitch (y)',r:'−20° … 40° (+ = look down)',act:'STS3250 servo',bus:'servo',p:[0,1.26],d:''});
addArm('left',1); addArm('right',-1);
const tstate={sel:'left_knee', locked:new Set()};
const TP={all:[], legs:J.filter(j=>!/(hip|knee|ankle)/.test(j.n)).map(j=>j.n), noyaw:['left_hip_yaw','right_hip_yaw'], fixed:J.filter(j=>/(hip|knee|ankle)/.test(j.n)).map(j=>j.n)};
let tpName='all';
function renderTree(){
  const groups={}; J.forEach(j=>(groups[j.g]=groups[j.g]||[]).push(j));
  let h='';
  for(const g of ['Left leg','Right leg','Torso and head','Left arm','Right arm']){
    h+=`<div><div class="node">pelvis → ${g.toLowerCase()}</div><ul>`;
    // render as nested chain
    let open=0;
    groups[g].forEach((j,i)=>{ if(g==='Torso and head' && i===1){ h+='<li><span class="node">torso → neck</span><ul>'; open++; }
      h+=`<li><button class="btn jbtn ${tstate.sel===j.n?'sel':''} ${tstate.locked.has(j.n)?'locked':''}" data-j="${j.n}">${j.n.replace(/^(left|right)_/,'')}</button>`;
      if(g!=='Torso and head' && i<groups[g].length-1){ h+='<ul>'; open++; } else h+='</li>'; });
    h+='</li></ul>'.repeat(open)+'</ul></div>';
  }
  $('tree').innerHTML=h;
  $('tree').querySelectorAll('[data-j]').forEach(b=>b.addEventListener('click',e=>{ const n=b.dataset.j; if(e.shiftKey){ toggleLock(n);} tstate.sel=n; renderTree(); }));
  const j=J.find(x=>x.n===tstate.sel);
  $('jinfo').innerHTML=`<h5>${j.n}</h5><div>Axis: <b>${j.ax}</b></div><div>Range: <b>${j.r}</b></div><div>Actuator: <b>${j.act}</b></div><div>Bus: <b>${j.bus}</b> (${BUS_NOTE[j.bus.replace(/^(left|right)_/,'')]||'CAN'})</div>${j.d?`<div class="hint" style="margin-top:4px">${j.d}</div>`:''}${tstate.locked.has(j.n)?'<div style="color:var(--bad)">Locked in this what-if.</div>':''}`;
  const n=J.length-tstate.locked.size;
  $('dofcount').textContent=n;
  const legOK=s=>J.filter(j=>j.n.startsWith(s+'_')&&/(hip|knee|ankle)/.test(j.n)).every(j=>!tstate.locked.has(j.n));
  const fixed=tpName==='fixed';
  $('dofsplit').innerHTML=`of 27 body joints${fixed?'':`<br>+ 6 floating base = <b>${n+6}</b> DOF in all`}<br>+ 16 finger servos in the two hands = ${n+(fixed?0:6)+16}<br>Left foot posable in 6-D: <b style="color:${legOK('left')?'var(--good)':'var(--bad)'}">${legOK('left')?'yes':'no'}</b> · right: <b style="color:${legOK('right')?'var(--good)':'var(--bad)'}">${legOK('right')?'yes':'no'}</b>`;
  // body figure
  const bx=x=>200+x*230, bz=z=>290-z*200;
  let s='';
  const line=(a,b)=>`<line x1="${bx(a[0])}" y1="${bz(a[1])}" x2="${bx(b[0])}" y2="${bz(b[1])}" stroke="#3a4552" stroke-width="5" stroke-linecap="round"/>`;
  for(const sx of [1,-1]){ s+=line([sx*0.1,0.70],[sx*0.1,0.06])+line([sx*0.1,0.06],[sx*0.1,0.0])+line([sx*0.158,1.165],[sx*0.158,0.75])+line([sx*0.158,0.75],[sx*0.158,0.57]); }
  s+=line([-0.1,0.70],[0.1,0.70])+line([0,0.70],[0,1.225])+line([-0.158,1.165],[0.158,1.165]);
  s+=`<rect x="${bx(-0.08)}" y="${bz(1.43)}" width="${0.16*230}" height="${0.17*200}" rx="12" fill="#232c37" stroke="#3a4552"/>`;
  s+=`<text x="10" y="18" fill="#6b7785" font-size="11">front view: the robot's left is on your right</text>`;
  // joints: offset duplicates
  const seen={};
  J.forEach(j=>{ const key=j.p.join(','); const k=seen[key]=(seen[key]||0)+1; const dx=(k-1)*9*(j.p[0]>=0?1:-1);
    const sel=tstate.sel===j.n, lk=tstate.locked.has(j.n);
    s+=`<circle data-j="${j.n}" cx="${bx(j.p[0])+dx}" cy="${bz(j.p[1])}" r="${sel?7:5}" fill="${lk?'#ff6b6b':sel?'#ffb454':'#4fb3ff'}" stroke="#0c1015" style="cursor:pointer"><title>${j.n}</title></circle>`; });
  $('body').innerHTML=s;
  $('body').querySelectorAll('[data-j]').forEach(c=>c.addEventListener('click',e=>{ if(e.shiftKey) toggleLock(c.dataset.j); tstate.sel=c.dataset.j; renderTree(); }));
  updateNote();
}
function toggleLock(n){ tstate.locked.has(n)?tstate.locked.delete(n):tstate.locked.add(n); tpName='custom'; }
$('lockbtn').addEventListener('click',()=>{ toggleLock(tstate.sel); renderTree(); });
document.querySelectorAll('[data-tp]').forEach(b=>b.addEventListener('click',()=>{ tpName=b.dataset.tp; tstate.locked=new Set(TP[tpName]); renderTree(); }));

// ---------- Playground C ----------
const hstate={hr:0,hpch:0,hy:0};
const HP={zero:{hr:0,hpch:0,hy:0},step:{hr:5,hpch:35,hy:25},sit:{hr:0,hpch:90,hy:0},splay:{hr:30,hpch:10,hy:-20}};
const Rx=a=>{const c=Math.cos(a),s=Math.sin(a);return [[1,0,0],[0,c,-s],[0,s,c]];};
const Ry=a=>{const c=Math.cos(a),s=Math.sin(a);return [[c,0,s],[0,1,0],[-s,0,c]];};
const Rz=a=>{const c=Math.cos(a),s=Math.sin(a);return [[c,-s,0],[s,c,0],[0,0,1]];};
const mm=(A,B)=>A.map((r,i)=>B[0].map((_,j)=>r.reduce((t,_,k)=>t+A[i][k]*B[k][j],0)));
const mv=(A,v)=>A.map(r=>r[0]*v[0]+r[1]*v[1]+r[2]*v[2]);
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const dot=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
// joint definitions (left leg): rotation matrix and local axis, with the page's sign convention
const JD={roll:{R:a=>Rx(a),ax:[1,0,0],c:'#ff6b6b'}, pitch:{R:a=>Ry(-a),ax:[0,1,0],c:'#4cc38a'}, yaw:{R:a=>Rz(-a),ax:[0,0,1],c:'#4fb3ff'}};
const ORDERS=[['roll','pitch','yaw','Roll, pitch, yaw','Marvin Bot'],['pitch','roll','yaw','Pitch, roll, yaw','Marvin Bot until 2026-10-26'],['yaw','roll','pitch','Yaw, roll, pitch','yaw first']];
function proj(v){ // isometric-ish: x forward to lower-left, y left to the right, z up
  return [ -v[0]*0.62 + v[1]*0.95 , -v[2] + v[0]*0.32 + v[1]*0.12 ]; }
function renderHip(){
  const ang={roll:hstate.hr*D2R,pitch:hstate.hpch*D2R,yaw:hstate.hy*D2R};
  let html='';
  ORDERS.forEach(o=>{
    let R=[[1,0,0],[0,1,0],[0,0,1]]; const axes=[];
    o.slice(0,3).forEach(n=>{ axes.push({n,v:mv(R,JD[n].ax),c:JD[n].c}); R=mm(R,JD[n].R(ang[n])); });
    const leg=mv(R,[0,0,-1]), toe=mv(R,[1,0,0]);
    const det=Math.abs(dot(axes[0].v,cross(axes[1].v,axes[2].v)));
    const S=90, cx=130, cy=70;
    const P=v=>{const p=proj(v);return [cx+p[0]*S, cy+p[1]*S];};
    let s=`<svg viewBox="0 0 260 230" role="img" aria-label="${o[3]} hip in perspective">`;
    // ground ref and pelvis
    const g=[[-0.6,-0.6,-1.15],[0.6,-0.6,-1.15],[0.6,0.6,-1.15],[-0.6,0.6,-1.15]].map(P);
    s+=`<polygon points="${g.map(p=>p.join(',')).join(' ')}" fill="#121820" stroke="#24303c"/>`;
    s+=`<polygon points="${[[0.25,-0.5,0.12],[0.25,0.1,0.12],[-0.25,0.1,0.12],[-0.25,-0.5,0.12]].map(P).map(p=>p.join(',')).join(' ')}" fill="#232c37" stroke="#3a4552"/>`;
    const O=P([0,0,0]), F=P(leg.map(x=>x*1.0));
    s+=`<line x1="${O[0]}" y1="${O[1]}" x2="${F[0]}" y2="${F[1]}" stroke="#cbd5df" stroke-width="7" stroke-linecap="round"/>`;
    const T=P(leg.map((x,i)=>x+toe[i]*0.3));
    s+=`<line x1="${F[0]}" y1="${F[1]}" x2="${T[0]}" y2="${T[1]}" stroke="#ffb454" stroke-width="5" stroke-linecap="round"/>`;
    axes.forEach((a,i)=>{ const A=P(a.v.map(x=>x*0.45)), B=P(a.v.map(x=>-x*0.45));
      s+=`<line x1="${B[0]}" y1="${B[1]}" x2="${A[0]}" y2="${A[1]}" stroke="${a.c}" stroke-width="${4-i}" opacity=".95"/><circle cx="${A[0]}" cy="${A[1]}" r="3.5" fill="${a.c}"/>`; });
    s+=`<circle cx="${O[0]}" cy="${O[1]}" r="5" fill="#0c1015" stroke="#e6edf3" stroke-width="2"/></svg>`;
    const cls=det<0.15?'var(--bad)':det<0.5?'var(--warn)':'var(--good)';
    html+=`<figure>${s}<figcaption><b>${o[3]}</b> (${o[4]})<br>axis spread <b style="color:${cls}">${det.toFixed(2)}</b>${det<0.15?' · gimbal lock':''}<br>sole ${['x','y','z'].map((k,i)=>(leg[i]*0.70).toFixed(2)).join(', ')} m</figcaption></figure>`;
  });
  $('three').innerHTML=html;
}
[['hr'],['hpch'],['hy']].forEach(([id])=>{ const el=$(id); el.addEventListener('input',()=>{hstate[id]=+el.value;$('o_'+id).textContent=el.value+'°';renderAll();}); $('o_'+id).textContent=el.value+'°'; });
document.querySelectorAll('[data-hp]').forEach(b=>b.addEventListener('click',()=>{ Object.assign(hstate,HP[b.dataset.hp]); for(const k in hstate){ $(k).value=hstate[k]; $('o_'+k).textContent=hstate[k]+'°'; } renderAll(); }));

// ---------- note ----------
function updateNote(){
  const parts=[];
  const r=fk(state.hip,state.knee,state.ankle);
  if(state.hip!==DEFAULTS.hip||state.knee!==DEFAULTS.knee||state.ankle!==DEFAULTS.ankle)
    parts.push(`In the planar leg I set hip pitch ${state.hip}°, knee ${state.knee}°, ankle pitch ${state.ankle}° (stance is 6/15/9°): the ankle sits ${r.ank[0].toFixed(3)} m ahead of and ${(-r.ank[1]).toFixed(3)} m below the hip axis, the foot is pitched ${r.f.toFixed(0)}° (toe up +), and the hip is ${(-r.sole[1]).toFixed(3)} m above the sole point.${Math.abs(state.knee-state.hip)>40?' A flat foot is impossible here: it would need ankle pitch '+(state.knee-state.hip)+'°, outside ±40°.':''}`);
  if(tstate.locked.size) parts.push(`In the joint tree I locked ${[...tstate.locked].join(', ')}, leaving ${J.length-tstate.locked.size} of 27 body DOF.`+([...tstate.locked].some(n=>/(hip|knee|ankle)/.test(n))?' At least one foot can then no longer be posed in all 6 directions (3 position, 3 orientation).':''));
  if(hstate.hr||hstate.hpch||hstate.hy){
    const sing=Math.abs(Math.abs(hstate.hpch)-90)<15;
    parts.push(`For the hip-order comparison I used roll ${hstate.hr}°, pitch ${hstate.hpch}°, yaw ${hstate.hy}°.${sing?' Near 90° hip pitch, Marvin Bot\'s roll-pitch-yaw hip is close to gimbal lock (roll and yaw axes parallel), so a controller or IK solver must avoid needing thigh rotation there.':''}`);
  }
  $('note').textContent = parts.length ? `Marvin Bot (1.44 m, 41 kg, 27-DOF humanoid; legs 0.325 m thigh, 0.315 m shin, 0.060 m ankle; hip order roll-pitch-yaw; ranges from description/robot.yaml).\n`+parts.join('\n')+`\nQuestion to resolve: does any planned task (stairs, sitting, getting up) need a pose outside these ranges or near the hip singularity?`
    : 'Change a control above to build a note.';
}
$('copy').addEventListener('click',()=>{ navigator.clipboard.writeText($('note').textContent).then(()=>{ const b=$('copy'); b.textContent='Copied!'; setTimeout(()=>b.textContent='Copy',1200); }); });
function renderAll(){ renderLeg(); renderHip(); updateNote(); }
renderTree(); renderAll();
