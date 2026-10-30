// Spend per phase: the 60 BOM lines mapped to phases at single prices (same mapping as page 21; total $17,531.03)
const SPEND=[0,0,4751.50,4342.50,5053.55,3288.48,95.00];
const PH=[
 {n:'Foundations',lo:1,hi:1,status:'done',
  work:'Repo, requirements, decision log, licences, v0 design: joints, sizes, mass budget, BOM.',
  pass:'Requirements and v0 layout agreed.',
  buys:'Nothing.',
  risks:'Scope: what the robot must do (requirements R1-R65) and whether it fits the mass and cost budgets on paper.',
  study:['01-dof-kinematics','02-balance','03-architecture','07-mass-proportions']},
 {n:'Robot in simulation',lo:3,hi:5,status:'prog',
  work:'Code-CAD of the frame, URDF and masses generated from it; standing, step-up and torque/thermal checks; walking and stair policies in Isaac Lab, cross-checked in MuJoCo.',
  pass:'Walks and climbs 15-18 cm steps in simulation; peak torques ≤70% of rating; continuous torques within rating.',
  buys:'Nothing in parts (a training GPU, not counted in the BOM, or a rented one).',
  risks:'Can this body walk and climb at all with these actuators? Knee heating partly (a stair policy on the old 1.29 m body at 73-75% RMS; flat v8 on the 1.44 m body at most 68%); ankle push-rod loads (item 9); mass and strength on paper.',
  study:['13-description','14-simulation','15-pd-control','16-rl-walking','05-leg-loads-stairs','06-ankle-linkage','08-structure-cad','17-motion-tracking']},
 {n:'Actuator bench',lo:2,hi:3,status:'todo',
  work:'One RS04 and one RS03, CAN adapter, bench supply with regen clamp. Measure torque constant, speed, heating, CAN timing, built-in position control; fit actuator models and feed them back into simulation.',
  pass:'Manual figures confirmed; models match measurements; CAN at the planned rates works on the Orin.',
  buys:'RS04 ×1, RS03 ×1 (single price), 48 V bench supply, regen clamp, USB-CAN adapter, RobStride debug adapter, wired e-stop, hand tools; the Orin, NVMe and CAN transceivers (inference: the pass check needs the Orin).',
  risks:'Datasheet vs reality for torque, speed and heat; RS04 continuous rating in a small mount (item 11); output bearing (2); bus current and regen (3); CAN timing (4, 13); IMU choice (6).',
  study:['04-actuators','10-can-bus','09-power-battery','12-safety','20-sim-to-real','15-pd-control']},
 {n:'One leg',lo:4,hi:6,status:'todo',
  work:'Remaining actuators for one leg; machine and print one leg; test on a jig: trajectory tracking, ankle linkage range, gait-like loads for heat, simple hops.',
  pass:'Leg tracks simulated motion; linkage reaches its range; nothing overheats.',
  buys:'The other 11 RS03 in one 10+ order, two more RS04, a dual-channel CAN adapter, 7075 bar, leg stock and service machining (half), bearings, push rods, rod ends, leg jig, end mills, printing tools, filament.',
  risks:'Machining accuracy (item 8); ankle linkage range and rod loads under real motion (9); leg heating under gait-like loads (11); sim-to-real tracking of one limb.',
  study:['05-leg-loads-stairs','06-ankle-linkage','08-structure-cad','15-pd-control','20-sim-to-real']},
 {n:'Lower body',lo:6,hi:8,status:'todo',
  work:'Second leg, pelvis, battery and power system, e-stop, Orin, IMU. Stand on a gantry, walk on the gantry, then walk free.',
  pass:'Walks at 0.5 m/s, turns, stops, recovers from pushes, handles low steps.',
  buys:'Second leg (three RS04 + the spare), pelvis machining, 39 cells, BMS, charger, power distribution, contactor, primary IMU, gantry, wireless e-stop, padding, spot welder.',
  risks:'Cell supply and pack mass (item 7); the walking policy’s sim-to-real gap on a real balancing robot; e-stop and power chain under load.',
  study:['02-balance','09-power-battery','12-safety','16-rl-walking','20-sim-to-real','11-sensors-compute']},
 {n:'Torso, arms, head',lo:4,hi:6,status:'todo',
  work:'Waist, arms, wrist pitch and hands, neck, camera; whole-body controller; fall recovery.',
  pass:'Walks while carrying objects; gets up after a fall.',
  buys:'RS06 ×5, RS00 ×2, four STS3250 servos, hand kits and converters, depth camera, waist crossed-roller bearing, torso machining, upper-body materials.',
  risks:'Camera driver on JetPack 7.2 (item 5); hand mass, wrist pattern and servo-bus timing (12); arm torques for getting up.',
  study:['18-wbc-vla','19-hands','17-motion-tracking','11-sensors-compute','07-mass-proportions']},
 {n:'Perception and intelligence',lo:12,hi:12,status:'todo',ongoing:true,
  work:'Terrain map from the depth camera for stairs, teleoperation, data collection, GR00T N1.7 fine-tuning; finish the shells.',
  pass:'Climbs real stairs using vision; picks up objects on command.',
  buys:'The last kilogram of carbon-fibre nylon for the shells; a training GPU if you have none (not in the BOM).',
  risks:'GR00T N1.7 closed-loop latency on the Orin while the 50 Hz controller runs (item 10); vision-based stairs.',
  study:['11-sensors-compute','18-wbc-vla','17-motion-tracking','16-rl-walking']},
];
const TITLES={'01-dof-kinematics':'Degrees of freedom','02-balance':'Balance','03-architecture':'System architecture','04-actuators':'Actuators','05-leg-loads-stairs':'Leg loads and stairs','06-ankle-linkage':'Ankle linkage','07-mass-proportions':'Mass budget','08-structure-cad':'Structure and code-CAD','09-power-battery':'Power and battery','10-can-bus':'CAN buses','11-sensors-compute':'Sensors and compute','12-safety':'Safety','13-description':'Robot description','14-simulation':'Simulation','15-pd-control':'PD control','16-rl-walking':'RL for walking','17-motion-tracking':'Motion tracking','18-wbc-vla':'Whole-body control and VLA','19-hands':'Hands','20-sim-to-real':'Sim-to-real','21-bom-costs':'BOM and costs'};
const RISKS=[
 [1,'RobStride direct quote, lead time and import tariffs vs Seeed; price the unquoted BOM lines','2 → 3','Before each phase’s order; settled for the actuators before the phase 3 bulk RS03 order*'],
 [2,'RobStride output-bearing type, backlash and moment ratings','2','Open up and load the bench units; decides the far-side bearing design before the leg is machined*'],
 [3,'RobStride bus current and regen behaviour','2','Bench test with supply and regen clamp; sizes the fuse, clamp and precharge'],
 [4,'Real CAN timing with 3-5 motors per bus on the Orin under JetPack 7.2; on-board CAN pin setup','2','Phase 2 pass check: "CAN at the planned rates works on the Orin"'],
 [5,'Depth camera driver support on JetPack 7.2; camera baseline figure','5','Before buying the camera for the head*'],
 [6,'Primary IMU choice (industrial AHRS or BMI088 on a microcontroller)','2 → 4','Decided after a bench test (decisions log), fitted in phase 4*'],
 [7,'Cell availability (genuine stock) and the built pack’s mass','4','When the battery is built for the lower body*'],
 [8,'Desktop-CNC feeds and accuracy in 6061 (coupon cuts)','3','Coupon cuts before machining the first leg*'],
 [9,'Ankle push-rod loads and rod-end sizing','1 → 3','Simulation with gait loads (phase 1), then linkage range on the leg jig (phase 3)*'],
 [10,'GR00T N1.7 closed-loop latency on the Orin while the 50 Hz controller runs','6','When manipulation moves onto the robot*'],
 [11,'Knee and hip-roll heating on slow stairs (static check: knee 118%, hip roll 105% of rated)','1 → 2 → 3','Stair policy RMS on the 1.44 m model (phase 1), RS04 thermal test in a thigh-sized mount (phase 2), gait-like loads on one leg (phase 3)'],
 [12,'Hands: weigh the kits, confirm the 6 × M3 on Ø42 wrist pattern, servo-bus timing with 8 servos per hand','5','The kits are owned, so this can be done any time; needed before the arms are built*'],
 [13,'CAN interfaces: does a dual-channel adapter carry two 500 Hz buses on one USB link? Find a US price','2','With the CAN timing test (item 4); otherwise 5 single-channel adapters and room for 2 more in the bay'],
];
const DEFAULTS={hours:10, est:0.5, lead:0, show6:true, sel:1};
const state={...DEFAULTS};
const $=id=>document.getElementById(id);
const BASEH=10;                 // assumed part-time hours/week behind the plan's ranges (this guide's assumption)
const P1START=new Date('2026-10-25T00:00:00Z'), TODAY=new Date('2026-10-29T00:00:00Z');
const DAY=86400000;
function weeks(i){ const p=PH[i]; let w=(p.lo+(p.hi-p.lo)*state.est)*(p.ongoing?1:BASEH/state.hours); const lt=(i>=2&&i<=5)?state.lead:0; return {work:w, lead:lt, tot:w+lt}; }
function schedule(){
  // phase 0 done: ended when phase 1 started. Phase 1 started 2026-10-25.
  const out=[]; let t=new Date(P1START.getTime()-7*DAY);
  for(let i=0;i<7;i++){ const w= i===0? {work:1,lead:0,tot:1} : weeks(i);
    const s=new Date(t), e=new Date(t.getTime()+w.tot*7*DAY); out.push({i,s,e,...w}); t=e; }
  return out;
}
function fmtD(d){ return d.toLocaleDateString('en-GB',{day:'numeric',month:'short',year:'numeric',timeZone:'UTC'}); }
function fmtM(d){ return d.toLocaleDateString('en-GB',{month:'short',year:'numeric',timeZone:'UTC'}); }
function svgEl(tag,a,txt){ const e=document.createElementNS('http://www.w3.org/2000/svg',tag); for(const k in a) e.setAttribute(k,a[k]); if(txt!=null) e.textContent=txt; return e; }
const COL={done:'#4cc38a',prog:'#ffb454',todo:'#4fb3ff'};
function money(v){ return '$'+Math.round(v).toLocaleString('en-US'); }

function render(){
  const S=schedule(); const vis=S.filter(x=>state.show6||x.i<6);
  const svg=$('gantt'); svg.innerHTML='';
  const W=Math.max(360,Math.min(700,svg.clientWidth||700)), narrow=W<560, left=narrow?118:190,right=14,top=30,rowH=38,H=top+vis.length*rowH+12; svg.setAttribute('viewBox',`0 0 ${W} ${H}`);
  const t0=S[0].s.getTime(), t1=vis[vis.length-1].e.getTime();
  const sx=t=>left+(W-left-right)*(t-t0)/(t1-t0);
  // month ticks
  const d=new Date(Date.UTC(S[0].s.getUTCFullYear(),S[0].s.getUTCMonth()+1,1)); const span=(t1-t0)/DAY; const stepM= span>900?6: span>450?3: span>200?2:1;
  for(;d.getTime()<t1; d.setUTCMonth(d.getUTCMonth()+stepM)){ const x=sx(d.getTime());
    svg.appendChild(svgEl('line',{x1:x,x2:x,y1:top-6,y2:H-6,stroke:'#2a3340'}));
    svg.appendChild(svgEl('text',{x:x+3,y:top-12,fill:'#9aa7b4','font-size':11},fmtM(d))); }
  vis.forEach((p,k)=>{ const y=top+k*rowH; const g=svgEl('g',{class:'ph','data-i':p.i,tabindex:0,role:'button','aria-label':'Phase '+p.i});
    if(p.i===state.sel) g.appendChild(svgEl('rect',{x:0,y:y-2,width:W,height:rowH-4,fill:'#1d2a38',rx:4}));
    g.appendChild(svgEl('text',{x:8,y:y+rowH/2,fill:p.i===state.sel?'#4fb3ff':'#e6edf3','font-size':13,'font-weight':p.i===state.sel?700:400},`${p.i} ${narrow&&PH[p.i].n.length>14?PH[p.i].n.slice(0,13)+'…':PH[p.i].n}`));
    const x0=sx(p.s.getTime()), xl=sx(p.s.getTime()+p.lead*7*DAY), x1=sx(p.e.getTime());
    if(p.lead>0) g.appendChild(svgEl('rect',{x:x0,y:y+6,width:Math.max(xl-x0,1),height:rowH-16,fill:'#3a4757',rx:3}));
    const bar=svgEl('rect',{class:'bar',x:xl,y:y+6,width:Math.max(x1-xl,2),height:rowH-16,fill:COL[PH[p.i].status],rx:3,'fill-opacity':PH[p.i].ongoing?0.45:1});
    g.appendChild(bar);
    const lbl=(PH[p.i].ongoing?'ongoing':(p.tot<10?p.tot.toFixed(1):Math.round(p.tot))+' wk');
    const tx= x1+4 > W-50 ? xl+4 : x1+4;
    g.appendChild(svgEl('text',{x:tx,y:y+rowH/2,fill: x1+4>W-50?'#0f1216':'#9aa7b4','font-size':11,'font-family':'ui-monospace,monospace'},lbl));
    g.addEventListener('click',()=>{ state.sel=p.i; $('sel').value=p.i; render(); });
    g.addEventListener('keydown',e=>{ if(e.key==='Enter'||e.key===' '){ e.preventDefault(); state.sel=p.i; $('sel').value=p.i; render(); } });
    svg.appendChild(g); });
  const xt=sx(TODAY.getTime());
  svg.appendChild(svgEl('line',{x1:xt,x2:xt,y1:top-6,y2:H-6,stroke:'#ff6b6b','stroke-width':2}));
  svg.appendChild(svgEl('text',{x:xt+3,y:H-4,fill:'#ff6b6b','font-size':10},'today'));
  // readouts
  $('r_p2').querySelector('.v').textContent=fmtD(S[2].s);
  $('r_walk').querySelector('.v').textContent=fmtD(S[4].e);
  $('r_full').querySelector('.v').textContent=fmtD(S[5].e);
  const wk=S.slice(1,6).reduce((s,p)=>s+p.tot,0);
  $('r_wk').querySelector('.v').textContent=wk.toFixed(0)+' wk';
  const spent=SPEND.slice(0,state.sel+1).reduce((a,b)=>a+b,0);
  $('r_spent').querySelector('.v').textContent=money(spent);
  $('r_spent').querySelector('.k').textContent=`Spent by end of phase ${state.sel}`;
  renderDetail(S); renderRisks(); updateNote(S);
}
function renderDetail(S){
  const i=state.sel,p=PH[i],s=S[i];
  const st={done:'<span class="pill done">done</span>',prog:'<span class="pill prog">in progress since 25 Oct 2026</span>',todo:'<span class="pill todo">not started</span>'}[p.status];
  const items=RISKS.filter(r=>r[2].split(/\s*→\s*/).includes(String(i))).map(r=>'#'+r[0]).join(', ')||'none of the 13';
  const study=p.study.map(id=>`<a href=${JSON.stringify("../"+id+"/")}>${id.slice(0,2)} ${TITLES[id]}</a>`).join(' · ');
  const dur = p.ongoing? 'ongoing (drawn as 12 weeks)' : (i===0?'1 week (done)':`plan ${p.lo===p.hi?p.lo:p.lo+'-'+p.hi} weeks part-time → about ${s.work.toFixed(1)} weeks of work at ${state.hours} h/week${s.lead?` + ${s.lead} weeks waiting for parts`:''}`);
  $('detail').innerHTML=`<h3>Phase ${i}: ${p.n} ${st}</h3><dl>
   <dt>Work</dt><dd>${p.work}</dd>
   <dt>Pass check</dt><dd><b>${p.pass}</b></dd>
   <dt>Duration</dt><dd>${dur}; ${fmtD(s.s)} to ${fmtD(s.e)}</dd>
   <dt>Spend (approx.)</dt><dd>${money(SPEND[i])} at single prices${i===3?' ($4,221 with the 10+ RS03 price)':''}${i===2?' ($1,252 without the Orin)':''}. ${p.buys}</dd>
   <dt>Risks retired</dt><dd>${p.risks}<br><span style="color:var(--muted)">Open items settled here: ${items}</span></dd>
   <dt>Study first</dt><dd>${study}</dd></dl>`;
}
let riskFilter=null;
function renderRisks(){
  const f=$('riskfilter'); if(!f.childElementCount){
    const all=document.createElement('button'); all.textContent='All phases'; all.dataset.p=''; f.appendChild(all);
    for(let i=1;i<7;i++){ const b=document.createElement('button'); b.textContent='Phase '+i; b.dataset.p=i; f.appendChild(b); }
    f.addEventListener('click',e=>{ const b=e.target.closest('button'); if(!b) return; riskFilter=b.dataset.p===''?null:+b.dataset.p; if(riskFilter!=null){ state.sel=riskFilter; $('sel').value=riskFilter; } render(); });
  }
  f.querySelectorAll('button').forEach(b=>b.classList.toggle('on', (b.dataset.p===''&&riskFilter==null)||(+b.dataset.p===riskFilter&&riskFilter!=null)));
  const tb=document.querySelector('#risktable tbody'); tb.innerHTML='';
  RISKS.forEach(r=>{ const phases=r[2].split(/\s*→\s*/).map(Number);
    if(riskFilter!=null && !phases.includes(riskFilter)) return;
    const tr=document.createElement('tr'); if(phases.includes(state.sel)) tr.className='hl';
    tr.innerHTML=`<td>${r[0]}</td><td>${r[1]}</td><td class="ph">${r[2]}</td><td>${r[3]}</td>`;
    tr.style.cursor='pointer'; tr.addEventListener('click',()=>{ state.sel=phases[phases.length-1]; $('sel').value=state.sel; render(); });
    tb.appendChild(tr); });
}
function renderTable(){
  const tb=$('phtable'); const S=schedule();
  PH.forEach((p,i)=>{ const tr=document.createElement('tr');
    const st={done:'<span class="pill done">done</span>',prog:'<span class="pill prog">in progress</span>',todo:'<span class="pill todo">not started</span>'}[p.status];
    tr.innerHTML=`<td>${i}</td><td>${p.n}</td><td>${p.pass}</td><td>${p.ongoing?'ongoing':(p.lo===p.hi?p.lo+' week':p.lo+'-'+p.hi+' weeks')}</td><td class="num">${money(SPEND[i])}</td><td>${st}</td>`; tb.appendChild(tr); });
  const tr=document.createElement('tr'); tr.innerHTML=`<th></th><th>Total</th><th></th><th></th><th class="num">${money(SPEND.reduce((a,b)=>a+b,0))}</th><th></th>`; tb.appendChild(tr);
}
function updateNote(S){
  const parts=[];
  if(state.hours!==DEFAULTS.hours) parts.push(`${state.hours} hours a week (the plan's "part-time" times are taken as about ${BASEH} h/week, an assumption)`);
  if(state.est!==DEFAULTS.est) parts.push(state.est===0?'the low end of each phase’s time range':'the high end of each phase’s time range');
  if(state.lead) parts.push(`${state.lead} weeks of parts lead time in each hardware phase (2-5)`);
  if(state.sel!==DEFAULTS.sel) parts.push(`a focus on phase ${state.sel} (${PH[state.sel].n})`);
  if(!parts.length){ $('note').textContent='Change a control above to build a note.'; return; }
  const i=state.sel;
  $('note').textContent=`For Marvin Bot (1.44 m, 41 kg, 27-DOF humanoid; currently in phase 1, robot in simulation, since 2026-10-25), I planned the build with ${parts.join(', ')}.
Projected: actuator bench (phase 2) starts ${fmtD(S[2].s)}; lower body walking (end of phase 4) ${fmtD(S[4].e)}; whole robot (end of phase 5) ${fmtD(S[5].e)}.
Phase ${i} pass check: ${PH[i].pass} Approximate parts spend: ${money(SPEND[i])} (single prices; guide's mapping of bom/bom.xlsx to phases). Risks it retires: ${PH[i].risks}
Before ordering, verify the docs/roadmap.md open items settled in or before this phase. Durations are indicative, from docs/plan.md, scaled by hours; not a forecast.`;
}
// wiring
function bind(id,fmt){ const el=$(id),o=$('o_'+id); el.addEventListener('input',()=>{ state[id]=+el.value; if(o) o.textContent=fmt(state[id]); render(); }); }
bind('hours',v=>v+' h'); bind('lead',v=>v+' wk');
$('est').addEventListener('change',e=>{ state.est=+e.target.value; render(); });
$('show6').addEventListener('change',e=>{ state.show6=e.target.checked; render(); });
PH.forEach((p,i)=>{ const o=document.createElement('option'); o.value=i; o.textContent=`${i} ${p.n}`; $('sel').appendChild(o); });
$('sel').addEventListener('change',e=>{ state.sel=+e.target.value; render(); });
function sync(){ $('hours').value=state.hours; $('o_hours').textContent=state.hours+' h'; $('lead').value=state.lead; $('o_lead').textContent=state.lead+' wk'; $('est').value=String(state.est); $('show6').checked=state.show6; $('sel').value=state.sel; }
const PRESETS={today:{...DEFAULTS},evenings:{...DEFAULTS,hours:5},fulltime:{...DEFAULTS,hours:40},pess:{...DEFAULTS,est:1,lead:3},shipping:{...DEFAULTS,lead:6,sel:2}};
document.querySelectorAll('[data-preset]').forEach(b=>b.addEventListener('click',()=>{ Object.assign(state,PRESETS[b.dataset.preset]); riskFilter=null; sync(); render(); }));
$('copy').addEventListener('click',()=>{ navigator.clipboard.writeText($('note').textContent).then(()=>{ const b=$('copy'); b.textContent='Copied!'; setTimeout(()=>b.textContent='Copy',1200); }); });
sync(); renderTable(); render();
let _rw=0; window.addEventListener('resize',()=>{ clearTimeout(_rw); _rw=setTimeout(render,150); });
