const $ = id => document.getElementById(id);
const D2R = Math.PI/180, s2 = x => Math.sin(x*D2R)**2;
const DT = 0.02, EP = 20;
const MULTS = [0, 0.1, 0.25, 0.5, 1, 2, 4, 10, 30];
const ONE = 4; // index of ×1
// [name, default weight (flat task), group, value function of gait features f]
const TERMS = [
 ['track_lin_vel_xy_exp', 1.0, 'Task', f => Math.exp(-(f.ev**2)/0.0625)],
 ['track_ang_vel_z_exp', 1.5, 'Task', f => Math.exp(-(f.ew**2)/0.25)],
 ['forward_progress', 1.0, 'Task', f => Math.min(1, Math.max(0, 1 - f.ev/0.5))],
 ['termination_penalty', -200, 'Task', null],
 ['feet_air_time', 0.5, 'Gait', f => { const T=f.Tss; const g = T<=0 ? 0 : (T<=0.35 ? T/2 : (0.35*T-0.06125)/T); return f.fss*g; }],
 ['foot_clearance', 1.0, 'Gait', f => (f.fss/2+f.ffl)*0.96*Math.exp(-((f.clr-0.08)**2)/0.001)],
 ['feet_air_time_variance', -1.0, 'Gait', f => f.asym**2/2],
 ['feet_slide', -0.1, 'Gait', f => f.slide*(2*(1-f.fss-f.ffl)+f.fss)],
 ['feet_flat', -2.0, 'Feet', f => (2*(1-f.fss-f.ffl)+f.fss)*s2(f.tiltS)],
 ['swing_feet_level', -2.0, 'Feet', f => (f.fss+2*f.ffl)*s2(f.tiltW)],
 ['pelvis_height', -10, 'Posture', f => Math.max(0, f.dz-0.02)],
 ['pelvis_roll', -10, 'Posture', f => s2(f.roll)],
 ['flat_orientation_l2', -1.0, 'Posture', f => s2(f.roll)+s2(f.pitch)],
 ['feet_width', -5, 'Posture', f => Math.max(0, Math.abs(f.w-0.16)-0.02)],
 ['joint_deviation_hip', -0.3, 'Posture', f => f.dev],
 ['lin_vel_z_l2', -0.2, 'Smoothness', f => f.vz**2],
 ['ang_vel_xy_l2', -0.05, 'Smoothness', f => f.wxy**2],
 ['action_rate_l2', -0.05, 'Smoothness', f => 12*f.da**2],
 ['joint_vel_legs', -1e-3, 'Smoothness', f => 12*f.qd**2],
 ['dof_torques_l2', -2e-5, 'Effort', f => 12*f.tau**2],
 ['torque_limits', -0.2, 'Effort', f => f.clip],
 ['sustained_torque', -2.0, 'Effort', f => 2*Math.max(0, f.k**2-0.8)],
];
// Illustrative gait features. ev speed error m/s, ew yaw-rate error rad/s, dz pelvis below stance m,
// roll / pitch RMS deg, w step width m, Tss single-support phase s, fss fraction single support,
// ffl fraction both feet off, asym L-R timing s, slide m/s, clr swing clearance m, tiltS / tiltW foot tilt deg,
// dev hip/ankle roll+yaw deviation rad (L1 sum), tau leg torque RMS N·m per joint, k knee RMS / rated,
// qd joint speed RMS rad/s, da action change RMS, vz vertical speed RMS m/s, wxy roll/pitch rate RMS rad/s,
// clip torque lost to saturation N·m, p chance of falling per episode, Tf mean time to fall s.
const GAITS = {
 upright:{name:'Upright walk', desc:'flat v7/v8-like: level pelvis, 15 cm steps, ~0.46 s swings', ev:.06,ew:.05,dz:.012,roll:2.1,pitch:2,w:.147,Tss:.46,fss:.72,ffl:0,asym:.01,slide:.02,clr:.07,tiltS:2,tiltW:8,dev:.12,tau:22,k:.6,qd:2.2,da:.06,vz:.08,wxy:.3,clip:0,p:0,Tf:20, knee:15, lift:.07},
 crouch:{name:'Crouched shuffle', desc:'rough v6-like: knees bent, pelvis 10 cm low, short steady steps', ev:.03,ew:.04,dz:.10,roll:1.5,pitch:5,w:.16,Tss:.40,fss:.66,ffl:0,asym:.01,slide:.03,clr:.075,tiltS:2,tiltW:7,dev:.10,tau:24,k:.85,qd:1.6,da:.04,vz:.04,wxy:.2,clip:0,p:0,Tf:20, knee:70, lift:.07},
 waddle:{name:'Waddle', desc:'flat v6-like: rocks 7° over each foot, 21 cm step width, no double support', ev:.04,ew:.06,dz:.03,roll:7,pitch:2,w:.21,Tss:.68,fss:.97,ffl:0,asym:.01,slide:.01,clr:.08,tiltS:3,tiltW:10,dev:.30,tau:22,k:.6,qd:1.8,da:.045,vz:.05,wxy:.45,clip:0,p:0,Tf:20, knee:20, lift:.08},
 toedown:{name:'Toe-down swing', desc:'flat v5-like: heel lifted, foot pitched 30° in swing', ev:.06,ew:.05,dz:.012,roll:2.1,pitch:2,w:.15,Tss:.46,fss:.72,ffl:0,asym:.01,slide:.02,clr:.08,tiltS:2,tiltW:30,dev:.12,tau:21,k:.58,qd:2.0,da:.055,vz:.08,wxy:.3,clip:0,p:0,Tf:20, knee:15, lift:.08},
 hop:{name:'Two-footed hop', desc:'both feet leave the ground 35% of the time; hard landings', ev:.10,ew:.10,dz:0,roll:1,pitch:4,w:.16,Tss:0,fss:0,ffl:.35,asym:0,slide:.05,clr:.06,tiltS:3,tiltW:5,dev:.1,tau:40,k:.9,qd:3.2,da:.10,vz:.40,wxy:.6,clip:1.5,p:.3,Tf:8, knee:30, lift:.06},
 stand:{name:'Stand still', desc:'ignores the command: no steps, almost no effort', ev:.5,ew:0,dz:0,roll:.5,pitch:1,w:.16,Tss:0,fss:0,ffl:0,asym:0,slide:0,clr:0,tiltS:1,tiltW:0,dev:.02,tau:12,k:.25,qd:.1,da:.01,vz:.005,wxy:.05,clip:0,p:0,Tf:20, knee:15, lift:0},
 fall:{name:'Fall over at once', desc:'ends the episode after ~1 s', ev:.3,ew:.3,dz:.2,roll:20,pitch:30,w:.25,Tss:0,fss:0,ffl:0,asym:0,slide:.2,clr:0,tiltS:10,tiltW:0,dev:.4,tau:40,k:.8,qd:3,da:.15,vz:.3,wxy:1.5,clip:2,p:1,Tf:1, knee:60, lift:0},
};
const GKEYS = Object.keys(GAITS);
const DEF = Object.fromEntries(TERMS.map(t => [t[0], ONE]));
const PPO_DEF = {eps:0.2, ratio:1.3, adv:1, gamma:0.99, lam:0.95};
const state = {w:{...DEF}, sel:'upright', ...PPO_DEF};
const PRESETS = {
 marvin: {},
 v6: {feet_air_time:6, pelvis_roll:0, feet_width:0, joint_deviation_hip:2},
 noposture: {pelvis_height:0},
 noswing: {swing_feet_level:0},
 lazy: {dof_torques_l2:8, action_rate_l2:7},
};
const weight = n => { const t = TERMS.find(t => t[0]===n); return t[1]*MULTS[state.w[n]]; };
function fmtW(x){ if(x===0) return '0'; const a=Math.abs(x); return a<0.01 ? x.toExponential(1).replace('e','e') : (+x.toPrecision(3)).toString(); }
function score(f){
  const parts = [];
  let r = 0;
  for(const [n,, , fn] of TERMS){ if(!fn) continue; const c = weight(n)*fn(f); parts.push([n,c]); r += c; }
  const term = weight('termination_penalty')*DT;
  const R = (1-f.p)*EP*r + f.p*(f.Tf*r + term);
  return {r, R, parts, term};
}

// sliders
(function buildSliders(){
  const box = $('wsliders'); let grp='';
  for(const [n,w0,g] of TERMS){
    if(g!==grp){ grp=g; const d=document.createElement('div'); d.className='grp'; d.textContent=g; box.appendChild(d); }
    const d = document.createElement('div'); d.className='wsl';
    d.innerHTML = `<label for="w_${n}"><span>${n}</span><output id="ow_${n}"></output></label><input type="range" id="w_${n}" min="0" max="${MULTS.length-1}" step="1" value="${ONE}" aria-label="${n} weight multiplier">`;
    box.appendChild(d);
    d.querySelector('input').addEventListener('input', e => { state.w[n] = +e.target.value; render(); });
  }
})();
(function buildGaitButtons(){
  const box = $('gaitbtns');
  for(const k of GKEYS){ const b=document.createElement('button'); b.textContent=GAITS[k].name; b.dataset.g=k;
    b.addEventListener('click',()=>{ state.sel=k; render(); }); box.appendChild(b); }
})();

function drawBars(results){
  const svg = $('bars'); const W=640,H=330, L=150, R=20, T=20, B=40;
  const vals = GKEYS.map(k => results[k].R);
  let lo = Math.min(0, ...vals), hi = Math.max(0, ...vals); if(hi-lo<1){hi+=1;}
  const pad=(hi-lo)*0.08; lo-= (lo<0?pad:0); hi+=pad;
  const x = v => L + (v-lo)/(hi-lo)*(W-L-R);
  const best = GKEYS.reduce((a,b)=> results[b].R>results[a].R?b:a);
  const bh = (H-T-B)/GKEYS.length;
  let s = '';
  // grid
  const step = niceStep((hi-lo)/6);
  for(let v=Math.ceil(lo/step)*step; v<=hi; v+=step){ s+=`<line x1="${x(v)}" y1="${T}" x2="${x(v)}" y2="${H-B}" stroke="#2a3340"/><text x="${x(v)}" y="${H-B+16}" fill="#9aa7b4" font-size="11" text-anchor="middle">${+v.toFixed(1)}</text>`; }
  s += `<line x1="${x(0)}" y1="${T}" x2="${x(0)}" y2="${H-B}" stroke="#9aa7b4"/>`;
  s += `<text x="${(L+W-R)/2}" y="${H-6}" fill="#9aa7b4" font-size="12" text-anchor="middle">expected return of a 20 s episode (illustrative features, real term weights)</text>`;
  GKEYS.forEach((k,i)=>{
    const v = results[k].R, y = T+i*bh+4, h = bh-8;
    const col = k===best ? '#ffb454' : (v>=0 ? '#4cc38a' : '#ff6b6b');
    const x0 = Math.min(x(0),x(v)), w = Math.abs(x(v)-x(0));
    const selc = k===state.sel ? ' stroke="#4fb3ff" stroke-width="2"' : '';
    s += `<g style="cursor:pointer" data-g="${k}"><rect x="0" y="${y-2}" width="${W}" height="${h+4}" fill="transparent"/>`;
    s += `<text x="${L-8}" y="${y+h/2+4}" fill="${k===state.sel?'#4fb3ff':'#e6edf3'}" font-size="13" text-anchor="end">${GAITS[k].name}</text>`;
    s += `<rect x="${x0}" y="${y}" width="${Math.max(w,1)}" height="${h}" rx="3" fill="${col}" opacity="${k===best?1:0.75}"${selc}/>`;
    const tx = v>=0 ? x(v)+5 : x(0)+5;
    s += `<text x="${tx}" y="${y+h/2+4}" fill="#e6edf3" font-size="12" text-anchor="start" font-family="ui-monospace,monospace">${v.toFixed(1)}${k===best?' ★':''}</text></g>`;
  });
  svg.innerHTML = s;
  svg.querySelectorAll('g[data-g]').forEach(g=>g.addEventListener('click',()=>{state.sel=g.dataset.g; render();}));
  return best;
}
function niceStep(raw){ const p=Math.pow(10,Math.floor(Math.log10(raw))); const m=raw/p; return (m<1.5?1:m<3.5?2:m<7.5?5:10)*p; }

function drawGlyph(f){
  // front view (left) and side view (right) stick figures, schematic
  const svg=$('glyph'); let s='';
  s+=`<line x1="10" y1="180" x2="290" y2="180" stroke="#2a3340" stroke-width="2"/>`;
  s+=`<text x="75" y="196" fill="#9aa7b4" font-size="11" text-anchor="middle">front</text><text x="215" y="196" fill="#9aa7b4" font-size="11" text-anchor="middle">side</text>`;
  const sc = 165; // px per m
  const thigh=.325*sc, shin=.315*sc, ank=.06*sc;
  // side: knee flexion -> pelvis height
  const kn = f.knee*D2R, hp = kn/2, ap = kn/2;
  const hipH = (thigh*Math.cos(hp)+shin*Math.cos(kn-hp)) + ank;
  // front view
  const cx=75, roll=(f.roll*1.4)*D2R, hw=0.10*sc, fw=f.w*sc/2;
  const pelvY = 180-Math.min(hipH,160)-(f.ffl>0?10:0);
  const lx = cx-hw*Math.cos(roll), ly = pelvY - hw*Math.sin(roll), rx = cx+hw*Math.cos(roll), ry = pelvY + hw*Math.sin(roll);
  const lift = f.lift*sc;
  const lfY = 180-(f.ffl>0?10:0), rfY = 180-lift-(f.ffl>0?10:0);
  s+=`<line x1="${lx}" y1="${ly}" x2="${rx}" y2="${ry}" stroke="#ffb454" stroke-width="5" stroke-linecap="round"/>`;
  s+=`<line x1="${cx}" y1="${pelvY}" x2="${cx+Math.sin(roll)*55}" y2="${pelvY-Math.cos(roll)*55}" stroke="#9aa7b4" stroke-width="4"/>`;
  s+=`<circle cx="${cx+Math.sin(roll)*66}" cy="${pelvY-Math.cos(roll)*66}" r="10" fill="none" stroke="#9aa7b4" stroke-width="3"/>`;
  s+=`<line x1="${lx}" y1="${ly}" x2="${cx-fw}" y2="${lfY}" stroke="#4fb3ff" stroke-width="4"/>`;
  s+=`<line x1="${rx}" y1="${ry}" x2="${cx+fw}" y2="${rfY}" stroke="#4fb3ff" stroke-width="4"/>`;
  s+=`<line x1="${cx-fw-10}" y1="${lfY}" x2="${cx-fw+10}" y2="${lfY}" stroke="#e6edf3" stroke-width="4"/>`;
  s+=`<line x1="${cx+fw-10}" y1="${rfY}" x2="${cx+fw+10}" y2="${rfY}" stroke="#e6edf3" stroke-width="4"/>`;
  // side view: stance leg and swing leg
  const sx=215, hipY=180-hipH-(f.ffl>0?10:0)+ (f.p===1?40:0);
  const knX = sx + thigh*Math.sin(hp), knY = hipY + thigh*Math.cos(hp);
  const anX = knX - shin*Math.sin(kn-hp), anY = knY + shin*Math.cos(kn-hp);
  s+=`<line x1="${sx}" y1="${hipY}" x2="${knX}" y2="${knY}" stroke="#4fb3ff" stroke-width="4"/><line x1="${knX}" y1="${knY}" x2="${anX}" y2="${anY}" stroke="#4fb3ff" stroke-width="4"/>`;
  s+=`<line x1="${anX-12}" y1="${anY+ank}" x2="${anX+40}" y2="${anY+ank}" stroke="#e6edf3" stroke-width="4"/>`;
  // swing leg (behind, lifted), foot tilted by tiltW (toe down)
  if(f.lift>0){
    const swX = sx-25, swY = 180-lift-ank, t=f.tiltW*D2R;
    s+=`<line x1="${sx}" y1="${hipY}" x2="${swX+10}" y2="${(hipY+swY)/2+8}" stroke="#4fb3ff" stroke-opacity=".55" stroke-width="4"/><line x1="${swX+10}" y1="${(hipY+swY)/2+8}" x2="${swX}" y2="${swY}" stroke="#4fb3ff" stroke-opacity=".55" stroke-width="4"/>`;
    s+=`<line x1="${swX-12*Math.cos(t)}" y1="${swY+ank-12*Math.sin(-t)}" x2="${swX+40*Math.cos(t)}" y2="${swY+ank+40*Math.sin(t)}" stroke="#e6edf3" stroke-opacity=".7" stroke-width="4"/>`;
  }
  s+=`<line x1="${sx}" y1="${hipY}" x2="${sx+Math.sin(f.pitch*2*D2R)*60}" y2="${hipY-60}" stroke="#9aa7b4" stroke-width="4"/>`;
  svg.innerHTML=s;
}

function tableFeatures(f){
  const rows = [
    ['Speed error', f.ev.toFixed(2)+' m/s'], ['Pelvis below stance', (f.dz*100).toFixed(1)+' cm'],
    ['Pelvis roll (RMS)', f.roll.toFixed(1)+'°'], ['Step width', (f.w*100).toFixed(1)+' cm'],
    ['Single-support phase', f.Tss.toFixed(2)+' s × '+Math.round(f.fss*100)+'%'], ['Both feet off', Math.round(f.ffl*100)+'%'],
    ['Swing clearance', (f.clr*100).toFixed(1)+' cm'], ['Swing / stance foot tilt', f.tiltW+'° / '+f.tiltS+'°'],
    ['Leg torque RMS', f.tau+' N·m; knee '+Math.round(f.k*100)+'% of rated'], ['Falls', f.p? Math.round(f.p*100)+'% of episodes, at ~'+f.Tf+' s':'none'],
  ];
  $('feat').innerHTML = `<tr><th colspan="2">${f.name} <span class="badge">illustrative</span><div class="small" style="font-weight:400">${f.desc}</div></th></tr>` + rows.map(r=>`<tr><td>${r[0]}</td><td class="num">${r[1]}</td></tr>`).join('');
}
function tableContrib(res){
  const parts = res.parts.filter(p=>Math.abs(p[1])>=0.0005).sort((a,b)=>Math.abs(b[1])-Math.abs(a[1]));
  let h = `<tr><th>Term</th><th class="num">weight</th><th class="num">reward/s</th></tr>`;
  for(const [n,c] of parts){ h += `<tr><td><code>${n}</code></td><td class="num">${fmtW(weight(n))}</td><td class="num ${c>=0?'pos':'neg'}">${c>=0?'+':''}${c.toFixed(3)}</td></tr>`; }
  h += `<tr><th>Total</th><th></th><th class="num">${res.r.toFixed(3)}</th></tr>`;
  const f = GAITS[state.sel];
  if(f.p) h += `<tr><td colspan="3" class="small">Falls in ${Math.round(f.p*100)}% of episodes after ~${f.Tf} s: return loses the remaining time and adds termination ${fmtW(res.term)} per fall.</td></tr>`;
  $('contrib').innerHTML = h;
}

function setRO(id, text, cls){ const e=$(id); e.className='ro'+(cls?' '+cls:''); e.querySelector('.v').textContent=text; }

// PPO plot
function drawPPO(){
  const svg=$('ppo'), W=640,H=260,L=46,R=14,T=14,B=34; const e=state.eps, A=state.adv, rr=state.ratio;
  const xr = r => L + r/2*(W-L-R); const ymax = 2.2*A; const y = v => T + (ymax - v)/(2*ymax)*(H-T-B);
  let s='';
  for(let r=0;r<=2.001;r+=0.25) s+=`<line x1="${xr(r)}" y1="${T}" x2="${xr(r)}" y2="${H-B}" stroke="#2a3340"/><text x="${xr(r)}" y="${H-B+14}" fill="#9aa7b4" font-size="11" text-anchor="middle">${r}</text>`;
  s+=`<rect x="${xr(1-e)}" y="${T}" width="${xr(1+e)-xr(1-e)}" height="${H-T-B}" fill="#4fb3ff" opacity=".08"/>`;
  s+=`<line x1="${L}" y1="${y(0)}" x2="${W-R}" y2="${y(0)}" stroke="#9aa7b4"/>`;
  s+=`<text x="${(L+W-R)/2}" y="${H-4}" fill="#9aa7b4" font-size="12" text-anchor="middle">probability ratio r = π_new / π_old</text>`;
  s+=`<text x="12" y="${T+10}" fill="#9aa7b4" font-size="11">L</text>`;
  const curve=(sign,clip)=>{ let p=''; for(let i=0;i<=200;i++){ const r=i/100; const a=sign*A; const v= clip? Math.min(r*a, Math.min(Math.max(r,1-e),1+e)*a) : r*a; p+=(i?'L':'M')+xr(r).toFixed(1)+','+y(v).toFixed(1); } return p; };
  s+=`<path d="${curve(1,false)}" stroke="#4cc38a" stroke-dasharray="5 4" fill="none" opacity=".6"/><path d="${curve(-1,false)}" stroke="#ff6b6b" stroke-dasharray="5 4" fill="none" opacity=".6"/>`;
  s+=`<path d="${curve(1,true)}" stroke="#4cc38a" stroke-width="3" fill="none"/><path d="${curve(-1,true)}" stroke="#ff6b6b" stroke-width="3" fill="none"/>`;
  const Lp = Math.min(rr*A, Math.min(Math.max(rr,1-e),1+e)*A), Ln = Math.min(-rr*A, -Math.min(Math.max(rr,1-e),1+e)*A);
  s+=`<line x1="${xr(rr)}" y1="${T}" x2="${xr(rr)}" y2="${H-B}" stroke="#ffb454" stroke-dasharray="3 3"/>`;
  s+=`<circle cx="${xr(rr)}" cy="${y(Lp)}" r="6" fill="#4cc38a" stroke="#0c1015" stroke-width="2"/><circle cx="${xr(rr)}" cy="${y(Ln)}" r="6" fill="#ff6b6b" stroke="#0c1015" stroke-width="2"/>`;
  s+=`<text x="${xr(1-e)+3}" y="${T+12}" fill="#4fb3ff" font-size="11">1−ε</text><text x="${xr(1+e)+3}" y="${T+12}" fill="#4fb3ff" font-size="11">1+ε</text>`;
  svg.innerHTML=s;
  const gpos = (rr < 1+e) ? A : 0;           // d/dr of min(rA, clip(r)A), A>0
  const gneg = (rr > 1-e) ? -A : 0;          // A<0
  setRO('r_gpos', gpos? '+'+gpos.toFixed(1)+' (learning)' : '0 (clipped)', gpos?'ok':'warn');
  setRO('r_gneg', gneg? gneg.toFixed(1)+' (learning)' : '0 (clipped)', gneg?'ok':'warn');
  // discount bars
  const d=$('disc'), DW=640, DH=150, N=150, dl=40, dr=30, dt=12, db=36; let t='';
  const bx = k => dl + k/N*(DW-dl-dr), by = v => dt + (1-v)*(DH-dt-db);
  for(let k=0;k<N;k+=25) t+=`<text x="${bx(k)}" y="${DH-db+14}" fill="#9aa7b4" font-size="11" text-anchor="middle">${k} (${(k*DT).toFixed(1)} s)</text>`;
  const g=state.gamma, gl=g*state.lam; const bw=(DW-dl-dr)/N;
  for(let k=0;k<N;k++){ t+=`<rect x="${bx(k)}" y="${by(g**k)}" width="${bw*0.9}" height="${(DH-db)-by(g**k)}" fill="#4fb3ff" opacity=".55"/>`;
    t+=`<rect x="${bx(k)}" y="${by(gl**k)}" width="${bw*0.9}" height="${(DH-db)-by(gl**k)}" fill="#b48cff" opacity=".8"/>`; }
  t+=`<text x="${dl}" y="${DH-2}" fill="#9aa7b4" font-size="11">steps ahead k (policy steps of 20 ms): weight of a reward or TD error k steps ahead</text>`;
  t+=`<text x="4" y="${dt+8}" fill="#9aa7b4" font-size="11">1</text><text x="4" y="${DH-db}" fill="#9aa7b4" font-size="11">0</text>`;
  d.innerHTML=t;
  const h=1/(1-g), hg=1/(1-gl);
  setRO('r_hor', `${h.toFixed(0)} steps = ${(h*DT).toFixed(2)} s`, h*DT<0.5?'warn':'');
  setRO('r_gae', `${hg.toFixed(1)} steps = ${(hg*DT).toFixed(2)} s`);
}

function render(){
  for(const [n] of TERMS){ const o=$('ow_'+n), el=$('w_'+n); el.value=state.w[n]; o.textContent = `×${MULTS[state.w[n]]} = ${fmtW(weight(n))}`; o.classList.toggle('chg', state.w[n]!==ONE); }
  const results = {}; for(const k of GKEYS) results[k] = score(GAITS[k]);
  const best = drawBars(results);
  const sorted = GKEYS.map(k=>results[k].R).sort((a,b)=>b-a);
  setRO('r_win', GAITS[best].name, best==='upright'?'ok':'bad');
  const m = sorted[0]-sorted[1];
  setRO('r_margin', m.toFixed(1)+' ('+(100*m/Math.max(Math.abs(sorted[0]),1e-9)).toFixed(0)+'%)', best==='upright' ? (m<2?'warn':'ok') : 'bad');
  setRO('r_up', results.upright.r.toFixed(3)+' /s');
  setRO('r_sel', results[state.sel].r.toFixed(3)+' /s');
  document.querySelectorAll('#gaitbtns button').forEach(b=>b.classList.toggle('on', b.dataset.g===state.sel));
  drawGlyph(GAITS[state.sel]); tableFeatures(GAITS[state.sel]); tableContrib(results[state.sel]);
  for(const k of ['eps','ratio','adv','gamma','lam']){ $(k).value=state[k]; $('o_'+k).textContent = k==='gamma'? state[k].toFixed(3) : state[k].toFixed(2); }
  drawPPO();
  updateNote(best, results);
}
function updateNote(best, results){
  const ch = TERMS.filter(t=>state.w[t[0]]!==ONE).map(t=>`${t[0]} ${fmtW(t[1])} → ${fmtW(weight(t[0]))}`);
  const pp = []; for(const k of ['eps','gamma','lam']) if(state[k]!==PPO_DEF[k]) pp.push(`${k==='eps'?'clip ε':k==='gamma'?'γ':'λ'} ${PPO_DEF[k]} → ${state[k]}`);
  if(!ch.length && !pp.length){ $('note').textContent='Change a weight or a PPO setting above to build a note.'; return; }
  let s = `Marvin Bot (1.44 m, 41 kg humanoid; walking policy trained with PPO in Isaac Lab, tasks in sim/isaaclab/marvin_bot/tasks/velocity_env_cfg.py). `;
  if(ch.length){
    s += `In a reward-shaping sandbox (illustrative gait features, walking at 0.5 m/s on flat ground) I changed these flat-task weights: ${ch.join('; ')}. `;
    s += best==='upright' ? `The upright walk still wins, by ${(results.upright.R - Math.max(...GKEYS.filter(k=>k!=='upright').map(k=>results[k].R))).toFixed(1)} return units per 20 s episode. `
      : `With these weights the optimiser would prefer "${GAITS[best].name}" over the upright walk (${results[best].R.toFixed(1)} vs ${results.upright.R.toFixed(1)} per 20 s episode): a reward-hacking risk if trained. `;
    s += `Before changing the real config, check the corresponding term in rewards.py, resume from the current checkpoint and verify posture, roll, step width and swing foot tilt in sim/mujoco/check_policy.py.`;
  }
  if(pp.length) s += ` PPO settings explored: ${pp.join(', ')} (agents.py uses clip 0.2, γ 0.99, λ 0.95; γ ${state.gamma} gives a planning horizon of about ${(DT/(1-state.gamma)).toFixed(2)} s at 50 Hz).`;
  $('note').textContent = s;
}
['eps','ratio','adv','gamma','lam'].forEach(k=>$(k).addEventListener('input',e=>{state[k]=parseFloat(e.target.value); render();}));
document.querySelectorAll('[data-preset]').forEach(b=>b.addEventListener('click',()=>{ state.w={...DEF, ...PRESETS[b.dataset.preset]}; render(); }));
$('copy').addEventListener('click',()=>{ navigator.clipboard.writeText($('note').textContent).then(()=>{ const b=$('copy'); b.textContent='Copied!'; setTimeout(()=>b.textContent='Copy',1200); }); });
render();
