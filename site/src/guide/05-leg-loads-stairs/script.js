const $ = id => document.getElementById(id);
const D2R=Math.PI/180, G=9.81, L1=0.325, L2=0.315, HA=0.06, TOE=0.15, HEEL=0.09, DXC=0.011;
// lumped leg masses from the URDF (kg) and their position along the link (fraction from the upper joint)
const M_PL=1.51, F_PL=0.52, M_TH=2.0, F_TH=0.95, M_SH=2.78, F_SH=0.54, M_FT=0.64;
const RS04={rated:40,peak:120,rpm:200,inertia:0.03};
const ANK={rated:32,peak:96}; // two RS03 through the linkage at its worst pose (tools/ankle_linkage.py; page 06). Neutral pose: 39.6 / 118.8.
const DEF={mode:'manual',qh:24.3,qk:35.3,h:0.18,prog:1,hb:0.665,xc:-0.062,mass:40.35,share:1,stage:false,rate:40, duty:0.6,tp:54,tb:0,tsw:5,tc:1.2};
const S={...DEF};
const PRE={today:{mode:'manual',qh:24.3,qk:35.3,h:0.18,mass:40.35,share:1,stage:false,rate:40},
  start:{mode:'auto',h:0.18,prog:0,hb:0.665,xc:-0.20,share:0.5,mass:40.35,stage:false,rate:40},
  mid:{mode:'auto',h:0.18,prog:0.5,hb:0.665,xc:-0.03,share:1,mass:40.35,stage:false,rate:40},
  stance:{mode:'manual',qh:6,qk:15,h:0.18,share:1,mass:40.35,stage:false,rate:40},
  stage:{mode:'manual',qh:24.3,qk:35.3,h:0.18,mass:40.35,share:1,stage:true,rate:40}};
const RPRE={hold:{duty:1,tp:47,tb:47,tsw:0},policy:{duty:0.6,tp:54,tb:0,tsw:5},flat:{duty:0.6,tp:49,tb:0,tsw:5},slow:{duty:0.8,tp:60,tb:30,tsw:5}};
function knee(){ const N=S.stage?1.5:1; return {rated:S.rate*N, peak:RS04.peak*N, w:RS04.rpm*2*Math.PI/60/N, I:RS04.inertia*N*N}; }
function solve(){
  let qh,qk,hip,kn,reach=true;
  const A=[0,HA];
  if(S.mode==='manual'){ qh=S.qh; qk=S.qk; const ps=(qh-qk)*D2R;
    kn=[-L2*Math.sin(ps), HA+L2*Math.cos(ps)]; hip=[kn[0]-L1*Math.sin(qh*D2R), kn[1]+L1*Math.cos(qh*D2R)];
  } else {
    const hz=S.hb-S.h*(1-S.prog); hip=[S.xc-DXC, hz];
    let dx=hip[0], dz=hip[1]-HA, d=Math.hypot(dx,dz);
    if(d>L1+L2-1e-4){ reach=false; const k=(L1+L2-1e-4)/d; dx*=k; dz*=k; d=L1+L2-1e-4; hip=[dx,dz+HA]; }
    if(d<Math.abs(L1-L2)+0.02){ d=Math.abs(L1-L2)+0.02; }
    const beta=Math.atan2(dx,dz), alpha=Math.acos(Math.max(-1,Math.min(1,(L2*L2+d*d-L1*L1)/(2*L2*d))));
    qk=180-Math.acos(Math.max(-1,Math.min(1,(L1*L1+L2*L2-d*d)/(2*L1*L2))))/D2R;
    const sh=beta+alpha; kn=[L2*Math.sin(sh), HA+L2*Math.cos(sh)];
    qh=qk-sh/D2R;
  }
  const qa=qk-qh;
  const xcom = S.mode==='manual' ? hip[0]+DXC : S.xc;
  const xp=Math.max(-HEEL,Math.min(TOE,xcom));
  const F=S.share*S.mass*G;
  // segment positions (x)
  const lerp=(a,b,f)=>a+(b-a)*f;
  const xPL=lerp(hip[0],kn[0],F_PL), xTH=lerp(hip[0],kn[0],F_TH), xSH=lerp(kn[0],0,F_SH), xFT=0;
  const tq=(xj,segs)=>F*(xp-xj)-segs.reduce((s,[m,x])=>s+m*G*(x-xj),0);
  const tH=tq(hip[0],[[M_PL,xPL],[M_TH,xTH],[M_SH,xSH],[M_FT,xFT]]);
  const tK=tq(kn[0],[[M_SH,xSH],[M_FT,xFT]]);
  const tA=tq(0,[]);
  return {qh,qk,qa,hip,kn,A,xcom,xp,F,tH,tK,tA,reach};
}
const SC=500, OX=330, OY=420; const X=x=>OX+x*SC, Y=z=>OY-z*SC;
function renderLeg(r){
  const h=S.h; let s='';
  const edge=-HEEL-0.01;
  // floor and stair
  s+=`<rect x="0" y="${Y(-h)}" width="600" height="${Math.max(0,520-Y(-h))}" fill="#151b22"/>`;
  s+=`<rect x="${X(edge)}" y="${Y(0)}" width="${600-X(edge)}" height="${Y(-h)-Y(0)}" fill="#202933" stroke="#3a4552"/>`;
  s+=`<line x1="0" y1="${Y(-h)}" x2="${X(edge)}" y2="${Y(-h)}" stroke="#3a4552" stroke-width="2"/>`;
  if(h>0) s+=`<text x="${X(edge)-6}" y="${(Y(0)+Y(-h))/2+4}" fill="#9aa7b4" font-size="11" text-anchor="end">riser ${(h*100).toFixed(1)} cm</text>`;
  // rear leg (illustrative): on the floor if it reaches, otherwise swinging with its toe 3 cm over the edge
  { const floorA=[edge-0.04-TOE, -h+HA], swingA=[edge-TOE, 0.03+HA];
    let ra=floorA, lab='rear foot on the floor';
    if(Math.hypot(floorA[0]-r.hip[0],floorA[1]-r.hip[1])>=L1+L2-0.002){ ra=swingA; lab='rear foot swinging over the edge'; }
    const dx=ra[0]-r.hip[0], dz=ra[1]-r.hip[1]; let d=Math.hypot(dx,dz); const ok=d<L1+L2-0.002;
    let kp, ae;
    if(ok){ const a=Math.acos((L1*L1+d*d-L2*L2)/(2*L1*d)); const base=Math.atan2(dx,-dz); const th=base+a; kp=[r.hip[0]+L1*Math.sin(th), r.hip[1]-L1*Math.cos(th)]; ae=ra; }
    else { kp=[r.hip[0]+dx/d*L1, r.hip[1]+dz/d*L1]; ae=[r.hip[0]+dx/d*(L1+L2), r.hip[1]+dz/d*(L1+L2)]; lab='rear foot lifted'; }
    s+=`<polyline points="${[r.hip,kp,ae].map(p=>X(p[0])+','+Y(p[1])).join(' ')}" fill="none" stroke="#3f4a57" stroke-width="7" stroke-linecap="round" stroke-linejoin="round"/>`;
    s+=`<line x1="${X(ae[0])}" y1="${Y(ae[1])}" x2="${X(ae[0])}" y2="${Y(ae[1]-HA)}" stroke="#3f4a57" stroke-width="4"/>`;
    s+=`<line x1="${X(ae[0]-HEEL)}" y1="${Y(ae[1]-HA)}" x2="${X(ae[0]+TOE)}" y2="${Y(ae[1]-HA)}" stroke="#3f4a57" stroke-width="6" stroke-linecap="round"/>`;
    s+=`<text x="${X(ae[0])}" y="${Y(ae[1]-HA)+16}" fill="#6b7785" font-size="10" text-anchor="middle">${lab}</text>`; }
  // stance foot
  s+=`<rect x="${X(-HEEL)}" y="${Y(0.012)}" width="${0.24*SC}" height="${0.012*SC}" fill="#ffb45455" stroke="#ffb454"/>`;
  s+=`<polygon points="${X(0)},${Y(HA)} ${X(-HEEL)},${Y(0.012)} ${X(TOE)},${Y(0.012)}" fill="#ffb45422" stroke="#ffb454"/>`;
  // force line
  const top=Math.max(r.hip[1]+0.15,0.75);
  s+=`<line x1="${X(r.xp)}" y1="${Y(0)}" x2="${X(r.xp)}" y2="${Y(top)}" stroke="#b48cff" stroke-dasharray="6 4" opacity=".7"/>`;
  s+=`<defs><marker id="ah" markerWidth="8" markerHeight="8" refX="4" refY="4" orient="auto"><path d="M0,0 L8,4 L0,8 z" fill="#b48cff"/></marker></defs>`;
  s+=`<line x1="${X(r.xp)}" y1="${Y(-0.09)}" x2="${X(r.xp)}" y2="${Y(-0.01)}" stroke="#b48cff" stroke-width="4" marker-end="url(#ah)"/>`;
  s+=`<text x="${X(r.xp)+6}" y="${Y(-0.07)}" fill="#b48cff" font-size="11">${r.F.toFixed(0)} N</text>`;
  // lever arms
  for(const [p,c] of [[r.hip,'#4fb3ff'],[r.kn,'#ffb454'],[r.A,'#4cc38a']]) s+=`<line x1="${X(p[0])}" y1="${Y(p[1])}" x2="${X(r.xp)}" y2="${Y(p[1])}" stroke="${c}" stroke-dasharray="2 3" stroke-width="2"/>`;
  // leg
  s+=`<polyline points="${[r.hip,r.kn,r.A].map(p=>X(p[0])+','+Y(p[1])).join(' ')}" fill="none" stroke="#cbd5df" stroke-width="10" stroke-linecap="round" stroke-linejoin="round"/>`;
  // body
  s+=`<rect x="${X(r.hip[0]-0.07)}" y="${Y(r.hip[1]+0.10)}" width="${0.14*SC}" height="${0.14*SC}" rx="10" fill="#232c37" stroke="#3a4552"/>`;
  s+=`<rect x="${X(r.hip[0]+DXC-0.08)}" y="${Y(r.hip[1]+0.52)}" width="${0.16*SC}" height="${0.40*SC}" rx="12" fill="#1b232c" stroke="#3a4552" opacity=".8"/>`;
  s+=`<circle cx="${X(r.xcom)}" cy="${Y(r.hip[1]+0.018)}" r="9" fill="#4fb3ff" stroke="#0c1015" stroke-width="2"/><text x="${X(r.xcom)+12}" y="${Y(r.hip[1]+0.018)-8}" fill="#4fb3ff" font-size="11">CoM</text>`;
  for(const [p,n] of [[r.hip,'hip'],[r.kn,'knee'],[r.A,'ankle']]) s+=`<circle cx="${X(p[0])}" cy="${Y(p[1])}" r="6" fill="#0c1015" stroke="#e6edf3" stroke-width="2"/><text x="${X(p[0])+9}" y="${Y(p[1])+15}" fill="#e6edf3" font-size="11">${n}</text>`;
  s+=`<text x="8" y="16" fill="#6b7785" font-size="11">side view · forward → · sole on the stair at z = 0</text>`;
  $('legview').innerHTML=s;
}
function cls(p){ return p<0.7?'var(--good)':p<=1.0?'var(--warn)':'var(--bad)'; }
function renderBars(r){
  const K=knee();
  const rows=[['Hip pitch',Math.abs(r.tH),S.rate,RS04.peak,'RS04'],['Knee',Math.abs(r.tK),K.rated,K.peak,S.stage?'RS04 + 1.5:1':'RS04'],['Ankle pitch',Math.abs(r.tA),ANK.rated,ANK.peak,'2 × RS03, worst pose']];
  const MAX=2.0; // bar spans 0..200% of continuous
  $('bars').innerHTML=rows.map(([n,t,rt,pk,a])=>{ const p=t/rt, w=Math.min(1,p/MAX)*100, p70=Math.min(100,0.7*pk/rt/MAX*100);
    return `<div class="bar"><div><b>${n}</b><div class="bv">${a}</div></div><div class="track"><div class="fill" style="width:${w}%;background:${cls(p)}"></div><div class="mark" style="left:${50}%"></div><div class="mark p70" style="left:${p70}%"></div></div><div class="bv"><b style="color:${cls(p)}">${t.toFixed(1)} N·m</b> · ${(p*100).toFixed(0)}% cont · ${(t/pk*100).toFixed(0)}% peak</div></div>`; }).join('')
    + `<div class="hint" style="font-size:12px;color:var(--muted)">Bar length: 0 to 200% of the continuous rating (white line = 100%). Violet line: 70% of peak.</div>`;
}
function setRO(id,txt,c){ const e=$(id); e.querySelector('.v').innerHTML=txt; e.className='ro '+(c||''); }
function render(){
  document.querySelectorAll('.m-man').forEach(e=>e.style.display=S.mode==='manual'?'':'none');
  document.querySelectorAll('.m-auto').forEach(e=>e.style.display=S.mode==='auto'?'':'none');
  if(S.mode==='manual') $('h').closest('.ctl').style.display='';
  const r=solve(); window._last=r;
  renderLeg(r); renderBars(r);
  const lim=(v,a,b)=>v<a-0.05||v>b+0.05;
  const bad=lim(r.qh,-30,120)||lim(r.qk,0,130)||lim(r.qa,-40,40);
  setRO('r_ang',`${r.qh.toFixed(0)}° / ${r.qk.toFixed(0)}° / ${r.qa.toFixed(0)}°`, bad?'bad':'');
  if(bad) $('r_ang').querySelector('.k').textContent='Hip / knee / ankle: OUT OF RANGE'; else $('r_ang').querySelector('.k').textContent='Hip / knee / ankle pitch';
  setRO('r_lev',`${(Math.abs(r.xp-r.kn[0])*100).toFixed(1)} cm`);
  const inside=r.xcom>=-HEEL+0.02&&r.xcom<=TOE-0.02;
  setRO('r_bal', inside?'over the sole':(S.share<1?'off this sole (shared)':'outside: tips'), inside?'ok':S.share<1?'warn':'bad');
  const K=knee(); setRO('r_kstage',`${K.w.toFixed(1)} rad/s · ${K.I.toFixed(3)} kg·m²`);
  renderRMS(); updateNote();
}
function renderRMS(){
  const K=knee(), N=400; let s2=0, pk=0; const pts=[];
  for(let i=0;i<N;i++){ const u=(i+0.5)/N; let t;
    if(u<S.duty) t=S.tb+(S.tp-S.tb)*Math.sin(Math.PI*u/S.duty); else t=S.tsw;
    s2+=t*t; pk=Math.max(pk,Math.abs(t)); pts.push([u,t]); }
  const rms=Math.sqrt(s2/N);
  const W=600,H=300, x0=50,x1=585,y0=20,y1=260, ymax=Math.max(130,K.peak*0.7+10,S.tp+10);
  const X=u=>x0+u*(x1-x0), Yv=v=>y1-v/ymax*(y1-y0);
  let s=`<rect x="${x0}" y="${y0}" width="${x1-x0}" height="${y1-y0}" fill="#0f141a" stroke="#24303c"/>`;
  for(let v=0;v<=ymax;v+=20) s+=`<line x1="${x0}" x2="${x1}" y1="${Yv(v)}" y2="${Yv(v)}" stroke="#1d2630"/><text x="${x0-6}" y="${Yv(v)+4}" fill="#6b7785" font-size="10" text-anchor="end">${v}</text>`;
  s+=`<rect x="${X(0)}" y="${y0}" width="${X(S.duty)-X(0)}" height="${y1-y0}" fill="#4fb3ff0d"/>`;
  s+=`<text x="${X(S.duty/2)}" y="${y0+14}" fill="#6b7785" font-size="11" text-anchor="middle">stance</text>${S.duty<0.97?`<text x="${X((1+S.duty)/2)}" y="${y0+14}" fill="#6b7785" font-size="11" text-anchor="middle">swing</text>`:''}`;
  s+=`<path d="M${X(0)} ${Yv(0)} ${pts.map(p=>'L'+X(p[0]).toFixed(1)+' '+Yv(p[1]).toFixed(1)).join(' ')} L${X(1)} ${Yv(0)} Z" fill="#4fb3ff22" stroke="#4fb3ff" stroke-width="2"/>`;
  const hl=(v,c,lab,d)=>`<line x1="${x0}" x2="${x1}" y1="${Yv(v)}" y2="${Yv(v)}" stroke="${c}" stroke-width="2" ${d?'stroke-dasharray="6 4"':''}/><text x="${x1-4}" y="${Yv(v)-5}" fill="${c}" font-size="11" text-anchor="end">${lab}</text>`;
  s+=hl(K.rated,'#e6edf3',`continuous ${K.rated.toFixed(0)} N·m`,true)+hl(0.7*K.peak,'#b48cff',`70% of peak ${(0.7*K.peak).toFixed(0)} N·m`,true)+hl(rms,'#ffb454',`RMS ${rms.toFixed(1)} N·m`,false);
  s+=`<text x="${(x0+x1)/2}" y="${H-12}" fill="#6b7785" font-size="11" text-anchor="middle">time over one step: 0 to ${S.tc.toFixed(2)} s</text><text x="14" y="${(y0+y1)/2}" fill="#6b7785" font-size="11" transform="rotate(-90 14 ${(y0+y1)/2})" text-anchor="middle">knee torque (N·m)</text>`;
  $('rmsview').innerHTML=s;
  const p=rms/K.rated;
  setRO('r_rms',`${rms.toFixed(1)} N·m`);
  setRO('r_rmsp',`${(p*100).toFixed(0)}%`, p<0.7?'ok':p<=1?'warn':'bad');
  setRO('r_heat',`${(p*p).toFixed(2)} ×`, p<=1?'ok':'bad');
  setRO('r_pk',`${(pk/K.peak*100).toFixed(0)}%`, pk/K.peak<=0.7?'ok':'bad');
  window._rms=rms;
}
const FMT={qh:v=>v.toFixed(1)+'°',qk:v=>v.toFixed(1)+'°',h:v=>(v*100).toFixed(1)+' cm',prog:v=>(v*100).toFixed(0)+'%',hb:v=>v.toFixed(3)+' m',xc:v=>v.toFixed(3)+' m',mass:v=>v.toFixed(2)+' kg',share:v=>(v*100).toFixed(0)+'%',rate:v=>v.toFixed(1)+' N·m',duty:v=>(v*100).toFixed(0)+'%',tp:v=>v.toFixed(1)+' N·m',tb:v=>v.toFixed(1)+' N·m',tsw:v=>v.toFixed(1)+' N·m',tc:v=>v.toFixed(2)+' s'};
function sync(){ for(const k in FMT){ $(k).value=S[k]; $('o_'+k).textContent=FMT[k](S[k]); } $('stage').checked=S.stage; document.querySelectorAll('#mode button').forEach(b=>b.classList.toggle('on',b.dataset.m===S.mode)); }
for(const k in FMT){ $(k).addEventListener('input',()=>{ S[k]=parseFloat($(k).value); $('o_'+k).textContent=FMT[k](S[k]); render(); }); }
$('stage').addEventListener('change',e=>{S.stage=e.target.checked; render();});
document.querySelectorAll('#mode button').forEach(b=>b.addEventListener('click',()=>{
  if(b.dataset.m==='auto' && S.mode==='manual'){ const r=solve(); S.xc=+r.xcom.toFixed(3); S.prog=1; S.hb=+Math.min(0.70,Math.max(0.55,r.hip[1]+S.h*0)).toFixed(3); }
  if(b.dataset.m==='manual' && S.mode==='auto'){ const r=solve(); S.qh=+r.qh.toFixed(1); S.qk=+r.qk.toFixed(1); }
  S.mode=b.dataset.m; sync(); render(); }));
document.querySelectorAll('[data-preset]').forEach(b=>b.addEventListener('click',()=>{ Object.assign(S,PRE[b.dataset.preset]); sync(); render(); }));
document.querySelectorAll('[data-rp]').forEach(b=>b.addEventListener('click',()=>{ Object.assign(S,RPRE[b.dataset.rp]); sync(); render(); }));
function updateNote(){
  const r=window._last, K=knee(), parts=[];
  const legKeys=['mode','qh','qk','h','prog','hb','xc','mass','share','stage','rate'];
  if(legKeys.some(k=>S[k]!==DEF[k])){
    parts.push(`Stance-leg statics (2-D sagittal model, foot flat, pelvis upright): hip ${r.qh.toFixed(0)}°, knee ${r.qk.toFixed(0)}°, ankle ${r.qa.toFixed(0)}°${S.mode==='auto'?` (auto pose on a ${(S.h*100).toFixed(1)} cm riser at ${(S.prog*100).toFixed(0)}% of the step-up, walking hip height ${S.hb} m)`:''}; mass ${S.mass} kg with ${(S.share*100).toFixed(0)}% on this leg; CoM ${r.xcom.toFixed(3)} m from the ankle. Torques: hip pitch ${Math.abs(r.tH).toFixed(1)} N·m, knee ${Math.abs(r.tK).toFixed(1)} N·m (${(Math.abs(r.tK)/K.rated*100).toFixed(0)}% of a ${K.rated.toFixed(0)} N·m continuous rating, ${(Math.abs(r.tK)/K.peak*100).toFixed(0)}% of peak), ankle pitch ${Math.abs(r.tA).toFixed(1)} N·m.${S.stage?' Includes the parked 1.5:1 knee stage (60/180 N·m, ~14 rad/s, 2.25 × reflected inertia).':''}${S.rate!==40?` Assumes the RS04 holds only ${S.rate} N·m continuous in the leg.`:''}`);
  }
  const rk=['duty','tp','tb','tsw','tc'];
  if(rk.some(k=>S[k]!==DEF[k])) parts.push(`Duty-cycle check: stance ${(S.duty*100).toFixed(0)}% of the cycle, knee torque rising from ${S.tb} to ${S.tp} N·m (half-sine), ${S.tsw} N·m in swing: RMS ${window._rms.toFixed(1)} N·m = ${(window._rms/K.rated*100).toFixed(0)}% of continuous; peak ${(S.tp/K.peak*100).toFixed(0)}% of peak (rule: ≤ 70%).`);
  $('note').textContent = parts.length ? `Marvin Bot knee on stairs (repo: static 18 cm check 47 N·m = 118% of the RS04's 40 N·m continuous; stair policy rough v5 measured 73–75% RMS on the 1.29 m model, rough v9 passes 14/15 stair runs on the 1.44 m model; 1.5:1 stage parked).\n`+parts.join('\n')+`\nNext step: measure the knee RMS of the stair policy (rough v9, 14/15 stair runs passed on the 1.44 m model) on the new body, and compare it and the bench-measured continuous rating before ordering leg actuators.` : 'Change a control above to build a note.';
}
$('copy').addEventListener('click',()=>{ navigator.clipboard.writeText($('note').textContent).then(()=>{ const b=$('copy'); b.textContent='Copied!'; setTimeout(()=>b.textContent='Copy',1200); }); });
sync(); render();
