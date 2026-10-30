const $ = id => document.getElementById(id);
const H = 0.002;      // bench sample period (500 Hz)
const SUB = 0.0005;   // integration step
const VS = 0.02;      // Coulomb smoothing speed, rad/s
const J_ARM = 0.012;  // test arm inertia
const TPK = 120, W0 = 200*2*Math.PI/60; // RS04 peak torque N·m and no-load speed rad/s
// DC-motor limit as in Isaac Lab's DCMotor: torque falls linearly to zero at no-load speed
const sat = (tau, om) => Math.min(Math.min(TPK, TPK*(1-om/W0)), Math.max(Math.max(-TPK, TPK*(-1-om/W0)), tau));
const PKEYS = ['J','b','tc','d','Kt'];
const PINFO = {J:{lab:'Inertia J',u:'kg·m²',min:0.01,max:0.1,f:v=>v.toFixed(3)}, b:{lab:'Viscous b',u:'N·m·s/rad',min:0,max:1,f:v=>v.toFixed(2)}, tc:{lab:'Coulomb τc',u:'N·m',min:0,max:1.5,f:v=>v.toFixed(2)}, d:{lab:'Delay d',u:'ms',min:0,max:20,f:v=>v.toFixed(1)}, Kt:{lab:'Torque const. Kt',u:'N·m/A',min:1.5,max:2.5,f:v=>v.toFixed(2)}};
const TODAY = {J:0.042, b:0, tc:0.3, d:5, Kt:2.1};
const DEFAULTS = {...TODAY, test:'both', noise:0.05, kp:40, kd:2};
const state = {...DEFAULTS, reveal:false, seed:7};
let TRUE = {J:0.050, b:0.35, tc:0.55, d:4, Kt:1.95};
let FIT = null;
const DR = {J:[0.042,0.042], b:[0,0], tc:[0.3,0.3], d:[0,10], Kt:[2.1,2.1]};
const DR_TODAY = JSON.parse(JSON.stringify(DR));

function rng(seed){ let a=seed>>>0; return ()=>{ a|=0; a=a+0x6D2B79F5|0; let t=Math.imul(a^a>>>15,1|a); t=t+Math.imul(t^t>>>7,61|t)^t; return ((t^t>>>14)>>>0)/4294967296; }; }
function gauss(r){ let u=0,v=0; while(u===0)u=r(); v=r(); return Math.sqrt(-2*Math.log(u))*Math.cos(2*Math.PI*v); }

// current command for a test, A
function current(test, t){
  if(t<0) return 0;
  const step = tt => { if(tt<0.1||tt>=2) return 0; const vals=[3,0,-3,1.5,-1.5]; return vals[Math.min(4,Math.floor((tt-0.1)/0.38))]; };
  const chirp = tt => { if(tt<0||tt>=2) return 0; const f0=0.5,f1=8,T=2; return 3*Math.sin(2*Math.PI*(f0*tt+(f1-f0)*tt*tt/(2*T))); };
  if(test==='step') return step(t);
  if(test==='chirp') return chirp(t);
  return t<2 ? step(t) : chirp(t-2);
}
const duration = test => test==='both' ? 4 : 2;

function simulateBench(p, test){
  const T=duration(test), n=Math.round(T/H), ratio=Math.round(H/SUB);
  const w=new Float64Array(n); let om=0, t=0;
  for(let k=0;k<n;k++){
    w[k]=om;
    for(let s=0;s<ratio;s++){ const tau=sat(p.Kt*current(test,t-p.d/1000),om) - p.b*om - p.tc*Math.tanh(om/VS); om += SUB*tau/p.J; t+=SUB; }
  }
  return w;
}
let DATA = null;
function makeData(){
  const r=rng(state.seed*1000+Math.round(state.noise*1000)+(state.test==='step'?1:state.test==='chirp'?2:3));
  const w=simulateBench(TRUE,state.test); const m=new Float64Array(w.length);
  for(let k=0;k<w.length;k++) m[k]=w[k]+state.noise*gauss(r);
  // locked-output test: 0..10 A in 1 A steps, load cell noise 0.3 N·m
  const lk=[]; for(let i=-10;i<=10;i+=1){ if(i===0) continue; lk.push([i, TRUE.Kt*i + 0.3*gauss(r)]); }
  DATA={w:m, lk, test:state.test};
}
function rms(a,b){ let s=0; for(let k=0;k<a.length;k++){ const e=a[k]-b[k]; s+=e*e; } return Math.sqrt(s/a.length); }

function fit(){
  const lk=DATA.lk; let sx=0, sy=0; for(const [i,tq] of lk){ sx+=i*i; sy+=i*tq; } const Kt=sy/sx;
  const w=DATA.w, n=w.length, test=DATA.test;
  // smoothed speed (5-point mean) and acceleration (central difference over ±2 samples of smoothed speed)
  const ws=new Float64Array(n); for(let k=2;k<n-2;k++) ws[k]=(w[k-2]+w[k-1]+w[k]+w[k+1]+w[k+2])/5;
  let best=null;
  for(let d=0; d<=20.0001; d+=0.5){
    const M=[[0,0,0],[0,0,0],[0,0,0]], v=[0,0,0]; let yy=0;
    for(let k=4;k<n-4;k++){
      const a=(ws[k+2]-ws[k-2])/(4*H), om=ws[k], sg=Math.tanh(om/VS);
      // sample k is recorded at the start of interval k; use the mean command over the surrounding sample
      const t=k*H, y=Kt*current(test,t-d/1000);
      const row=[a,om,sg];
      for(let i=0;i<3;i++){ v[i]+=row[i]*y; for(let j=0;j<3;j++) M[i][j]+=row[i]*row[j]; } yy+=y*y;
    }
    const x=solve3(M,v); if(!x) continue;
    let res=yy; for(let i=0;i<3;i++){ res-=2*x[i]*v[i]; for(let j=0;j<3;j++) res+=x[i]*M[i][j]*x[j]; }
    if(!best || res<best.res) best={res, d, J:x[0], b:x[1], tc:x[2]};
  }
  const clamp=(v,k)=>Math.min(PINFO[k].max,Math.max(PINFO[k].min,v));
  FIT={J:+clamp(best.J,'J').toFixed(3), b:+clamp(best.b,'b').toFixed(2), tc:+clamp(best.tc,'tc').toFixed(2), d:best.d, Kt:+clamp(Kt,'Kt').toFixed(2)};
  return FIT;
}
function solve3(A,b){
  const m=A.map((r,i)=>[...r,b[i]]);
  for(let c=0;c<3;c++){ let p=c; for(let r=c+1;r<3;r++) if(Math.abs(m[r][c])>Math.abs(m[p][c])) p=r; if(Math.abs(m[p][c])<1e-12) return null; [m[c],m[p]]=[m[p],m[c]];
    for(let r=0;r<3;r++){ if(r===c) continue; const f=m[r][c]/m[c][c]; for(let k=c;k<4;k++) m[r][k]-=f*m[c][k]; } }
  return [m[0][3]/m[0][0], m[1][3]/m[1][1], m[2][3]/m[2][2]];
}

// deploy: PD tracking with model-based feed-forward, run on a plant
const TD=3, CT=0.002;
const qref = t => 0.3*Math.sin(2*Math.PI*1.2*t) + 0.15*Math.sin(2*Math.PI*2.7*t);
const dqref = t => 0.3*2*Math.PI*1.2*Math.cos(2*Math.PI*1.2*t) + 0.15*2*Math.PI*2.7*Math.cos(2*Math.PI*2.7*t);
const ddqref = t => -0.3*(2*Math.PI*1.2)**2*Math.sin(2*Math.PI*1.2*t) - 0.15*(2*Math.PI*2.7)**2*Math.sin(2*Math.PI*2.7*t);
function deploy(model, plant, kp, kd){
  const n=Math.round(TD/SUB), every=Math.round(CT/SUB); const q=[], cmds=[]; let x=0, om=0, peak=0;
  const dsteps=Math.round(plant.d/1000/SUB);
  let cur=0;
  for(let s=0;s<n;s++){
    const t=s*SUB;
    if(s%every===0){
      const ff = model.J*ddqref(t) + model.b*dqref(t) + model.tc*Math.tanh(dqref(t)/VS);
      const tau = ff + kp*(qref(t)-x) + kd*(dqref(t)-om);
      cmds.push([s, tau/model.Kt]);
    }
    // apply the newest command issued at least dsteps ago
    while(cmds.length>1 && cmds[1][0] <= s-dsteps) cmds.shift();
    cur = (cmds[0][0] <= s-dsteps) ? cmds[0][1] : 0;
    peak=Math.max(peak,Math.abs(cur));
    const tq = sat(plant.Kt*cur,om) - plant.b*om - plant.tc*Math.tanh(om/VS);
    om += SUB*tq/plant.J; x += SUB*om;
    if(s%every===0) q.push(x);
    if(!isFinite(x)||Math.abs(x)>50){ while(q.length<n/every) q.push(NaN); break; }
  }
  let e=0, c=0; q.forEach((v,k)=>{ const t=k*CT; if(t>0.5){ e+=(v-qref(t))**2; c++; } });
  return {q, err: Math.sqrt(e/c), peak};
}
const model = () => ({J:state.J, b:state.b, tc:state.tc, d:state.d, Kt:state.Kt});

// drawing helpers
function plotFrame(W,Hh,L,R,T,B,xmin,xmax,ymin,ymax,xl,yl,xticks){
  const X=v=>L+(v-xmin)/(xmax-xmin)*(W-L-R), Y=v=>T+(ymax-v)/(ymax-ymin)*(Hh-T-B);
  let s=''; for(const xv of xticks) s+=`<line x1="${X(xv)}" y1="${T}" x2="${X(xv)}" y2="${Hh-B}" stroke="#2a3340"/><text x="${X(xv)}" y="${Hh-B+13}" fill="#9aa7b4" font-size="10" text-anchor="middle">${xv}</text>`;
  s+=`<line x1="${L}" y1="${Y(0)}" x2="${W-R}" y2="${Y(0)}" stroke="#3a4552"/>`;
  s+=`<text x="${(L+W-R)/2}" y="${Hh-2}" fill="#9aa7b4" font-size="11" text-anchor="middle">${xl}</text><text x="4" y="${T+8}" fill="#9aa7b4" font-size="11">${yl}</text>`;
  return {X,Y,s};
}
const path=(xs,ys,X,Y)=>{ let p='', on=false; for(let k=0;k<xs.length;k++){ const y=ys[k]; if(!isFinite(y)){on=false;continue;} p+=(on?'L':'M')+X(xs[k]).toFixed(1)+','+Y(Math.max(-1e3,Math.min(1e3,y))).toFixed(1); on=true; } return p; };

let LAST={};
function render(){
  for(const k of [...PKEYS,'noise','kp','kd']){ $(k).value=state[k]; }
  $('o_J').textContent=state.J.toFixed(3)+' kg·m²'; $('o_b').textContent=state.b.toFixed(2)+' N·m·s/rad'; $('o_tc').textContent=state.tc.toFixed(2)+' N·m';
  $('o_d').textContent=state.d.toFixed(1)+' ms'; $('o_Kt').textContent=state.Kt.toFixed(2)+' N·m/A'; $('o_noise').textContent=state.noise.toFixed(2)+' rad/s';
  $('o_kp').textContent=state.kp+' N·m/rad'; $('o_kd').textContent=state.kd.toFixed(1)+' N·m·s/rad';
  $('test').value=state.test; $('reveal').checked=state.reveal;
  if(!DATA || DATA.test!==state.test || DATA.noise!==state.noise || DATA.seed!==state.seed){ makeData(); DATA.noise=state.noise; DATA.seed=state.seed; }
  const m=model(); const w=simulateBench(m,state.test); const n=w.length; const ts=Array.from({length:n},(_,k)=>k*H);
  // bench plot
  let ymax=0; for(const v of DATA.w) ymax=Math.max(ymax,Math.abs(v)); ymax=Math.max(1,Math.ceil(ymax*1.15));
  const T=duration(state.test);
  const fr=plotFrame(640,200,40,10,10,22,0,T,-ymax,ymax,'','ω rad/s',T===4?[0,1,2,3,4]:[0,0.5,1,1.5,2]);
  let s='<defs><clipPath id="cpb"><rect x="40" y="10" width="590" height="168"/></clipPath></defs>'+fr.s;
  let dots=''; for(let k=0;k<n;k+=2) dots+=`M${fr.X(ts[k]).toFixed(1)},${fr.Y(DATA.w[k]).toFixed(1)}h1`;
  s+=`<path d="${dots}" stroke="#9aa7b4" stroke-width="2" opacity=".55"/>`;
  s+=`<path clip-path="url(#cpb)" d="${path(ts,w,fr.X,fr.Y)}" stroke="#4fb3ff" stroke-width="2" fill="none"/>`;
  if(state.reveal){ const wt=simulateBench(TRUE,state.test); s+=`<path d="${path(ts,wt,fr.X,fr.Y)}" stroke="#ffb454" stroke-width="1.2" stroke-dasharray="4 3" fill="none"/>`; }
  // residual strip
  const res=Array.from(w,(v,k)=>DATA.w[k]-v); let rmax=0; for(const v of res) rmax=Math.max(rmax,Math.abs(v)); rmax=Math.max(rmax,4*state.noise,0.2);
  const fr2=plotFrame(640,300,40,10,214,22,0,T,-rmax,rmax,'time (s)','',[]);
  s+=`<g>${fr2.s.replace(/<text x="320[^>]*><\/text>/,'')}</g><path d="${path(ts,res,fr2.X,fr2.Y)}" stroke="#ff6b6b" stroke-width="1" fill="none"/>`;
  s+=`<text x="40" y="208" fill="#9aa7b4" font-size="10">residual (measured − model), ±${rmax.toFixed(2)} rad/s</text>`;
  $('sid').innerHTML=s;
  const R=rms(DATA.w,w);
  const floor=state.noise;
  ro('r_res', R.toFixed(3)+' rad/s', R<2.5*floor+0.02?'ok':R<6*floor+0.1?'warn':'bad');
  ro('r_fl', floor.toFixed(3)+' rad/s');
  // locked test
  let kt=''; const Xk=v=>40+(v+10)/20*590, Yk=v=>60-v/25*50;
  kt+=`<line x1="40" y1="60" x2="630" y2="60" stroke="#3a4552"/><line x1="${Xk(0)}" y1="8" x2="${Xk(0)}" y2="112" stroke="#3a4552"/>`;
  for(const [i,tq] of DATA.lk) kt+=`<circle cx="${Xk(i)}" cy="${Yk(tq)}" r="3" fill="#9aa7b4"/>`;
  kt+=`<line x1="${Xk(-10)}" y1="${Yk(-10*state.Kt)}" x2="${Xk(10)}" y2="${Yk(10*state.Kt)}" stroke="#4fb3ff" stroke-width="2"/>`;
  kt+=`<text x="44" y="16" fill="#9aa7b4" font-size="11">locked output: load-cell torque (N·m, ±25) vs current (A, ±10)</text>`;
  $('kt').innerHTML=kt;
  let kr=0; for(const [i,tq] of DATA.lk) kr+=(tq-state.Kt*i)**2; kr=Math.sqrt(kr/DATA.lk.length);
  ro('r_ktr', kr.toFixed(2)+' N·m', kr<0.45?'ok':kr<1?'warn':'bad');
  const errs=PKEYS.map(k=> k==='d' ? Math.abs(m.d-TRUE.d)/Math.max(TRUE.d,1) : Math.abs(m[k]-TRUE[k])/Math.max(Math.abs(TRUE[k]),1e-3));
  const wi=errs.indexOf(Math.max(...errs));
  ro('r_truth', state.reveal ? `${PINFO[PKEYS[wi]].lab.split(' ')[0]} ${(errs[wi]*100).toFixed(0)}%` : 'hidden', state.reveal ? (errs[wi]<0.15?'ok':errs[wi]<0.4?'warn':'bad') : '');
  // deploy
  const sim=deploy(m,m,state.kp,state.kd), real=deploy(m,TRUE,state.kp,state.kd);
  const nq=sim.q.length, tq=Array.from({length:nq},(_,k)=>k*CT), rq=tq.map(qref);
  const fd=plotFrame(640,260,40,10,10,22,0,TD,-0.6,0.6,'time (s)','q rad',[0,0.5,1,1.5,2,2.5,3]);
  let ds=fd.s;
  ds+=`<path d="${path(tq,rq,fd.X,fd.Y)}" stroke="#b48cff" stroke-width="2" fill="none"/>`;
  ds+=`<path d="${path(tq,sim.q,fd.X,fd.Y)}" stroke="#4fb3ff" stroke-width="2" fill="none"/>`;
  ds+=`<path d="${path(tq,real.q,fd.X,fd.Y)}" stroke="#ffb454" stroke-width="2" fill="none"/>`;
  const gapArr=sim.q.map((v,k)=>(real.q[k]-v)*5); ds+=`<path d="${path(tq,gapArr,fd.X,fd.Y)}" stroke="#ff6b6b" stroke-width="1" fill="none" opacity=".8"/>`;
  ds+=`<text x="80" y="22" fill="#9aa7b4" font-size="10">red: real − sim, ×5</text>`;
  $('dep').innerHTML=ds;
  let g=0,c=0; sim.q.forEach((v,k)=>{ if(k*CT>0.5){ g+=(real.q[k]-v)**2; c++; } }); g=Math.sqrt(g/c);
  const deg=v=>isFinite(v)?(v*180/Math.PI).toFixed(2)+'°':'unstable';
  ro('r_esim', deg(sim.err), sim.err<0.02?'ok':'warn');
  ro('r_ereal', deg(real.err), !isFinite(real.err)?'bad':real.err<0.02?'ok':real.err<0.06?'warn':'bad');
  ro('r_gap', deg(g), !isFinite(g)?'bad':g<0.005?'ok':g<0.02?'warn':'bad');
  ro('r_peak', real.peak.toFixed(1)+' A');
  LAST={R, floor, sim:sim.err, real:real.err, gap:g};
  renderDR();
  updateNote();
}
function ro(id,text,cls){ const e=$(id); e.className='ro'+(cls?' '+cls:''); e.querySelector('.v').textContent=text; }

// domain randomisation picker
(function buildDR(){
  const box=$('drctl'); let h='';
  for(const k of PKEYS){ const P=PINFO[k]; const st=(P.max-P.min)/200;
    h+=`<div class="drrow"><div class="small">${P.lab} (${P.u}) <output id="dro_${k}"></output></div><div class="two"><input type="range" id="drlo_${k}" min="${P.min}" max="${P.max}" step="${st}" aria-label="${P.lab} minimum"><input type="range" id="drhi_${k}" min="${P.min}" max="${P.max}" step="${st}" aria-label="${P.lab} maximum"></div></div>`; }
  box.innerHTML='<h4>Randomised ranges (min, max)</h4>'+h;
  for(const k of PKEYS){
    $('drlo_'+k).addEventListener('input',e=>{ DR[k][0]=Math.min(+e.target.value, DR[k][1]); renderDR(); updateNote(); });
    $('drhi_'+k).addEventListener('input',e=>{ DR[k][1]=Math.max(+e.target.value, DR[k][0]); renderDR(); updateNote(); });
  }
})();
let DRSTAT={};
function renderDR(){
  const W=640, L=150, R=20; let s=''; let cov=0; let logw=0;
  PKEYS.forEach((k,i)=>{
    const P=PINFO[k], y=20+i*54, X=v=>L+(v-P.min)/(P.max-P.min)*(W-L-R);
    const [lo,hi]=DR[k]; $('drlo_'+k).value=lo; $('drhi_'+k).value=hi; $('dro_'+k).textContent=`${P.f(lo)} – ${P.f(hi)}`;
    const inside = TRUE[k]>=lo-1e-9 && TRUE[k]<=hi+1e-9; if(inside) cov++;
    const span=P.max-P.min; logw += Math.log(Math.max((hi-lo)/span, 0.005));
    s+=`<text x="${L-8}" y="${y+18}" fill="#e6edf3" font-size="13" text-anchor="end">${P.lab}</text>`;
    s+=`<text x="${L-8}" y="${y+33}" fill="${inside?'#4cc38a':'#ff6b6b'}" font-size="11" text-anchor="end">${inside?'covered':'truth OUTSIDE'}</text>`;
    s+=`<line x1="${L}" y1="${y+20}" x2="${W-R}" y2="${y+20}" stroke="#2a3340" stroke-width="10" stroke-linecap="round"/>`;
    s+=`<rect x="${X(lo)}" y="${y+12}" width="${Math.max(X(hi)-X(lo),3)}" height="16" rx="3" fill="#4fb3ff" opacity=".55"/>`;
    s+=`<line x1="${X(state[k])}" y1="${y+6}" x2="${X(state[k])}" y2="${y+34}" stroke="#e6edf3" stroke-width="2"/>`;
    s+=`<path d="M${X(TRUE[k])-6},${y+2} L${X(TRUE[k])+6},${y+2} L${X(TRUE[k])},${y+11} z" fill="#ffb454"/>`;
    s+=`<text x="${L}" y="${y+46}" fill="#9aa7b4" font-size="10">${P.f(P.min)}</text><text x="${W-R}" y="${y+46}" fill="#9aa7b4" font-size="10" text-anchor="end">${P.f(P.max)}</text>`;
  });
  $('drsvg').innerHTML=s;
  ro('r_cov', `${cov} of 5`, cov===5?'ok':'bad');
  ro('r_vol', (100*Math.exp(logw/5)).toFixed(0)+'% of slider span');
  // worst corner: controller built on the model, plant at each corner of the box
  let worst=0;
  for(let mask=0; mask<32; mask++){
    const p={}; PKEYS.forEach((k,i)=>{ p[k]=DR[k][(mask>>i)&1]; });
    const r=deploy(model(), p, state.kp, state.kd); const e=isFinite(r.err)?r.err:Infinity; worst=Math.max(worst,e);
  }
  const real=deploy(model(),TRUE,state.kp,state.kd).err;
  ro('r_wc', isFinite(worst)?(worst*180/Math.PI).toFixed(2)+'°':'unstable', !isFinite(worst)?'bad':worst<0.03?'ok':'warn');
  ro('r_tr', isFinite(real)?(real*180/Math.PI).toFixed(2)+'°':'unstable', isFinite(real) && real<=worst+1e-9 ? 'ok':'bad');
  DRSTAT={cov, worst, real};
}
function setDR(kind){
  for(const k of PKEYS){ const P=PINFO[k]; const v=state[k];
    if(kind==='today'){ DR[k]=[...DR_TODAY[k]]; continue; }
    const f = kind==='p10'?0.1 : kind==='p30'?0.3 : null;
    let lo,hi; if(f!==null){ if(k==='d'){ lo=v-Math.max(f*v,1); hi=v+Math.max(f*v,1);} else { lo=v*(1-f); hi=v*(1+f);} } else { lo=P.min+(P.max-P.min)*0.05; hi=P.max-(P.max-P.min)*0.05; }
    DR[k]=[Math.max(P.min,+lo.toFixed(4)), Math.min(P.max,+hi.toFixed(4))];
  }
  renderDR(); updateNote();
}

function updateNote(){
  const chg=PKEYS.filter(k=>state[k]!==TODAY[k]);
  const drchg=PKEYS.filter(k=>DR[k][0]!==DR_TODAY[k][0]||DR[k][1]!==DR_TODAY[k][1]);
  if(!chg.length && !drchg.length && state.kp===DEFAULTS.kp && state.kd===DEFAULTS.kd && state.test===DEFAULTS.test){ $('note').textContent='Change a model parameter, press Fit, or pick randomisation ranges to build a note.'; return; }
  const deg=v=>isFinite(v)?(v*180/Math.PI).toFixed(2)+'°':'unstable';
  let s=`Marvin Bot (1.44 m, 41 kg humanoid, RobStride actuators; sim motor gains, armature and friction in sim/isaaclab/marvin_bot/robot_cfg.py are placeholders until the phase 2 bench). In a one-joint system-identification toy (illustrative true values, ${state.test==='both'?'step + chirp':state.test} current test, ${state.noise} rad/s speed noise) `;
  s+= chg.length ? `my model is J ${state.J.toFixed(3)} kg·m², b ${state.b.toFixed(2)} N·m·s/rad, Coulomb ${state.tc.toFixed(2)} N·m, delay ${state.d} ms, Kt ${state.Kt.toFixed(2)} N·m/A (placeholders: J 0.042, b 0, 0.3, 5 ms, 2.1)${FIT && PKEYS.every(k=>state[k]===FIT[k])?', from a least-squares fit':''}. ` : `I kept the placeholder model. `;
  s+=`Speed residual ${LAST.R.toFixed(3)} rad/s against a ${LAST.floor.toFixed(3)} rad/s noise floor. A PD tracking test (kp ${state.kp}, kd ${state.kd}, model-based feed-forward) predicts ${deg(LAST.sim)} RMS error in sim and gives ${deg(LAST.real)} on the true joint, a sim-to-real gap of ${deg(LAST.gap)}. `;
  { const r=PKEYS.map(k=>`${k} ${PINFO[k].f(DR[k][0])}–${PINFO[k].f(DR[k][1])}`).join(', '); s+=`Randomisation ranges: ${r}; they cover ${DRSTAT.cov} of 5 true values${DRSTAT.cov<5?' (the policy would never train on the real motor)':''}, worst-corner tracking ${deg(DRSTAT.worst)}. `; }
  s+=`Next step: run bench_robstride.py step/latency/thermal on the RS04 and RS03, add a chirp and a locked-output load-cell test, fit, update robot_cfg.py and actuators.yaml (verified: bench), and centre randomisation on the fit.`;
  $('note').textContent=s;
}

// events
for(const k of [...PKEYS,'noise','kp','kd']) $(k).addEventListener('input',e=>{ state[k]=parseFloat(e.target.value); render(); });
$('test').addEventListener('change',e=>{ state.test=e.target.value; render(); });
$('reveal').addEventListener('change',e=>{ state.reveal=e.target.checked; render(); });
$('fitbtn').addEventListener('click',()=>{ Object.assign(state, fit()); render(); });
$('newmotor').addEventListener('click',()=>{ state.seed++; const r=rng(state.seed*7919);
  TRUE={J:+(0.04+0.025*r()).toFixed(3), b:+(0.2+0.4*r()).toFixed(2), tc:+(0.3+0.6*r()).toFixed(2), d:+(1+7*r()).toFixed(1), Kt:+(1.85+0.3*r()).toFixed(2)}; FIT=null; DATA=null; render(); });
const PRESETS={
  today:()=>({...TODAY}),
  nodelay:()=>({J:0.042,b:0,tc:0,d:0,Kt:2.1}),
  nofric:()=>({...(FIT||fit()), b:0, tc:0}),
  fit:()=>fit(),
};
document.querySelectorAll('[data-preset]').forEach(b=>b.addEventListener('click',()=>{ Object.assign(state, PRESETS[b.dataset.preset]()); render(); }));
document.querySelectorAll('[data-dr]').forEach(b=>b.addEventListener('click',()=>setDR(b.dataset.dr)));
$('copy').addEventListener('click',()=>{ navigator.clipboard.writeText($('note').textContent).then(()=>{ const b=$('copy'); b.textContent='Copied!'; setTimeout(()=>b.textContent='Copy',1200); }); });
render();
