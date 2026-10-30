const $ = id => document.getElementById(id);
const D2R=Math.PI/180;
// log sliders: Il, Ir, kp, kd stored as log10
const LOG = {Il:1, Ir:1, kp:1, kd:1};
const DEFAULTS = {Il:0.184, mgr:6.9, Ir:3.7e-4, N:9, tmax:120, w0:20.9, fr:0.3, kp:150, kd:5, fn:10, zeta:2, ruleI:'arm', mode:'gain', rate:200, delay:0, dwhere:'cmd', q0:0, q1:60};
const PRESETS = {
  knee:{...DEFAULTS},
  wrist:{Il:0.00067, mgr:0, Ir:5e-5, N:10, tmax:14, w0:33.0, fr:0.05, kp:15, kd:0.8, mode:'gain', rate:200, delay:0, dwhere:'cmd', q0:0, q1:45},
  stiff:{...DEFAULTS, kp:1500, kd:5},
  soft:{...DEFAULTS, kp:15, kd:1.5},
  delay:{...DEFAULTS, delay:10, dwhere:'loop'},
  bm:{...DEFAULTS, mode:'freq', fn:10, zeta:2, ruleI:'arm'},
};
const state = {...DEFAULTS};
function fmtv(k,v){
  switch(k){
    case 'Il': return v.toPrecision(3)+' kg·m²';
    case 'Ir': return v.toExponential(2)+' kg·m²';
    case 'mgr': return v.toFixed(1)+' N·m';
    case 'N': return v+':1';
    case 'tmax': return v+' N·m';
    case 'w0': return v>0? v.toFixed(1)+' rad/s ('+(v*60/2/Math.PI).toFixed(0)+' rpm)':'off';
    case 'fr': return v.toFixed(2)+' N·m';
    case 'kp': return v.toPrecision(3)+' N·m/rad';
    case 'kd': return v.toPrecision(3)+' N·m·s/rad';
    case 'fn': return v.toFixed(1)+' Hz';
    case 'zeta': return v.toFixed(2);
    case 'delay': return v+' ms';
    case 'q0': case 'q1': return v+'°';
  } return v;
}
function syncUI(){
  for(const k of ['Il','mgr','Ir','N','tmax','w0','fr','kp','kd','fn','zeta','delay','q0','q1']){
    const el=$(k); el.value = LOG[k]? Math.log10(state[k]) : state[k]; const o=$('o_'+k); if(o) o.textContent=fmtv(k,state[k]); }
  $('rate').value=String(state.rate); $('dwhere').value=state.dwhere; $('ruleI').value=state.ruleI;
  $('m_gain').classList.toggle('on',state.mode==='gain'); $('m_freq').classList.toggle('on',state.mode==='freq');
  $('gainbox').style.display=state.mode==='gain'?'':'none'; $('freqbox').style.display=state.mode==='freq'?'':'none';
}
for(const k of ['Il','mgr','Ir','N','tmax','w0','fr','kp','kd','fn','zeta','delay','q0','q1']){
  $(k).addEventListener('input',()=>{ const v=parseFloat($(k).value); state[k]= LOG[k]? Math.pow(10,v) : v; if(k==='kp'||k==='kd') state.mode='gain'; render(); });
}
$('rate').addEventListener('change',()=>{state.rate=parseFloat($('rate').value); render();});
$('dwhere').addEventListener('change',()=>{state.dwhere=$('dwhere').value; render();});
$('ruleI').addEventListener('change',()=>{state.ruleI=$('ruleI').value; render();});
$('m_gain').addEventListener('click',()=>{state.mode='gain'; render();});
$('m_freq').addEventListener('click',()=>{state.mode='freq'; render();});
document.querySelectorAll('[data-preset]').forEach(b=>b.addEventListener('click',()=>{ Object.assign(state,DEFAULTS,PRESETS[b.dataset.preset]); render(); }));

function gains(){
  const arm=state.Ir*state.N*state.N, J=state.Il+arm;
  if(state.mode==='freq'){ const w=2*Math.PI*state.fn, I= state.ruleI==='arm'?arm:J; state.kp=I*w*w; state.kd=2*I*state.zeta*w; }
  return {arm,J,kp:state.kp,kd:state.kd};
}
// simulate. returns arrays
function simulate(withArm){
  const g=gains(); const J = withArm? g.J : state.Il; const kp=g.kp, kd=g.kd;
  const dt=1e-4, T=1.2, n=Math.round(T/dt), tStep=0.05;
  const q0=state.q0*D2R, q1=state.q1*D2R, mgr=state.mgr, fr=state.fr, tmax=state.tmax, w0=state.w0;
  const ctrlEvery=Math.max(1,Math.round(1/(state.rate*dt)));
  const dSteps=Math.round(state.delay*1e-3/dt);
  const qs=new Float32Array(n+1), vs=new Float32Array(n+1), taus=new Float32Array(n+1), tgt=new Float32Array(n+1);
  let q=q0, v=0, tau=0;
  // start in equilibrium under gravity at q0 (pre-loaded): solve q so that kp(q0 - q) = mgr sin q
  for(let it=0;it<50;it++){ const f=kp*(q0-q)-mgr*Math.sin(q); const df=-kp-mgr*Math.cos(q); if(Math.abs(df)<1e-12) break; q-=f/df; }
  if(!isFinite(q)) q=q0;
  const acc=(qq,vv)=> (tau - mgr*Math.sin(qq) - fr*Math.tanh(vv/0.02))/J;
  let blown=false;
  for(let i=0;i<=n;i++){
    const t=i*dt;
    const target = (t>=tStep)? q1 : q0;
    const tTgtSeen = (state.dwhere==='cmd') ? ((t-state.delay*1e-3)>=tStep? q1:q0) : target;
    tgt[i]=tTgtSeen;
    qs[i]=q; vs[i]=v;
    if(i%ctrlEvery===0){
      let qm=q, vm=v;
      if(state.dwhere==='loop' && dSteps>0){ const j=Math.max(0,i-dSteps); qm=qs[j]; vm=vs[j]; }
      let u = kp*(tTgtSeen-qm) - kd*vm;
      let hi=tmax, lo=-tmax;
      if(w0>0){ hi=Math.min(Math.max(tmax*(1-v/w0),0),tmax); lo=Math.max(Math.min(tmax*(-1-v/w0),0),-tmax); }
      tau=Math.min(hi,Math.max(lo,u));
    }
    taus[i]=tau;
    if(i===n) break;
    // RK4 with tau held
    const k1v=acc(q,v), k1q=v;
    const k2v=acc(q+0.5*dt*k1q, v+0.5*dt*k1v), k2q=v+0.5*dt*k1v;
    const k3v=acc(q+0.5*dt*k2q, v+0.5*dt*k2v), k3q=v+0.5*dt*k2v;
    const k4v=acc(q+dt*k3q, v+dt*k3v), k4q=v+dt*k3v;
    q+=dt/6*(k1q+2*k2q+2*k3q+k4q); v+=dt/6*(k1v+2*k2v+2*k3v+k4v);
    if(!isFinite(q)||Math.abs(q)>50){ blown=true; for(let j=i+1;j<=n;j++){qs[j]=NaN;taus[j]=NaN;tgt[j]=target;} break; }
  }
  return {qs,taus,tgt,dt,n,tStep,blown,kp,kd,J};
}
function metrics(s){
  const {qs,dt,n,tStep}=s; const q0=state.q0*D2R, q1=state.q1*D2R, step=q1-q0;
  if(s.blown) return {unstable:true};
  const iS=Math.round(tStep/dt);
  const tail=Math.round(0.15/dt);
  let mean=0; for(let i=n-tail;i<=n;i++) mean+=qs[i]; mean/= (tail+1);
  // oscillation check in last 0.3 s vs prior 0.3 s
  const amp=(a,b)=>{let mx=-1e9,mn=1e9; for(let i=a;i<b;i++){mx=Math.max(mx,qs[i]);mn=Math.min(mn,qs[i]);} return mx-mn;};
  const w=Math.round(0.3/dt); const a2=amp(n-w,n), a1=amp(n-2*w,n-w);
  const sustained = Math.abs(step)>1e-6 && a2>0.05*Math.abs(step) && a2>0.7*a1;
  let peak=-Infinity; for(let i=iS;i<=n;i++){ const d=(qs[i]-q0)*Math.sign(step||1); if(d>peak) peak=d; }
  const os = Math.abs(step)>1e-6 ? Math.max(0,(peak-Math.abs(step))/Math.abs(step))*100 : 0;
  const osFinal = Math.abs(step)>1e-6 ? Math.max(0,(peak-(mean-q0)*Math.sign(step))/Math.abs(step))*100 : 0;
  let ts=null; const band=0.02*Math.abs(step);
  for(let i=n;i>=iS;i--){ if(Math.abs(qs[i]-mean)>band){ ts=(i+1)*dt-tStep; break; } }
  if(ts===null) ts=0;
  let tp=0; for(let i=iS;i<=n;i++) tp=Math.max(tp,Math.abs(s.taus[i]));
  return {unstable:sustained, os, osFinal, ts, ss:(q1-mean)/D2R, tp, settled: ts < 1.2-tStep-0.16};
}
const cv=$('cv'), ctx=cv.getContext('2d');
function plot(sA,sB){
  const W=cv.width,H=cv.height; ctx.fillStyle='#0c1015'; ctx.fillRect(0,0,W,H);
  const L=90,R=W-20; const p1={t:20,b:430}, p2={t:480,b:H-50};
  const T=1.2, X=t=>L+t/T*(R-L);
  // ranges
  let qmin=Infinity,qmax=-Infinity; for(const s of [sA,sB]) for(let i=0;i<=s.n;i+=10){ const v=s.qs[i]; if(isFinite(v)){qmin=Math.min(qmin,v,s.tgt[i]);qmax=Math.max(qmax,v,s.tgt[i]);} }
  qmin/=D2R; qmax/=D2R; const pad=Math.max(5,(qmax-qmin)*0.12); qmin-=pad; qmax+=pad;
  qmin=Math.max(qmin,-400); qmax=Math.min(qmax,400);
  const Y1=v=>p1.b-(v-qmin)/(qmax-qmin)*(p1.b-p1.t);
  const tl=state.tmax*1.15; const Y2=v=>(p2.t+p2.b)/2 - v/tl*(p2.b-p2.t)/2;
  ctx.font='22px system-ui,sans-serif'; ctx.lineWidth=1;
  // grids
  ctx.strokeStyle='#1d2530'; ctx.fillStyle='#9aa7b4';
  for(let t=0;t<=T+1e-9;t+=0.2){ ctx.beginPath(); ctx.moveTo(X(t),p1.t); ctx.lineTo(X(t),p1.b); ctx.moveTo(X(t),p2.t); ctx.lineTo(X(t),p2.b); ctx.stroke(); ctx.textAlign='center'; ctx.fillText(t.toFixed(1)+' s',X(t),H-18); }
  const stepq=niceStep((qmax-qmin)/6);
  ctx.textAlign='right';
  for(let v=Math.ceil(qmin/stepq)*stepq; v<=qmax; v+=stepq){ ctx.beginPath(); ctx.moveTo(L,Y1(v)); ctx.lineTo(R,Y1(v)); ctx.stroke(); ctx.fillText(Math.round(v)+'°',L-8,Y1(v)+7); }
  const stept=niceStep(tl/2.5);
  for(let v=-Math.floor(tl/stept)*stept; v<=tl; v+=stept){ ctx.beginPath(); ctx.moveTo(L,Y2(v)); ctx.lineTo(R,Y2(v)); ctx.stroke(); ctx.fillText(Math.round(v)+'',L-8,Y2(v)+7); }
  ctx.save(); ctx.translate(24,(p1.t+p1.b)/2); ctx.rotate(-Math.PI/2); ctx.textAlign='center'; ctx.fillText('joint angle',0,0); ctx.restore();
  ctx.save(); ctx.translate(24,(p2.t+p2.b)/2); ctx.rotate(-Math.PI/2); ctx.textAlign='center'; ctx.fillText('torque N·m',0,0); ctx.restore();
  // 2% band around target
  const q1=state.q1, band=Math.abs(state.q1-state.q0)*0.02;
  ctx.fillStyle='rgba(255,180,84,0.10)'; ctx.fillRect(L,Y1(q1+band),R-L,Math.max(1,Y1(q1-band)-Y1(q1+band)));
  // torque limits
  ctx.strokeStyle='#ff6b6b'; ctx.setLineDash([8,6]); ctx.beginPath(); ctx.moveTo(L,Y2(state.tmax)); ctx.lineTo(R,Y2(state.tmax)); ctx.moveTo(L,Y2(-state.tmax)); ctx.lineTo(R,Y2(-state.tmax)); ctx.stroke(); ctx.setLineDash([]);
  const line=(s,arr,Yf,col,wd,scale)=>{ ctx.strokeStyle=col; ctx.lineWidth=wd; ctx.beginPath(); let pen=false; for(let i=0;i<=s.n;i+=4){ const v=arr[i]*(scale||1); if(!isFinite(v)){pen=false;continue;} const y=Math.max(-50,Math.min(H+50,Yf(v))); if(!pen){ctx.moveTo(X(i*s.dt),y);pen=true;} else ctx.lineTo(X(i*s.dt),y);} ctx.stroke(); };
  line(sA,sA.tgt,Y1,'#ffb454',2,1/D2R);
  line(sB,sB.qs,Y1,'#b48cff',2,1/D2R);
  line(sA,sA.qs,Y1,'#4fb3ff',3.5,1/D2R);
  ctx.save(); ctx.beginPath(); ctx.rect(L,p2.t-2,R-L,p2.b-p2.t+4); ctx.clip(); line(sA,sA.taus,Y2,'#4cc38a',2.5,1); ctx.restore();
  ctx.lineWidth=1;
}
function niceStep(x){ const p=Math.pow(10,Math.floor(Math.log10(x))); const m=x/p; return (m<1.5?1:m<3.5?2:m<7.5?5:10)*p; }
function ro(id,v,cls){ const e=$(id); e.className='ro '+(cls||''); e.querySelector('.v').textContent=v; }
let last={};
function render(){
  const g=gains(); syncUI();
  const sA=simulate(true), sB=simulate(false);
  plot(sA,sB);
  const m=metrics(sA);
  const wn=Math.sqrt(g.kp/g.J), z=g.kd/(2*Math.sqrt(g.kp*g.J));
  ro('r_arm', g.arm.toPrecision(2)+' kg·m², '+(g.arm/g.J*100).toFixed(0)+'% of J','');
  ro('r_wn', (wn/2/Math.PI).toFixed(1)+' Hz ('+wn.toFixed(0)+' rad/s)', wn/state.rate>0.5?'warn':'');
  ro('r_z', z.toFixed(2), z<0.3?'warn':'');
  if(m.unstable||sA.blown){
    ro('r_os','–','bad'); ro('r_ts','never','bad'); ro('r_ss','–','bad'); ro('r_tp', sA.blown?'–':m.tp.toFixed(1)+' N·m','bad');
    ro('r_st', sA.blown?'diverges':'keeps oscillating','bad');
  } else {
    ro('r_os', m.os.toFixed(1)+'%', m.os>25?'warn':'');
    ro('r_ts', m.settled? (m.ts*1000).toFixed(0)+' ms' : '> 1 s', m.settled?'':'warn');
    ro('r_ss', m.ss.toFixed(2)+'°', Math.abs(m.ss)>3?'warn':'');
    ro('r_tp', m.tp.toFixed(1)+' N·m'+(m.tp>=state.tmax*0.999?' (saturated)':''), m.tp>=state.tmax*0.999?'warn':'');
    const verdict = z<0.3?'underdamped': (m.os>25?'ringing':(Math.abs(m.ss)>3?'sags':(m.settled?'good':'slow')));
    ro('r_st', verdict, verdict==='good'?'ok':'warn');
  }
  last={g,wn,z,m,blown:sA.blown};
  updateNote();
}
function updateNote(){
  const ch=[]; const g=last.g;
  const names={Il:'link inertia',mgr:'gravity moment',Ir:'rotor inertia',N:'gear ratio',tmax:'peak torque',w0:'no-load speed',fr:'friction',kp:'kp',kd:'kd',rate:'PD update rate',delay:'delay',q0:'step start',q1:'step end'};
  for(const k in names){ const a=state[k], b=DEFAULTS[k]; if(Math.abs(a-b)>1e-9*Math.max(1,Math.abs(b))) ch.push(`${names[k]} ${fmtv(k,a)||a}${k==='rate'?' Hz':''} (knee default ${fmtv(k,b)||b}${k==='rate'?' Hz':''})`); }
  if(state.mode==='freq') ch.push(`gains from the rule kp = I·ω², kd = 2·I·ζ·ω with f = ${state.fn} Hz, ζ = ${state.zeta}, I = ${state.ruleI==='arm'?'armature only (BeyondMimic)':'link + armature'}`);
  if(state.delay>0) ch.push(`delay placed ${state.dwhere==='cmd'?'on the target (as DelayedDCMotor)':'inside the PD loop'}`);
  if(!ch.length){ $('note').textContent='Change a control above to build a note.'; return; }
  const m=last.m;
  const res = last.blown||m.unstable ? 'The loop is UNSTABLE (sustained or growing oscillation).' :
    `Result: overshoot ${m.os.toFixed(1)}%, 2% settling ${m.settled?(m.ts*1000).toFixed(0)+' ms':'over 1 s'}, steady-state error ${m.ss.toFixed(2)}°, peak torque ${m.tp.toFixed(1)} N·m.`;
  $('note').textContent = `Marvin Bot (1.44 m, 41 kg humanoid), single-joint PD step-response study. Starting from the knee as trained (RS04, kp 150, kd 5, armature 0.03 kg·m² placeholder, link 0.184 kg·m², sim/isaaclab/marvin_bot/robot_cfg.py), I changed: ${ch.join('; ')}. Resulting gains kp ${g.kp.toPrecision(3)} N·m/rad, kd ${g.kd.toPrecision(3)} N·m·s/rad; armature ${g.arm.toPrecision(3)} kg·m²; total inertia ${g.J.toPrecision(3)} kg·m²; natural frequency ${(last.wn/2/Math.PI).toFixed(1)} Hz; damping ratio ${last.z.toFixed(2)}. ${res} Caveats: one joint with the rest of the robot fixed, linear torque-speed line, armature not yet measured (phase 2 bench).`;
}
$('copy').addEventListener('click',()=>{ navigator.clipboard.writeText($('note').textContent).then(()=>{ const b=$('copy'); b.textContent='Copied!'; setTimeout(()=>b.textContent='Copy',1200); }); });
render();
