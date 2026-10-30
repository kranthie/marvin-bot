const $ = id => document.getElementById(id);
const G=9.81;
function ro(id,v,cls){ const e=$(id); e.className='ro '+(cls||''); e.querySelector('.v').textContent=v; }
function niceStep(x){ const p=Math.pow(10,Math.floor(Math.log10(x))); const m=x/p; return (m<1.5?1:m<3.5?2:m<7.5?5:10)*p; }
/* =============== Playground A =============== */
const DA={model:'soft', k:1e5, c:500, tc:0.02, dr:1, e:0.3, dt:0.002, integ:'se', m:41, h0:0.2};
const PA={ today:{...DA},
  stiffok:{...DA, model:'penalty', k:1e7, c:0, dt:0.0005, integ:'se'},
  blow:{...DA, model:'penalty', k:1e7, c:0, dt:0.005, integ:'se'},
  euler:{...DA, model:'penalty', k:1e5, c:0, dt:0.002, integ:'ee'},
  hard:{...DA, model:'hard', e:0.3, dt:0.002} };
const A={...DA};
const LOGA={k:1,dt:1};
function fa(k,v){ switch(k){case 'k':return v.toExponential(1)+' N/m'; case 'c':return v.toFixed(0)+' N·s/m'; case 'tc':return (v*1000).toFixed(0)+' ms'; case 'dr':return v.toFixed(2); case 'e':return v.toFixed(2); case 'dt':return v>=0.001?(v*1000).toFixed(2)+' ms':(v*1e6).toFixed(0)+' µs'; case 'm':return v+' kg'; case 'h0':return (v*100).toFixed(0)+' cm';} return v;}
const KA=['k','c','tc','dr','e','dt','m','h0'];
KA.forEach(k=>$(k).addEventListener('input',()=>{ const v=parseFloat($(k).value); A[k]=LOGA[k]?Math.pow(10,v):v; renderA(); }));
$('model').addEventListener('change',()=>{A.model=$('model').value; renderA();});
$('integ').addEventListener('change',()=>{A.integ=$('integ').value; renderA();});
document.querySelectorAll('[data-preset]').forEach(b=>b.addEventListener('click',()=>{ Object.assign(A,PA[b.dataset.preset]); renderA(); }));
function softParams(){ const dw=0.95; return {ks:1/(dw*dw*A.tc*A.tc*A.dr*A.dr), bs:2/(dw*A.tc), d0:0.9, dw, width:0.001}; }
function simA(){
  const m=A.m, dt=A.dt, T=1.5, n=Math.min(Math.round(T/dt),200000);
  const sp=softParams();
  const tc=A.tc; // MuJoCo refsafe is not applied here, so the user can see the problem
  function acc(z,v){
    let a=-G; const pen=-z;
    if(pen>0){
      if(A.model==='penalty'){ const F=Math.max(0,A.k*pen - A.c*v); a+=F/m; }
      else if(A.model==='soft'){ const d=sp.d0+(sp.dw-sp.d0)*Math.min(pen/sp.width,1); const f=d*(-sp.bs*v + sp.ks*pen + G); if(f>0) a+=f; }
    }
    return a;
  }
  function energy(z,v){ let E=0.5*m*v*v+m*G*z; const pen=-z; if(pen>0){ if(A.model==='penalty') E+=0.5*A.k*pen*pen; else if(A.model==='soft') E+=0.5*m*0.95*sp.ks*pen*pen; } return E; }
  const ts=[],zs=[],es=[];
  let z=A.h0, v=0; const E0=Math.max(energy(z,v), 1e-9);
  let maxPen=0, Emax=1, blown=false, inC=false, contacts=0, h1=null, apexAfter=-1e9, firstEnd=false, contactStart=null, contactDur=null;
  const every=Math.max(1,Math.floor(n/2000));
  for(let i=0;i<=n;i++){
    const t=i*dt;
    if(i%every===0){ ts.push(t); zs.push(z); es.push(energy(z,v)/E0); }
    const pen=-z; if(pen>maxPen) maxPen=pen;
    const Er=energy(z,v)/E0; if(Er>Emax) Emax=Er;
    if(!isFinite(z)||Math.abs(z)>100||Er>3){ blown=true; break; }
    const nowC = A.model==='hard'? false : pen>0;
    if(nowC&&!inC){ contacts++; if(contacts===1) contactStart=t; }
    if(!nowC&&inC&&contacts===1&&!firstEnd){ firstEnd=true; contactDur=t-contactStart; }
    if(firstEnd&&contacts===1) apexAfter=Math.max(apexAfter,z);
    inC=nowC;
    if(i===n) break;
    if(A.model==='hard'){
      v+=-G*dt;
      if(z+v*dt<0 && v<0){ v=-A.e*v; if(contacts===0){ contacts=1; firstEnd=true; contactDur=0; } else if(!firstEnd){} else if(apexAfter>-1e8 && v>1e-3) contacts++; }
      z=Math.max(0,z+v*dt);
    } else if(A.integ==='ee'){ const a=acc(z,v); z+=dt*v; v+=dt*a; }
    else if(A.integ==='se'){ v+=dt*acc(z,v); z+=dt*v; }
    else { const k1z=v,k1v=acc(z,v); const k2z=v+0.5*dt*k1v,k2v=acc(z+0.5*dt*k1z,v+0.5*dt*k1v); const k3z=v+0.5*dt*k2v,k3v=acc(z+0.5*dt*k2z,v+0.5*dt*k2v); const k4z=v+dt*k3v,k4v=acc(z+dt*k3z,v+dt*k3v); z+=dt/6*(k1z+2*k2z+2*k3z+k4z); v+=dt/6*(k1v+2*k2v+2*k3v+k4v); }
  }
  if(firstEnd && apexAfter>0) h1=apexAfter;
  return {ts,zs,es,maxPen,Emax,blown,h1,contactDur,E0};
}
const cA=$('cvA'), xA=cA.getContext('2d');
function plotA(s){
  const W=cA.width,H=cA.height; xA.fillStyle='#0c1015'; xA.fillRect(0,0,W,H);
  const L=140,R=W-20,T=1.5,X=t=>L+t/T*(R-L);
  const P=[{t:20,b:300},{t:340,b:540},{t:580,b:H-50}];
  xA.font='22px system-ui,sans-serif';
  // ranges
  const hmax=Math.max(A.h0,0.01)*1.15;
  let pmax=Math.max(s.maxPen*1000*1.2, 1); if(!isFinite(pmax)||s.blown) pmax=Math.max(1, Math.min(pmax, 1000));
  let emax=Math.min(Math.max(1.2, s.Emax*1.05), 5);
  const Ys=[v=>P[0].b-(v)/(hmax)*(P[0].b-P[0].t), v=>P[1].t+(v)/(pmax)*(P[1].b-P[1].t), v=>P[2].b-v/emax*(P[2].b-P[2].t)];
  xA.strokeStyle='#1d2530'; xA.fillStyle='#9aa7b4'; xA.lineWidth=1;
  for(let t=0;t<=T+1e-9;t+=0.25){ for(const p of P){ xA.beginPath(); xA.moveTo(X(t),p.t); xA.lineTo(X(t),p.b); xA.stroke(); } xA.textAlign='center'; xA.fillText(t.toFixed(2)+' s',X(t),H-18); }
  const axes=[[0,hmax,v=>(v*100).toFixed(0)+' cm'],[0,pmax,v=>v.toFixed(v<10?1:0)+' mm'],[0,emax,v=>(v*100).toFixed(0)+'%']];
  axes.forEach(([lo,hi,f],i)=>{ const st=niceStep((hi-lo)/4); xA.textAlign='right'; for(let v=lo; v<=hi+1e-12; v+=st){ xA.beginPath(); xA.moveTo(L,Ys[i](v)); xA.lineTo(R,Ys[i](v)); xA.stroke(); xA.fillText(f(v),L-8,Ys[i](v)+7);} });
  const labels=['height','penetration ↓','energy'];
  labels.forEach((l,i)=>{ xA.save(); xA.translate(24,(P[i].t+P[i].b)/2); xA.rotate(-Math.PI/2); xA.textAlign='center'; xA.fillText(l,0,0); xA.restore(); });
  xA.strokeStyle='#ffb454'; xA.setLineDash([6,6]); xA.beginPath(); xA.moveTo(L,Ys[2](1)); xA.lineTo(R,Ys[2](1)); xA.stroke(); xA.setLineDash([]);
  const draw=(arr,Y,col,p,mapf)=>{ xA.save(); xA.beginPath(); xA.rect(L,p.t-1,R-L,p.b-p.t+2); xA.clip(); xA.strokeStyle=col; xA.lineWidth=3; xA.beginPath(); arr.forEach((v,i)=>{ const y=Y(mapf(v)); const x=X(s.ts[i]); if(i===0) xA.moveTo(x,y); else xA.lineTo(x,y); }); xA.stroke(); xA.restore(); };
  draw(s.zs,Ys[0],'#4fb3ff',P[0],v=>Math.max(v,0));
  draw(s.zs,Ys[1],'#ff6b6b',P[1],v=>Math.max(-v,0)*1000);
  draw(s.es,Ys[2],'#4cc38a',P[2],v=>v);
  if(s.blown){ xA.fillStyle='#ff6b6b'; xA.font='bold 30px system-ui'; xA.textAlign='center'; xA.fillText('unstable: the simulation blew up',(L+R)/2,(P[0].t+P[0].b)/2); }
}
function renderA(){
  KA.forEach(k=>{ $(k).value=LOGA[k]?Math.log10(A[k]):A[k]; $('o_'+k).textContent=fa(k,A[k]); });
  $('model').value=A.model; $('integ').value=A.integ;
  $('box_pen').style.display=A.model==='penalty'?'':'none'; $('box_soft').style.display=A.model==='soft'?'':'none'; $('box_hard').style.display=A.model==='hard'?'':'none';
  $('integ').disabled = A.model==='hard';
  const s=simA(); plotA(s);
  let w, zeta, statPen;
  const sp=softParams();
  if(A.model==='penalty'){ w=Math.sqrt(A.k/A.m); zeta=A.c/(2*Math.sqrt(A.k*A.m)); statPen=A.m*G/A.k; }
  else if(A.model==='soft'){ w=Math.sqrt(0.95*sp.ks); zeta=(0.95*sp.bs)/(2*w); statPen=0.05*G/(0.95*sp.ks); }
  ro('a_pen', s.blown?'–':(s.maxPen*1000).toFixed(s.maxPen<0.01?2:1)+' mm', s.blown?'bad':'');
  if(A.model==='hard'){
    ro('a_stat','0 mm',''); ro('a_wdt','n/a (no spring)',''); ro('a_spc','1 (an impulse)','');
    const eT=A.e; const eS = s.h1? Math.sqrt(s.h1/A.h0):0;
    ro('a_e', eS.toFixed(2)+' / '+eT.toFixed(2),'');
  } else {
    ro('a_stat', (statPen*1000).toFixed(statPen<0.001?3:2)+' mm','');
    const wdt=w*A.dt;
    const lim = A.integ==='ee'?0:(A.integ==='se'?2:2.8);
    ro('a_wdt', wdt.toFixed(2)+(A.integ==='ee'?' (always drifts)':' (limit '+lim+')'), (s.blown||wdt>lim)?'bad':(A.integ==='ee'||wdt>0.5*lim?'warn':'ok'));
    const dur = s.contactDur;
    ro('a_spc', s.blown?'–':(dur? (dur/A.dt).toFixed(dur/A.dt<10?1:0)+' ('+(dur*1000).toFixed(0)+' ms)':'stays in contact'), dur&&dur/A.dt<10?'warn':'');
    const eT = zeta<1? Math.exp(-zeta*Math.PI/Math.sqrt(1-zeta*zeta)) : 0;
    const eS = s.h1? Math.sqrt(s.h1/A.h0):0;
    ro('a_e', s.blown?'–':eS.toFixed(2)+' / '+eT.toFixed(2), '');
  }
  ro('a_en', s.blown?'∞':(s.Emax*100).toFixed(1)+'%', s.blown||s.Emax>1.1?'bad':(s.Emax>1.01?'warn':'ok'));
  lastA=s; updateNote();
}
let lastA=null;
/* =============== Playground B =============== */
const DB={push:60, akp:800, akd:80, kA:Math.pow(10,4.7), muA:1, fA:'reg', kB:1e6, muB:1, fB:'stick'};
const PB={ today:{...DB}, rock:{...DB, push:140}, edge:{...DB, push:150}, slip:{...DB, push:100, muA:0.15, muB:0.15} };
const B={...DB};
const LOGB={kA:1,kB:1};
function fb(k,v){ switch(k){case 'push':return v+' N'; case 'akp':return v+' N·m/rad'; case 'akd':return v+' N·m·s/rad'; case 'kA': case 'kB': return v.toExponential(1)+' N/m'; case 'muA': case 'muB': return v.toFixed(2);} return v; }
const KB=['push','akp','akd','kA','muA','kB','muB'];
KB.forEach(k=>$(k).addEventListener('input',()=>{ const v=parseFloat($(k).value); B[k]=LOGB[k]?Math.pow(10,v):v; renderB(); }));
['fA','fB'].forEach(k=>$(k).addEventListener('change',()=>{B[k]=$(k).value; renderB();}));
document.querySelectorAll('[data-preset2]').forEach(b=>b.addEventListener('click',()=>{ Object.assign(B,PB[b.dataset.preset2]); renderB(); }));
const RB={mf:2, cf:[0.03,0], heel:[-0.09,-0.03], toe:[0.15,-0.03], ank:[0,0.03], mp:39, L:0.75, Ip:39*0.25*0.25, tmax:238};
RB.If = RB.mf*(0.24*0.24+0.06*0.06)/12;
function solve4(M,b){ const n=4; const a=M.map((r,i)=>[...r,b[i]]);
  for(let c=0;c<n;c++){ let p=c; for(let r=c+1;r<n;r++) if(Math.abs(a[r][c])>Math.abs(a[p][c])) p=r; [a[c],a[p]]=[a[p],a[c]];
    for(let r=c+1;r<n;r++){ const f=a[r][c]/a[c][c]; for(let k=c;k<=n;k++) a[r][k]-=f*a[c][k]; } }
  const x=new Array(n).fill(0); for(let r=n-1;r>=0;r--){ let s=a[r][n]; for(let k=r+1;k<n;k++) s-=a[r][k]*x[k]; x[r]=s/a[r][r]; } return x; }
const SB={dt:1e-4, meff:5, vs:0.05};
function simB(kn,mu,law){
  const dt=SB.dt, T=3, n=Math.round(T/dt);
  const cn=2*Math.sqrt(kn*SB.meff), kt=kn, ct=cn;
  let q=[0, 0.03 - RB.mp*0+0, 0, 0], qd=[0,0,0,0];
  q[1]=0.03 + 0; // sole at z=0
  const anchors=[null,null];
  const out={t:[],ph:[],heel:[],slide:[],pose:[]};
  let fell=false, x0=null, ph0=0, maxLean=0, maxHeel=0, failT=null;
  const pts=[RB.heel,RB.toe];
  for(let i=0;i<=n;i++){
    const t=i*dt;
    const [x,z,th,ph]=q, [xd,zd,thd,phd]=qd; const c=Math.cos(th), s=Math.sin(th);
    // foot point helper
    const P=(ab)=>{ const [a,b]=ab; return {p:[x+a*c-b*s, z+a*s+b*c], J:[[1,0,-a*s-b*c,0],[0,1,a*c-b*s,0]], jd:[-thd*thd*(a*c-b*s), -thd*thd*(a*s+b*c)]}; };
    const F=P(RB.cf);
    const A0=P(RB.ank); const sp=Math.sin(ph), cp=Math.cos(ph);
    const Pp={p:[A0.p[0]+RB.L*sp, A0.p[1]+RB.L*cp], J:[[1,0,A0.J[0][2],RB.L*cp],[0,1,A0.J[1][2],-RB.L*sp]], jd:[A0.jd[0]-RB.L*sp*phd*phd, A0.jd[1]-RB.L*cp*phd*phd]};
    if(i===Math.round(1.0/dt)) x0=x;
    if(i%Math.round(0.004/dt)===0){ out.t.push(t); out.ph.push(ph*180/Math.PI); const hz=P(RB.heel).p[1]; out.heel.push(hz*1000); out.slide.push(x0===null?0:(x-x0)*1000); }
    if(i===Math.round(1.0/dt)) ph0=ph;
    if(t>=1.0){ maxLean=Math.max(maxLean,Math.abs(ph-ph0)*180/Math.PI); maxHeel=Math.max(maxHeel,P(RB.heel).p[1]); }
    if(Math.abs(ph)>Math.PI/6 || !isFinite(ph)){ fell=true; failT=t; break; }
    if(i===n) break;
    // mass matrix
    const M=[[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]], rhs=[0,0,0,0];
    const addBody=(B,m)=>{ for(let r=0;r<4;r++){ for(let k=0;k<4;k++) M[r][k]+=m*(B.J[0][r]*B.J[0][k]+B.J[1][r]*B.J[1][k]); rhs[r]+= B.J[0][r]*(-m*B.jd[0]) + B.J[1][r]*(m*(-G) - m*B.jd[1]); } };
    addBody(F,RB.mf); addBody(Pp,RB.mp); M[2][2]+=RB.If; M[3][3]+=RB.Ip;
    // ankle PD
    let tau=B.akp*(ph+th)+B.akd*(phd+thd); tau=Math.max(-RB.tmax,Math.min(RB.tmax,tau));
    rhs[2]-=tau; rhs[3]-=tau;
    // push on body COM
    if(t>=1.0&&t<1.15){ for(let r=0;r<4;r++) rhs[r]+=Pp.J[0][r]*B.push; }
    // contacts
    pts.forEach((ab,j)=>{ const C=P(ab); const vx=C.J[0][0]*xd+C.J[0][2]*thd, vz=C.J[1][1]*zd+C.J[1][2]*thd; const pen=-C.p[1];
      if(pen>0){ const Fn=Math.max(0,kn*pen - cn*vz); let Ft;
        if(law==='reg'){ Ft=-mu*Fn*Math.tanh(vx/SB.vs); anchors[j]=null; }
        else { if(anchors[j]===null) anchors[j]=C.p[0]; let tr=-kt*(C.p[0]-anchors[j]) - ct*vx; const lim=mu*Fn; if(Math.abs(tr)>lim){ tr=Math.sign(tr)*lim; anchors[j]=C.p[0]+tr/kt; } Ft=tr; }
        for(let r=0;r<4;r++) rhs[r]+=C.J[0][r]*Ft + C.J[1][r]*Fn;
      } else anchors[j]=null; });
    const qdd=solve4(M,rhs);
    for(let r=0;r<4;r++){ qd[r]+=dt*qdd[r]; q[r]+=dt*qd[r]; }
  }
  const slide = out.slide.length? out.slide[out.slide.length-1]:0;
  return {...out, fell, failT, maxLean, maxHeel:maxHeel*1000, slideEnd:slide, final:q};
}
const cB=$('cvB'), xB=cB.getContext('2d');
function plotB(a,b){
  const W=cB.width,H=cB.height; xB.fillStyle='#0c1015'; xB.fillRect(0,0,W,H);
  xB.font='22px system-ui,sans-serif';
  // left: side view sketch of the final pose of both
  const gx=150, gy=640, sc=600;
  xB.strokeStyle='#3a4757'; xB.lineWidth=2; xB.beginPath(); xB.moveTo(20,gy); xB.lineTo(300,gy); xB.stroke();
  [[a,'#4fb3ff'],[b,'#ffb454']].forEach(([r,col])=>{
    const [x,z,th,ph]=r.final; const c=Math.cos(th), s=Math.sin(th);
    const W2=(ab)=>[gx+(x+ab[0]*c-ab[1]*s)*sc*0.5, gy-(z+ab[0]*s+ab[1]*c)*sc*0.5];
    const h=W2(RB.heel), t=W2(RB.toe), an=W2(RB.ank);
    const top=[an[0]+RB.L*Math.sin(ph)*sc*0.5, an[1]-RB.L*Math.cos(ph)*sc*0.5];
    xB.strokeStyle=col; xB.lineWidth=5; xB.beginPath(); xB.moveTo(h[0],h[1]); xB.lineTo(t[0],t[1]); xB.lineTo(an[0],an[1]); xB.closePath(); xB.stroke();
    xB.lineWidth=4; xB.beginPath(); xB.moveTo(an[0],an[1]); xB.lineTo(top[0],top[1]); xB.stroke();
    xB.fillStyle=col; xB.beginPath(); xB.arc(top[0],top[1],14,0,7); xB.fill();
  });
  xB.fillStyle='#9aa7b4'; xB.textAlign='left'; xB.fillText('end pose (side view)',20,40); xB.fillText('push →',40,gy-260);
  // right: plots
  const L=420,R=W-20,T=3, X=t=>L+t/T*(R-L);
  const P1={t:20,b:360}, P2={t:410,b:H-50};
  let lmax=2; [a,b].forEach(r=>r.ph.forEach(v=>{ if(isFinite(v)) lmax=Math.max(lmax,Math.abs(v)); })); lmax=Math.min(lmax*1.15,35);
  let mm=2; [a,b].forEach(r=>{ r.heel.forEach(v=>mm=Math.max(mm,Math.abs(v))); r.slide.forEach(v=>mm=Math.max(mm,Math.abs(v))); }); mm=Math.min(mm*1.15,400);
  const Y1=v=>(P1.t+P1.b)/2 - v/lmax*(P1.b-P1.t)/2, Y2=v=>(P2.t+P2.b)/2 - v/mm*(P2.b-P2.t)/2;
  xB.strokeStyle='#1d2530'; xB.lineWidth=1; xB.fillStyle='#9aa7b4';
  for(let t=0;t<=T;t+=0.5){ xB.beginPath(); xB.moveTo(X(t),P1.t); xB.lineTo(X(t),P1.b); xB.moveTo(X(t),P2.t); xB.lineTo(X(t),P2.b); xB.stroke(); xB.textAlign='center'; xB.fillText(t.toFixed(1)+' s',X(t),H-18); }
  const s1=niceStep(lmax/2), s2=niceStep(mm/2); xB.textAlign='right';
  for(let v=-Math.floor(lmax/s1)*s1; v<=lmax; v+=s1){ xB.beginPath(); xB.moveTo(L,Y1(v)); xB.lineTo(R,Y1(v)); xB.stroke(); xB.fillText(v.toFixed(s1<1?1:0)+'°',L-8,Y1(v)+7); }
  for(let v=-Math.floor(mm/s2)*s2; v<=mm; v+=s2){ xB.beginPath(); xB.moveTo(L,Y2(v)); xB.lineTo(R,Y2(v)); xB.stroke(); xB.fillText(v.toFixed(s2<1?1:0),L-8,Y2(v)+7); }
  xB.fillStyle='rgba(255,107,107,0.12)'; xB.fillRect(X(1.0),P1.t,X(1.15)-X(1.0),P1.b-P1.t); xB.fillRect(X(1.0),P2.t,X(1.15)-X(1.0),P2.b-P2.t);
  xB.fillStyle='#9aa7b4'; xB.textAlign='left'; xB.fillText('push',X(1.16),P1.t+24); xB.fillText('body lean',L+8,P1.t+24); xB.fillText('heel lift, foot slide (mm)',L+8,P2.t+24);
  const line=(r,arr,Y,col,dash,w)=>{ xB.save(); xB.beginPath(); xB.rect(L,0,R-L,H); xB.clip(); xB.strokeStyle=col; xB.lineWidth=w; xB.setLineDash(dash); xB.beginPath(); arr.forEach((v,i)=>{ const x=X(r.t[i]), y=Y(v); if(i===0) xB.moveTo(x,y); else xB.lineTo(x,y); }); xB.stroke(); xB.restore(); };
  [[a,'#4fb3ff'],[b,'#ffb454']].forEach(([r,col])=>{ line(r,r.ph,Y1,col,[],3); line(r,r.heel,Y2,col,[10,6],2.5); line(r,r.slide,Y2,col,[2,5],3); if(r.fell){ xB.fillStyle=col; xB.textAlign='left'; xB.fillText('falls at '+r.failT.toFixed(2)+' s',X(Math.min(r.failT,2.3)),Y1(0)+(col==='#4fb3ff'?-30:40)); } });
}
let lastB=null;
function renderB(){
  KB.forEach(k=>{ $(k).value=LOGB[k]?Math.log10(B[k]):B[k]; $('o_'+k).textContent=fb(k,B[k]); });
  $('fA').value=B.fA; $('fB').value=B.fB;
  const a=simB(B.kA,B.muA,B.fA), b=simB(B.kB,B.muB,B.fB);
  plotB(a,b);
  const f1=v=>v.toFixed(1);
  ro('b_lean', f1(a.maxLean)+'° / '+f1(b.maxLean)+'°','');
  ro('b_lift', f1(a.maxHeel)+' / '+f1(b.maxHeel)+' mm','');
  ro('b_slide', f1(a.slideEnd)+' / '+f1(b.slideEnd)+' mm', Math.abs(a.slideEnd-b.slideEnd)>5?'warn':'');
  ro('b_out', (a.fell?'falls':'recovers')+' / '+(b.fell?'falls':'recovers'), a.fell!==b.fell?'bad':(a.fell?'bad':'ok'));
  let gap=0; const nn=Math.min(a.ph.length,b.ph.length); const i0=Math.round(1.0/0.004); for(let i=i0;i<nn;i++) gap=Math.max(gap,Math.abs((a.ph[i]-a.ph[i0])-(b.ph[i]-b.ph[i0])));
  if(a.fell!==b.fell) gap=Infinity;
  ro('b_gap', isFinite(gap)? gap.toFixed(2)+'°':'one falls', gap>1?'warn':'ok');
  lastB={a,b,gap}; updateNote();
}
function updateNote(){
  if(!lastA||!lastB) return;
  const parts=[];
  const chA=[]; for(const k in DA){ if(A[k]!==DA[k]) chA.push(`${k} ${fa(k,A[k])||A[k]}`); }
  if(chA.length){
    const s=lastA;
    parts.push(`Contact/integrator study (one 1-D body dropped on the floor): ${chA.join(', ')} (defaults: MuJoCo-style soft contact, solref 0.02 s / 1, 2 ms, semi-implicit Euler, 41 kg, 20 cm). Result: ${s.blown?'UNSTABLE, the step is too large for the contact stiffness':`max penetration ${(s.maxPen*1000).toFixed(2)} mm, peak energy ${(s.Emax*100).toFixed(1)}% of the start${s.Emax>1.01?' (energy created by the integrator)':''}`}. Marvin Bot's MuJoCo checks use 2 ms with implicitfast; Isaac Lab training uses a 5 ms step on Newton's MuJoCo Warp solver.`);
  }
  const chB=[]; for(const k in DB){ if(B[k]!==DB[k]) chB.push(`${k} ${fb(k,B[k])||B[k]}`); }
  if(chB.length){
    const {a,b,gap}=lastB;
    parts.push(`Sim-to-sim gap study (sagittal foot + inverted pendulum, 41 kg, ankle PD, push for 0.15 s): ${chB.join(', ')}. Model A (${B.fA==='reg'?'regularised friction':'stick-slip'}, k ${B.kA.toExponential(1)} N/m, μ ${B.muA}) ${a.fell?'falls':'recovers'} with max lean ${a.maxLean.toFixed(1)}°, heel lift ${a.maxHeel.toFixed(1)} mm, slide ${a.slideEnd.toFixed(1)} mm; model B (${B.fB==='reg'?'regularised':'stick-slip'}, k ${B.kB.toExponential(1)} N/m, μ ${B.muB}) ${b.fell?'falls':'recovers'} with ${b.maxLean.toFixed(1)}°, ${b.maxHeel.toFixed(1)} mm, ${b.slideEnd.toFixed(1)} mm. Largest lean difference ${isFinite(gap)?gap.toFixed(2)+'°':'one falls, the other does not'}. Implication: contact parameters (stiffness, friction law, μ) should be randomised in training or checked in a second simulator before trusting the policy near this regime.`);
  }
  $('note').textContent = parts.length? 'Marvin Bot (1.44 m, 41 kg humanoid, simulation stage), physics simulation. '+parts.join('\n\n') : 'Change a control above to build a note.';
}
$('copy').addEventListener('click',()=>{ navigator.clipboard.writeText($('note').textContent).then(()=>{ const b=$('copy'); b.textContent='Copied!'; setTimeout(()=>b.textContent='Copy',1200); }); });
renderA(); renderB();
