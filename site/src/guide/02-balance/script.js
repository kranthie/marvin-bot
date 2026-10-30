const $ = id => document.getElementById(id);
const G=9.81, FL=0.24, FW=0.11, ANK2CEN=0.03, ANKX=-0.015; // foot centre = ankle + 0.03; ankle 15 mm behind the hip line in the stance
// ---------- Playground A ----------
const DA={mode:'both',width:0.20,stride:0,cx:0.004,cy:0,ax:0,ay:0,zh:0.71};
const A={...DA};
const PA={today:{...DA}, single:{mode:'left',width:0.20,stride:0,cx:0.01,cy:0.09,ax:0,ay:0,zh:0.71},
  walk:{mode:'both',width:0.20,stride:0.15,cx:0.02,cy:0.0,ax:0,ay:0,zh:0.71}, narrow:{mode:'both',width:0.12,stride:0,cx:0.004,cy:0,ax:0,ay:0,zh:0.71},
  accel:{mode:'both',width:0.20,stride:0,cx:0.004,cy:0,ax:2.0,ay:0,zh:0.71}};
function feet(){ const c=ANKX+ANK2CEN, h=A.stride/2, out=[];
  // world: f = forward, l = left
  if(A.mode!=='right') out.push({side:'L',f:c+h,l:A.width/2});
  if(A.mode!=='left') out.push({side:'R',f:c-h,l:-A.width/2});
  return out; }
function allFeet(){ const c=ANKX+ANK2CEN, h=A.stride/2; return [{side:'L',f:c+h,l:A.width/2,on:A.mode!=='right'},{side:'R',f:c-h,l:-A.width/2,on:A.mode!=='left'}]; }
function corners(ft){ return [[ft.f+FL/2,ft.l+FW/2],[ft.f+FL/2,ft.l-FW/2],[ft.f-FL/2,ft.l-FW/2],[ft.f-FL/2,ft.l+FW/2]]; }
function hull(pts){ pts=pts.slice().sort((a,b)=>a[0]-b[0]||a[1]-b[1]); const cr=(o,a,b)=>(a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0]);
  const lo=[],up=[]; for(const p of pts){ while(lo.length>=2&&cr(lo[lo.length-2],lo[lo.length-1],p)<=0) lo.pop(); lo.push(p);} for(const p of pts.slice().reverse()){ while(up.length>=2&&cr(up[up.length-2],up[up.length-1],p)<=0) up.pop(); up.push(p);} up.pop(); lo.pop(); return lo.concat(up); }
function segDist(p,a,b){ const dx=b[0]-a[0],dy=b[1]-a[1]; let t=((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy); t=Math.max(0,Math.min(1,t)); return Math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy); }
function signedMargin(p,H){ let inside=true, d=Infinity; for(let i=0;i<H.length;i++){ const a=H[i],b=H[(i+1)%H.length];
    const cr=(b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0]); if(cr<0) inside=false; d=Math.min(d,segDist(p,a,b)); } return inside?d:-d; }
function area(H){ let s=0; for(let i=0;i<H.length;i++){const a=H[i],b=H[(i+1)%H.length]; s+=a[0]*b[1]-b[0]*a[1];} return Math.abs(s)/2; }
const SA=620, AX0=300, AY0=230; // screen: left on screen-left => x_screen = AX0 - l*SA ; forward up => y = AY0 - f*SA
const sx=l=>AX0-l*SA, sy=f=>AY0-f*SA;
function renderA(){
  const F=feet(); const H=hull(F.flatMap(corners));
  const com=[A.cx,A.cy], zmp=[A.cx-A.zh/G*A.ax, A.cy-A.zh/G*A.ay];
  const m=signedMargin(com,H), mz=signedMargin(zmp,H);
  let s='';
  for(let g=-0.5;g<=0.5001;g+=0.1){ s+=`<line x1="${sx(g)}" y1="0" x2="${sx(g)}" y2="460" stroke="#1a222b"/>`; }
  for(let g=-0.3;g<=0.3801;g+=0.1){ s+=`<line x1="0" y1="${sy(g)}" x2="600" y2="${sy(g)}" stroke="#1a222b"/>`; }
  s+=`<text x="8" y="16" fill="#6b7785" font-size="11">grid 0.1 m · forward ↑ · robot's left ←</text>`;
  s+=`<polygon points="${H.map(p=>sx(p[1])+','+sy(p[0])).join(' ')}" fill="#4cc38a22" stroke="#4cc38a" stroke-width="1.5"/>`;
  allFeet().forEach(ft=>{ const c=corners(ft); s+=`<polygon points="${c.map(p=>sx(p[1])+','+sy(p[0])).join(' ')}" fill="${ft.on?'#ffb45444':'none'}" stroke="${ft.on?'#ffb454':'#56606c'}" stroke-dasharray="${ft.on?'':'5 4'}" stroke-width="2"/>`;
    s+=`<text x="${sx(ft.l)}" y="${sy(ft.f-FL/2)+14}" fill="#9aa7b4" font-size="11" text-anchor="middle">${ft.side}${ft.on?'':' (in the air)'}</text>`;
    s+=`<circle cx="${sx(ft.l)}" cy="${sy(ft.f-ANK2CEN)}" r="3" fill="#9aa7b4"/>`; });
  // hips
  s+=`<line x1="${sx(0.1)}" y1="${sy(0)}" x2="${sx(-0.1)}" y2="${sy(0)}" stroke="#3a4552" stroke-width="3"/>`;
  // zmp arrow
  if(A.ax||A.ay) s+=`<line x1="${sx(com[1])}" y1="${sy(com[0])}" x2="${sx(zmp[1])}" y2="${sy(zmp[0])}" stroke="#b48cff" stroke-dasharray="4 3"/>`;
  s+=`<rect x="${sx(zmp[1])-6}" y="${sy(zmp[0])-6}" width="12" height="12" transform="rotate(45 ${sx(zmp[1])} ${sy(zmp[0])})" fill="#b48cff"/>`;
  s+=`<circle id="comdot" cx="${sx(com[1])}" cy="${sy(com[0])}" r="10" fill="${m>0?'#4fb3ff':'#ff6b6b'}" stroke="#0c1015" stroke-width="2" style="cursor:grab"/>`;
  s+=`<text x="${sx(com[1])+14}" y="${sy(com[0])+4}" fill="#e6edf3" font-size="12">CoM</text>`;
  $('topview').innerHTML=s;
  const set=(id,txt,cls)=>{const e=$(id); e.querySelector('.v').textContent=txt; e.className='ro '+(cls||'');};
  set('r_m',`${(m*1000).toFixed(0)} mm`, m>0.02?'ok':m>0?'warn':'bad');
  set('r_zmp',`${zmp[0].toFixed(3)}, ${zmp[1].toFixed(3)} m`);
  set('r_zm',`${(mz*1000).toFixed(0)} mm`, mz>0.02?'ok':mz>0?'warn':'bad');
  set('r_area',`${(area(H)*1e4).toFixed(0)} cm²`);
  let v;
  if(m<=0) v=`<b style="color:var(--bad)">Not statically balanced.</b> The CoM projection is ${(-m*1000).toFixed(0)} mm outside the support polygon: standing still, the robot tips towards that side.`;
  else if(mz<=0) v=`<b style="color:var(--warn)">Statically fine, dynamically not.</b> The CoM is inside, but this acceleration needs the ZMP ${(-mz*1000).toFixed(0)} mm outside the feet. The feet would roll over an edge; the robot must accelerate less or step.`;
  else v=`<b style="color:var(--good)">Balanced.</b> Static margin ${(m*1000).toFixed(0)} mm${(A.ax||A.ay)?`, ZMP margin ${(mz*1000).toFixed(0)} mm`:''}.`;
  $('verdictA').innerHTML=v; updateNote();
}
function syncA(){ for(const k of ['width','stride','cx','cy','ax','ay','zh']){ $(k).value=A[k]; outA(k);} document.querySelectorAll('#mode button').forEach(b=>b.classList.toggle('on',b.dataset.m===A.mode)); }
function outA(k){ const v=A[k]; $('o_'+k).textContent = (k==='ax'||k==='ay')?`${(+v).toFixed(1)} m/s²`:`${(+v).toFixed(k==='cx'||k==='cy'?3:2)} m`; }
for(const k of ['width','stride','cx','cy','ax','ay','zh']){ $(k).addEventListener('input',()=>{A[k]=parseFloat($(k).value); outA(k); renderA();}); outA(k); }
document.querySelectorAll('#mode button').forEach(b=>b.addEventListener('click',()=>{A.mode=b.dataset.m; syncA(); renderA();}));
document.querySelectorAll('[data-preset]').forEach(b=>b.addEventListener('click',()=>{Object.assign(A,PA[b.dataset.preset]); syncA(); renderA();}));
(function(){ const svg=$('topview'); let drag=false;
  const toW=e=>{ const r=svg.getBoundingClientRect(); const x=(e.clientX-r.left)/r.width*600, y=(e.clientY-r.top)/r.height*460; return [(AY0-y)/SA,(AX0-x)/SA]; };
  svg.addEventListener('pointerdown',e=>{ drag=true; svg.setPointerCapture(e.pointerId); mv(e); });
  svg.addEventListener('pointermove',e=>{ if(drag) mv(e); });
  svg.addEventListener('pointerup',()=>drag=false);
  function mv(e){ const w=toW(e); A.cx=Math.max(-0.3,Math.min(0.3,+w[0].toFixed(3))); A.cy=Math.max(-0.3,Math.min(0.3,+w[1].toFixed(3))); $('cx').value=A.cx; $('cy').value=A.cy; outA('cx'); outA('cy'); renderA(); }
})();

// ---------- Playground B ----------
const DB={mass:41.3,z:0.71,J:30,dir:'fwd',strat:'step',ts:0.30,smax:0.40};
const B={...DB};
const PB={today:{...DB}, ankle:{...DB,strat:'ankle'}, max:{...DB,J:20,strat:'ankle'}, train:{...DB,J:20.65,strat:'ankle'}, side:{...DB,dir:'side'}};
// distances from the CoM (stance default) to the sole edges, in the push direction
function edges(){ const comx=0.004, cen=ANKX+ANK2CEN;
  if(B.dir==='fwd') return {front:cen+FL/2-comx, back:comx-(cen-FL/2), half:FL/2, label:'toes'};
  if(B.dir==='back') return {front:comx-(cen-FL/2), back:cen+FL/2-comx, half:FL/2, label:'heels'};
  return {front:0.10+FW/2, back:0.10+FW/2, half:FW/2, label:'outer edge of the far foot'}; }
let traj=null;
function simulate(){
  const w=Math.sqrt(G/B.z), v0=B.J/B.mass, E=edges();
  let x=0, v=v0, lo=-E.back, hi=E.front, t=0, dt=0.0005, stepped=false, out=[], status='', stepLen=null, stepNeeded=null;
  const cen0=(hi+lo)/2; let footC=cen0, footHalf=(hi-lo)/2;
  // in side mode, double support spans both feet; after a step the new single foot is only FW wide
  while(t<2.5){
    let xi=x+v/w;
    if(B.strat==='step' && !stepped && t>=B.ts){
      stepped=true;
      if(xi>hi || xi<lo){ stepNeeded=xi-cen0; const wantC = B.dir==='side' ? xi : xi; // place sole centre on the capture point
        let d=wantC-cen0; const dd=Math.sign(d)*Math.min(Math.abs(d),B.smax); stepLen=dd;
        footC=cen0+dd; footHalf=E.half; lo=footC-footHalf; hi=footC+footHalf;
        if(B.dir==='side'){ /* new foot replaces the polygon: sole centred where it landed */ }
      } else { stepNeeded=0; stepLen=0; }
    }
    const p=Math.max(lo,Math.min(hi,xi));
    out.push([t,x,xi,p,lo,hi]);
    if(Math.abs(x-p)>0.45){ status='fell'; break; }
    const a=w*w*(x-p); x+=v*dt+0.5*a*dt*dt; v+=a*dt; t+=dt;
  }
  if(!status){ const last=out[out.length-1]; status=(last[2]>=last[4]-1e-6&&last[2]<=last[5]+1e-6)?'captured':'fell'; }
  traj={w,v0,E,out,status,stepLen,stepNeeded,cen0};
}
const LX=v=>300+v*560, GY=200; // scene
function drawB(k){
  const T=traj, row=T.out[Math.min(k,T.out.length-1)], [t,x,xi,p,lo,hi]=row;
  const zpx=Math.min(170,B.z*190);
  let s='';
  s+=`<rect x="0" y="${GY}" width="600" height="6" fill="#2a3340"/>`;
  s+=`<text x="8" y="16" fill="#6b7785" font-size="11">side view · push ${B.dir==='side'?'sideways (frontal plane)':B.dir==='back'?'backward (shown left to right)':'forward →'} · t = ${t.toFixed(2)} s</text>`;
  // original foot (edges) and new foot
  const E=T.E; s+=`<rect x="${LX(-E.back)}" y="${GY-8}" width="${(E.front+E.back)*560}" height="8" fill="${t<B.ts||B.strat==='ankle'||!T.stepLen?'#ffb45466':'#ffb45422'}" stroke="#ffb454"/>`;
  if(B.strat==='step' && t>=B.ts && T.stepLen) s+=`<rect x="${LX(lo)}" y="${GY-8}" width="${(hi-lo)*560}" height="8" fill="#ffb45488" stroke="#ffb454"/><text x="${LX((lo+hi)/2)}" y="${GY+22}" fill="#ffb454" font-size="11" text-anchor="middle">step ${(T.stepLen*100).toFixed(0)} cm</text>`;
  // capture point
  s+=`<line x1="${LX(xi)}" y1="${GY-zpx-20}" x2="${LX(xi)}" y2="${GY}" stroke="#ffb454" stroke-dasharray="4 4"/><polygon points="${LX(xi)},${GY-1} ${LX(xi)-7},${GY+9} ${LX(xi)+7},${GY+9}" fill="#ffb454"/><text x="${LX(xi)+8}" y="${GY-zpx-8}" fill="#ffb454" font-size="11">capture point ξ</text>`;
  // pendulum
  const ang=Math.atan2(x-p,B.z); const fallen=T.status==='fell'&&k>=T.out.length-1;
  const cx=fallen?LX(p)+Math.sin(Math.sign(x-p)*1.2)*zpx:LX(x), cy=fallen?GY-Math.cos(1.2)*zpx:GY-zpx;
  s+=`<line x1="${LX(p)}" y1="${GY}" x2="${cx}" y2="${cy}" stroke="#cbd5df" stroke-width="5" stroke-linecap="round"/>`;
  s+=`<circle cx="${cx}" cy="${cy}" r="14" fill="#4fb3ff" stroke="#0c1015" stroke-width="2"/><text x="${cx}" y="${cy+4}" fill="#0c1015" font-size="10" text-anchor="middle" font-weight="700">CoM</text>`;
  s+=`<circle cx="${LX(p)}" cy="${GY}" r="5" fill="#b48cff"/>`;
  if(k<12) s+=`<path d="M ${LX(x)-70} ${GY-zpx} l 45 0" stroke="#ff6b6b" stroke-width="4" marker-end="url(#ar)"/><text x="${LX(x)-74}" y="${GY-zpx-8}" fill="#ff6b6b" font-size="12">${B.J} N·s</text>`;
  s+=`<defs><marker id="ar" markerWidth="8" markerHeight="8" refX="4" refY="4" orient="auto"><path d="M0,0 L8,4 L0,8 z" fill="#ff6b6b"/></marker></defs>`;
  // plot
  const P0=250, PH=160, tmax=2.5, X=t=>40+t/tmax*545, Yp=v=>P0+PH/2-v*PH/1.2;
  s+=`<rect x="40" y="${P0}" width="545" height="${PH}" fill="#0f141a" stroke="#24303c"/>`;
  for(const v of [-0.4,0,0.4]) s+=`<line x1="40" x2="585" y1="${Yp(v)}" y2="${Yp(v)}" stroke="#1e2731"/><text x="34" y="${Yp(v)+4}" fill="#6b7785" font-size="10" text-anchor="end">${v.toFixed(1)}</text>`;
  s+=`<text x="44" y="${P0+12}" fill="#6b7785" font-size="10">position (m) vs time (0–2.5 s)</text>`;
  const pathOf=(i,c,dash)=>{ let d=''; for(let j=0;j<T.out.length;j+=20){ const r=T.out[j]; const v=Math.max(-0.6,Math.min(0.6,r[i])); d+=(j?'L':'M')+X(r[0]).toFixed(1)+' '+Yp(v).toFixed(1); } return `<path d="${d}" fill="none" stroke="${c}" stroke-width="2" ${dash?'stroke-dasharray="4 3"':''}/>`; };
  s+=pathOf(5,'#ffb45488',true)+pathOf(3,'#b48cff')+pathOf(2,'#ffb454')+pathOf(1,'#4fb3ff');
  if(B.strat==='step') s+=`<line x1="${X(B.ts)}" x2="${X(B.ts)}" y1="${P0}" y2="${P0+PH}" stroke="#56606c" stroke-dasharray="3 3"/><text x="${X(B.ts)+3}" y="${P0+PH-4}" fill="#6b7785" font-size="10">touchdown</text>`;
  s+=`<line x1="${X(t)}" x2="${X(t)}" y1="${P0}" y2="${P0+PH}" stroke="#e6edf3" opacity=".5"/>`;
  $('lipm').innerHTML=s;
}
let anim=null;
function play(){ if(anim) cancelAnimationFrame(anim); const n=traj.out.length, t0=performance.now(), dur=traj.out[n-1][0]*1000;
  const f=now=>{ const fr=Math.min(1,(now-t0)/Math.max(dur,1)); drawB(Math.floor(fr*(n-1))); if(fr<1) anim=requestAnimationFrame(f); }; anim=requestAnimationFrame(f); }
function renderB(animate){
  simulate(); const T=traj, E=T.E;
  const off=T.v0/T.w, jmax=B.mass*T.w*E.front;
  const set=(id,txt,cls)=>{const e=$(id); e.querySelector('.v').textContent=txt; e.className='ro '+(cls||'');};
  set('r_dv',`${T.v0.toFixed(2)} m/s`);
  set('r_w',`${T.w.toFixed(2)} s⁻¹ · ${(1/T.w).toFixed(2)} s`);
  set('r_cp',`${(off*1000).toFixed(0)} mm`, off<E.front?'ok':'bad');
  set('r_edge',`${(E.front*1000).toFixed(0)} mm`);
  set('r_jmax',`${jmax.toFixed(1)} N·s`);
  // capture point at touchdown with CoP held at the edge
  const xiT = off<=E.front ? off : E.front+(off-E.front)*Math.exp(T.w*B.ts);
  const need = off<=E.front ? 0 : xiT-T.cen0;
  set('r_step', off<=E.front?'none':`${(need*100).toFixed(0)} cm`, off<=E.front?'ok':need<=B.smax?'warn':'bad');
  $('dirhint').textContent=`Distance from the CoM to the ${E.label}: ${(E.front*1000).toFixed(0)} mm (from the stance; ${B.dir==='side'?'both feet down, 0.20 m apart':'feet side by side'}).`;
  let v;
  if(T.status==='captured' && !T.stepLen) v=`<b style="color:var(--good)">Recovered without stepping.</b> The capture point stays on the sole; the ankle shifts the CoP onto it and the CoM settles above it.`;
  else if(T.status==='captured') v=`<b style="color:var(--good)">Recovered with one step</b> of ${(T.stepLen*100).toFixed(0)} cm (foot-centre travel) landing at ${B.ts.toFixed(2)} s.`;
  else if(B.strat==='ankle') v=`<b style="color:var(--bad)">Falls.</b> The capture point is ${((off-E.front)*1000).toFixed(0)} mm beyond the ${E.label}; no CoP on this foot can stop it. A step is needed.`;
  else v=`<b style="color:var(--bad)">Falls.</b> By touchdown the capture point is ${(need*100).toFixed(0)} cm away, more than the ${(B.smax*100).toFixed(0)} cm step allowed. Step sooner, or take a second step.`;
  $('verdictB').innerHTML=v+` <span style="color:var(--muted)">(Linear inverted pendulum: point mass, constant height, ideal ankle; real recovery is somewhat worse.)</span>`;
  if(animate) play(); else drawB(traj.out.length-1);
  updateNote();
}
function outB(k){ const v=B[k]; $('o_'+k).textContent = k==='mass'?`${(+v).toFixed(2)} kg`: k==='J'?`${(+v).toFixed(1)} N·s`: k==='ts'?`${(+v).toFixed(2)} s`:`${(+v).toFixed(2)} m`; }
function syncB(){ for(const k of ['mass','z','J','ts','smax']){ $(k).value=B[k]; outB(k);} document.querySelectorAll('#dir button').forEach(b=>b.classList.toggle('on',b.dataset.d===B.dir)); document.querySelectorAll('#strat button').forEach(b=>b.classList.toggle('on',b.dataset.s===B.strat)); }
for(const k of ['mass','z','J','ts','smax']){ $(k).addEventListener('input',()=>{B[k]=parseFloat($(k).value); outB(k); renderB(false);}); $(k).addEventListener('change',()=>renderB(true)); outB(k); }
document.querySelectorAll('#dir button').forEach(b=>b.addEventListener('click',()=>{B.dir=b.dataset.d; syncB(); renderB(true);}));
document.querySelectorAll('#strat button').forEach(b=>b.addEventListener('click',()=>{B.strat=b.dataset.s; syncB(); renderB(true);}));
document.querySelectorAll('[data-p2]').forEach(b=>b.addEventListener('click',()=>{Object.assign(B,PB[b.dataset.p2]); syncB(); renderB(true);}));
$('play').addEventListener('click',()=>play());

function updateNote(){
  const parts=[];
  const chA=Object.keys(DA).filter(k=>A[k]!==DA[k]);
  if(chA.length){ const F=feet(); const H=hull(F.flatMap(corners)); const m=signedMargin([A.cx,A.cy],H), mz=signedMargin([A.cx-A.zh/G*A.ax,A.cy-A.zh/G*A.ay],H);
    parts.push(`Support check: ${A.mode==='both'?'both feet':A.mode+' foot only'}, step width ${A.width.toFixed(2)} m, left foot ${A.stride.toFixed(2)} m ahead, CoM at (${A.cx.toFixed(3)} fwd, ${A.cy.toFixed(3)} left) m from the hip midpoint${(A.ax||A.ay)?`, accelerating (${A.ax}, ${A.ay}) m/s² at CoM height ${A.zh} m`:''}: static margin ${(m*1000).toFixed(0)} mm${(A.ax||A.ay)?`, ZMP margin ${(mz*1000).toFixed(0)} mm`:''}.`); }
  const chB=Object.keys(DB).filter(k=>B[k]!==DB[k]);
  if(chB.length && traj){ const off=traj.v0/traj.w;
    parts.push(`Push check (linear inverted pendulum): ${B.J} N·s ${B.dir==='side'?'sideways':B.dir==='back'?'backward':'forward'} on ${B.mass} kg with the CoM at ${B.z} m gives ${traj.v0.toFixed(2)} m/s and a capture point ${(off*1000).toFixed(0)} mm ahead of the CoM, against ${(traj.E.front*1000).toFixed(0)} mm to the sole edge. Strategy: ${B.strat==='ankle'?'ankle only':`one step landing after ${B.ts} s, at most ${B.smax} m`}. Result: ${traj.status==='captured'?'recovers':'falls'}${traj.stepLen?`, step ${(traj.stepLen*100).toFixed(0)} cm`:''}. For comparison, R14 asks for 30 N·s and training pushes reach 0.5 m/s (~21 N·s).`); }
  $('note').textContent = parts.length ? `Marvin Bot balance (41.3 kg with shells, CoM ~0.71 m in the stance, feet 0.24 × 0.11 m, hips 0.20 m apart).\n`+parts.join('\n') : 'Change a control above to build a note.';
}
$('copy').addEventListener('click',()=>{ navigator.clipboard.writeText($('note').textContent).then(()=>{ const b=$('copy'); b.textContent='Copied!'; setTimeout(()=>b.textContent='Copy',1200); }); });
renderA(); renderB(false);
