const $ = id => document.getElementById(id);
const NS='http://www.w3.org/2000/svg', D2R=Math.PI/180, TAU=2*Math.PI;
const MOTORS={RS03:{peak:60,rated:20,kg:0.90,dia:0.100},RS06:{peak:36,rated:11,kg:0.621,dia:0.085},RS04:{peak:120,rated:40,kg:1.42,dia:0.120}};
const DEFAULTS={pitch:0,roll:0,r:60,back:60,side:40,bx:40,model:'RS03',dp:60,dr:30};
const state={...DEFAULTS};
const PRESETS={
 today:{...DEFAULTS},
 worst:{...DEFAULTS,pitch:-40,roll:20},
 narrow:{...DEFAULTS,side:25},
 short:{...DEFAULTS,back:35},
 calf:{...DEFAULTS,bx:-50},
 rs06:{...DEFAULTS,model:'RS06'}
};
const FIX={crank_y:0.041,below:0.010,ax:-0.010,az:0.080,bz:0.180,knee_z:0.315,tilt:0};
const rem=x=>x-TAU*Math.round(x/TAU);
function rotY(a,v){const c=Math.cos(a),s=Math.sin(a);return [c*v[0]+s*v[2],v[1],-s*v[0]+c*v[2]];}
function rotX(a,v){const c=Math.cos(a),s=Math.sin(a);return [v[0],c*v[1]-s*v[2],s*v[1]+c*v[2]];}
const sub=(a,b)=>[a[0]-b[0],a[1]-b[1],a[2]-b[2]], dot=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2], norm=a=>Math.sqrt(dot(a,a));
// Linkage, as tools/ankle_linkage.py (left ankle, shin frame: x forward, y left, z up, origin on the ankle axis)
function makeLinkage(g){
 const L={r:g.r};
 L.motor=[[g.ax,FIX.crank_y,g.az],[g.bx,-FIX.crank_y,g.bz]];
 L.attach=[[-g.back,g.side,-FIX.below],[-g.back,-g.side,-FIX.below]];
 L.theta0=Math.PI+FIX.tilt;
 L.tip=(i,t)=>[L.motor[i][0]+L.r*Math.cos(t),L.motor[i][1],L.motor[i][2]+L.r*Math.sin(t)];
 L.foot=(i,p,r)=>rotY(-p,rotX(r,L.attach[i]));
 L.length=[0,1].map(i=>norm(sub(L.tip(i,L.theta0),L.attach[i])));
 L.crank=(i,p,r,guess)=>{ const a=L.foot(i,p,r), d=sub(a,L.motor[i]);
   const P=-2*L.r*d[0], Q=-2*L.r*d[2], K=L.length[i]**2-dot(d,d)-L.r**2, amp=Math.hypot(P,Q);
   if(amp<1e-12||Math.abs(K)>amp) return null;
   const base=Math.atan2(Q,P), spread=Math.acos(K/amp); const s=[base+spread,base-spread];
   return Math.abs(rem(s[0]-guess))<=Math.abs(rem(s[1]-guess))?s[0]:s[1]; };
 L.tx=(i,t,p,r)=>{ const tan=[-Math.sin(t),0,Math.cos(t)], rod=sub(L.foot(i,p,r),L.tip(i,t)); return Math.abs(dot(tan,rod))/norm(rod); };
 L.jac=(p,r,guess,h=1e-5)=>{ const j=[[0,0],[0,0]]; const steps=[[h,0],[0,h]];
   for(let k=0;k<2;k++){ for(let i=0;i<2;i++){ const hi=L.crank(i,p+steps[k][0],r+steps[k][1],guess[i]), lo=L.crank(i,p-steps[k][0],r-steps[k][1],guess[i]);
     if(hi===null||lo===null) return null; j[i][k]=rem(hi-lo)/(2*h);} } return j; };
 return L;
}
function geom(){ return {r:state.r/1000,back:state.back/1000,side:state.side/1000,ax:FIX.ax,az:FIX.az,bx:state.bx/1000,bz:FIX.bz}; }
const RANGE={p:[-40*D2R,40*D2R],r:[-20*D2R,20*D2R]};
function evaluate(L,mot,demand,steps=17){
 let worst=1,cp=[Infinity,Infinity],cr=[Infinity,Infinity],rf=0,lo=Infinity,hi=-Infinity,wpose=null;
 for(let a=0;a<steps;a++) for(let b=0;b<steps;b++){
   const p=RANGE.p[0]+(RANGE.p[1]-RANGE.p[0])*a/(steps-1), r=RANGE.r[0]+(RANGE.r[1]-RANGE.r[0])*b/(steps-1);
   const th=[0,1].map(i=>L.crank(i,p,r,L.theta0)); if(th.includes(null)) return null;
   for(let i=0;i<2;i++){ const t=L.tx(i,th[i],p,r); if(t<worst){worst=t;wpose=[p,r];} const dt=rem(th[i]-L.theta0); lo=Math.min(lo,dt); hi=Math.max(hi,dt); }
   const J=L.jac(p,r,th); if(!J) return null;
   for(let k=0;k<2;k++){ const col=Math.abs(J[0][k])+Math.abs(J[1][k]); cp[k]=Math.min(cp[k],mot.peak*col); cr[k]=Math.min(cr[k],mot.rated*col); }
   const m=solveT(J,demand); if(!m) return null;
   for(let i=0;i<2;i++){ rf=Math.max(rf,Math.abs(m[i])/(L.r*Math.max(L.tx(i,th[i],p,r),1e-6))); }
 }
 const J0=L.jac(0,0,[L.theta0,L.theta0]);
 return {worst,wpose,peak:cp,rated:cr,rod:rf,swing:[lo,hi],ratios:[(Math.abs(J0[0][0])+Math.abs(J0[1][0]))/2,(Math.abs(J0[0][1])+Math.abs(J0[1][1]))/2]};
}
function solveT(J,d){ // solve J^T m = d
 const a=J[0][0],b=J[1][0],c=J[0][1],e=J[1][1]; const det=a*e-b*c; if(Math.abs(det)<1e-12) return null;
 return [(d[0]*e-b*d[1])/det,(a*d[1]-c*d[0])/det]; }
function el(tag,attrs,parent,text){ const e=document.createElementNS(NS,tag); for(const k in attrs) e.setAttribute(k,attrs[k]); if(text!==undefined) e.textContent=text; if(parent) parent.appendChild(e); return e; }
const fmt=(x,d=0)=>Number(x).toFixed(d);
function setRO(id,v,cls){ const e=$(id); e.className='ro'+(cls?' '+cls:''); e.querySelector('.v').textContent=v; }

let cache={key:null,res:null};
function render(){
 const g=geom(), L=makeLinkage(g), mot=MOTORS[state.model], p=state.pitch*D2R, r=state.roll*D2R, demand=[state.dp,state.dr];
 const key=JSON.stringify([g,state.model,demand]);
 if(cache.key!==key){ cache={key,res:evaluate(L,mot,demand)}; }
 const W=cache.res;
 const th=[0,1].map(i=>L.crank(i,p,r,L.theta0));
 const ok=!th.includes(null);
 const J= ok? L.jac(p,r,th):null;
 drawViews(L,p,r,th,ok,mot);
 drawMaps(L,J,mot,demand,p,r);
 // pose readouts
 const st=$('status');
 const clash = Math.hypot(g.bx-g.ax,g.bz-g.az) < mot.dia+0.006;
 if(!ok||!J){ st.className='statusline bad'; st.textContent='This pose cannot be reached: a rod is too short or too long to close the loop.'; ['r1','r2','r3','r4','r5','r6'].forEach(id=>setRO(id,'–')); }
 else {
  const tx=[0,1].map(i=>L.tx(i,th[i],p,r));
  const capP=[mot.rated,mot.peak].map(l=>l*(Math.abs(J[0][0])+Math.abs(J[1][0])));
  const capR=[mot.rated,mot.peak].map(l=>l*(Math.abs(J[0][1])+Math.abs(J[1][1])));
  const m=solveT(J,demand); const F=m? [0,1].map(i=>Math.abs(m[i])/(L.r*Math.max(tx[i],1e-6))):null;
  setRO('r1',`${fmt(rem(th[0]-L.theta0)/D2R)}° / ${fmt(rem(th[1]-L.theta0)/D2R)}°`);
  const mt=Math.min(...tx); setRO('r2',`${fmt(tx[0],2)} / ${fmt(tx[1],2)}`, mt>=0.7?'ok':mt>=0.5?'warn':'bad');
  setRO('r3',`${fmt(capP[0])} / ${fmt(capP[1])} N·m`);
  setRO('r4',`${fmt(capR[0])} / ${fmt(capR[1])} N·m`);
  setRO('r5', F? `${fmt(F[0])} / ${fmt(F[1])} N` : 'singular', F&&Math.max(...F)<=1900?'ok':'warn');
  $('r6').querySelector('.v').textContent=`[${fmt(J[0][0],2)}, ${fmt(J[0][1],2)}]\n[${fmt(J[1][0],2)}, ${fmt(J[1][1],2)}]`;
  $('r6').querySelector('.v').style.whiteSpace='pre';
  const mm=m? m.map(x=>Math.abs(x)):[Infinity,Infinity];
  let msg=`Motor torques for the ${fmt(state.dp)} / ${fmt(state.dr)} N·m demand here: a ${m?fmt(m[0],1):'–'} N·m, b ${m?fmt(m[1],1):'–'} N·m`;
  let cls='ok';
  if(Math.max(...mm)>mot.peak){cls='bad'; msg+=` — beyond the ${state.model}'s ${mot.peak} N·m peak.`;}
  else if(Math.max(...mm)>0.7*mot.peak){cls='warn'; msg+=` — above 70% of peak (the repo's rule for peaks).`;}
  else msg+=` — within 70% of peak.`;
  if(mt<0.5){cls='bad'; msg+=' Transmission below 0.5: near a singularity.';}
  if(clash){cls=cls==='ok'?'warn':cls; msg+=' The two motor housings overlap at this spacing.';}
  st.className='statusline '+cls; st.textContent=msg;
 }
 if(!W){ ['w1','w2','w3','w4','w5','w6'].forEach(id=>setRO(id,'–','bad')); setRO('w3','unreachable','bad'); }
 else {
  setRO('w1',`${fmt(W.rated[0])} / ${fmt(W.peak[0])} N·m`);
  setRO('w2',`${fmt(W.rated[1])} / ${fmt(W.peak[1])} N·m`);
  setRO('w3',`${fmt(W.worst,2)}`, W.worst>=0.6?'ok':W.worst>=0.5?'warn':'bad');
  setRO('w4',`${fmt(W.rod)} N`, W.rod<=1900?'ok':W.rod<=2500?'warn':'bad');
  setRO('w5',`${fmt(W.ratios[0],2)} / ${fmt(W.ratios[1],2)}`);
  setRO('w6',`${fmt(L.length[0]*1000)}, ${fmt(L.length[1]*1000)} mm; ${fmt(W.swing[0]/D2R)}°..${fmt(W.swing[1]/D2R)}°`);
 }
 updateNote(W);
}
function drawViews(L,p,r,th,ok,mot){
 const svg=$('view'); svg.innerHTML='';
 // side view: x forward → screen right; z up. Rear view: robot's left (+y) → screen left.
 const s=760, sx0=180, sy0=300, rx0=480;
 const S=(v)=>[sx0+v[0]*s, sy0-v[2]*s], Rv=(v)=>[rx0-v[1]*s, sy0-v[2]*s];
 el('text',{x:20,y:20,fill:'#9aa7b4','font-size':12},svg,'SIDE (toes to the right)');
 el('text',{x:350,y:20,fill:'#9aa7b4','font-size':12},svg,'REAR (robot\'s left on the left)');
 el('line',{x1:330,x2:330,y1:10,y2:350,stroke:'#1f2833'},svg);
 const line=(f,a,b,attrs)=>{ const A=f(a),B=f(b); return el('line',Object.assign({x1:A[0],y1:A[1],x2:B[0],y2:B[1]},attrs),svg); };
 const poly=(f,pts,attrs)=>el('polygon',Object.assign({points:pts.map(v=>f(v).join(',')).join(' ')},attrs),svg);
 // foot plate (sole 0.06 below the ankle axis), rotated
 const foot=(x,y,z)=>rotY(-p,rotX(r,[x,y,z]));
 const sideFoot=[[-0.09,0,-0.03],[0.15,0,-0.03],[0.15,0,-0.06],[-0.09,0,-0.06]].map(v=>foot(...v));
 const rearFoot=[[0,0.055,-0.03],[0,-0.055,-0.03],[0,-0.055,-0.06],[0,0.055,-0.06]].map(v=>foot(...v));
 // ground line
 el('line',{x1:20,x2:320,y1:S([0,0,-0.06])[1]+ (0),y2:S([0,0,-0.06])[1],stroke:'#2a3340','stroke-dasharray':'4 4'},svg);
 el('line',{x1:340,x2:630,y1:S([0,0,-0.06])[1],y2:S([0,0,-0.06])[1],stroke:'#2a3340','stroke-dasharray':'4 4'},svg);
 // shin (side): from ankle up to knee
 poly(S,[[-0.03,0,0],[0.03,0,0],[0.03,0,0.33],[-0.03,0,0.33]],{fill:'#1c232c',stroke:'#2a3340'});
 poly(Rv,[[0,0.047,0.02],[0,-0.047,0.02],[0,-0.047,0.33],[0,0.047,0.33]],{fill:'#1c232c',stroke:'#2a3340'});
 // knee motor
 const kc=S([0,0,FIX.knee_z]); el('circle',{cx:kc[0],cy:kc[1],r:0.06*s,fill:'none',stroke:'#3a4656','stroke-dasharray':'3 3'},svg);
 el('text',{x:kc[0]+0.065*s,y:kc[1]+4,fill:'#5b6b7c','font-size':10},svg,'knee RS04');
 poly(S,sideFoot,{fill:'#24303c',stroke:'#9aa7b4'}); poly(Rv,rearFoot,{fill:'#24303c',stroke:'#9aa7b4'});
 // heel posts: rod-end brackets from the foot top up to the rod ends
 for(let i=0;i<2;i++){ const A=L.foot(i,p,r), B=foot(L.attach[i][0],L.attach[i][1],-0.03); line(S,A,B,{stroke:'#9aa7b4','stroke-width':3}); line(Rv,A,B,{stroke:'#9aa7b4','stroke-width':3}); }
 // ankle axis
 const a0=S([0,0,0]), a1=Rv([0,0,0]);
 el('circle',{cx:a0[0],cy:a0[1],r:4,fill:'#e6edf3'},svg); el('circle',{cx:a1[0],cy:a1[1],r:4,fill:'#e6edf3'},svg);
 const cols=['#4fb3ff','#ffb454'];
 for(let i=0;i<2;i++){
   const M=L.motor[i], c=S(M);
   el('circle',{cx:c[0],cy:c[1],r:mot.dia/2*s,fill:'rgba(79,179,255,0.06)',stroke:cols[i],'stroke-width':1.2,opacity:.8},svg);
   el('text',{x:c[0]+mot.dia/2*s+3,y:c[1]-mot.dia/2*s+10,fill:cols[i],'font-size':11},svg,i?'b (high)':'a (low)');
   // rear view motor: a box along y (thickness ~56 mm) on the side opposite its crank
   const yc=M[1], sgn=Math.sign(yc)||1; const bodyY0=yc-sgn*0.006, bodyY1=yc-sgn*0.062;
   poly(Rv,[[0,bodyY0,M[2]+mot.dia/2],[0,bodyY1,M[2]+mot.dia/2],[0,bodyY1,M[2]-mot.dia/2],[0,bodyY0,M[2]-mot.dia/2]],{fill:'rgba(79,179,255,0.05)',stroke:cols[i],opacity:.7});
   if(ok){
     const T=L.tip(i,th[i]), A=L.foot(i,p,r);
     line(S,M,T,{stroke:cols[i],'stroke-width':5,'stroke-linecap':'round'});
     line(S,T,A,{stroke:cols[i],'stroke-width':2.5});
     line(Rv,M,T,{stroke:cols[i],'stroke-width':5,'stroke-linecap':'round'});
     line(Rv,T,A,{stroke:cols[i],'stroke-width':2.5});
     [S,Rv].forEach(f=>{ const t=f(T),aa=f(A); el('circle',{cx:t[0],cy:t[1],r:3.5,fill:'#0c1015',stroke:cols[i]},svg); el('circle',{cx:aa[0],cy:aa[1],r:3.5,fill:'#0c1015',stroke:cols[i]},svg); });
   }
   el('circle',{cx:c[0],cy:c[1],r:3,fill:cols[i]},svg);
 }
 el('text',{x:20,y:345,fill:'#9aa7b4','font-size':11},svg,`pitch ${state.pitch}°, roll ${state.roll}°`);
}
function drawMaps(L,J,mot,demand,p,r){
 const svg=$('maps'); svg.innerHTML='';
 // left: transmission map
 const x0=46,y0=20,w=250,h=200, np=17, nr=9;
 const cw=w/np, ch=h/nr;
 for(let a=0;a<np;a++) for(let b=0;b<nr;b++){
   const pp=RANGE.p[0]+(RANGE.p[1]-RANGE.p[0])*a/(np-1), rr=RANGE.r[0]+(RANGE.r[1]-RANGE.r[0])*b/(nr-1);
   const th=[0,1].map(i=>L.crank(i,pp,rr,L.theta0));
   let fill='#000';
   if(!th.includes(null)){ const t=Math.min(L.tx(0,th[0],pp,rr),L.tx(1,th[1],pp,rr));
     fill= t<0.5? `rgb(255,${Math.round(107*t/0.5)},107)` : t<0.7? '#ffb454' : `rgba(76,195,138,${0.35+0.65*(t-0.7)/0.3})`; }
   el('rect',{x:x0+a*cw,y:y0+(nr-1-b)*ch,width:cw-1,height:ch-1,fill},svg);
 }
 const px=x0+(p-RANGE.p[0])/(RANGE.p[1]-RANGE.p[0])*(w-cw)+cw/2, py=y0+(1-(r-RANGE.r[0])/(RANGE.r[1]-RANGE.r[0]))*(h-ch)+ch/2;
 el('circle',{cx:px,cy:py,r:6,fill:'none',stroke:'#e6edf3','stroke-width':2},svg);
 el('text',{x:x0+w/2,y:y0+h+16,fill:'#9aa7b4','font-size':11,'text-anchor':'middle'},svg,'pitch −40° … +40°');
 el('text',{x:14,y:y0+h/2,fill:'#9aa7b4','font-size':11,'text-anchor':'middle',transform:`rotate(-90 14 ${y0+h/2})`},svg,'roll −20° … +20°');
 el('text',{x:x0,y:y0+h+34,fill:'#9aa7b4','font-size':10},svg,'green ≥ 0.7 · orange 0.5-0.7 · red < 0.5 · black unreachable');
 // right: torque map
 const X0=330,Y0=20,WW=290,HH=230; const cx=X0+WW/2, cy=Y0+HH/2;
 const sim={p:2*mot.peak*0.99,r:2*mot.peak*0.67};
 const lim=Math.max(sim.p, demand[0], J? mot.peak*(Math.abs(J[0][0])+Math.abs(J[1][0])):0)*1.12;
 const limr=Math.max(sim.r, demand[1], J? mot.peak*(Math.abs(J[0][1])+Math.abs(J[1][1])):0)*1.12;
 const sc=Math.min((WW/2)/lim,(HH/2)/limr);
 const T=(tp,tr)=>[cx+tp*sc, cy-tr*sc];
 el('line',{x1:X0,x2:X0+WW,y1:cy,y2:cy,stroke:'#2a3340'},svg); el('line',{x1:cx,x2:cx,y1:Y0,y2:Y0+HH,stroke:'#2a3340'},svg);
 el('text',{x:X0+WW,y:cy+14,fill:'#9aa7b4','font-size':10,'text-anchor':'end'},svg,'pitch torque →');
 el('text',{x:cx+4,y:Y0+10,fill:'#9aa7b4','font-size':10},svg,'roll torque ↑');
 const b0=T(-sim.p,-sim.r), b1=T(sim.p,sim.r);
 el('rect',{x:b0[0],y:b1[1],width:b1[0]-b0[0],height:b0[1]-b1[1],fill:'none',stroke:'#b48cff','stroke-dasharray':'5 4','stroke-width':1.5},svg);
 if(J){
   const corners=l=>[[l,l],[l,-l],[-l,-l],[-l,l]].map(([ta,tb])=>T(J[0][0]*ta+J[1][0]*tb, J[0][1]*ta+J[1][1]*tb));
   el('polygon',{points:corners(mot.peak).map(c=>c.join(',')).join(' '),fill:'rgba(76,195,138,.12)',stroke:'#4cc38a','stroke-width':1.8},svg);
   el('polygon',{points:corners(mot.rated).map(c=>c.join(',')).join(' '),fill:'rgba(46,107,79,.5)',stroke:'#2e6b4f'},svg);
 }
 const dpt=T(demand[0],demand[1]); el('circle',{cx:dpt[0],cy:dpt[1],r:5,fill:'#ff6b6b'},svg);
 el('text',{x:X0+4,y:Y0+HH+16,fill:'#9aa7b4','font-size':10},svg,`box ±${fmt(sim.p)} / ±${fmt(sim.r)} N·m (simulator, neutral ratios)`);
}
function updateNote(W){
 const ch=[];
 if(state.r!==DEFAULTS.r) ch.push(`crank radius ${state.r} mm (repo 60)`);
 if(state.back!==DEFAULTS.back) ch.push(`rod ends ${state.back} mm behind the ankle axis (repo 60)`);
 if(state.side!==DEFAULTS.side) ch.push(`rod ends ±${state.side} mm beside it (repo ±40)`);
 if(state.bx!==DEFAULTS.bx) ch.push(`the upper motor ${state.bx} mm forward of the shin axis (repo 40)`);
 if(state.model!==DEFAULTS.model) ch.push(`two ${state.model} motors instead of two RS03`);
 const pose=(state.pitch!==0||state.roll!==0)? ` I looked at the pose pitch ${state.pitch}°, roll ${state.roll}°.`:'';
 const dem=(state.dp!==DEFAULTS.dp||state.dr!==DEFAULTS.dr)? ` Demand case ${state.dp} N·m pitch / ${state.dr} N·m roll (repo rod-sizing case 60 / 30).`:'';
 if(!ch.length && !pose && !dem){ $('note').textContent='Change a control above to build a note.'; return; }
 let res = W? `Over the full range (pitch ±40°, roll ±20°), the playground's port of tools/ankle_linkage.py gives: worst transmission ${fmt(W.worst,2)} (rule ≥ 0.5), worst-pose torque ${fmt(W.rated[0])} N·m pitch / ${fmt(W.rated[1])} N·m roll continuous (${fmt(W.peak[0])} / ${fmt(W.peak[1])} peak), ratios at neutral ${fmt(W.ratios[0],2)} / ${fmt(W.ratios[1],2)}, largest rod force ${fmt(W.rod)} N. Repo today: 0.53; 32 / 25 rated; 96 / 74 peak; 0.99 / 0.67; 1857 N. Need from the static 18 cm step check: 24.4 / 17.5 N·m.` : 'This geometry cannot reach the whole range (a rod cannot close the loop somewhere).';
 $('note').textContent=`For Marvin Bot's two-motor parallel ankle (left ankle, staggered RS03 motors on the shin, cranks driving push rods to the foot), I explored ${ch.length?ch.join(', '):'the current geometry'}.${pose}${dem} ${res} Remember the simulator models pitch and roll as independent joints with neutral-pose limits; if this geometry is adopted, update description/skeleton.yaml's ankle_linkage and rerun tools/ankle_linkage.py --check and the URDF build.`;
}
// controls
const F={pitch:v=>v+'°',roll:v=>v+'°',r:v=>v+' mm',back:v=>v+' mm',side:v=>'±'+v+' mm',bx:v=>v+' mm',dp:v=>v+' N·m',dr:v=>v+' N·m'};
Object.keys(DEFAULTS).forEach(id=>{ const e=$(id), out=$('o_'+id);
 const upd=()=>{ state[id]= e.tagName==='SELECT'? e.value : parseFloat(e.value); if(out) out.textContent=F[id]?F[id](state[id]):state[id]; render(); };
 e.addEventListener('input',upd); e.addEventListener('change',upd); if(out) out.textContent=F[id]?F[id](state[id]):state[id]; });
function setState(s){ Object.assign(state,s); for(const k in state){ const e=$(k); if(e){ e.value=state[k]; const o=$('o_'+k); if(o) o.textContent=F[k]?F[k](state[k]):state[k]; } } render(); }
document.querySelectorAll('[data-preset]').forEach(b=>b.addEventListener('click',()=>setState(PRESETS[b.dataset.preset])));
$('copy').addEventListener('click',()=>{ navigator.clipboard.writeText($('note').textContent).then(()=>{ const b=$('copy'); b.textContent='Copied!'; setTimeout(()=>b.textContent='Copy',1200); }); });
render();
