"use strict";
const $ = id => document.getElementById(id);
const D2R = Math.PI/180, R2D = 180/Math.PI;
const NS = 'http://www.w3.org/2000/svg';

/* ---------------- vector helpers (x forward, y left, z up) ---------------- */
const add=(a,b)=>[a[0]+b[0],a[1]+b[1],a[2]+b[2]], sub=(a,b)=>[a[0]-b[0],a[1]-b[1],a[2]-b[2]];
const mul=(a,s)=>[a[0]*s,a[1]*s,a[2]*s], norm=a=>Math.hypot(a[0],a[1],a[2]);
// pitch: rotate the "down" vector forward by p (Ry(-p))
const P=(v,p)=>{const c=Math.cos(p),s=Math.sin(p);return [v[0]*c-v[2]*s,v[1],v[0]*s+v[2]*c];};
const Rl=(v,r)=>{const c=Math.cos(r),s=Math.sin(r);return [v[0],v[1]*c-v[2]*s,v[1]*s+v[2]*c];};
const Yw=(v,y)=>{const c=Math.cos(y),s=Math.sin(y);return [v[0]*c-v[1]*s,v[0]*s+v[1]*c,v[2]];};
const torsoRot=(v,lean)=>P(v,-lean); // pelvis pitched forward by lean

/* ---------------- bodies ---------------- */
const HH=1.75;
const HUMAN={ thigh:0.4265,shin:0.4265,ankle:0.079,hipHalf:0.09,heel:0.07,toe:0.19,
  sh:[-0.03,0.18,0.4725],off:[0,0,0],upper:0.315,fore:0.245,hand:0.19,neck:0.56,headTop:HH };
const ROBOT={ thigh:0.325,shin:0.315,ankle:0.060,hipHalf:0.100,heel:0.09,toe:0.15,
  sh:[0.025,0.110,0.465],off:[0,0.048,-0.025],upper:0.200,fore:0.188,hand:0.181,neck:0.525,headTop:1.438,
  lim:{hip:[-30,120],roll:[-30,45],knee:[0,130],ankle:[-40,40],sp:[-60,180],sr:[-20,120],sy:[-150,150],el:[0,135]} };
const COUPLED=[[20,0],[45,-10],[75,-10],[90,-20],[135,-20],[150,0]];
function rollMin(p){ // degrees
  if(p<=COUPLED[0][0]||p>=COUPLED[COUPLED.length-1][0]) return 0;
  for(let i=0;i<COUPLED.length-1;i++){const [a,va]=COUPLED[i],[b,vb]=COUPLED[i+1]; if(p>=a&&p<=b) return va+(vb-va)*(p-a)/(b-a);}
  return 0;}

/* leg FK in pelvis frame: returns knee, ankle (relative to pelvis) */
function legFK(B,side,th,ph,kn){ // radians; side +1 left, -1 right
  const hip=[0,side*B.hipHalf,0];
  const t=Rl(P([0,0,-B.thigh],th),side*ph), s=Rl(P([0,0,-B.shin],th-kn),side*ph);
  const knee=add(hip,t); return {hip,knee,ank:add(knee,s)};}
/* arm FK in torso frame (relative to pelvis) */
function armFK(B,side,q){ // q=[pitch,roll,yaw,elbow] radians
  const S=[B.sh[0],side*B.sh[1],B.sh[2]];
  const r0=add(S,P([B.off[0],side*B.off[1],B.off[2]],q[0]));
  const up=v=>P(Rl(v,side*q[1]),q[0]);
  const el=add(r0,up([0,0,-B.upper]));
  const fd=up(Yw(P([0,0,-1],q[3]),side*q[2]));
  const wr=add(el,mul(fd,B.fore)), hd=add(wr,mul(fd,B.hand*0.6));
  return {S,r0,el,wr,hd};}

/* ---------------- human motions (degrees; keyframes) ---------------- */
const STAND={p:0,lth:4,lph:2,lkn:6,rth:4,rph:2,rkn:6,la:[5,8,0,15],ra:[5,8,0,15]};
function K(t,o){return [t,Object.assign(JSON.parse(JSON.stringify(STAND)),o)];}
const MOTIONS={
 reach:[K(0,{}),K(0.4,{}),K(1.4,{p:12,lth:14,rth:14,lkn:8,rkn:8,la:[95,-6,0,4],ra:[95,-6,0,4]}),K(2.6,{p:12,lth:14,rth:14,lkn:8,rkn:8,la:[95,-6,0,4],ra:[95,-6,0,4]}),K(3.6,{}),K(4,{})],
 clap:(()=>{const a={la:[62,6,0,78],ra:[62,6,0,78]},m={la:[62,-30,0,78],ra:[62,-30,0,78]};
   return [K(0,{}),K(0.6,a),K(1.0,m),K(1.4,a),K(1.8,m),K(2.2,a),K(2.6,m),K(3.2,a),K(4,{})];})(),
 squat:[K(0,{la:[85,0,0,5],ra:[85,0,0,5]}),K(0.3,{la:[88,0,0,5],ra:[88,0,0,5]}),
   K(1.2,{p:25,lth:93,rth:93,lkn:108,rkn:108,la:[85,0,0,5],ra:[85,0,0,5]}),K(1.6,{la:[85,0,0,5],ra:[85,0,0,5]}),
   K(2.0,{la:[85,0,0,5],ra:[85,0,0,5]}),K(3.0,{p:24,lth:124,rth:124,lkn:140,rkn:140,la:[90,0,0,5],ra:[90,0,0,5]}),K(3.3,{p:24,lth:124,rth:124,lkn:140,rkn:140,la:[90,0,0,5],ra:[90,0,0,5]}),K(4,{la:[85,0,0,5],ra:[85,0,0,5]})],
 wave:(()=>{const u={ra:[15,100,90,70]},v={ra:[15,100,90,112]};
   return [K(0,{}),K(0.7,u),K(1.1,v),K(1.5,u),K(1.9,v),K(2.3,u),K(2.7,v),K(3.2,u),K(4,{})];})(),
 wave2:(()=>{const u={la:[10,150,90,30],ra:[10,150,90,30]},v={la:[10,150,90,70],ra:[10,150,90,70]};
   return [K(0,{}),K(0.8,u),K(1.2,v),K(1.6,u),K(2.0,v),K(2.4,u),K(2.8,v),K(3.3,u),K(4,{})];})(),
 march:(()=>{const L={lth:70,lkn:85,la:[-20,8,0,30],ra:[30,8,0,40]},R={rth:70,rkn:85,la:[30,8,0,40],ra:[-20,8,0,30]};
   return [K(0,{}),K(0.5,L),K(1.0,{}),K(1.5,R),K(2.0,{}),K(2.5,L),K(3.0,{}),K(3.5,R),K(4,{})];})()
};
const FPS=25, NF=100;
function lerpPose(a,b,u){const s=u*u*(3-2*u),o={};for(const k in a){o[k]=Array.isArray(a[k])?a[k].map((x,i)=>x+(b[k][i]-x)*s):a[k]+(b[k]-a[k])*s;}return o;}
function poseAt(keys,t){for(let i=0;i<keys.length-1;i++){if(t<=keys[i+1][0]){return lerpPose(keys[i][1],keys[i+1][1],(t-keys[i][0])/(keys[i+1][0]-keys[i][0]));}}return keys[keys.length-1][1];}

/* human frame: world keypoints; pelvis placed so the stance (lower) foot stays at its rest spot */
function humanFrame(ps){
  const B=HUMAN,lean=ps.p*D2R;
  const legs=[1,-1].map(sd=>{const k=sd>0?'l':'r';const f=legFK(B,sd,ps[k+'th']*D2R,ps[k+'ph']*D2R,ps[k+'kn']*D2R);
    const a=(ps[k+'kn']-ps[k+'th']+ps.p)*D2R; // foot flat
    return {hip:torsoRot(f.hip,lean),knee:torsoRot(f.knee,lean),ank:torsoRot(f.ank,lean),fp:(ps[k+'th']-ps[k+'kn']+(a*R2D))*D2R-lean};});
  // stance = lower ankle
  const st=legs[0].ank[2]<=legs[1].ank[2]+1e-9?0:1;
  const restA=legFK(B,1,STAND.lth*D2R,STAND.lph*D2R,STAND.lkn*D2R).ank;
  const pel=[restA[0]-legs[st].ank[0],0,B.ankle-legs[st].ank[2]];
  const W=v=>add(pel,v);
  const arms=[1,-1].map(sd=>{const q=(sd>0?ps.la:ps.ra).map(x=>x*D2R);const f=armFK(B,sd,q);
    return {S:W(torsoRot(f.S,lean)),el:W(torsoRot(f.el,lean)),wr:W(torsoRot(f.wr,lean)),hd:W(torsoRot(f.hd,lean)),q};});
  const neck=W(torsoRot([B.sh[0],0,B.neck],lean)), head=W(torsoRot([B.sh[0]+0.02,0,B.neck+0.2],lean));
  return {pel,lean,legs:legs.map(l=>({hip:W(l.hip),knee:W(l.knee),ank:W(l.ank),fp:l.fp})),arms,neck,head,ps};
}

/* ---------------- scaling (as scale_keypoints) ---------------- */
function scaleFrame(h,S){
  const T={};const g=v=>[v[0],v[1],v[2]];
  if(S.mode==='uni'){const s=S.su; const sc=v=>mul(v,s);
    T.pel=sc(h.pel);T.legs=h.legs.map(l=>({knee:sc(l.knee),ank:sc(l.ank)}));
    T.arms=h.arms.map(a=>({S:sc(a.S),el:sc(a.el),wr:sc(a.wr)}));}
  else{const sl=S.sl,sa=S.sa;const sc=v=>mul(v,sl);
    T.pel=sc(h.pel);
    T.legs=h.legs.map((l,i)=>{const side=i===0?1:-1;const kn=sc(l.knee),an=sc(l.ank);
      if(S.fixTorso){kn[1]+=side*0.015;an[1]+=side*0.03;} return {knee:kn,ank:an};});
    const sw=S.fixTorso?0.62:sl, fwd=S.fixTorso?0.065:0;
    T.arms=h.arms.map(a=>{const d=sub(a.S,h.pel);const Sn=add(T.pel,[d[0]*sw+fwd,d[1]*sw,d[2]*S.st]);
      const f=v=>add(Sn,mul(sub(v,a.S),sa));return {S:Sn,el:f(a.el),wr:f(a.wr)};});}
  return T;}

/* ---------------- robot solve ---------------- */
function legIK(B,side,dl){ // dl: target ankle relative to hip joint, pelvis frame
  const ph=Math.atan2(side*dl[1],-dl[2]); const zp=-Math.hypot(dl[1],dl[2]);
  const r=Math.hypot(dl[0],zp),T=B.thigh,S=B.shin;
  let kn=0; if(r<T+S){const c=(T*T+S*S-r*r)/(2*T*S);kn=Math.PI-Math.acos(Math.max(-1,Math.min(1,c)));}
  const g=Math.atan2(dl[0],-zp); const th=g+Math.atan2(S*Math.sin(kn),T+S*Math.cos(kn));
  return [th,ph,kn];}
const clampD=(v,l)=>Math.max(l[0]*D2R,Math.min(l[1]*D2R,v));
function solve4(x){ // 4x4 linear system, x = augmented [A|b]
  const n=4;for(let i=0;i<n;i++){let m=i;for(let r=i+1;r<n;r++)if(Math.abs(x[r][i])>Math.abs(x[m][i]))m=r;[x[i],x[m]]=[x[m],x[i]];
    const pv=x[i][i]||1e-12;for(let r=0;r<n;r++){if(r===i)continue;const f=x[r][i]/pv;for(let c=i;c<=n;c++)x[r][c]-=f*x[i][c];}}
  return x.map((row,i)=>row[n]/(row[i]||1e-12));}
function armIK(B,side,tS,tEl,tWr,q0,limited){ // targets relative to pelvis (torso frame)
  let q=q0.slice();const wE=0.3,wR=0.003;
  const proj=q=>{if(!limited)return q;const L=B.lim;q[0]=clampD(q[0],L.sp);
    const rmin=Math.max(L.sr[0],rollMin(q[0]*R2D));q[1]=Math.max(rmin*D2R,Math.min(L.sr[1]*D2R,q[1]));q[2]=clampD(q[2],L.sy);q[3]=clampD(q[3],L.el);return q;};
  const res=q=>{const f=armFK(B,side,q);const a=sub(f.wr,tWr),b=mul(sub(f.el,tEl),Math.sqrt(wE));
    return [...a,...b,...q.map((v,i)=>(v-q0[i])*Math.sqrt(wR))];};
  q=proj(q);
  for(let it=0;it<30;it++){const r=res(q);const J=[];const h=1e-4;
    for(let j=0;j<4;j++){const qq=q.slice();qq[j]+=h;const r2=res(qq);J.push(r2.map((v,i)=>(v-r[i])/h));}
    const A=[];for(let i=0;i<4;i++){A.push([]);for(let j=0;j<4;j++){let s=0;for(let k=0;k<r.length;k++)s+=J[i][k]*J[j][k];A[i].push(s+(i===j?1e-3:0));}
      let b=0;for(let k=0;k<r.length;k++)b-=J[i][k]*r[k];A[i].push(b);}
    const d=solve4(A);for(let j=0;j<4;j++)q[j]+=Math.max(-0.3,Math.min(0.3,d[j]));q=proj(q);}
  return q;}

function robotFrame(h,T){
  const B=ROBOT,lean=h.lean,L=B.lim;
  const inv=v=>P(v,lean); // world->pelvis frame rotation (inverse of torsoRot)
  const viol=[];
  const legs=[1,-1].map((sd,i)=>{const dl=sub(inv(sub(T.legs[i].ank,T.pel)),[0,sd*B.hipHalf,0]);
    const need=legIK(B,sd,dl); const q=[clampD(need[0],L.hip),Math.max(L.roll[0]*D2R,Math.min(L.roll[1]*D2R,need[1])),clampD(need[2],L.knee)];
    const nm=sd>0?'left':'right';
    if(need[2]*R2D>L.knee[1]+0.5)viol.push([nm+' knee',need[2]*R2D,L.knee[1]]);
    if(need[0]*R2D>L.hip[1]+0.5)viol.push([nm+' hip pitch',need[0]*R2D,L.hip[1]]);
    if(need[0]*R2D<L.hip[0]-0.5)viol.push([nm+' hip pitch',need[0]*R2D,L.hip[0]]);
    const aNeed=(q[2]-q[0])*R2D+lean*R2D; const a=Math.max(L.ankle[0],Math.min(L.ankle[1],aNeed));
    if(Math.abs(aNeed)>L.ankle[1]+0.5)viol.push([nm+' ankle pitch',aNeed,aNeed>0?L.ankle[1]:L.ankle[0]]);
    const f=legFK(B,sd,q[0],q[1],q[2]); const fp=(q[0]-q[2])+a*D2R-lean;
    return {hip:add(T.pel,torsoRot(f.hip,lean)),knee:add(T.pel,torsoRot(f.knee,lean)),ank:add(T.pel,torsoRot(f.ank,lean)),fp,q,hit:{knee:need[2]*R2D>L.knee[1]+0.5,hip:need[0]*R2D>L.hip[1]+0.5||need[0]*R2D<L.hip[0]-0.5,ankle:Math.abs(aNeed)>L.ankle[1]+0.5}};});
  const arms=[1,-1].map((sd,i)=>{const t=T.arms[i];const rel=v=>inv(sub(v,T.pel));
    const q0=h.arms[i].q;const free=armIK(B,sd,rel(t.S),rel(t.el),rel(t.wr),q0,false);
    const q=armIK(B,sd,rel(t.S),rel(t.el),rel(t.wr),free,true);
    const nm=sd>0?'left':'right';const fd=free.map(x=>x*R2D);const hit={sp:false,sr:false,el:false};
    const rmin=Math.max(L.sr[0],rollMin(Math.max(L.sp[0],Math.min(L.sp[1],fd[0]))));
    if(fd[1]>L.sr[1]+1){viol.push([nm+' shoulder roll',fd[1],L.sr[1]]);hit.sr=true;}
    if(fd[1]<rmin-1){viol.push([nm+' shoulder roll',fd[1],rmin,'coupled']);hit.sr=true;}
    if(fd[0]>L.sp[1]+1||fd[0]<L.sp[0]-1){viol.push([nm+' shoulder pitch',fd[0],fd[0]>0?L.sp[1]:L.sp[0]]);hit.sp=true;}
    if(fd[3]>L.el[1]+1){viol.push([nm+' elbow',fd[3],L.el[1]]);hit.el=true;}
    const f=armFK(B,sd,q);const W=v=>add(T.pel,torsoRot(v,lean));
    return {S:W(f.S),r0:W(f.r0),el:W(f.el),wr:W(f.wr),hd:W(f.hd),q,free,hit};});
  const neck=add(T.pel,torsoRot([B.sh[0],0,B.neck],lean)),head=add(T.pel,torsoRot([B.sh[0]+0.02,0,B.neck+0.13],lean));
  return {pel:T.pel,lean,legs,arms,neck,head,viol};}

function soleZ(B,leg){const heel=add(leg.ank,P([-B.heel,0,-B.ankle],-leg.fp));const toe=add(leg.ank,P([B.toe,0,-B.ankle],-leg.fp));
  return {heel,toe,z:Math.min(heel[2],toe[2]),flat:add(leg.ank,[0,0,-B.ankle])};}
function shiftF(F,dz){const s=v=>[v[0],v[1],v[2]+dz];F.pel=s(F.pel);F.neck=s(F.neck);F.head=s(F.head);
  F.legs.forEach(l=>{l.hip=s(l.hip);l.knee=s(l.knee);l.ank=s(l.ank);});F.arms.forEach(a=>{for(const k of ['S','r0','el','wr','hd'])if(a[k])a[k]=s(a[k]);});}

/* ---------------- whole clip ---------------- */
let CLIP=null;
function computeClip(){
  const keys=MOTIONS[state.motion];const S={mode:state.mode,su:state.su,sl:state.sl,sa:state.sa,st:state.st,fixTorso:state.fixTorso};
  const H=[],R=[],T=[];
  for(let i=0;i<NF;i++){const h=humanFrame(poseAt(keys,i/FPS));const t=scaleFrame(h,S);H.push(h);T.push(t);R.push(robotFrame(h,t));}
  // human contacts
  const hc=H.map((h,i)=>h.legs.map((l,k)=>{const j=Math.max(0,i-1),m=Math.min(NF-1,i+1);const v=Math.hypot(H[m].legs[k].ank[0]-H[j].legs[k].ank[0],H[m].legs[k].ank[2]-H[j].legs[k].ank[2])*FPS/Math.max(1,m-j);return soleZ(HUMAN,l).z<0.02&&v<0.15;}));
  // robot soles before fix
  let minS=R.map(r=>Math.min(...r.legs.map(l=>soleZ(ROBOT,l).z)));
  let dz=0;
  if(state.fixSole){const ms=minS.slice().sort((a,b)=>a-b);dz=-ms[Math.floor(ms.length*0.5)];}
  // the fix moves the robot, not the targets (targets stay where scaling put them)
  R.forEach((r,i)=>{shiftF(r,dz);});
  // metrics
  let he=0,hm=0,n=0,slide=0,ns=0,stance=[],lim=0;
  for(let i=0;i<NF;i++){const r=R[i],t=T[i];
    r.arms.forEach((a,k)=>{const e=norm(sub(a.wr,t.arms[k].wr));he+=e;hm=Math.max(hm,e);n++;});
    if(r.viol.length)lim++;
    r.legs.forEach((l,k)=>{if(hc[i][k]){stance.push(soleZ(ROBOT,l).z);
      if(i>0&&hc[i-1][k]){const a=R[i-1].legs[k].ank,b=l.ank;slide+=Math.hypot(b[0]-a[0],b[1]-a[1])*FPS;ns++;}}});}
  stance.sort((a,b)=>a-b);
  const med=stance.length?stance[Math.floor(stance.length/2)]:0;
  const pen=stance.length?Math.min(0,stance[0], ...R.map(r=>Math.min(...r.legs.map(l=>soleZ(ROBOT,l).z)))):0;
  const all=[];R.forEach(r=>r.viol.forEach(v=>all.push(v)));
  const worst={};all.forEach(v=>{const k=v[0]+(v[3]||'');const ex=Math.abs(v[1]-v[2]);if(!worst[k]||ex>Math.abs(worst[k][1]-worst[k][2]))worst[k]=v;});
  CLIP={H,R,T,dz,handMean:he/n,handMax:hm,soleMed:med,pen,slide:ns?slide/ns:0,knee0:R[0].legs[0].q[2]*R2D,lim,worst:Object.values(worst),pelT:T[0].pel[2]};
}

/* ---------------- drawing ---------------- */
function el(tag,attrs,parent){const e=document.createElementNS(NS,tag);for(const k in attrs)e.setAttribute(k,attrs[k]);if(parent)parent.appendChild(e);return e;}
function drawFrame(i){
  const svg=$('view');svg.innerHTML='';
  const side=state.view==='side';const sc=200, floorY=392;
  const proj=(v,cx)=>side?[cx+v[0]*sc,floorY-v[2]*sc]:[cx-v[1]*sc,floorY-v[2]*sc];
  el('rect',{x:0,y:0,width:760,height:420,fill:'#0c1015'},svg);
  // grid
  for(let m=0;m<=2;m+=0.25){const y=floorY-m*sc;el('line',{x1:0,x2:760,y1:y,y2:y,stroke:'#1a222c','stroke-width':1},svg);
    if(m>0)el('text',{x:4,y:y-3,fill:'#56636f','font-size':10,'font-family':'monospace'},svg).textContent=m.toFixed(2)+' m';}
  el('line',{x1:0,x2:760,y1:floorY,y2:floorY,stroke:'#56636f','stroke-width':2},svg);
  el('line',{x1:380,x2:380,y1:10,y2:410,stroke:'#2a3340','stroke-dasharray':'4 4'},svg);
  const h=CLIP.H[i],r=CLIP.R[i],t=CLIP.T[i];
  const cxH=190-(side?h.pel[0]*sc*0:0), cxR=575;
  const line=(a,b,c,w,cx,dash)=>{const p=proj(a,cx),q=proj(b,cx);el('line',{x1:p[0],y1:p[1],x2:q[0],y2:q[1],stroke:c,'stroke-width':w,'stroke-linecap':'round','stroke-dasharray':dash||''},svg);};
  const dot=(a,c,rad,cx,fill)=>{const p=proj(a,cx);el('circle',{cx:p[0],cy:p[1],r:rad,fill:fill===undefined?c:fill,stroke:c,'stroke-width':1.5},svg);};
  const order=side?[1,0]:[0,1]; // draw far side first
  // human
  const hc='#cfd8e3', hcF='#7f8b97';
  const drawBody=(F,B,cx,col,colF,isRobot)=>{
    order.forEach(k=>{const l=F.legs[k];const c=(side&&k===1)?colF:col;const s=soleZ(B,l);
      line(F.pel,l.hip,c,4,cx);line(l.hip,l.knee,c,5,cx);line(l.knee,l.ank,c,5,cx);
      if(side){line(s.heel,s.toe,c,4,cx);line(l.ank,s.heel,c,2,cx);line(l.ank,s.toe,c,2,cx);}else{line(l.ank,s.flat,c,4,cx);const p=add(s.flat,[0,0.05,0]),q=add(s.flat,[0,-0.05,0]);line(p,q,c,4,cx);}
      if(isRobot){if(l.hit.knee)dot(l.knee,'var(--bad)',6,cx);if(l.hit.hip)dot(l.hip,'var(--bad)',6,cx);if(l.hit.ankle)dot(l.ank,'var(--bad)',6,cx);}});
    const shc=[(F.arms[0].S[0]+F.arms[1].S[0])/2,(F.arms[0].S[1]+F.arms[1].S[1])/2,(F.arms[0].S[2]+F.arms[1].S[2])/2];
    line(F.pel,shc,col,6,cx);line(F.arms[0].S,F.arms[1].S,col,4,cx);line(shc,F.neck,col,4,cx);
    const hp=proj(F.head,cx);el('circle',{cx:hp[0],cy:hp[1],r:(isRobot?0.11:0.11)*sc,fill:'none',stroke:col,'stroke-width':3},svg);
    order.forEach(k=>{const a=F.arms[k];const c=(side&&k===1)?colF:col;
      if(a.r0)line(a.S,a.r0,c,4,cx);line(a.r0||a.S,a.el,c,4,cx);line(a.el,a.wr,c,4,cx);line(a.wr,a.hd,c,3,cx);
      if(isRobot){if(a.hit.sr||a.hit.sp)dot(a.S,'var(--bad)',6,cx);if(a.hit.el)dot(a.el,'var(--bad)',6,cx);}});
  };
  drawBody(h,HUMAN,cxH,hc,hcF,false);
  // robot targets
  const tc='#4fb3ff';
  [0,1].forEach(k=>{dot(t.legs[k].ank,tc,4,cxR,'none');dot(t.legs[k].knee,tc,4,cxR,'none');dot(t.arms[k].S,tc,4,cxR,'none');dot(t.arms[k].el,tc,4,cxR,'none');dot(t.arms[k].wr,tc,5,cxR,'none');});
  dot(t.pel,tc,5,cxR,'none');
  drawBody(r,ROBOT,cxR,'#ffb454','#a7783a',true);
  // errors: wrist
  [0,1].forEach(k=>{const e=norm(sub(r.arms[k].wr,t.arms[k].wr));if(e>0.02)line(r.arms[k].wr,t.arms[k].wr,'#ff6b6b',2,cxR,'3 3');
    const ea=norm(sub(r.legs[k].ank,t.legs[k].ank));if(ea>0.02)line(r.legs[k].ank,t.legs[k].ank,'#ff6b6b',2,cxR,'3 3');});
  el('text',{x:12,y:22,fill:'#cfd8e3','font-size':13},svg).textContent=`Human source, ${HH.toFixed(2)} m`;
  el('text',{x:392,y:22,fill:'#ffb454','font-size':13},svg).textContent=`Marvin Bot, 1.44 m · ${side?'side':'front'} view`;
  el('text',{x:392,y:40,fill:'#9aa7b4','font-size':11},svg).textContent=state.fixSole?`foot-on-floor fix: clip moved ${(CLIP.dz*100>=0?'+':'')}${(CLIP.dz*100).toFixed(1)} cm`:'no foot-on-floor fix (pelvis pinned to its target)';
  $('o_time').textContent=(i/FPS).toFixed(1)+' s';
  drawCoupled(i);
}
function drawCoupled(i){
  const svg=$('coupled');svg.innerHTML='';
  el('rect',{x:0,y:0,width:760,height:230,fill:'#0c1015'},svg);
  const x0=60,x1=740,y0=22,y1=205;const px=p=>x0+(p+60)/(240)*(x1-x0), py=r=>y1-(r+40)/(200)*(y1-y0);
  // forbidden regions: below rollMin(p) and below -20, above 120
  let d=`M${px(-60)},${py(-40)}`;for(let p=-60;p<=180;p+=2)d+=` L${px(p)},${py(Math.max(-20,rollMin(p)))}`;d+=` L${px(180)},${py(-40)} Z`;
  el('path',{d,fill:'rgba(255,107,107,.18)',stroke:'#ff6b6b','stroke-width':1},svg);
  el('rect',{x:px(-60),y:py(160),width:px(180)-px(-60),height:py(120)-py(160),fill:'rgba(255,107,107,.18)'},svg);
  for(const r of [-20,0,60,120]){el('line',{x1:x0,x2:x1,y1:py(r),y2:py(r),stroke:'#1f2933'},svg);el('text',{x:x0-6,y:py(r)+4,fill:'#56636f','font-size':10,'text-anchor':'end'},svg).textContent=r+'°';}
  for(const p of [-60,0,45,90,135,180]){el('text',{x:px(p),y:y1+14,fill:'#56636f','font-size':10,'text-anchor':'middle'},svg).textContent=p+'°';}
  el('text',{x:x0,y:13,fill:'#9aa7b4','font-size':11},svg).textContent='Shoulder roll (up) vs shoulder pitch (along) the motion asks for · shaded: outside the limits';
  [0,1].forEach(k=>{let dd='';CLIP.R.forEach((r,j)=>{const q=r.arms[k].free;dd+=(j?' L':'M')+px(q[0]*R2D)+','+py(Math.max(-40,Math.min(160,q[1]*R2D)));});
    el('path',{d:dd,fill:'none',stroke:'#b48cff','stroke-width':1.5,opacity:k?0.6:1},svg);
    const q=CLIP.R[i].arms[k].free;el('circle',{cx:px(q[0]*R2D),cy:py(Math.max(-40,Math.min(160,q[1]*R2D))),r:5,fill:CLIP.R[i].arms[k].hit.sr||CLIP.R[i].arms[k].hit.sp?'#ff6b6b':'#b48cff'},svg);});
}

/* ---------------- state, readouts ---------------- */
const DEFAULTS={motion:'reach',view:'side',mode:'sep',su:0.822,sl:0.75,sa:0.745,st:0.955,fixSole:true,fixTorso:true,
  sig:0.3,err:0.03,nb:15,noff:15};
const state={...DEFAULTS};
const PRESETS={
 today:{mode:'sep',sl:0.75,sa:0.745,st:0.955,fixSole:true,fixTorso:true},
 uniform:{mode:'uni',su:0.822,fixSole:true,fixTorso:false},
 none:{mode:'uni',su:1.0,fixSole:false,fixTorso:false},
 small:{mode:'sep',sl:0.72,sa:0.68,st:0.85,fixSole:true,fixTorso:true},
 wave2:{motion:'wave2',view:'front',mode:'sep',sl:0.75,sa:0.745,st:0.955,fixSole:true,fixTorso:true},
 squat:{motion:'squat',view:'side',mode:'sep',sl:0.75,sa:0.745,st:0.955,fixSole:true,fixTorso:true}
};
function setRO(id,txt,cls){const e=$(id);e.querySelector('.v').textContent=txt;e.className='ro'+(cls?' '+cls:'');}
function readouts(){
  const c=CLIP;
  setRO('r_hand',`${(c.handMean*100).toFixed(1)} / ${(c.handMax*100).toFixed(1)} cm`,c.handMean>0.05?'bad':c.handMean>0.02?'warn':'ok');
  const s=c.soleMed*100; setRO('r_sole',(s>=0?'+':'')+s.toFixed(1)+' cm'+(c.pen<-0.005?` (dips ${(c.pen*100).toFixed(1)})`:''),Math.abs(s)>2||c.pen<-0.02?'bad':Math.abs(s)>0.5?'warn':'ok');
  setRO('r_slide',(c.slide*100).toFixed(1)+' cm/s',c.slide>0.04?'bad':c.slide>0.02?'warn':'ok');
  setRO('r_knee',c.knee0.toFixed(0)+'°',c.knee0>25?'bad':c.knee0>12?'warn':'ok');
  setRO('r_pel',`${(c.pelT*100).toFixed(0)} cm (robot 70)`,Math.abs(c.pelT-0.70)>0.04?'warn':'ok');
  setRO('r_lim',`${c.lim} of ${NF}`,c.lim>0?'bad':'ok');
  const v=$('viol');
  if(!c.worst.length){v.textContent='No joint limit asked to be broken in this clip.';v.className='viol okv';}
  else{v.className='viol';v.textContent='Motion asks for: '+c.worst.map(w=>`${w[0]} ${w[1].toFixed(0)}° (${w[3]?'coupled limit':'limit'} ${w[2].toFixed(0)}°)`).join(' · ');}
}
let frame=0,playing=true,last=0;
function recompute(){computeClip();readouts();drawFrame(frame);updateNote();}
function syncUI(){
  $('motion').value=state.motion;
  $('v_side').classList.toggle('on',state.view==='side');$('v_front').classList.toggle('on',state.view==='front');
  $('m_uni').classList.toggle('on',state.mode==='uni');$('m_sep').classList.toggle('on',state.mode==='sep');
  $('uniBox').style.display=state.mode==='uni'?'':'none';$('sepBox').style.display=state.mode==='sep'?'':'none';
  for(const k of ['su','sl','sa','st','sig','err','nb','noff']){$(k).value=state[k];}
  $('o_su').textContent=state.su.toFixed(3);$('o_sl').textContent=state.sl.toFixed(3);$('o_sa').textContent=state.sa.toFixed(3);$('o_st').textContent=state.st.toFixed(3);
  $('fixSole').checked=state.fixSole;$('fixTorso').checked=state.fixTorso;
}
['su','sl','sa','st'].forEach(k=>$(k).addEventListener('input',e=>{state[k]=parseFloat(e.target.value);syncUI();recompute();}));
$('motion').addEventListener('change',e=>{state.motion=e.target.value;if(['wave','wave2','clap'].includes(state.motion))state.view='front';else state.view='side';syncUI();recompute();});
$('v_side').onclick=()=>{state.view='side';syncUI();drawFrame(frame);};
$('v_front').onclick=()=>{state.view='front';syncUI();drawFrame(frame);};
$('m_uni').onclick=()=>{state.mode='uni';syncUI();recompute();};
$('m_sep').onclick=()=>{state.mode='sep';syncUI();recompute();};
$('fixSole').onchange=e=>{state.fixSole=e.target.checked;recompute();};
$('fixTorso').onchange=e=>{state.fixTorso=e.target.checked;recompute();};
document.querySelectorAll('[data-preset]').forEach(b=>b.addEventListener('click',()=>{Object.assign(state,PRESETS[b.dataset.preset]);syncUI();recompute();}));
$('time').addEventListener('input',e=>{frame=parseInt(e.target.value);playing=false;$('play').textContent='Play';drawFrame(frame);});
$('play').onclick=()=>{playing=!playing;$('play').textContent=playing?'Pause':'Play';};
function tick(ts){if(playing&&ts-last>1000/FPS){last=ts;frame=(frame+1)%NF;$('time').value=frame;drawFrame(frame);}requestAnimationFrame(tick);}

/* ---------------- Playground B: kernel ---------------- */
const TERMS=[
 {k:'pos',name:'Body positions (15 bodies, relative to anchor)',w:1.0,s:0.3,u:'m',max:0.5,step:0.005,v:0.03,src:'BeyondMimic'},
 {k:'ori',name:'Body orientations',w:1.0,s:0.4,u:'rad',max:1.5,step:0.01,v:0.05,src:'BeyondMimic'},
 {k:'lv',name:'Body linear velocities',w:1.0,s:0.5,u:'m/s',max:2,step:0.01,v:0.1,src:'Marvin Bot: σ 0.5 (BeyondMimic 1.0)'},
 {k:'av',name:'Body angular velocities',w:1.0,s:3.14,u:'rad/s',max:8,step:0.05,v:0.5,src:'BeyondMimic'},
 {k:'ap',name:'Anchor (torso) position, global',w:0.5,s:0.3,u:'m',max:0.6,step:0.005,v:0.05,src:'BeyondMimic; added back by Marvin Bot'},
 {k:'ao',name:'Anchor orientation, global',w:0.5,s:0.4,u:'rad',max:1.5,step:0.01,v:0.05,src:'BeyondMimic'},
 {k:'jp',name:'Joint angles, all joints (RMS)',w:0.5,s:0.3,u:'rad',max:1.2,step:0.005,v:0.05,src:'Marvin Bot addition'},
 {k:'ar',name:'Action rate ‖aₜ − aₜ₋₁‖²',w:-0.1,s:null,u:'',max:3,step:0.05,v:0.3,src:'BeyondMimic'},
 {k:'cm',name:'Feet whose contact differs from the reference',w:-2.0,s:null,u:'feet',max:2,step:1,v:0,src:'Marvin Bot (upstream −0.1)'}
];
const RP={
 crouch:{sig:0.3,err:0.03,nb:15,noff:7,t:{pos:0.03*Math.sqrt(7/15),ori:0.05,lv:0.1,av:0.5,ap:0.03,ao:0.03,jp:0.113*Math.sqrt(6/27),ar:0.3,cm:0}},
 arm:{sig:0.3,err:0.6,nb:15,noff:1,t:{pos:0.6/Math.sqrt(15),ori:0.3,lv:0.3,av:1,ap:0.03,ao:0.03,jp:0.25,ar:0.3,cm:0}},
 hop:{sig:0.3,err:0.06,nb:15,noff:15,t:{pos:0.06,ori:0.08,lv:0.4,av:1.5,ap:0.06,ao:0.05,jp:0.12,ar:0.8,cm:1}},
 behind:{sig:0.3,err:0.4,nb:15,noff:15,t:{pos:0.05,ori:0.08,lv:0.5,av:1,ap:0.4,ao:0.1,jp:0.1,ar:0.4,cm:1}},
 good:{sig:0.3,err:0.02,nb:15,noff:15,t:{pos:0.02,ori:0.04,lv:0.08,av:0.4,ap:0.02,ao:0.02,jp:0.03,ar:0.2,cm:0}}
};
const tv={};TERMS.forEach(t=>tv[t.k]=t.v);
function buildTerms(){const tb=$('terms').querySelector('tbody');
  TERMS.forEach(t=>{const tr=document.createElement('tr');
    tr.innerHTML=`<td>${t.name}<div style="font-size:11px;color:var(--muted)">${t.src}</div></td><td class="num">${t.w}</td><td class="num">${t.s===null?'–':t.s+' '+t.u}</td>
    <td><input type="range" id="t_${t.k}" min="0" max="${t.max}" step="${t.step}" value="${t.v}" aria-label="${t.name} error"></td><td class="num mono" id="tv_${t.k}"></td><td class="num mono" id="tc_${t.k}"></td><td><span class="bar" id="tb_${t.k}"></span></td>`;
    tb.appendChild(tr);$('t_'+t.k).addEventListener('input',e=>{tv[t.k]=parseFloat(e.target.value);drawTerms();updateNote();});});}
function termVal(t,e){return t.s===null?t.w*(t.k==='ar'?e*e:e):t.w*Math.exp(-(e*e)/(t.s*t.s));}
function drawTerms(){let tot=0;TERMS.forEach(t=>{const e=tv[t.k];const c=termVal(t,e);tot+=c;
  $('tv_'+t.k).textContent=t.k==='cm'?e.toFixed(0):e.toFixed(3)+' '+t.u;$('tc_'+t.k).textContent=(c>=0?'+':'')+c.toFixed(3);
  const b=$('tb_'+t.k);b.style.width=Math.min(110,Math.abs(c)*55)+'px';b.style.background=c<0?'var(--bad)':'var(--accent)';
  $('t_'+t.k).value=e;});
  $('tot').textContent=tot.toFixed(3);}
function drawKernel(){
  const svg=$('kern');svg.innerHTML='';el('rect',{x:0,y:0,width:760,height:300,fill:'#0c1015'},svg);
  const x0=56,x1=740,y0=20,y1=260,E=0.6;const px=e=>x0+e/E*(x1-x0),py=r=>y1-r*(y1-y0);
  for(const r of [0,0.25,0.5,0.75,1]){el('line',{x1:x0,x2:x1,y1:py(r),y2:py(r),stroke:'#1f2933'},svg);el('text',{x:x0-6,y:py(r)+4,fill:'#56636f','font-size':11,'text-anchor':'end'},svg).textContent=r.toFixed(2);}
  for(let e=0;e<=E+1e-9;e+=0.1){el('text',{x:px(e),y:y1+16,fill:'#56636f','font-size':11,'text-anchor':'middle'},svg).textContent=e.toFixed(1)+' m';}
  el('text',{x:x1,y:y1+32,fill:'#9aa7b4','font-size':11,'text-anchor':'end'},svg).textContent='error of the bodies that are off';
  const s=state.sig,N=state.nb,no=Math.min(state.noff,state.nb);
  // curve for one-body-everything (all off) and diluted
  const curve=(f,col,w,dash)=>{let d='';for(let i=0;i<=200;i++){const e=E*i/200;d+=(i?' L':'M')+px(e)+','+py(f(e));}el('path',{d,fill:'none',stroke:col,'stroke-width':w,'stroke-dasharray':dash||''},svg);};
  curve(e=>Math.exp(-e*e/(s*s)),'#4fb3ff',2.5);
  curve(e=>Math.exp(-(no/N)*e*e/(s*s)),'#b48cff',2,'5 4');
  // sigma marker
  el('line',{x1:px(s),x2:px(s),y1:y0,y2:y1,stroke:'#56636f','stroke-dasharray':'2 3'},svg);
  el('text',{x:px(s)+4,y:y0+12,fill:'#9aa7b4','font-size':11},svg).textContent='σ → 0.37';
  const e=state.err,r=Math.exp(-(no/N)*e*e/(s*s));
  el('circle',{cx:px(Math.min(e,E)),cy:py(r),r:6,fill:'#ffb454'},svg);
  el('text',{x:px(Math.min(e,E))+9,y:py(r)-8,fill:'#ffb454','font-size':12},svg).textContent=`r = ${r.toFixed(3)}`;
  el('text',{x:x0+8,y:y1-10,fill:'#4fb3ff','font-size':11},svg).textContent='solid: all scored bodies off by e';
  el('text',{x:x0+8,y:y1-26,fill:'#b48cff','font-size':11},svg).textContent=`dashed: ${no} of ${N} bodies off by e (the mean dilutes it)`;
  const g=-(2*(no/N)*e/(s*s))*r;
  setRO('k_r',r.toFixed(3),r>0.9?'ok':r>0.5?'warn':'bad');
  setRO('k_lost',((1-r)*100).toFixed(1)+'%');
  setRO('k_g',g.toFixed(2)+' /m');
  setRO('k_rms',(e*Math.sqrt(no/N)*100).toFixed(1)+' cm');
  $('o_sig').textContent=s.toFixed(2)+' m';$('o_err').textContent=(e*100).toFixed(1)+' cm';$('o_nb').textContent=N;$('o_noff').textContent=no;
}
['sig','err','nb','noff'].forEach(k=>$(k).addEventListener('input',e=>{state[k]=parseFloat(e.target.value);drawKernel();updateNote();}));
document.querySelectorAll('[data-rp]').forEach(b=>b.addEventListener('click',()=>{const p=RP[b.dataset.rp];
  state.sig=p.sig;state.err=p.err;state.nb=p.nb;state.noff=p.noff;Object.assign(tv,p.t);syncUI();drawKernel();drawTerms();updateNote();}));

/* ---------------- skills table ---------------- */
const SK=[['Walk','Walking and turning',8.5,6.0],['Walk slow','Walking and turning',8.1,6.6],['Walk back','Walking and turning',7.9,7.0],
['Jog','Walking and turning',8.8,5.8],['March','Walking and turning',9.2,4.4],['Start stop','Walking and turning',8.8,5.6],
['Carry walk','Walking and turning',11.4,6.8],['Side left','Walking and turning',10.7,7.8],['Side right','Walking and turning',10.9,7.5],
['Turn left','Walking and turning',8.2,6.3],['Turn right','Walking and turning',8.4,7.5],['Turn in place','Walking and turning',8.2,4.9],
['Kick','Whole body',10.7,5.7],['Squat','Whole body',15.8,8.6],['Pick up','Whole body',12.6,3.9],['Bow','Whole body',8.1,4.4],
['Dance sway','Whole body',9.9,4.4],['Idle','Whole body',9.4,3.7],['Look around','Whole body',8.3,3.2],
['Clap','Arms and gestures',12.6,11.0],['Wave','Arms and gestures',11.7,6.9],['Wave both','Arms and gestures',34.6,12.1],
['Point','Arms and gestures',9.3,6.5],['Reach forward','Arms and gestures',18.4,9.3],['Arms raise','Arms and gestures',12.3,8.3]];
let sortK='j',sortDir=-1;
function drawSkills(){const tb=$('skills').querySelector('tbody');tb.innerHTML='';
  const idx={name:0,group:1,j:2,h:3}[sortK];
  const rows=SK.slice().sort((a,b)=>(a[idx]>b[idx]?1:a[idx]<b[idx]?-1:0)*sortDir);
  rows.forEach(r=>{const tr=document.createElement('tr');
    tr.innerHTML=`<td>${r[0]}</td><td>${r[1]}</td><td class="num">${r[2].toFixed(1)}</td><td><span class="bar" style="width:${r[2]*4}px;${r[2]>15?'background:var(--bad)':''}"></span></td><td class="num">${r[3].toFixed(1)}</td><td><span class="bar h" style="width:${r[3]*8}px"></span></td>`;tb.appendChild(tr);});}
document.querySelectorAll('#skills th.sort').forEach(th=>th.addEventListener('click',()=>{const k=th.dataset.k;if(sortK===k)sortDir*=-1;else{sortK=k;sortDir=(k==='name'||k==='group')?1:-1;}drawSkills();}));

/* ---------------- note ---------------- */
function updateNote(){
  const parts=[];const c=CLIP;
  const motionName=$('motion').selectedOptions[0].textContent;
  if(state.mode==='uni')parts.push(`uniform keypoint scale ${state.su.toFixed(3)} (height ratio is 0.822)`);
  else if(state.sl!==DEFAULTS.sl||state.sa!==DEFAULTS.sa||state.st!==DEFAULTS.st)parts.push(`per-part scale: legs ${state.sl.toFixed(3)}, arms ${state.sa.toFixed(3)}, shoulder height ${state.st.toFixed(3)} (repo today: 0.750, 0.746, 0.955)`);
  if(!state.fixSole)parts.push('no foot-on-floor fix');
  if(!state.fixTorso&&state.mode==='sep')parts.push('no shoulder/feet adjustment');
  const kparts=[];
  if(state.sig!==DEFAULTS.sig)kparts.push(`body-position σ ${state.sig.toFixed(2)} m (repo 0.3)`);
  if(state.nb!==DEFAULTS.nb)kparts.push(`${state.nb} scored bodies (repo 15)`);
  const changedTerms=TERMS.filter(t=>tv[t.k]!==t.v);
  if(!parts.length&&!kparts.length&&!changedTerms.length&&state.motion===DEFAULTS.motion){$('note').textContent='Change a control above to build a note.';return;}
  let s=`Marvin Bot (1.44 m, 27-DOF humanoid) motion-tracking pipeline: Kimodo clips retargeted with PyRoki (sim/motion/retarget_marvin_bot.py), tracked BeyondMimic-style in ProtoMotions.\n`;
  s+=`Retargeting test, "${motionName}" from a ${HH} m source`+(parts.length?`, with ${parts.join('; ')}`:' with the repo\'s current per-part scales')+`:\n`;
  s+=`- hands ${(c.handMean*100).toFixed(1)} cm from their scaled targets on average (max ${(c.handMax*100).toFixed(1)} cm); stance sole ${(c.soleMed*100).toFixed(1)} cm from the floor; foot sliding ${(c.slide*100).toFixed(1)} cm/s in contact; standing knee bend ${c.knee0.toFixed(0)}°; pelvis target ${(c.pelT*100).toFixed(0)} cm vs the robot's 70 cm hip axis.\n`;
  s+=c.worst.length?`- the motion asks for: ${c.worst.map(w=>`${w[0]} ${w[1].toFixed(0)}° (limit ${w[2].toFixed(0)}°${w[3]?', coupled':''})`).join('; ')}.\n`:'- no joint limit is asked to be broken.\n';
  if(kparts.length||changedTerms.length){let tot=0;TERMS.forEach(t=>tot+=termVal(t,tv[t.k]));
    s+=`Reward explored: ${kparts.join('; ')||'repo kernels'}; one-body kernel at e = ${(state.err*100).toFixed(1)} cm gives r = ${Math.exp(-(Math.min(state.noff,state.nb)/state.nb)*state.err*state.err/(state.sig*state.sig)).toFixed(3)}; weighted sum with my errors = ${tot.toFixed(2)} of 5.5.\n`;}
  s+=`Question: is this scaling/reward choice better than the repo's (sim/motion/README.md, docs/decisions.md 2026-10-29), and what would it change for the weakest clips (two-hand wave 34.6° joint RMS, reach 18.4°, clap hands 11 cm)?`;
  $('note').textContent=s;}
$('copy').addEventListener('click',()=>{navigator.clipboard.writeText($('note').textContent).then(()=>{const b=$('copy');b.textContent='Copied!';setTimeout(()=>b.textContent='Copy',1200);});});

buildTerms();syncUI();recompute();drawKernel();drawTerms();drawSkills();updateNote();
requestAnimationFrame(tick);
