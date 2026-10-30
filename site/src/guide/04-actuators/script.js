const $ = id => document.getElementById(id);
const NS='http://www.w3.org/2000/svg';
const RPM=2*Math.PI/60;
// Manual data. tn: digitised 48 V T-N curve at the output (rpm, N·m). arm: repo armature placeholder (kg·m², joint side).
const MODELS={
 RS04:{peak:120,rated:40,rpm:200,N:9,kg:1.42,Kt:2.1,Ir:27/Math.SQRT2,Ip:90/Math.SQRT2,ke:16.9,mitV:15,arm:0.03,price:255,
   tn:[[0,120],[95,120],[110,110],[130,95],[150,78],[160,62],[170,53],[180,46],[186,38],[189,25],[190,10],[200,0]],th:{f:0.10,tw:2,th:250}},
 RS03:{peak:60,rated:20,rpm:195,N:9,kg:0.90,Kt:2.36,Ir:13/Math.SQRT2,Ip:43/Math.SQRT2,ke:17,mitV:20,arm:0.02,price:225,
   tn:[[0,60],[120,60],[135,55],[150,47.5],[165,39],[180,29],[186,21],[187,10],[195,0]],th:{f:0.20,tw:20,th:1000}},
 RS06:{peak:36,rated:11,rpm:480,N:9,kg:0.621,Kt:1.09,Ir:14.3/Math.SQRT2,Ip:57/Math.SQRT2,ke:7.6,mitV:50,arm:0.01,price:210,
   tn:[[0,36],[280,36],[300,33],[320,30],[350,26],[380,22],[400,18],[420,12.5],[428,10],[430,5],[480,0]],th:{f:0.20,tw:20,th:1000}},
 RS00:{peak:14,rated:5,rpm:315,N:10,kg:0.31,Kt:1.48,Ir:4.7/Math.SQRT2,Ip:15.5/Math.SQRT2,ke:9.5,mitV:33,arm:0.005,price:139,
   tn:[[0,14],[100,14],[150,12],[200,9.3],[250,6.5],[300,2],[315,0.5],[318,0]],th:{f:0.20,tw:20,th:1000}}
};
function interp(tab,x){ if(x<=tab[0][0]) return tab[0][1]; for(let i=1;i<tab.length;i++){ if(x<=tab[i][0]){ const [x0,y0]=tab[i-1],[x1,y1]=tab[i]; return y0+(y1-y0)*(x-x0)/(x1-x0);} } return 0; }
// envelope at the joint (N·m) for joint speed w (rad/s), ratio N, bus V
function envelope(m,w,N,V){ const k=N/m.N; const rpmNative=(w/RPM)*k; return k*interp(m.tn, rpmNative*48/V); }
function w0(m,N,V){ return m.tn[m.tn.length-1][0]*RPM*(V/48)/(N/m.N); }
function el(tag,attrs,parent,text){ const e=document.createElementNS(NS,tag); for(const k in attrs) e.setAttribute(k,attrs[k]); if(text!==undefined) e.textContent=text; if(parent) parent.appendChild(e); return e; }
function fmt(x,d=0){ return Number(x).toFixed(d); }
function niceStep(range,n){ const raw=range/n, p=Math.pow(10,Math.floor(Math.log10(raw))); const f=raw/p; return (f<1.5?1:f<3?2:f<7?5:10)*p; }

/* ---------- Playground 1: torque-speed ---------- */
const TS_DEF={tsModel:'RS04',tsRatio:9,tsV:48,tsT:47,tsW:0.5,tsDC:true};
const ts={...TS_DEF};
const TS_PRESETS={
 today:{tsModel:'RS04',tsRatio:9,tsV:48,tsT:47,tsW:0.5},
 swing:{tsModel:'RS04',tsRatio:9,tsV:48,tsT:15,tsW:8},
 stage:{tsModel:'RS04',tsRatio:13.5,tsV:48,tsT:47,tsW:0.5},
 hd:{tsModel:'RS04',tsRatio:50,tsV:48,tsT:47,tsW:0.5},
 lowbat:{tsModel:'RS04',tsRatio:9,tsV:42,tsT:60,tsW:13}
};
function renderTS(){
 const m=MODELS[ts.tsModel], N=ts.tsRatio, V=ts.tsV, k=N/m.N;
 const svg=$('tsView'); svg.innerHTML='';
 const W=640,H=400,L=58,R=16,T=14,B=46, pw=W-L-R, ph=H-T-B;
 const wmax=Math.max(w0(m,N,Math.max(V,48))*1.08, ts.tsW*1.1, 1);
 const tmax=Math.max(m.peak*k*1.12, ts.tsT*1.1, 1);
 const X=w=>L+pw*w/wmax, Y=t=>T+ph*(1-t/tmax);
 // grid
 const xs=niceStep(wmax,8), ys=niceStep(tmax,6);
 for(let x=0;x<=wmax+1e-9;x+=xs){ el('line',{x1:X(x),x2:X(x),y1:T,y2:T+ph,stroke:'#1f2833'},svg); el('text',{x:X(x),y:T+ph+16,fill:'#9aa7b4','font-size':11,'text-anchor':'middle'},svg,fmt(x,xs<1?1:0)); }
 for(let y=0;y<=tmax+1e-9;y+=ys){ el('line',{x1:L,x2:L+pw,y1:Y(y),y2:Y(y),stroke:'#1f2833'},svg); el('text',{x:L-6,y:Y(y)+4,fill:'#9aa7b4','font-size':11,'text-anchor':'end'},svg,fmt(y,ys<1?1:0)); }
 el('text',{x:L+pw/2,y:H-8,fill:'#9aa7b4','font-size':12,'text-anchor':'middle'},svg,'joint speed (rad/s)');
 el('text',{x:14,y:T+ph/2,fill:'#9aa7b4','font-size':12,'text-anchor':'middle',transform:`rotate(-90 14 ${T+ph/2})`},svg,'joint torque (N·m)');
 const n=160;
 const path=(fn,wEnd)=>{ let d=''; for(let i=0;i<=n;i++){ const w=wEnd*i/n; d+=(i?'L':'M')+X(w).toFixed(1)+','+Y(Math.max(0,fn(w))).toFixed(1); } return d; };
 // continuous region
 const rated=m.rated*k; const w0V=w0(m,N,V);
 let dC='M'+X(0)+','+Y(0); for(let i=0;i<=n;i++){ const w=w0V*i/n; dC+='L'+X(w).toFixed(1)+','+Y(Math.min(rated,envelope(m,w,N,V))).toFixed(1); } dC+='L'+X(w0V)+','+Y(0)+'Z';
 el('path',{d:dC,fill:'rgba(76,195,138,.16)',stroke:'none'},svg);
 el('line',{x1:X(0),x2:X(w0V),y1:Y(rated),y2:Y(rated),stroke:'#4cc38a','stroke-dasharray':'5 4','stroke-width':1.5},svg);
 el('text',{x:X(w0V*0.55),y:Y(rated)-5,fill:'#4cc38a','font-size':11},svg,`rated (continuous) ${fmt(rated,rated<20?1:0)} N·m`);
 // 48 V reference
 if(Math.abs(V-48)>0.01) el('path',{d:path(w=>envelope(m,w,N,48),w0(m,N,48)),fill:'none',stroke:'#5b6b7c','stroke-width':2,'stroke-dasharray':'6 4'},svg);
 // DC model
 if(ts.tsDC){ const wd=m.rpm*RPM/k, pk=m.peak*k; el('path',{d:`M${X(0)},${Y(pk)}L${X(wd)},${Y(0)}`,stroke:'#b48cff','stroke-width':1.8,fill:'none','stroke-dasharray':'2 3'},svg); }
 // envelope at V
 el('path',{d:path(w=>envelope(m,w,N,V),w0V),fill:'none',stroke:'#4fb3ff','stroke-width':2.5},svg);
 el('text',{x:X(0)+6,y:Y(m.peak*k)-6,fill:'#4fb3ff','font-size':11},svg,`peak ${fmt(m.peak*k)} N·m at ${fmt(V,1)} V`);
 // MIT speed range
 const wm=m.mitV/k; if(wm<wmax){ el('line',{x1:X(wm),x2:X(wm),y1:T,y2:T+ph,stroke:'#ffb454','stroke-dasharray':'3 5'},svg); el('text',{x:X(wm)-4,y:T+12,fill:'#ffb454','font-size':10,'text-anchor':'end'},svg,'MIT-mode speed range'); }
 // operating point
 const avail=envelope(m,ts.tsW,N,V);
 let cls,msg;
 if(ts.tsT<=Math.min(rated,avail)+1e-9){cls='ok';msg='Inside the continuous region: the actuator can do this indefinitely (with the manual\'s heat sink).';}
 else if(ts.tsT<=avail+1e-9){cls='warn';msg=`Inside the peak envelope but above rated: possible for a limited time (heat ${fmt((curr(m,ts.tsT/k)/m.Ir)**2,2)}× rated). See Playground 2.`;}
 else {cls='bad';msg=`Outside the envelope: at ${fmt(ts.tsW,1)} rad/s the actuator gives at most ${fmt(avail,1)} N·m.`;}
 if(ts.tsW>wm && cls!=='bad') msg+=' Note: faster than the MIT-mode velocity encoding range.';
 if(ts.tsDC){ const dc=Math.max(0,m.peak*k*(1-ts.tsW/(m.rpm*RPM/k))); if(ts.tsT>dc+1e-9 && cls!=='bad') msg+=` The simulator's straight-line model would clip it to ${fmt(dc,1)} N·m.`; }
 const col={ok:'#4cc38a',warn:'#ffb454',bad:'#ff6b6b'}[cls];
 el('circle',{cx:X(ts.tsW),cy:Y(ts.tsT),r:7,fill:col,stroke:'#0c1015','stroke-width':2},svg);
 el('text',{x:Math.min(X(ts.tsW)+10,W-120),y:Math.max(Y(ts.tsT)-10,T+24),fill:col,'font-size':12},svg,`${fmt(ts.tsT,1)} N·m @ ${fmt(ts.tsW,1)} rad/s`);
 const st=$('tsStatus'); st.className='statusline '+cls; st.textContent=msg;
 // readouts
 const I=curr(m,ts.tsT/k);
 setRO('tsR1',`${fmt(m.peak*k,m.peak*k<20?1:0)} / ${fmt(rated,rated<20?1:0)} N·m`);
 setRO('tsR2',`${fmt(w0V,1)} rad/s (${fmt(w0V/RPM)} rpm)`);
 setRO('tsR3',`${fmt(avail,1)} N·m`, avail>=ts.tsT?'ok':'bad');
 setRO('tsR4',`${fmt(I,1)} A`);
 const heat=(I/m.Ir)**2; setRO('tsR5',`${fmt(heat,2)}×`, heat<=1?'ok':heat<=1.5?'warn':'bad');
 setRO('tsR6',`${fmt(ts.tsT*ts.tsW)} W`);
 const Jr=m.arm*k*k; setRO('tsR7',`${Jr<0.1?fmt(Jr,3):fmt(Jr,2)} kg·m²`);
 setRO('tsR8',`${fmt(Jr/(0.375**2),2)} kg`, Jr/0.1406<1?'ok':Jr/0.1406<3?'warn':'bad');
 updateNote();
}
// rms current for an actuator-native torque (output of the native gearbox), piecewise through rated and peak currents
function curr(m,tau){ tau=Math.abs(tau); return tau<=m.rated? tau/m.rated*m.Ir : m.Ir+(tau-m.rated)/(m.peak-m.rated)*(m.Ip-m.Ir); }
function setRO(id,v,cls){ const e=$(id); e.className='ro'+(cls?' '+cls:''); e.querySelector('.v').textContent=v; }

/* ---------- Playground 2: thermal ---------- */
const TH_DEF={thModel:'RS04',thSink:100,thAmb:25,thProf:'stairs',thT:44,thStall:false,thDur:15};
const th={...TH_DEF};
const TH_PRESETS={
 today:{thModel:'RS04',thSink:100,thAmb:25,thProf:'stairs',thT:44,thStall:false,thDur:15},
 static:{thModel:'RS04',thSink:100,thAmb:25,thProf:'const',thT:47,thStall:false,thDur:20},
 thigh:{thModel:'RS04',thSink:87,thAmb:25,thProf:'const',thT:47,thStall:false,thDur:20},
 walk:{thModel:'RS04',thSink:100,thAmb:25,thProf:'walk',thT:34.7,thStall:false,thDur:15},
 yaw:{thModel:'RS03',thSink:100,thAmb:25,thProf:'const',thT:17.7,thStall:false,thDur:30}
};
const ALPHA=0.00393, LIM=145, RISE=120;
function profile(t){ const T=th.thT; switch(th.thProf){
  case 'const': return T;
  case 'walk': return (t%0.6)<0.36? T : 0.2*T;
  case 'stairs': return (t%3)<1.2? T : 0.3*T;
  case 'hold': return (t%60)<30? T : 0; } return T; }
function simulate(m,sink,amb,dur,stall){
 const p=m.th, a=RISE/(1+ALPHA*RISE);         // rated heat × R_total = a (K), cold-referenced
 const Rtot=1/(sink*sink);                     // normalised total resistance (1 at the manual plate)
 const Rwh=p.f, Rha=Rtot-p.f; const Cw=p.tw/p.f, Ch=p.th/(1-p.f);
 let Tw=0,Th=0,t=0,dt=0.05, tlim=null, sumsq=0, nS=0; const out=[]; const every=Math.max(1,Math.round(dur/400/dt));
 let i=0;
 while(t<=dur){
   const tau=profile(t); sumsq+=tau*tau; nS++;
   const x=curr(m,tau)/m.Ir; const P=a*x*x*(1+ALPHA*(Tw+amb-25))*(stall?Math.SQRT2:1);
   const dTw=(P-(Tw-Th)/Rwh)/Cw, dTh=((Tw-Th)/Rwh-Th/Rha)/Ch;
   Tw+=dTw*dt; Th+=dTh*dt; t+=dt;
   if(tlim===null && Tw+amb>=LIM) tlim=t;
   if(i++%every===0) out.push([t,Tw+amb,Th+amb]);
 }
 return {out,tlim,rms:Math.sqrt(sumsq/nS),Tend:Tw+amb,Rtot,a};
}
function steadyState(m,rmsHeat,Rtot,amb,stall){ // solve ΔT = a·h·(1+α(ΔT+amb-25))·Rtot
 const a=RISE/(1+ALPHA*RISE); const g=a*rmsHeat*Rtot*(stall?Math.SQRT2:1); const den=1-g*ALPHA; if(den<=0) return Infinity; return amb+g*(1+ALPHA*(amb-25))/den; }
function renderTH(){
 const m=MODELS[th.thModel], sink=th.thSink/100, dur=th.thDur*60;
 const r=simulate(m,sink,th.thAmb,dur,th.thStall);
 // mean of (I/Ir)^2 over one period for steady state
 let h=0,nn=0; for(let t=0;t<60;t+=0.01){ const x=curr(m,profile(t))/m.Ir; h+=x*x; nn++; } h/=nn;
 const Tss=steadyState(m,h,r.Rtot,th.thAmb,th.thStall);
 const svg=$('thView'); svg.innerHTML='';
 const W=640,H=360,L=50,R=16,T=14,B=44,pw=W-L-R,ph=H-T-B;
 const ymin=Math.min(20,th.thAmb-5), ymax=Math.max(160, Math.min(220, Math.max(...r.out.map(o=>o[1]))+10));
 const X=t=>L+pw*t/dur, Y=v=>T+ph*(1-(v-ymin)/(ymax-ymin));
 const xs=niceStep(th.thDur,8);
 for(let x=0;x<=th.thDur+1e-9;x+=xs){ el('line',{x1:X(x*60),x2:X(x*60),y1:T,y2:T+ph,stroke:'#1f2833'},svg); el('text',{x:X(x*60),y:T+ph+16,fill:'#9aa7b4','font-size':11,'text-anchor':'middle'},svg,fmt(x)); }
 for(let y=Math.ceil(ymin/20)*20;y<=ymax;y+=20){ el('line',{x1:L,x2:L+pw,y1:Y(y),y2:Y(y),stroke:'#1f2833'},svg); el('text',{x:L-6,y:Y(y)+4,fill:'#9aa7b4','font-size':11,'text-anchor':'end'},svg,y); }
 el('text',{x:L+pw/2,y:H-8,fill:'#9aa7b4','font-size':12,'text-anchor':'middle'},svg,'time (minutes)');
 el('text',{x:14,y:T+ph/2,fill:'#9aa7b4','font-size':12,'text-anchor':'middle',transform:`rotate(-90 14 ${T+ph/2})`},svg,'temperature (°C)');
 el('line',{x1:L,x2:L+pw,y1:Y(LIM),y2:Y(LIM),stroke:'#ffb454','stroke-dasharray':'6 4','stroke-width':1.5},svg);
 el('text',{x:L+pw-4,y:Y(LIM)-5,fill:'#ffb454','font-size':11,'text-anchor':'end'},svg,'145 °C winding limit');
 let dW='',dH=''; r.out.forEach((o,i)=>{ const c=i?'L':'M'; dW+=c+X(o[0]).toFixed(1)+','+Y(Math.min(o[1],ymax)).toFixed(1); dH+=c+X(o[0]).toFixed(1)+','+Y(Math.min(o[2],ymax)).toFixed(1); });
 el('path',{d:dH,fill:'none',stroke:'#4fb3ff','stroke-width':2},svg);
 el('path',{d:dW,fill:'none',stroke:'#ff6b6b','stroke-width':2.2},svg);
 if(r.tlim!==null){ el('line',{x1:X(r.tlim),x2:X(r.tlim),y1:T,y2:T+ph,stroke:'#ff6b6b','stroke-dasharray':'3 4'},svg); el('text',{x:Math.min(X(r.tlim)+5,W-120),y:T+14,fill:'#ff6b6b','font-size':11},svg,`limit at ${fmtT(r.tlim)}`); }
 const ratedM=m.rated*sink;
 setRO('thR1',`${fmt(r.rms,1)} N·m`);
 const pr=r.rms/ratedM; setRO('thR2',`${fmt(pr*100)}%`, pr<=0.85?'ok':pr<=1?'warn':'bad');
 setRO('thR3',`${fmt(r.Tend)} °C`, r.Tend<LIM-15?'ok':r.Tend<LIM?'warn':'bad');
 setRO('thR4', r.tlim===null? (Tss>=LIM?'later than shown':'never') : fmtT(r.tlim), r.tlim===null&&Tss<LIM?'ok':'bad');
 setRO('thR5', isFinite(Tss)&&Tss<400? `${fmt(Tss)} °C`:'runaway', Tss<LIM-15?'ok':Tss<LIM?'warn':'bad');
 setRO('thR6', `${fmt(ratedM,1)} N·m`);
 const st=$('thStatus');
 if(r.tlim!==null){ st.className='statusline bad'; st.textContent=`The winding reaches the 145 °C protection limit after ${fmtT(r.tlim)}; the driver would cut torque there.`; }
 else if(Tss>=LIM){ st.className='statusline warn'; st.textContent=`Below the limit within ${th.thDur} min, but the steady state (${isFinite(Tss)&&Tss<400?fmt(Tss)+' °C':'runaway'}) is above it: it would trip if this went on.`; }
 else { st.className='statusline ok'; st.textContent=`Sustainable: the winding levels off around ${fmt(Tss)} °C, ${fmt(LIM-Tss)} K below the limit.`; }
 updateNote();
}
function fmtT(s){ return s<120? `${fmt(s)} s` : `${fmt(s/60,1)} min`; }

/* ---------- Playground 3: joint assignment ---------- */
const JA_DEF={jaMass:40.35,jaSink:100,jaStage:false};
const ja={...JA_DEF};
const JA_PRESETS={today:{jaMass:40.35,jaSink:100,jaStage:false},target:{jaMass:35,jaSink:100,jaStage:false},thigh:{jaMass:40.35,jaSink:85,jaStage:false},stage:{jaMass:40.35,jaSink:85,jaStage:true}};
// demand (N·m), rated, peak of the actuator(s) at the joint; leg: scales with mass
const JOINTS=[
 {n:'Knee',m:'RS04',d:47.1,r:40,p:120,leg:1,src:'static 18 cm step'},
 {n:'Hip roll',m:'RS04',d:41.9,r:40,p:120,leg:1,src:'static 18 cm step'},
 {n:'Hip pitch',m:'RS04',d:12.3,r:40,p:120,leg:1,src:'static 18 cm step'},
 {n:'Hip yaw',m:'RS03',d:17.7,r:20,p:60,leg:1,src:'static 18 cm step'},
 {n:'Ankle pitch (2×RS03)',m:'RS03',d:24.4,r:32,p:96,leg:1,src:'step vs worst pose'},
 {n:'Ankle roll (2×RS03)',m:'RS03',d:17.5,r:25,p:74,leg:1,src:'step vs worst pose'},
 {n:'Shoulder pitch',m:'RS03',d:9.4,r:20,p:60,leg:0,src:'arm out, 0.5 kg'},
 {n:'Shoulder roll',m:'RS03',d:9.4,r:20,p:60,leg:0,src:'arm out sideways, 0.5 kg'},
 {n:'Elbow',m:'RS06',d:3.4,r:11,p:36,leg:0,src:'forearm level, 0.5 kg'},
 {n:'Wrist pitch (servo)',m:'STS3250',d:1.18,r:1.57,p:4.9,leg:0,src:'0.5 kg at fingertips (R20)'}
];
function renderJA(){
 const svg=$('jaView'); svg.innerHTML='';
 const W=640,L=170,R=60,T=26,rowH=38,pw=W-L-R; const maxPct=1.6;
 const X=p=>L+pw*Math.min(p,maxPct)/maxPct;
 [0,0.5,1,1.5].forEach(p=>{ el('line',{x1:X(p),x2:X(p),y1:T-8,y2:T+rowH*JOINTS.length,stroke:p===1?'#ffb454':'#1f2833','stroke-dasharray':p===1?'5 4':''},svg); el('text',{x:X(p),y:T-12,fill:p===1?'#ffb454':'#9aa7b4','font-size':11,'text-anchor':'middle'},svg,(p*100)+'%'+(p===1?' of rated':'')); });
 const scale=ja.jaMass/40.35, sink=ja.jaSink/100;
 JOINTS.forEach((j,i)=>{
   let d=j.d*(j.leg?scale:1), r=j.r, p=j.p;
   if(j.m!=='STS3250') r*=sink;
   if(ja.jaStage && j.n==='Knee'){ r*=1.5; p*=1.5; }
   const pr=d/r, pp=d/p, y=T+i*rowH+6;
   const col=pr<=0.85?'#4cc38a':pr<=1?'#ffb454':'#ff6b6b';
   el('text',{x:L-8,y:y+12,fill:'#e6edf3','font-size':12,'text-anchor':'end'},svg,j.n);
   el('text',{x:L-8,y:y+25,fill:'#9aa7b4','font-size':10,'text-anchor':'end'},svg,`${j.m} · ${j.src}`);
   el('rect',{x:L,y:y+2,width:Math.max(1,X(pr)-L),height:14,rx:3,fill:col,opacity:.85},svg);
   el('rect',{x:X(pp)-2,y:y+19,width:4,height:9,fill:'#b48cff'},svg);
   el('line',{x1:L,x2:X(pp),y1:y+23.5,y2:y+23.5,stroke:'#b48cff','stroke-width':1,opacity:.5},svg);
   el('text',{x:Math.min(X(pr)+6,W-R+4),y:y+14,fill:col,'font-size':11},svg,`${fmt(d,1)} / ${fmt(r,1)} N·m = ${fmt(pr*100)}%`+(pr>maxPct?' ▶':''));
   el('text',{x:X(pp)+6,y:y+29,fill:'#b48cff','font-size':10},svg,`${fmt(pp*100)}% of peak`);
 });
 updateNote();
}

/* ---------- binding ---------- */
function bindAll(stateObj, ids, renderFn, fmts){
 ids.forEach(id=>{ const e=$(id); const out=$('o_'+id);
   const upd=()=>{ stateObj[id]= e.type==='checkbox'? e.checked : (e.tagName==='SELECT'? e.value : parseFloat(e.value)); if(out) out.textContent=fmts[id]?fmts[id](stateObj[id]):stateObj[id]; renderFn(); };
   e.addEventListener('input',upd); e.addEventListener('change',upd); });
}
function setUI(stateObj,s,fmts,renderFn){ Object.assign(stateObj,s); for(const k in stateObj){ const e=$(k); if(!e) continue; if(e.type==='checkbox') e.checked=!!stateObj[k]; else e.value=stateObj[k]; const o=$('o_'+k); if(o) o.textContent=fmts[k]?fmts[k](stateObj[k]):stateObj[k]; } renderFn(); }
const TSF={tsRatio:v=>v+':1',tsV:v=>fmt(v,1)+' V',tsT:v=>fmt(v,1)+' N·m',tsW:v=>`${fmt(v,1)} rad/s (${fmt(v/RPM)} rpm)`};
const THF={thSink:v=>v+'%',thAmb:v=>v+' °C',thT:v=>fmt(v,1)+' N·m',thDur:v=>v+' min'};
const JAF={jaMass:v=>fmt(v,2)+' kg',jaSink:v=>v+'%'};
bindAll(ts,['tsModel','tsRatio','tsV','tsT','tsW','tsDC'],renderTS,TSF);
bindAll(th,['thModel','thSink','thAmb','thProf','thT','thStall','thDur'],renderTH,THF);
bindAll(ja,['jaMass','jaSink','jaStage'],renderJA,JAF);
// model change in P1 resets ratio to native
$('tsModel').addEventListener('change',()=>{ const m=MODELS[ts.tsModel]; setUI(ts,{tsRatio:m.N},TSF,renderTS); });
document.querySelectorAll('[data-ts]').forEach(b=>b.addEventListener('click',()=>setUI(ts,TS_PRESETS[b.dataset.ts],TSF,renderTS)));
document.querySelectorAll('[data-th]').forEach(b=>b.addEventListener('click',()=>setUI(th,TH_PRESETS[b.dataset.th],THF,renderTH)));
document.querySelectorAll('[data-ja]').forEach(b=>b.addEventListener('click',()=>setUI(ja,JA_PRESETS[b.dataset.ja],JAF,renderJA)));

function updateNote(){
 const parts=[];
 if(!$('note')) return;
 const m=MODELS[ts.tsModel], k=ts.tsRatio/m.N;
 const tsChanged=['tsModel','tsRatio','tsV','tsT','tsW'].some(x=>ts[x]!==TS_DEF[x]);
 if(tsChanged){
   const avail=envelope(m,ts.tsW,ts.tsRatio,ts.tsV), rated=m.rated*k;
   const verdict= ts.tsT<=Math.min(rated,avail)? 'inside the continuous (rated) region' : ts.tsT<=avail? `inside the peak envelope but above rated (${fmt((ts.tsT/rated)**2,2)}× rated heat), so time-limited` : `outside the envelope (max ${fmt(avail,1)} N·m at that speed)`;
   parts.push(`Torque-speed: a ${ts.tsModel}${ts.tsRatio!==m.N?` rescaled to a ${ts.tsRatio}:1 total ratio (peak ${fmt(m.peak*k)} N·m, rated ${fmt(rated,1)} N·m, reflected inertia ~${fmt(m.arm*k*k,3)} kg·m² from the repo's armature placeholder)`:''} at ${fmt(ts.tsV,1)} V; a joint demand of ${fmt(ts.tsT,1)} N·m at ${fmt(ts.tsW,1)} rad/s is ${verdict}.`);
 }
 const thChanged=Object.keys(TH_DEF).some(x=>th[x]!==TH_DEF[x]);
 if(thChanged){
   const mm=MODELS[th.thModel]; const r=simulate(mm,th.thSink/100,th.thAmb,th.thDur*60,th.thStall);
   const prof={const:'constant',walk:'walking-like (60% at full, 40% at 20%)',stairs:'stair-like (40% of a 3 s step at full, 60% at 30%)',hold:'30 s hold / 30 s rest'}[th.thProf];
   parts.push(`Thermal (two-node model fitted to RobStride's overload tables, 145 °C limit): ${th.thModel}, ${prof} profile at ${fmt(th.thT,1)} N·m${th.thStall?' while holding still':''}, heat path ${th.thSink}% of the manual's plate rating (${fmt(mm.rated*th.thSink/100,1)} N·m continuous), ambient ${th.thAmb} °C. RMS torque ${fmt(r.rms,1)} N·m; ${r.tlim!==null?`reaches 145 °C after ${fmtT(r.tlim)}`:`stays below 145 °C for ${th.thDur} min (${fmt(r.Tend)} °C at the end)`}.`);
 }
 const jaChanged=Object.keys(JA_DEF).some(x=>ja[x]!==JA_DEF[x]);
 if(jaChanged){
   const scale=ja.jaMass/40.35, r=40*ja.jaSink/100*(ja.jaStage?1.5:1); const kn=47.1*scale, hr=41.9*scale, hrr=40*ja.jaSink/100;
   parts.push(`Joint assignment: at ${fmt(ja.jaMass,2)} kg bare, heat path ${ja.jaSink}%${ja.jaStage?' and the parked 1.5:1 knee stage':''}, the static 18 cm step puts the knee at ${fmt(kn,1)} N·m = ${fmt(kn/r*100)}% of its continuous rating and hip roll at ${fmt(hr/hrr*100)}% (repo rule: RMS within rating, peaks ≤ 70% of peak).`);
 }
 $('note').textContent = parts.length ? `For Marvin Bot (1.44 m, 40.35 kg bare humanoid with RobStride QDD actuators, 48 V), I explored the actuator limits:\n- ${parts.join('\n- ')}\nOpen question: confirm against the phase 2 bench test (RS04 thermal hold at 47 N·m in a thigh-sized mount, bench/actuator/bench_robstride.py) before deciding on the 1.5:1 knee stage.`
   : 'Change a control above to build a note.';
}
$('copy').addEventListener('click',()=>{ navigator.clipboard.writeText($('note').textContent).then(()=>{ const b=$('copy'); b.textContent='Copied!'; setTimeout(()=>b.textContent='Copy',1200); }); });
setUI(ts,{},TSF,renderTS); setUI(th,{},THF,renderTH); setUI(ja,{},JAF,renderJA);
