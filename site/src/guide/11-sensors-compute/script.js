const $=id=>document.getElementById(id);
const NS='http://www.w3.org/2000/svg';
function el(tag,a,p,txt){const e=document.createElementNS(NS,tag);for(const k in a)e.setAttribute(k,a[k]);if(txt!=null)e.textContent=txt;if(p)p.appendChild(e);return e;}
function ro(id,v,c){const r=$(id);r.querySelector('.v').textContent=v;r.className='ro '+(c||'');}
function bindAll(state,fmt,cb){ for(const k in fmt){ const e=$(k),o=$('o_'+k); const ev=e.tagName==='SELECT'?'change':'input';
  e.addEventListener(ev,()=>{state[k]=parseFloat(e.value); if(o) o.textContent=fmt[k](state[k]); cb();}); if(o) o.textContent=fmt[k](parseFloat(e.value)); } }
function setAll(state,fmt,s,cb){ Object.assign(state,s); for(const k in fmt){ $(k).value=state[k]; const o=$('o_'+k); if(o) o.textContent=fmt[k](state[k]); } cb(); }

/* ---------- A: stereo ---------- */
const SD={B:95,W:1280,F:90,sd:0.24,D:128,tol:1};
const S={...SD};
const SF={B:v=>v+' mm',W:v=>v+' px',F:v=>v+'°',sd:v=>(+v).toFixed(2)+' px',D:v=>v+' px',tol:v=>(+v).toFixed(1)+' cm'};
const SP={today:{...SD},low:{...SD,W:848},g335:{...SD,B:50},wide:{...SD,B:150},poor:{...SD,sd:0.6}};
function stereo(){
  const f=(S.W/2)/Math.tan(S.F/2*Math.PI/180), B=S.B/1000;
  const sz=z=>z*z*S.sd/(f*B), q=z=>z*z/(f*B);
  const zmax=Math.sqrt(S.tol/100*f*B/S.sd), zmin=f*B/S.D;
  const sv=$('sview'); sv.innerHTML='';
  const L=56,R=620,T=26,Bo=316, zMax=6, eMax=Math.max(0.03, Math.min(0.5, sz(4)*1.4));
  const X=z=>L+(R-L)*z/zMax, Y=e=>Bo-(Bo-T)*Math.min(e,eMax)/eMax;
  el('rect',{x:X(0.5),y:T,width:X(2)-X(0.5),height:Bo-T,fill:'#ffb454',opacity:.10},sv);
  el('text',{x:X(1.25),y:T+14,'text-anchor':'middle',fill:'#ffb454'},sv,'stairs ahead');
  if(zmin>0) el('rect',{x:L,y:T,width:X(Math.min(zmin,zMax))-L,height:Bo-T,fill:'#ff6b6b',opacity:.10},sv);
  for(let z=0;z<=6;z++){ el('line',{x1:X(z),y1:Bo,x2:X(z),y2:T,stroke:'#1f2731'},sv); el('text',{x:X(z),y:Bo+16,'text-anchor':'middle'},sv,z+' m'); }
  const step=eMax>0.3?0.1:eMax>0.12?0.05:0.02;
  for(let e=0;e<=eMax+1e-9;e+=step){ el('line',{x1:L,y1:Y(e),x2:R,y2:Y(e),stroke:'#1f2731'},sv); el('text',{x:L-6,y:Y(e)+4,'text-anchor':'end'},sv,(e*100).toFixed(0)+' cm'); }
  el('text',{x:L,y:16,class:'ttl'},sv,'Depth error σz (1 standard deviation) vs distance');
  el('text',{x:(L+R)/2,y:Bo+34,'text-anchor':'middle'},sv,'distance z');
  const path=fn=>{let d='';for(let i=0;i<=300;i++){const z=Math.max(0.05,i/300*zMax);d+=(i?'L':'M')+X(z).toFixed(1)+','+Y(fn(z)).toFixed(1);}return d;};
  el('path',{d:path(q),stroke:'#5d6b7a','stroke-width':1.5,fill:'none','stroke-dasharray':'5 4'},sv);
  el('path',{d:path(sz),stroke:'#4fb3ff','stroke-width':2.5,fill:'none'},sv);
  el('line',{x1:L,y1:Y(S.tol/100),x2:R,y2:Y(S.tol/100),stroke:'#4cc38a','stroke-dasharray':'2 3'},sv);
  el('text',{x:R-4,y:Y(S.tol/100)-5,'text-anchor':'end',fill:'#4cc38a'},sv,'tolerance '+S.tol.toFixed(1)+' cm');
  [[2,0.016],[4,0.064]].forEach(([z,e])=>{ if(Y(e)>T){ el('circle',{cx:X(z),cy:Y(e),r:5,fill:'none',stroke:'#4cc38a','stroke-width':2},sv);} });
  [0.5,1,1.5,2].forEach(z=>{ el('circle',{cx:X(z),cy:Y(sz(z)),r:3.5,fill:'#ffb454'},sv); });
  if(zmin>0.05) el('text',{x:L+4,y:Bo-8,fill:'#ff6b6b'},sv,'no depth < '+zmin.toFixed(2)+' m');
  ro('s_f',f.toFixed(0)+' px');
  ro('s_1',(sz(1)*1000).toFixed(1)+' mm', sz(1)*100<=S.tol?'ok':'warn');
  ro('s_2',(sz(2)*1000).toFixed(1)+' mm ('+(sz(2)/2*100).toFixed(2)+'%)', sz(2)*100<=S.tol?'ok':'warn');
  ro('s_4',(sz(4)*1000).toFixed(0)+' mm ('+(sz(4)/4*100).toFixed(2)+'%)');
  ro('s_r','up to '+zmax.toFixed(2)+' m', zmax>=2?'ok':zmax>=1?'warn':'bad');
  ro('s_m',zmin.toFixed(2)+' m', zmin<=0.6?'ok':'warn');
  S._r={f,sz2:sz(2),zmax,zmin};
  note();
}
bindAll(S,SF,stereo);
document.querySelectorAll('[data-sp]').forEach(b=>b.addEventListener('click',()=>setAll(S,SF,SP[b.dataset.sp],stereo)));

/* ---------- B: IMU ---------- */
const ID={gn:0.014,gb:0.2,an:175,fs:500,dl:3,bump:0.5,tau:0.5};
const I={...ID}; let seed=7;
const IF={gn:v=>(+v).toFixed(3),gb:v=>(+v).toFixed(2)+' °/s',an:v=>v+' µg',fs:v=>v+' Hz',dl:v=>(+v).toFixed(1)+' ms',bump:v=>(+v).toFixed(1)+' m/s²',tau:v=>(+v).toFixed(2)+' s'};
const IP={today:{...ID},bmi:{...ID,fs:1000,dl:1.5,gb:0.1},usb:{...ID,gn:0.03,an:250,fs:200,dl:12,gb:0.3},ind:{...ID,gn:0.005,an:60,fs:1000,dl:1,gb:0.02},trust:{...ID,tau:0.02}};
function rng(s){return function(){s|=0;s=s+0x6D2B79F5|0;let t=Math.imul(s^s>>>15,1|s);t=t+Math.imul(t^t>>>7,61|t)^t;return((t^t>>>14)>>>0)/4294967296;};}
function imu(){
  const r=rng(seed), gauss=()=>Math.sqrt(-2*Math.log(r()+1e-12))*Math.cos(2*Math.PI*r());
  const fs=I.fs, dt=1/fs, N=Math.round(4*fs), D2R=Math.PI/180;
  const A=4, fw=0.9, g=9.81; // 4° rock at 0.9 Hz (a step every ~0.55 s)
  const th=t=>A*Math.sin(2*Math.PI*fw*t)+1.2*Math.sin(2*Math.PI*2.3*fw*t+0.7); // deg
  const om=t=>A*2*Math.PI*fw*Math.cos(2*Math.PI*fw*t)+1.2*2*Math.PI*2.3*fw*Math.cos(2*Math.PI*2.3*fw*t+0.7); // deg/s
  const sg=I.gn*Math.sqrt(fs/2), sa=I.an*1e-6*g*Math.sqrt(fs/2);
  const tt=[],tr=[],acc=[],gyr=[],flt=[],wm=[]; let est=th(0), gint=th(0); const alpha=I.tau/(I.tau+dt);
  for(let i=0;i<N;i++){ const t=i*dt; const truth=th(t);
    const w=om(t)+I.gb+sg*gauss();
    const alin=I.bump*Math.sin(2*Math.PI*2*fw*t)*Math.abs(Math.sin(2*Math.PI*fw*t*1.0)); // footstep-like pulses
    const ax=g*Math.sin(truth*D2R)+alin+sa*gauss(), az=g*Math.cos(truth*D2R)+sa*gauss();
    const ta=Math.atan2(ax,az)/D2R;
    if(i>0){ est=alpha*(est+w*dt)+(1-alpha)*ta; gint+=w*dt; }
    tt.push(t);tr.push(truth);acc.push(ta);gyr.push(gint);flt.push(est);wm.push(w);
  }
  // delayed, sample-held view the policy gets: value at time t - delay, from samples
  const dS=I.dl/1000;
  const seen=t=>{ const k=Math.floor((t-dS)/dt); return k<0?null:k; };
  let e2=0,ed2=0,ea2=0,ew2=0,n=0;
  for(let i=Math.round(1*fs);i<N;i++){ const t=tt[i], k=seen(t); if(k==null) continue;
    e2+=(flt[k]-tr[i])**2; ed2+=(tr[k]-tr[i])**2; ea2+=(acc[i]-tr[i])**2; ew2+=(wm[k]-om(t))**2; n++; }
  const rms=Math.sqrt(e2/n), rmsD=Math.sqrt(ed2/n), rmsA=Math.sqrt(ea2/n), rmsW=Math.sqrt(ew2/n), drift=gyr[N-1]-tr[N-1];
  // plot
  const sv=$('iview'); sv.innerHTML='';
  const L=48,R=625,T=26,Bo=300, X=t=>L+(R-L)*t/4; const ymax=9; const Y=v=>(T+Bo)/2-(Bo-T)/2*Math.max(-ymax,Math.min(ymax,v))/ymax;
  for(let v=-8;v<=8;v+=4){ el('line',{x1:L,y1:Y(v),x2:R,y2:Y(v),stroke:'#1f2731'},sv); el('text',{x:L-6,y:Y(v)+4,'text-anchor':'end'},sv,v+'°'); }
  for(let s=0;s<=4;s++) el('text',{x:X(s),y:Bo+16,'text-anchor':'middle'},sv,s+' s');
  el('text',{x:L,y:16,class:'ttl'},sv,'Pelvis pitch tilt (simulated walking rock)');
  const step=Math.max(1,Math.floor(N/700));
  const line=(arr,col,w,dash,shift)=>{ let d=''; for(let i=0;i<N;i+=step){ const j=shift?seen(tt[i]):i; if(j==null) continue; d+=(d?'L':'M')+X(tt[i]).toFixed(1)+','+Y(arr[j]).toFixed(1);} el('path',{d,stroke:col,'stroke-width':w,fill:'none','stroke-dasharray':dash||'none'},sv); };
  line(acc,'#5d6b7a',1); line(gyr,'#b48cff',1.5,'5 3'); line(tr,'#e6edf3',2); line(flt,'#4fb3ff',2,null,true);
  // zoom inset showing delay
  ro('i_f',rms.toFixed(2)+'°', rms<0.3?'ok':rms<0.8?'warn':'bad');
  ro('i_d',rmsD.toFixed(2)+'°');
  ro('i_a',rmsA.toFixed(2)+'°');
  ro('i_g',drift.toFixed(2)+'°', Math.abs(drift)<0.5?'ok':'warn');
  ro('i_w',rmsW.toFixed(1)+' °/s');
  I._r={rms,rmsD,rmsA,rmsW};
  note();
}
bindAll(I,IF,imu);
document.querySelectorAll('[data-ip]').forEach(b=>b.addEventListener('click',()=>setAll(I,IF,IP[b.dataset.ip],imu)));
$('reseed').addEventListener('click',()=>{seed=(seed*7919+13)%100000; imu();});

/* ---------- C: compute ---------- */
const PM={maxn:{cpu:12,ff:1,gpu:1,lab:'MAXN'},50:{cpu:12*1497.6/2201.6,ff:1497.6/2201.6,gpu:816/1301,lab:'50 W'},30:{cpu:8*1728/2201.6,ff:1728/2201.6,gpu:612/1301,lab:'30 W'},15:{cpu:4*1113.6/2201.6,ff:1113.6/2201.6,gpu:408/1301,lab:'15 W'}};
const NETS={flat:{inp:75,h:[256,128,128],lab:'256-128-128'},rough:{inp:210,h:[512,256,128],lab:'512-256-128'},big:{inp:75*64,h:[1024,1024,512],lab:'1024-1024-512 with 64-step history'}};
const CD={pm:'maxn',w_rt:true,w_pol:true,w_cam:true,w_map:false,w_log:true,w_tel:false,w_vla:false,pr:50,net:'flat',lg:10,vh:2};
const C={...CD};
const CP={today:{...CD},stairs:{...CD,net:'rough',w_map:true},vla:{...CD,w_vla:true,vh:2},low:{...CD,pm:'30',w_vla:true,vh:2,w_map:true,net:'rough'}};
function readC(){ C.pm=$('pm').value; C.net=$('net').value; ['w_rt','w_pol','w_cam','w_map','w_log','w_tel','w_vla'].forEach(k=>C[k]=$(k).checked); C.pr=+$('pr').value; C.lg=+$('lg').value; C.vh=+$('vh').value; }
function writeC(){ $('pm').value=C.pm; $('net').value=C.net; ['w_rt','w_pol','w_cam','w_map','w_log','w_tel','w_vla'].forEach(k=>$(k).checked=C[k]); $('pr').value=C.pr; $('lg').value=C.lg; $('vh').value=C.vh; }
function comp(){
  readC(); $('o_pr').textContent=C.pr+' Hz'; $('o_lg').textContent=C.lg+' MB/s'; $('o_vh').textContent=C.vh.toFixed(1)+' /s';
  const pm=PM[C.pm], net=NETS[C.net];
  let macs=0, prev=net.inp; net.h.forEach(h=>{macs+=prev*h;prev=h;}); macs+=prev*12;
  const flops=2*macs, cpuGF=5; // effective GFLOP/s per core for small MLP (estimate)
  const tInf=(flops/(cpuGF*1e9)*1000 + 0.1)/pm.ff; // ms, incl. ~0.1 ms runtime overhead, slower at lower clocks
  const W=[];
  if(C.w_rt) W.push({k:'Real-time process',cpu:1.0,gpu:0,mem:0.2,c:'#ffb454'});
  if(C.w_pol) W.push({k:'Policy '+net.lab+' at '+C.pr+' Hz',cpu:tInf*C.pr/1000,gpu:0,mem:0.05,c:'#4cc38a'});
  if(C.w_cam) W.push({k:'Depth stream',cpu:0.3,gpu:0,mem:0.3,c:'#4fb3ff'});
  if(C.w_map) W.push({k:'Height map (GPU)',cpu:0.4,gpu:0.10,mem:1.0,c:'#7fc8ff'});
  if(C.w_log) W.push({k:'Logging '+C.lg+' MB/s',cpu:0.1+C.lg*0.01,gpu:0,mem:0.5,c:'#9aa7b4'});
  if(C.w_tel) W.push({k:'Teleop video',cpu:0.5,gpu:0.03,mem:0.3,c:'#d2b8ff'});
  const vlaG=C.vh*0.2755; if(C.w_vla) W.push({k:'VLA '+C.vh.toFixed(1)+'/s',cpu:1.0,gpu:vlaG,mem:16,c:'#b48cff'});
  W.push({k:'OS + ROS 2 base',cpu:0.5,gpu:0,mem:4,c:'#56606c'});
  const capC=pm.cpu, capG=pm.gpu; // gpu demand in MAXN-fractions
  const totC=W.reduce((a,w)=>a+w.cpu,0), totG=W.reduce((a,w)=>a+w.gpu,0), totM=W.reduce((a,w)=>a+w.mem,0);
  const bar=(id,items,key,cap,scale)=>{ const b=$(id); b.innerHTML=''; let x=0; items.forEach(w=>{ const v=w[key]; if(v<=0) return; const s=document.createElement('span'); s.style.left=(x/scale*100)+'%'; s.style.width=(Math.min(v,scale-x)/scale*100)+'%'; s.style.background=w.c; s.title=w.k+': '+v.toFixed(3); b.appendChild(s); x+=v; });
    const c=document.createElement('i'); c.className='cap'; c.style.left='calc('+(cap/scale*100)+'% - 1px)'; b.appendChild(c); };
  bar('cpubar',W,'cpu',capC,12); bar('gpubar',W,'gpu',capG,1); bar('membar',W,'mem',64,64);
  $('cpuv').textContent=totC.toFixed(2)+' of '+capC.toFixed(1); $('gpuv').textContent=(totG*100).toFixed(0)+'% of '+(capG*100).toFixed(0)+'%'; $('memv').textContent=totM.toFixed(1)+' GB';
  $('cpuv').style.color=totC>capC*0.8?'var(--bad)':''; $('gpuv').style.color=totG>capG*0.9?'var(--bad)':'';
  $('clegend').innerHTML=W.map(w=>`<span><i style="background:${w.c}"></i>${w.k}</span>`).join('')+'<span>white tick = this power mode\'s capacity</span>';
  ro('c_pol',tInf.toFixed(2)+' ms ('+(flops/1e6).toFixed(2)+' MFLOP)', tInf<1?'ok':'warn');
  const left=capG-(totG-(C.w_vla?vlaG:0)); const vmax=Math.max(0,left)/0.2755;
  ro('c_vmax',vmax.toFixed(1)+' chunks/s', vmax>=2?'ok':vmax>=1?'warn':'bad');
  ro('c_head',(capC-totC).toFixed(1)+' cores', capC-totC>2?'ok':capC-totC>0?'warn':'bad');
  ro('c_ghead',((capG-totG)*100).toFixed(0)+'%', capG-totG>0.2?'ok':capG-totG>0?'warn':'bad');
  C._r={totC,totG,totM,capC,capG,tInf,vmax,pm:pm.lab,net:net.lab};
  note();
}
['pm','net','w_rt','w_pol','w_cam','w_map','w_log','w_tel','w_vla','pr','lg','vh'].forEach(k=>$(k).addEventListener($(k).tagName==='SELECT'||$(k).type==='checkbox'?'change':'input',comp));
document.querySelectorAll('[data-cp]').forEach(b=>b.addEventListener('click',()=>{ Object.assign(C,CP[b.dataset.cp]); writeC(); comp(); }));

/* ---------- note ---------- */
function note(){
  if(!S._r||!I._r||!C._r) return;
  const p=[];
  const sd=[]; for(const k in SD) if(S[k]!==SD[k]) sd.push(k);
  if(sd.length) p.push(`Depth camera: baseline ${S.B} mm, ${S.W} px wide, ${S.F}° horizontal FOV, ${S.sd} px disparity precision, ${S.D} px search range gives f = ${S._r.f.toFixed(0)} px, depth error ${(S._r.sz2*100).toFixed(1)} cm at 2 m, within ${S.tol} cm out to ${S._r.zmax.toFixed(2)} m, no depth closer than ${S._r.zmin.toFixed(2)} m (Gemini 335L datasheet: 95 mm, ≤0.8% at 2 m).`);
  const id=[]; for(const k in ID) if(I[k]!==ID[k]) id.push(k);
  if(id.length) p.push(`IMU: gyro ${I.gn} °/s/√Hz with ${I.gb} °/s residual bias, accelerometer ${I.an} µg/√Hz, ${I.fs} Hz, ${I.dl} ms delay to the policy, complementary filter τ = ${I.tau} s, ${I.bump} m/s² footstep accelerations: tilt error the policy sees ${I._r.rms.toFixed(2)}° RMS (${I._r.rmsD.toFixed(2)}° of it from delay), angular-velocity error ${I._r.rmsW.toFixed(1)} °/s RMS. Training randomises 0–20 ms of observation delay; R42 asks for ≥500 Hz.`);
  const cd=[]; for(const k in CD) if(C[k]!==CD[k]) cd.push(k);
  if(cd.length) p.push(`Orin budget (${C._r.pm}, rough estimates): ${C.w_vla?'VLA at '+C.vh.toFixed(1)+' chunks/s, ':''}${C.w_map?'GPU height map, ':''}policy ${C._r.net} at ${C.pr} Hz (${C._r.tInf.toFixed(2)} ms): CPU ${C._r.totC.toFixed(1)} of ${C._r.capC.toFixed(1)} core-equivalents, GPU ${(C._r.totG*100).toFixed(0)}% of ${(C._r.capG*100).toFixed(0)}% available, memory ${C._r.totM.toFixed(0)} GB of 64; room for ${C._r.vmax.toFixed(1)} VLA chunks/s.`);
  $('note').textContent=p.length?`For Marvin Bot (1.44 m, 41 kg humanoid; pelvis IMU, Orbbec Gemini 335L in the head, Jetson AGX Orin 64 GB, 50 Hz policy): ${p.join(' ')} To act on: confirm with bench measurements (IMU end-to-end delay, depth noise on real stairs, tegrastats on the Orin).`:'Change a control in any playground above to build a note.';
}
$('copy').addEventListener('click',()=>{navigator.clipboard.writeText($('note').textContent).then(()=>{const b=$('copy');b.textContent='Copied!';setTimeout(()=>b.textContent='Copy',1200);});});
stereo(); imu(); comp();
