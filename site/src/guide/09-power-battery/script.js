const $=id=>document.getElementById(id);
/* ---------------- Playground 1: runtime ---------------- */
const DEFAULTS={S:13,P:3,Q:4.5,Vc:3.6,Vch:4.1,stop:15,mStand:30,mWalk:60,mStair:10,fW:45,fS:75,orin:30,pk:70};
const state={...DEFAULTS};
const PRESETS={today:{...DEFAULTS},s2p:{...DEFAULTS,P:2},full42:{...DEFAULTS,Vch:4.2},stairs:{...DEFAULTS,mStand:20,mWalk:40,mStair:40},maxn:{...DEFAULTS,orin:60,mStand:50,mWalk:50,mStair:0}};
const FMT={S:v=>v+'S',P:v=>v+'P',Q:v=>(+v).toFixed(1)+' Ah',Vc:v=>(+v).toFixed(2)+' V',Vch:v=>v+' V',stop:v=>v+'%',mStand:v=>v+'%',mWalk:v=>v+'%',mStair:v=>v+'%',orin:v=>(+v===60?'MAXN':v+' W'),pk:v=>v+'%'};
const MASS=41.3, G=9.81, CELL_G=70, OVERHEAD=0.25, CELL_A=45, CELL_R=0.0138;
// rated copper loss per motor model, W (RS04 from 0.16 ohm line-line; others scaled by mass: estimates)
const RS04=1.5*Math.pow(40/2.1,2)*0.16, RS03=RS04*0.90/1.42, RS06=RS04*0.621/1.42, RS00=RS04*0.31/1.42;
const LEG_CU=6*RS04+6*RS03;                 // 12 leg motors at rated
const ARM_CU=(5*RS03+4*RS06+2*RS00)*0.15*0.15; // waist + arms at 15% RMS
const ELEC=30;
function modePower(f, mech){ // f = leg RMS fraction
  const comp=state.orin/0.9, cu=LEG_CU*f*f+ARM_CU, m=mech/0.85; const base=comp+ELEC+cu+m; const loss=0.10*base;
  return {comp,elec:ELEC,cu,mech:m,loss,total:base+loss};
}
function calc(){
  const s=state;
  const Vnom=s.S*s.Vc, Ah=s.P*s.Q, E=Vnom*Ah;
  const chargeFrac= +s.Vch===4.1?0.86:1.0;
  const usable=E*Math.max(0,chargeFrac - s.stop/100);
  const walkMech=0.1*MASS*G*0.5, stairMech=MASS*G*0.09;
  const modes={stand:modePower(0.15,0),walk:modePower(s.fW/100,walkMech),stair:modePower(s.fS/100,stairMech)};
  const sum=s.mStand+s.mWalk+s.mStair||1; const w={stand:s.mStand/sum,walk:s.mWalk/sum,stair:s.mStair/sum};
  const Pavg=w.stand*modes.stand.total+w.walk*modes.walk.total+w.stair*modes.stair.total;
  const T=usable/Pavg;
  const mass=s.S*s.P*CELL_G*(1+OVERHEAD)/1000;
  // peak: stance leg 3 RS04 + 3 RS03 at pk% of peak torque, 4 rad/s
  const om=4, k=s.pk/100;
  const t4=120*k, t3=60*k;
  const p4=1.5*Math.pow(t4/2.1,2)*0.16 + t4*om/0.85;
  const p3=RS03*Math.pow(t3/20,2) + t3*om/0.85;
  const Ppk=3*p4+3*p3+modes.walk.comp+ELEC;
  const Vlow=s.S*3.5; const Rp=s.S*CELL_R/s.P;
  const Ipk=Ppk/Vlow; const Ilim=s.P*CELL_A;
  return {Vnom,Ah,E,usable,modes,w,Pavg,T,mass,Ipk,Ilim,Ppk,Rp,chargeFrac};
}
function bind(id,st,fmt,cb,outId){ const el=$(id), out=$(outId||('o_'+id));
  const upd=()=>{ st[id]= el.type==='checkbox'? el.checked : parseFloat(el.value); if(out) out.textContent=fmt[id]?fmt[id](st[id]):st[id]; cb(); };
  el.addEventListener('input',upd); el.addEventListener('change',upd); if(out) out.textContent=fmt[id]?fmt[id](el.type==='checkbox'?el.checked:el.value):el.value; }
function setAll(st,s,fmt,cb,outMap){ Object.assign(st,s); for(const k in s){ const el=$(k); if(!el) continue; if(el.type==='checkbox') el.checked=!!s[k]; else el.value=s[k]; const o=$((outMap&&outMap[k])||('o_'+k)); if(o) o.textContent=fmt[k]?fmt[k](s[k]):s[k]; } cb(); }
function ro(id,txt,cls){ const e=$(id); e.querySelector('.v').textContent=txt; e.className='ro'+(cls?' '+cls:''); }
function renderRT(){
  const r=calc(), s=state;
  ro('r_E',r.E.toFixed(0)+' Wh'); ro('r_U',r.usable.toFixed(0)+' Wh');
  ro('r_P',r.Pavg.toFixed(0)+' W');
  ro('r_T',(r.T).toFixed(2)+' h', r.T>=1.2?'ok':r.T>=1?'warn':'bad');
  ro('r_M',r.mass.toFixed(2)+' kg');
  ro('r_I',r.Ipk.toFixed(0)+' / '+r.Ilim+' A', r.Ipk<=0.7*r.Ilim?'ok':r.Ipk<=r.Ilim?'warn':'bad');
  ro('r_Vr',(s.S*4.2).toFixed(1)+' / '+(s.S*s.Vch).toFixed(1)+' / ~'+(s.S*3.45).toFixed(0)+' V', s.S*4.2<58?'':'bad');
  ro('r_C',(r.Ipk/r.Ah).toFixed(1)+' C');
  // drawing
  let g='';
  // pack grid
  g+=`<text x="16" y="20" fill="#9aa7b4" font-size="12">Pack: ${s.S}S${s.P}P = ${s.S*s.P} cells, ${r.Vnom.toFixed(1)} V nominal, ${r.Ah.toFixed(1)} Ah</text>`;
  const cw=Math.min(18,560/ s.S), ch=Math.min(18,70/s.P);
  for(let i=0;i<s.S;i++) for(let j=0;j<s.P;j++){ g+=`<rect x="${20+i*(cw+3)}" y="${30+j*(ch+3)}" width="${cw}" height="${ch}" rx="3" fill="#24527a" stroke="#4fb3ff" stroke-width=".8"/>`; }
  g+=`<text x="${24+s.S*(cw+3)}" y="${36+ch/2}" fill="#9aa7b4" font-size="11">← series adds volts</text>`;
  g+=`<text x="${24+s.S*(cw+3)}" y="${52+ch/2}" fill="#9aa7b4" font-size="11">↕ parallel adds amp-hours</text>`;
  // stacked bars per mode
  const y0=130, bh=26, maxP=Math.max(700,r.modes.stair.total*1.05); const sx=v=>130+430*v/maxP;
  g+=`<text x="16" y="${y0-8}" fill="#9aa7b4" font-size="12">Power by activity (estimates), W</text>`;
  [['standing','stand'],['walking','walk'],['stairs','stair']].forEach(([lab,k],i)=>{
    const m=r.modes[k], y=y0+i*(bh+12); let x=sx(0);
    g+=`<text x="16" y="${y+18}" fill="#e6edf3" font-size="12">${lab} (${Math.round(r.w[k]*100)}%)</text>`;
    [['comp','#4fb3ff'],['elec','#9aa7b4'],['cu','#ff6b6b'],['mech','#4cc38a'],['loss','#56606c']].forEach(([kk,c])=>{ const ww=sx(m[kk])-sx(0); g+=`<rect x="${x}" y="${y}" width="${Math.max(0,ww)}" height="${bh}" fill="${c}"/>`; x+=ww; });
    g+=`<text x="${x+6}" y="${y+18}" fill="#e6edf3" font-size="12">${m.total.toFixed(0)} W</text>`;
  });
  // avg line
  g+=`<line x1="${sx(r.Pavg)}" x2="${sx(r.Pavg)}" y1="${y0-4}" y2="${y0+3*(bh+12)-6}" stroke="#ffb454" stroke-dasharray="4 3"/><text x="${sx(r.Pavg)+4}" y="${y0+3*(bh+12)+6}" fill="#ffb454" font-size="11">mix average ${r.Pavg.toFixed(0)} W</text>`;
  // runtime bar
  const ry=300, maxT=3; const tx=v=>130+430*Math.min(v,maxT)/maxT;
  g+=`<text x="16" y="${ry-10}" fill="#9aa7b4" font-size="12">Runtime on this mix (usable ${r.usable.toFixed(0)} Wh: charge to ${s.Vch} V/cell ≈ ${Math.round(r.chargeFrac*100)}% of full, stop at ${s.stop}%)</text>`;
  g+=`<text x="16" y="${ry+20}" fill="#e6edf3" font-size="12">runtime</text>`;
  g+=`<rect x="${tx(0)}" y="${ry}" width="${tx(maxT)-tx(0)}" height="28" fill="#121820" stroke="#2a3340"/>`;
  g+=`<rect x="${tx(0)}" y="${ry}" width="${tx(r.T)-tx(0)}" height="28" fill="${r.T>=1.2?'#4cc38a':r.T>=1?'#ffb454':'#ff6b6b'}" opacity=".85"/>`;
  for(let h=0;h<=maxT;h+=0.5){ g+=`<line x1="${tx(h)}" x2="${tx(h)}" y1="${ry+28}" y2="${ry+34}" stroke="#56606c"/><text x="${tx(h)}" y="${ry+47}" fill="#9aa7b4" font-size="10" text-anchor="middle">${h} h</text>`; }
  g+=`<line x1="${tx(1)}" x2="${tx(1)}" y1="${ry-4}" y2="${ry+32}" stroke="#ff6b6b" stroke-width="2"/><text x="${tx(1)}" y="${ry+64}" fill="#ff6b6b" font-size="11" text-anchor="middle">R30: ≥ 1 h</text>`;
  g+=`<text x="${Math.min(tx(r.T)+6,560)}" y="${ry+19}" fill="#e6edf3" font-size="13" font-weight="600">${r.T.toFixed(2)} h${r.T>maxT?' →':''}</text>`;
  $('rt').innerHTML=g;
  updateNote();
}
['S','P','Q','Vc','Vch','stop','mStand','mWalk','mStair','orin','pk'].forEach(id=>bind(id,state,FMT,renderRT));
FMT.fW=v=>v+'%'; FMT.fS=v=>v+'%';
bind('fW',state,FMT,renderRT,'fW_o'); bind('fS',state,FMT,renderRT,'fS_o');
document.querySelectorAll('[data-preset]').forEach(b=>b.addEventListener('click',()=>{ setAll(state,PRESETS[b.dataset.preset],FMT,renderRT,{fW:'fW_o',fS:'fS_o'}); document.querySelectorAll('[data-preset]').forEach(x=>x.classList.toggle('on',x===b)); }));

/* ---------------- Playground 2: regen ---------------- */
const RDEF={src:0,soc:86,vb:48,J:0.8,w:6,n:1,tb:150,eta:0.7,C:5,R:75,clamp:false,Vcl:57};
const rs={...RDEF};
const RPRE={normal:{...RDEF},hard:{...RDEF,soc:100,w:10,n:4,tb:20},blocked:{...RDEF,src:1,soc:100,n:2},bench:{...RDEF,src:2,vb:48,n:2,clamp:false},benchc:{...RDEF,src:2,vb:48,n:2,clamp:true,Vcl:57}};
const RFMT={src:v=>['battery','blocked','bench'][v],soc:v=>v+'%',vb:v=>(+v).toFixed(1)+' V',J:v=>(+v).toFixed(2)+' kg·m²',w:v=>(+v).toFixed(1)+' rad/s',n:v=>v,tb:v=>v+' ms',eta:v=>(+v).toFixed(2),C:v=>(+v).toFixed(1)+' mF',R:v=>v+' mΩ',Vcl:v=>(+v).toFixed(1)+' V'};
// typical NMC open-circuit voltage per cell vs SoC (approximate shape)
const OCV=[[0,3.0],[5,3.3],[10,3.45],[20,3.56],[30,3.63],[40,3.69],[50,3.75],[60,3.83],[70,3.91],[80,3.99],[86,4.1],[90,4.13],[100,4.2]];
function ocv(soc){ for(let i=1;i<OCV.length;i++){ if(soc<=OCV[i][0]){ const [a,va]=OCV[i-1],[b,vb]=OCV[i]; return va+(vb-va)*(soc-a)/(b-a);} } return 4.2; }
function renderRG(){
  const s=rs; const S=13;
  const V0 = s.src==2 ? s.vb : S*ocv(s.soc);
  const tmin=s.J*s.w/120; const tb=Math.max(s.tb/1000,tmin); const limited=tb>s.tb/1000+1e-9; const J=s.J*s.n, eta=s.eta, C=s.C/1000, R=s.R/1000;
  const E=0.5*J*s.w*s.w, Eb=eta*E;
  const p0=eta*J*s.w*s.w/tb; // power at t=0 (constant deceleration)
  const N=200; const pts=[]; let Vpk=V0, dump=0, Ecap=0;
  let V=V0;
  for(let i=0;i<=N;i++){
    const t=tb*i/N; const p=p0*(1-t/tb);
    if(s.src==0){ V=(V0+Math.sqrt(V0*V0+4*p*R))/2; if(s.clamp&&V>s.Vcl) V=s.Vcl; }
    else { const Et=p0*(t - t*t/(2*tb)); let v=Math.sqrt(V0*V0+2*Et/C); if(s.clamp && v>s.Vcl){ v=s.Vcl; } V=v; }
    pts.push([t,V]); if(V>Vpk) Vpk=V;
  }
  if(s.src!=0 && s.clamp){ const cap=0.5*C*(s.Vcl*s.Vcl-V0*V0); dump=Math.max(0,Eb-Math.max(0,cap)); }
  if(s.src==0 && s.clamp){ /* clamp only conducts above Vcl */ }
  ro('g_V0',V0.toFixed(1)+' V');
  ro('g_E',Eb.toFixed(1)+' J');
  ro('g_P',p0.toFixed(0)+' W / '+(p0/V0).toFixed(0)+' A');
  ro('g_Vp',Vpk.toFixed(1)+' V', Vpk<56?'ok':Vpk<60?'warn':'bad');
  ro('g_M',(60-Vpk).toFixed(1)+' V', 60-Vpk>=3?'ok':60-Vpk>0?'warn':'bad');
  ro('g_D',s.clamp? dump.toFixed(1)+' J':'no clamp');
  // plot
  const x0=60,x1=620,yT=20,yB=280; const vmin=40, vmax=Math.max(66,Math.min(120,Vpk*1.05));
  const sx=t=>x0+(x1-x0)*t/tb, sy=v=>yB-(yB-yT)*(Math.min(v,vmax)-vmin)/(vmax-vmin);
  let g=`<rect x="${x0}" y="${yT}" width="${x1-x0}" height="${yB-yT}" fill="#0c1015" stroke="#2a3340"/>`;
  for(let v=40; v<=vmax; v+= vmax>80?10:5){ g+=`<line x1="${x0}" x2="${x1}" y1="${sy(v)}" y2="${sy(v)}" stroke="#1c232c"/><text x="${x0-6}" y="${sy(v)+4}" fill="#9aa7b4" font-size="10" text-anchor="end">${v} V</text>`; }
  g+=`<line x1="${x0}" x2="${x1}" y1="${sy(60)}" y2="${sy(60)}" stroke="#ff6b6b" stroke-width="2"/><text x="${x1-4}" y="${sy(60)-5}" fill="#ff6b6b" font-size="11" text-anchor="end">60 V actuator limit</text>`;
  g+=`<line x1="${x0}" x2="${x1}" y1="${sy(54.6)}" y2="${sy(54.6)}" stroke="#9aa7b4" stroke-dasharray="3 3"/><text x="${x0+4}" y="${sy(54.6)-4}" fill="#9aa7b4" font-size="10">54.6 V full pack</text>`;
  if(s.clamp) g+=`<line x1="${x0}" x2="${x1}" y1="${sy(s.Vcl)}" y2="${sy(s.Vcl)}" stroke="#ffb454" stroke-dasharray="6 3"/><text x="${x0+4}" y="${sy(s.Vcl)-4}" fill="#ffb454" font-size="10">clamp ${s.Vcl} V</text>`;
  g+=`<polyline fill="none" stroke="#4fb3ff" stroke-width="2.5" points="${pts.map(p=>sx(p[0]).toFixed(1)+','+sy(p[1]).toFixed(1)).join(' ')}"/>`;
  if(Vpk>vmax) g+=`<text x="${(x0+x1)/2}" y="${yT+16}" fill="#ff6b6b" font-size="12" text-anchor="middle">off the chart: ${Vpk.toFixed(0)} V</text>`;
  for(let i=0;i<=5;i++){ const t=tb*i/5; g+=`<text x="${sx(t)}" y="${yB+16}" fill="#9aa7b4" font-size="10" text-anchor="middle">${(t*1000).toFixed(0)}</text>`; }
  g+=`<text x="${x1}" y="${yB+32}" fill="#9aa7b4" font-size="10" text-anchor="end">time during braking, ms</text>`;
  if(limited) g+=`<text x="${x1}" y="${yT+16}" fill="#ffb454" font-size="11" text-anchor="end">braking stretched to ${(tb*1000).toFixed(0)} ms: 120 N·m peak torque limit</text>`;
  g+=`<text x="${x0}" y="${yB+32}" fill="#e6edf3" font-size="11">${['Battery absorbs: V = V_OC + I·R','Blocked: energy fills the bus capacitors','Bench supply: cannot sink current'][s.src]}</text>`;
  $('rg').innerHTML=g;
  updateNote();
}
['src','soc','vb','J','w','n','tb','eta','C','R','clamp','Vcl'].forEach(id=>bind(id,rs,RFMT,renderRG));
document.querySelectorAll('[data-rpreset]').forEach(b=>b.addEventListener('click',()=>{ setAll(rs,RPRE[b.dataset.rpreset],RFMT,renderRG); document.querySelectorAll('[data-rpreset]').forEach(x=>x.classList.toggle('on',x===b)); }));

/* ---------------- Playground 3: topology ---------------- */
const topo={estop:false,pulled:false};
const BLOCKS={
 bat:{x:20,y:170,w:110,h:60,t:['13S3P pack','+ BMS'],side:'main',info:'<b>13S3P pack + BMS</b><p>39 Molicel P45B 21700 cells, 46.8 V nominal, 54.6 V full, about 630 Wh and 3.4 kg. The battery management system (BMS) watches every series group\'s voltage and the pack temperature and current, balances the cells, and opens its own MOSFET switches on over-current, over- or under-voltage and over-temperature. The BOM lists a 13S 150 A smart BMS with CAN telemetry ($145); it reports state of charge to the computer on the battery CAN bus at 10 Hz. The pack slides into the back of the torso on a cradle with an anti-spark AS150-class blind-mate connector.</p>'},
 fuse:{x:160,y:170,w:90,h:60,t:['Class-T fuse','100 A'],side:'main',info:'<b>Class-T fuse, 100 A</b><p>The first thing after the pack. Its job is the worst case: a hard short, where a lithium pack can deliver thousands of amps. Class-T fuses are fast, current-limiting fuses with a high DC interrupting rating (20 kA DC for the Littelfuse JLLN range<sup class="cite"><a href="#s8">[8]</a></sup>), so they break such a fault without sustaining an arc. A fuse only protects the wire and the pack; it does nothing for small overloads. <code>harness.yaml</code>: main path 78.5 A continuous estimate, 100 A fuse, 6 AWG.</p>'},
 pre:{x:290,y:110,w:120,h:50,t:['Precharge','R + relay'],side:'act',info:'<b>Precharge resistor and relay</b><p>Sits across the contactor. Before the contactor closes, the relay connects the bus to the pack through a resistor of a few tens of ohms, so the drivers\' input capacitors fill in about a second instead of drawing hundreds of amps for a few microseconds. When the bus is within a few volts of the pack, the contactor closes. See "Precharge, in numbers" below.</p>'},
 con:{x:290,y:200,w:120,h:50,t:['Main contactor','48 V DC'],side:'act',info:'<b>Main contactor</b><p>A heavy-duty relay rated for DC that switches all actuator power. Its coil is held closed only while the whole e-stop chain is closed: wired button, wireless receiver and watchdog relay in series. Open any of them and the coil drops out, the contacts open, and every actuator loses power: an IEC 60204-1 category 0 stop. The computer is fed from before the contactor, so it stays up.</p>'},
 tpdb:{x:450,y:200,w:120,h:50,t:['Torso power board','bus bars, fuses'],side:'act',info:'<b>Torso power board</b><p>Bus bars and branch fuses: 20 A to each arm harness, the 12 V DC-DC for the servos, and an 80 A trunk across the waist to the pelvis board. Branch fuses protect each limb\'s thinner wire: a short in one arm blows its 20 A fuse and leaves the rest of the robot running. <code>harness.yaml</code>: each arm 13.6 A continuous estimate, 16 AWG.</p>'},
 arms:{x:610,y:150,w:130,h:44,t:['Arms 2 × 20 A'],side:'act',info:'<b>Arm branches, 20 A each</b><p>Power is daisy-chained motor to motor down each arm, with the arm CAN pair alongside. The waist motor sits on the right-arm CAN bus but takes its power from the pelvis board.</p>'},
 ppdb:{x:610,y:205,w:130,h:44,t:['Pelvis board','legs 2 × 40 A'],side:'act',info:'<b>Pelvis power board</b><p>Fed by the 80 A trunk (8 AWG, 0.68 m, across the waist). Feeds each leg through a 40 A fuse (24.3 A continuous estimate; 12 AWG at the root, 16 AWG at the ankle) and the waist motor.</p>'},
 dc12:{x:610,y:262,w:130,h:44,t:['DC-DC 12 V','servos'],side:'act',info:'<b>12 V DC-DC for the servos</b><p>A compact 10 A converter in the pelvis channel (at most 60 × 34 × 25 mm) that feeds the Feetech servos (neck, wrist pitch) and, through 5 V converters at the wrists, the hands. Rated for 75 V input so a regen spike cannot destroy it. After the contactor, so an e-stop also stops the servos.</p>'},
 dcc:{x:290,y:320,w:120,h:50,t:['DC-DC 19-20 V','75 V input'],side:'main',info:'<b>Computer DC-DC, 19-20 V, ≥120 W</b><p>Fed straight from the fuse, <i>ahead</i> of the contactor, so an e-stop never reboots the computer and logs survive. Rated for 75 V input because the bus can rise above the pack voltage during regen.</p>'},
 or:{x:450,y:320,w:100,h:50,t:['Ideal-diode OR'],side:'comp',info:'<b>Ideal-diode OR (LTC4359 class)</b><p>Two supplies feed the computer through two "ideal diodes": MOSFETs driven by a controller so they behave like diodes with almost no voltage drop.<sup class="cite"><a href="#s7">[7]</a></sup> Whichever source is higher supplies the load, and neither can back-feed the other. When the main pack is pulled, the keep-alive battery takes over without a gap.</p>'},
 ka:{x:290,y:380,w:120,h:46,t:['Keep-alive','50-100 Wh'],side:'ka',info:'<b>Keep-alive battery</b><p>A small 50-100 Wh pack low in the back of the torso that feeds only the computer. It keeps the Orin, its state and logs alive while the main pack is swapped (requirement R31). At the Orin\'s 30 W mode, 50 Wh is well over an hour of standby.</p>'},
 orin:{x:600,y:320,w:140,h:50,t:['Jetson AGX Orin','15-60 W'],side:'comp',info:'<b>Jetson AGX Orin 64 GB</b><p>Runs the policy, the 500 Hz CAN loop and the safety logic. Power modes of 15, 30 and 50 W plus MAXN.<sup class="cite"><a href="#s4">[4]</a></sup> Stays powered through any e-stop.</p>'},
 es:{x:20,y:20,w:230,h:60,t:['E-stop chain','wired + wireless + watchdog'],side:'es',info:'<b>E-stop chain</b><p>A latching wired mushroom button, a wireless e-stop receiver and a watchdog relay, all in series with the contactor coil. Any one opening drops the contactor. The watchdog relay opens if the control loop stops sending heartbeats. See <a href="../12-safety/">12 Safety</a> for the fault tree.</p>'}
};
function powered(id){
  const b=BLOCKS[id]; const packOn=!topo.pulled;
  if(b.side==='main') return packOn;
  if(b.side==='act') return packOn && !topo.estop;
  if(b.side==='comp') return true; // keep-alive OR
  if(b.side==='ka') return true;
  if(b.side==='es') return true;
}
function renderTopo(){
  let g='<defs><marker id="ar" markerUnits="userSpaceOnUse" markerWidth="10" markerHeight="10" refX="9" refY="5" orient="auto"><path d="M0,0 L10,5 L0,10 z" fill="#9aa7b4"/></marker></defs>';
  const line=(x1,y1,x2,y2,on,dash)=>`<line x1="${x1}" y1="${y1}" x2="${x2}" y2="${y2}" stroke="${on?'#4cc38a':'#3a424d'}" stroke-width="3" ${dash?'stroke-dasharray="6 4"':''} marker-end="url(#ar)"/>`;
  const P=id=>powered(id);
  g+=line(130,200,158,200,P('bat'));
  g+=line(250,200,288,225,P('bat'));
  g+=line(250,195,288,140,P('bat'));
  g+=line(410,135,450,215,P('con')); g+=line(410,225,448,225,P('con'));
  g+=line(570,215,608,172,P('tpdb')); g+=line(570,225,608,227,P('tpdb')); g+=line(570,235,608,284,P('tpdb'));
  g+=line(205,230,288,345,P('bat'));
  g+=line(410,345,448,345,P('dcc')); g+=line(410,405,448,355,true); g+=line(550,345,598,345,true);
  // contactor state drawing
  const open=topo.estop;
  g+=`<line x1="20" y1="80" x2="20" y2="80" />`;
  g+=`<path d="M135,50 C300,50 330,120 350,198" fill="none" stroke="#ff6b6b" stroke-width="2" stroke-dasharray="5 4"/><text x="268" y="70" fill="#ff6b6b" font-size="11">opens the contactor</text>`;
  for(const id in BLOCKS){ const b=BLOCKS[id]; const on=P(id);
    const stroke= id==='es'? '#ff6b6b' : on? '#4cc38a':'#3a424d';
    const fill= on? (id==='es'?'#2a1416':'#13261d') : '#161b22';
    g+=`<g class="blk" data-b="${id}" tabindex="0" role="button" aria-label="${b.t.join(' ')}"><rect x="${b.x}" y="${b.y}" width="${b.w}" height="${b.h}" rx="8" fill="${fill}" stroke="${stroke}" stroke-width="2"/>`;
    b.t.forEach((t,i)=>{ g+=`<text x="${b.x+b.w/2}" y="${b.y+b.h/2+(b.t.length===1?4:i?14:-4)}" fill="${on?'#e6edf3':'#6b7682'}" font-size="${i?11:12.5}" text-anchor="middle">${t}</text>`; });
    g+='</g>'; }
  if(open) g+=`<text x="350" y="268" fill="#ff6b6b" font-size="12" text-anchor="middle">contactor OPEN</text>`;
  if(topo.pulled) g+=`<text x="75" y="250" fill="#ff6b6b" font-size="12" text-anchor="middle">pack removed</text>`;
  g+=`<text x="600" y="395" fill="#9aa7b4" font-size="11">${topo.pulled?'Orin runs from keep-alive':'Orin runs from the pack'}</text>`;
  $('topo').innerHTML=g;
  document.querySelectorAll('#topo .blk').forEach(el=>{ const f=()=>{$('topoInfo').innerHTML=BLOCKS[el.dataset.b].info;}; el.addEventListener('click',f); el.addEventListener('keydown',e=>{ if(e.key==='Enter'||e.key===' ') f(); }); });
}
$('t_estop').addEventListener('click',()=>{ topo.estop=!topo.estop; $('t_estop').textContent=topo.estop?'Release e-stop':'Press e-stop'; $('t_estop').classList.toggle('on',topo.estop); renderTopo(); updateNote(); });
$('t_pull').addEventListener('click',()=>{ topo.pulled=!topo.pulled; $('t_pull').textContent=topo.pulled?'Insert the pack':'Pull the main pack (hot swap)'; $('t_pull').classList.toggle('on',topo.pulled); renderTopo(); updateNote(); });
$('t_reset').addEventListener('click',()=>{ topo.estop=false; topo.pulled=false; $('t_estop').textContent='Press e-stop'; $('t_pull').textContent='Pull the main pack (hot swap)'; $('t_estop').classList.remove('on'); $('t_pull').classList.remove('on'); renderTopo(); updateNote(); });

/* ---------------- note ---------------- */
function updateNote(){
  const parts=[], r=calc(), s=state;
  const lbl={S:'cells in series',P:'cells in parallel',Q:'cell capacity',Vc:'cell nominal voltage',Vch:'daily charge voltage',stop:'stop SoC',mStand:'standing share',mWalk:'walking share',mStair:'stairs share',fW:'walking leg RMS torque',fS:'stairs leg RMS torque',orin:'Orin mode',pk:'peak torque share'};
  for(const k in DEFAULTS) if(+s[k]!==+DEFAULTS[k]) parts.push(`${lbl[k]} ${(FMT[k]||(v=>v+'%'))(s[k])}`);
  const rparts=[]; const rl={src:'bus source',soc:'state of charge',vb:'bench voltage',J:'limb inertia',w:'joint speed',n:'limbs braking',tb:'braking time',eta:'regen share',C:'bus capacitance',R:'pack resistance',clamp:'clamp',Vcl:'clamp voltage'};
  for(const k in RDEF) if(String(rs[k])!==String(RDEF[k])) rparts.push(`${rl[k]} ${k==='clamp'?(rs[k]?'fitted':'none'):RFMT[k](rs[k])}`);
  if(!parts.length && !rparts.length && !topo.estop && !topo.pulled){ $('note').textContent='Change a control above to build a note. (Defaults: Marvin Bot\'s 13S3P pack of Molicel P45B cells, 4.1 V daily charge, 15% reserve, 30/60/10 standing/walking/stairs.)'; return; }
  let t=`For Marvin Bot (1.44 m, 41 kg humanoid; 48 V nominal 13S3P 21700 pack in the repo, ~630 Wh, 4.1 V/cell daily charge, actuators limited to 60 V, R30 runtime ≥ 1 h, R32 regen margin), I explored`;
  if(parts.length) t+=` a pack/load variant with ${parts.join(', ')}: ${s.S}S${s.P}P, ${r.E.toFixed(0)} Wh nominal, ${r.usable.toFixed(0)} Wh usable, ${r.Pavg.toFixed(0)} W average (first-principles estimate), runtime ${r.T.toFixed(2)} h, pack ~${r.mass.toFixed(2)} kg, peak ~${r.Ipk.toFixed(0)} A against a ${r.Ilim} A cell limit.`;
  else t+=` the default pack (${r.T.toFixed(2)} h estimated runtime).`;
  if(rparts.length){ const V0=rs.src==2?rs.vb:13*ocv(rs.soc); t+=` Regen case: ${rparts.join(', ')}; the peak bus voltage readout was ${$('g_Vp').querySelector('.v').textContent} from ${V0.toFixed(1)} V.`; }
  if(topo.estop||topo.pulled) t+=` Topology check: ${topo.estop?'with the e-stop pressed the contactor opens and all actuators, the 12 V servo rail and branch boards lose power while the computer stays up':''}${topo.estop&&topo.pulled?'; ':''}${topo.pulled?'with the pack pulled the computer runs from the keep-alive battery through the ideal-diode OR':''}.`;
  t+=' All power figures are estimates to be replaced by phase 2 bench measurements.';
  $('note').textContent=t;
}
$('copy').addEventListener('click',()=>{ navigator.clipboard.writeText($('note').textContent).then(()=>{ const b=$('copy'); b.textContent='Copied!'; setTimeout(()=>b.textContent='Copy',1200); }); });
renderRT(); renderRG(); renderTopo();
