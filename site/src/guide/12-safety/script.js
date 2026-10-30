const $=id=>document.getElementById(id);
/* ---------- Playground 1 ---------- */
const REG=[ // name, Fqs N, k N/mm, mH kg
 ['Skull and forehead',130,150,4.4],['Face',65,75,4.4],['Neck',150,50,1.2],['Back and shoulders',210,35,40],['Chest',140,25,40],['Abdomen',110,10,40],['Pelvis',180,25,40],['Upper arms and elbows',150,30,3],['Lower arms and wrists',160,40,2],['Hands and fingers',140,75,0.6],['Thighs and knees',220,50,75],['Lower legs',130,60,75]];
$('reg').innerHTML=REG.map((r,i)=>`<option value="${i}">${r[0]} (${r[1]} N, ${r[2]} N/mm, ${r[3]} kg)</option>`).join('');
const DEFAULTS={sc:0,lm:3,ll:0.57,w:5,r:0.57,fm:20,h:0.6,act:120,tl:100,pr:5,reg:4};
$('reg').value=4;
const state={...DEFAULTS};
const PRESETS={arm:{...DEFAULTS},fall:{...DEFAULTS,sc:1,reg:11},knee:{...DEFAULTS,sc:2,act:120,pr:5,reg:9},wrist:{...DEFAULTS,sc:2,act:4.9,pr:2,reg:9},slow:{...DEFAULTS,w:2.8}};
const FMT={sc:v=>['swing','fall','pinch'][v],lm:v=>(+v).toFixed(1)+' kg',ll:v=>(+v).toFixed(2)+' m',w:v=>(+v).toFixed(1)+' rad/s',r:v=>(+v).toFixed(2)+' m',fm:v=>(+v).toFixed(1)+' kg',h:v=>(+v).toFixed(2)+' m',act:v=>v+' N·m',tl:v=>v+'%',pr:v=>(+v).toFixed(1)+' cm',reg:v=>''};
function bind(id,st,fmt,cb){ const el=$(id), out=$('o_'+id);
  const upd=()=>{ st[id]= el.type==='checkbox'? el.checked : parseFloat(el.value); if(out) out.textContent=fmt[id]?fmt[id](st[id]):st[id]; cb(); };
  el.addEventListener('input',upd); el.addEventListener('change',upd); if(out) out.textContent=fmt[id]?fmt[id](el.value):el.value; }
function setAll(st,s,fmt,cb){ Object.assign(st,s); for(const k in s){ const el=$(k); if(!el) continue; if(el.type==='checkbox') el.checked=!!s[k]; else el.value=s[k]; const o=$('o_'+k); if(o) o.textContent=fmt[k]?fmt[k](s[k]):s[k]; } cb(); }
function ro(id,txt,cls){ const e=$(id); e.querySelector('.v').textContent=txt; e.className='ro'+(cls?' '+cls:''); }
function hazard(){
  const s=state, R=REG[s.reg]; const k=R[2]*1000, mH=R[3], Fqs=R[1], headReg=s.reg<=1, Ftr=(headReg?1:2)*Fqs;
  let v=0,mR=0,E=0,F=0,lim=Ftr,limName=headReg?'transient (no factor for skull and face)':'transient (2× quasi-static)',vmax=null,extra='';
  if(s.sc==0){ const r=Math.min(s.r,s.ll); const J=s.lm*s.ll*s.ll/3; mR=J/(r*r); v=s.w*r; E=0.5*J*s.w*s.w; const mu=1/(1/mH+1/mR); F=v*Math.sqrt(mu*k); vmax=Ftr/Math.sqrt(mu*k); extra=`ω for the threshold ≈ ${(vmax/r).toFixed(1)} rad/s`; }
  else if(s.sc==1){ mR=s.fm; v=Math.sqrt(2*9.81*s.h); E=s.fm*9.81*s.h; const mu=1/(1/mH+1/mR); F=v*Math.sqrt(mu*k); vmax=Ftr/Math.sqrt(mu*k); extra=`drop height for the threshold ≈ ${(vmax*vmax/(2*9.81)*100).toFixed(1)} cm`; }
  else { const tau=s.act*s.tl/100; F=tau/(s.pr/100); lim=Fqs; limName='quasi-static (clamping)'; E=0; v=0; mR=NaN; extra=`distance for the threshold ≈ ${(tau/Fqs*100).toFixed(1)} cm`; }
  return {R,v,mR,E,F,lim,limName,vmax,Fqs,Ftr,extra,k};
}
function renderHZ(){
  const s=state, h=hazard();
  $('g_swing').style.display=s.sc==0?'':'none'; $('g_fall').style.display=s.sc==1?'':'none'; $('g_pinch').style.display=s.sc==2?'':'none';
  ro('h_v', s.sc==2?'– (squeeze)':h.v.toFixed(2)+' m/s');
  ro('h_mr', s.sc==2?'–':h.mR.toFixed(2)+' kg');
  ro('h_E', s.sc==2?'–':h.E.toFixed(1)+' J');
  ro('h_F', h.F.toFixed(0)+' N', h.F<=h.lim?'ok':'bad');
  ro('h_L', h.lim.toFixed(0)+' N');
  ro('h_vmax', s.sc==2? h.extra : h.vmax.toFixed(2)+' m/s', '');
  // chart: log scale force bars
  const x0=170,x1=620; const lo=10, hi=Math.max(10000,h.F*1.5); const lx=f=>x0+(x1-x0)*Math.log10(Math.max(f,lo)/lo)/Math.log10(hi/lo);
  let g=`<text x="14" y="22" fill="#9aa7b4" font-size="12">${h.R[0]}: force, logarithmic scale (N)</text>`;
  for(const t of [10,30,100,300,1000,3000,10000,30000]) if(t<=hi){ g+=`<line x1="${lx(t)}" x2="${lx(t)}" y1="36" y2="190" stroke="#1c232c"/><text x="${lx(t)}" y="204" fill="#9aa7b4" font-size="10" text-anchor="middle">${t>=1000?t/1000+'k':t}</text>`; }
  const bar=(y,lab,val,col)=>`<text x="14" y="${y+17}" fill="#e6edf3" font-size="12">${lab}</text><rect x="${x0}" y="${y}" width="${lx(val)-x0}" height="24" fill="${col}" opacity=".85"/><text x="${lx(val)+6}" y="${y+17}" fill="#e6edf3" font-size="12">${val.toFixed(0)} N</text>`;
  g+=bar(44,'Estimated force',h.F,h.F<=h.lim?'#4cc38a':'#ff6b6b');
  g+=bar(84,'Quasi-static limit',h.Fqs,'#ffb454');
  g+=bar(124,'Transient limit',h.Ftr,'#b48cff');
  g+=`<text x="14" y="176" fill="#9aa7b4" font-size="11">${s.sc==2?'A pinch clamps the body, so the quasi-static limit applies.':'An impact where the body can move away: compared with the transient limit.'}</text>`;
  const ratio=h.F/h.lim;
  g+=`<text x="14" y="236" fill="${ratio>1?'#ff6b6b':'#4cc38a'}" font-size="16" font-weight="600">${ratio>1? ratio.toFixed(1)+'× over the '+h.limName+' threshold' : 'below the '+h.limName+' threshold ('+(ratio*100).toFixed(0)+'%)'}</text>`;
  g+=`<text x="14" y="260" fill="#e6edf3" font-size="12">${h.extra}${s.sc!=2?' · energy the region tolerates at the transient limit: '+(h.Ftr*h.Ftr/(2*h.k)).toFixed(2)+' J':''}</text>`;
  if(s.sc!=2) g+=`<text x="14" y="282" fill="#9aa7b4" font-size="12">Kinetic energy ${h.E.toFixed(1)} J (all of it, not only what reaches the body). Equivalent to dropping 1 kg from ${(h.E/9.81).toFixed(1)} m.</text>`;
  g+=`<text x="14" y="312" fill="#9aa7b4" font-size="11">Teaching model: spring contact, no padding, no clothing, pain-onset thresholds for cobots. Not an injury prediction.</text>`;
  $('hz').innerHTML=g; updateNote();
}
['sc','lm','ll','w','r','fm','h','act','tl','pr','reg'].forEach(id=>bind(id,state,FMT,renderHZ));
document.querySelectorAll('[data-preset]').forEach(b=>b.addEventListener('click',()=>{ setAll(state,PRESETS[b.dataset.preset],FMT,renderHZ); document.querySelectorAll('[data-preset]').forEach(x=>x.classList.toggle('on',x===b)); }));

/* ---------- Playground 2: fault tree ---------- */
const LAYERS=[
 ['clamp','Software limits and clamps','joint, velocity and torque limits on every command','sw'],
 ['timeout','Command timeout','no new command in time: hold pose','sw'],
 ['mon','Motor and CAN monitoring','missing replies, fault bits, temperatures: controlled stop','sw'],
 ['fall','Fall detection','IMU tilt and rates: switch every joint to damping','sw'],
 ['drv','Actuator driver protection','built-in over-current, over-temperature, under-voltage cut-outs','hw'],
 ['bms','BMS','low charge warning; over-current, voltage and temperature cut-off','hw'],
 ['wd','Watchdog relay','heartbeats stop: relay opens the contactor','hw'],
 ['estop','E-stop (wired + wireless)','a person presses it, or the wireless link drops','hw'],
 ['fuse','Branch fuses and Class-T fuse','short circuit: fuse blows','hw'],
 ['gantry','Gantry and harness','catches the robot before its knees touch the floor','phys']];
const LIDX=Object.fromEntries(LAYERS.map((l,i)=>[l[0],i]));
// For each fault: list of [layer, action, end state{act, comp, pose, cat}]
const FAULTS={
 wild:{name:'Policy outputs a wild target',desc:'A bug or out-of-distribution input makes the policy ask for a joint angle far outside its range, or NaN.',
  chain:[['clamp','The command is clamped to joint, velocity and torque limits (NaN replaced by holding the last good target) and logged.',{act:'on',comp:'up',pose:'keeps standing or walking',cat:'none (fault masked)'}],
         ['drv','The motor tries to follow; its driver trips on over-current and stops that one joint.',{act:'one joint off',comp:'up',pose:'may buckle and fall',cat:'local'}],
         ['estop','The e-stop holder sees the lurch and presses: the contactor opens.',{act:'off',comp:'up',pose:'limp',cat:'0'}]]},
 cmd:{name:'Teleoperation or high-level command stream stops',desc:'The VLA, joystick or network link stops sending new targets.',
  chain:[['timeout','After the timeout the controller holds the current pose (standing).',{act:'on',comp:'up',pose:'stands still, balancing',cat:'2 (controlled, power on)'}],
         ['estop','The e-stop holder notices the robot is not responding and presses.',{act:'off',comp:'up',pose:'limp',cat:'0'}]]},
 hang:{name:'Control process hangs',desc:'The real-time control process freezes (deadlock, kernel stall, computer crash). It stops sending CAN commands and heartbeats.',
  chain:[['wd','No heartbeat within the watchdog timeout: the relay opens the contactor. The motors may keep acting on their last command until then.',{act:'off',comp:'up (but hung)',pose:'limp',cat:'0'}],
         ['estop','Without a watchdog, only a person can stop it, after they notice.',{act:'off',comp:'up (but hung)',pose:'limp',cat:'0'}]]},
 can:{name:'CAN cable to one shank bus breaks',desc:'The knee and two ankle motors of one leg stop answering. They may keep executing their last command.',
  chain:[['mon','Replies missing for two cycles: the controller switches the reachable joints to damping, then opens the contactor.',{act:'off after damping',comp:'up',pose:'folds down under damping',cat:'1'}],
         ['wd','(The controller is still alive, so the watchdog does not trip.)',null],
         ['estop','The e-stop holder sees the leg go wrong and presses.',{act:'off',comp:'up',pose:'limp',cat:'0'}]]},
 temp:{name:'Knee motor over-temperature on stairs',desc:'A long stair session heats the knee winding towards its limit.',
  chain:[['mon','The temperature in the motor replies passes a warning level: the controller finishes the step, sits down, damps and cuts power.',{act:'off after sitting',comp:'up',pose:'seated',cat:'1'}],
         ['drv','The driver\'s own over-temperature cut-out stops the knee.',{act:'knee off',comp:'up',pose:'knee buckles: falls',cat:'local'}],
         ['estop','The e-stop holder presses.',{act:'off',comp:'up',pose:'limp',cat:'0'}]]},
 fallen:{name:'Robot is pushed and falls',desc:'A push beyond what the policy can recover, or a missed step.',
  chain:[['fall','Tilt and angular rate cross the fall threshold: every joint switches to damping so the robot crumples instead of thrashing.',{act:'on (damping)',comp:'up',pose:'falls softly',cat:'2 then 0'}],
         ['estop','The e-stop holder presses: power off, limp fall.',{act:'off',comp:'up',pose:'falls limp',cat:'0'}]]},
 short:{name:'Short circuit in a leg harness',desc:'A chafed wire in a service loop shorts the leg\'s power to ground.',
  chain:[['fuse','The leg\'s 40 A branch fuse blows (the Class-T fuse if the short is upstream).',{act:'leg off',comp:'up',pose:'leg goes limp',cat:'local'}],
         ['bms','The BMS opens on over-current: all power from the pack stops; the computer runs on the keep-alive battery.',{act:'off',comp:'up (keep-alive)',pose:'limp',cat:'0'}]]},
 low:{name:'Battery reaches 15%',desc:'Normal use runs the pack down.',
  chain:[['bms','The BMS reports 15% over CAN: the controller sits the robot down and cuts actuator power.',{act:'off after sitting',comp:'up',pose:'seated',cat:'1'}],
         ['estop','Without BMS telemetry, someone must watch the voltage and press.',{act:'off',comp:'up',pose:'limp',cat:'0'}]]},
 rf:{name:'Wireless e-stop out of range',desc:'The handheld transmitter\'s battery dies or it goes out of range.',
  chain:[['estop','The receiver is fail-safe: losing the link opens its relay, which opens the contactor.',{act:'off',comp:'up',pose:'limp',cat:'0'}]]}
};
const FDEF={fault:'hang',off:[],gantry:true};
const fs={fault:'hang',off:new Set(),gantry:true};
const FPRE={today:{off:[],gantry:true},nowd:{off:['wd'],gantry:true},nogantry:{off:[],gantry:false},swonly:{off:['drv','bms','wd','estop','fuse'],gantry:false}};
$('faults').innerHTML=Object.entries(FAULTS).map(([k,f])=>`<button data-fault="${k}">${f.name}</button>`).join('');
$('layerbox').innerHTML=LAYERS.map(l=>`<label><input type="checkbox" data-layer="${l[0]}" checked> ${l[1]}</label>`).join('');
function renderFT(){
  document.querySelectorAll('[data-fault]').forEach(b=>b.classList.toggle('on',b.dataset.fault===fs.fault));
  document.querySelectorAll('[data-layer]').forEach(c=>{ const id=c.dataset.layer; c.checked= id==='gantry'? fs.gantry : !fs.off.has(id); });
  const F=FAULTS[fs.fault];
  const chain=F.chain.filter(c=>c[2]); // catching entries
  const seen=new Set(F.chain.map(c=>c[0]));
  let catcher=null;
  for(const c of chain){ const on= c[0]==='gantry'? fs.gantry : !fs.off.has(c[0]); if(on){ catcher=c; break; } }
  // gantry as final catch for limp/falling outcomes
  let end=catcher?{...catcher[2]}:null;
  const falls = end && /limp|buckle|falls|folds/.test(end.pose);
  if(end && falls && fs.gantry) end.pose += ' → caught by the gantry harness';
  // draw
  const rowH=40, y0=40; let g=`<text x="14" y="22" fill="#e6edf3" font-size="13" font-weight="600">${F.name}</text>`;
  const ci = catcher? LIDX[catcher[0]] : LAYERS.length;
  const stopY = catcher? y0+ci*rowH+rowH/2 : y0+LAYERS.length*rowH+10;
  g+=`<line x1="40" y1="${y0-6}" x2="40" y2="${stopY}" stroke="${catcher?'#ffb454':'#ff6b6b'}" stroke-width="4"/>`;
  LAYERS.forEach((l,i)=>{
    const y=y0+i*rowH; const isG=l[0]==='gantry'; const on= isG? fs.gantry : !fs.off.has(l[0]);
    const rel = seen.has(l[0]) || (isG && falls);
    const isC = catcher && catcher[0]===l[0];
    const passed = i<ci;
    const col = isC? '#4cc38a' : (!on? '#3a424d' : rel? '#4fb3ff':'#2a3340');
    const fill = isC? '#13261d' : '#121820';
    g+=`<rect x="60" y="${y}" width="560" height="${rowH-6}" rx="6" fill="${fill}" stroke="${col}" stroke-width="${isC?2.5:1.2}" opacity="${rel||isC?1:0.55}"/>`;
    g+=`<text x="72" y="${y+15}" fill="${on?'#e6edf3':'#6b7682'}" font-size="12.5" ${!on?'text-decoration="line-through"':''}>${i+1}. ${l[1]} <tspan fill="#9aa7b4" font-size="10.5">[${l[3]==='sw'?'software':l[3]==='hw'?'hardware':'physical'}]</tspan></text>`;
    const tag = isC? 'CATCHES IT' : !on? 'removed' : !rel? 'does not see this fault' : passed? 'missed / not applicable' : 'backup';
    g+=`<text x="610" y="${y+15}" fill="${isC?'#4cc38a':!on?'#ff6b6b':'#9aa7b4'}" font-size="10.5" text-anchor="end">${tag}</text>`;
    g+=`<text x="72" y="${y+29}" fill="#9aa7b4" font-size="10.5">${l[2]}</text>`;
    if(isC||(passed&&rel)) g+=`<circle cx="40" cy="${y+rowH/2-3}" r="5" fill="${isC?'#4cc38a':'#ffb454'}"/>`;
  });
  if(!catcher) g+=`<text x="60" y="${y0+LAYERS.length*rowH+20}" fill="#ff6b6b" font-size="13" font-weight="600">Nothing catches this fault.</text>`;
  $('ft').innerHTML=g;
  let h=`<div class="k">What happens</div><p style="margin:.2em 0 .6em">${F.desc} ${catcher? catcher[1] : ''}</p>`;
  if(catcher){ h+=`<div class="readouts" style="margin-top:0"><div class="ro"><div class="k">Actuator power</div><div class="v" style="font-size:14px">${end.act}</div></div><div class="ro"><div class="k">Computer</div><div class="v" style="font-size:14px">${end.comp}</div></div><div class="ro"><div class="k">Robot ends</div><div class="v" style="font-size:14px">${end.pose}</div></div><div class="ro"><div class="k">Stop category (IEC 60204-1 analogy)</div><div class="v" style="font-size:14px">${end.cat}</div></div></div>`;
    if(falls && !fs.gantry) h+=`<p style="color:var(--warn);margin:.6em 0 0">Off the gantry the robot hits the floor: padding (R53), clearance of 2 m and fall detection limit the damage.</p>`; }
  else h+=`<p style="color:var(--bad)">With these layers removed, the fault runs on until something breaks or someone gets hurt. Add a layer back.</p>`;
  h+=`<p class="small" style="color:var(--muted);font-size:12px;margin:.6em 0 0">Design intent from docs/safety.md; timings and thresholds are to be set and verified on the bench.</p>`;
  $('endcard').innerHTML=h; updateNote();
}
document.querySelectorAll('[data-fault]').forEach(b=>b.addEventListener('click',()=>{ fs.fault=b.dataset.fault; renderFT(); }));
document.querySelectorAll('[data-layer]').forEach(c=>c.addEventListener('change',()=>{ const id=c.dataset.layer; if(id==='gantry') fs.gantry=c.checked; else { if(c.checked) fs.off.delete(id); else fs.off.add(id); } document.querySelectorAll('[data-fpreset]').forEach(x=>x.classList.remove('on')); renderFT(); }));
document.querySelectorAll('[data-fpreset]').forEach(b=>b.addEventListener('click',()=>{ const p=FPRE[b.dataset.fpreset]; fs.off=new Set(p.off); fs.gantry=p.gantry; document.querySelectorAll('[data-fpreset]').forEach(x=>x.classList.toggle('on',x===b)); renderFT(); }));

/* ---------- note ---------- */
function updateNote(){
  if(!$('endcard')) return;
  const s=state, h=hazard(), parts=[];
  const changed=Object.keys(DEFAULTS).some(k=>+s[k]!==+DEFAULTS[k]);
  const ftChanged= fs.fault!=='hang' || fs.off.size || !fs.gantry;
  if(!changed && !ftChanged){ $('note').textContent='Change a control above to build a note. (Defaults: a 3 kg arm swung at 5 rad/s hitting the chest; the fault tree on "control process hangs" with all layers fitted.)'; return; }
  let t='For Marvin Bot (1.44 m, 41 kg humanoid, 48 V pack, joints up to 120 N·m; safety design in docs/safety.md, nothing built or verified yet), ';
  if(changed){
    const sc=['a limb swing','a fall','a pinch'][s.sc];
    const det= s.sc==0? `${s.lm} kg limb, ${s.ll} m long, at ${s.w} rad/s, contact ${Math.min(s.r,s.ll)} m from the joint`
              : s.sc==1? `${s.fm} kg dropping ${s.h} m` : `${s.act} N·m actuator limited to ${s.tl}% at ${s.pr} cm from the axis`;
    t+=`I modelled ${sc} (${det}) against the ISO/TS 15066 ${h.R[0].toLowerCase()} region: estimated ${h.F.toFixed(0)} N vs a ${h.limName} threshold of ${h.lim.toFixed(0)} N (${(h.F/h.lim).toFixed(1)}×); ${h.extra}. `;
  }
  if(ftChanged){ const F=FAULTS[fs.fault]; const txt=$('endcard').innerText.replace(/\s+/g,' ').slice(0,400);
    t+=`Fault tree: "${F.name}" with ${fs.off.size?'these layers removed: '+[...fs.off].map(x=>LAYERS[LIDX[x]][1]).join(', '):'all layers fitted'}${fs.gantry?' and the gantry in use':' and no gantry'}. Outcome: ${txt} `;
  }
  t+='Thresholds are cobot pain-onset values used for scale, not a compliance claim; verify every layer on the bench.';
  $('note').textContent=t;
}
$('copy').addEventListener('click',()=>{ navigator.clipboard.writeText($('note').textContent).then(()=>{ const b=$('copy'); b.textContent='Copied!'; setTimeout(()=>b.textContent='Copy',1200); }); });
renderHZ(); renderFT();
