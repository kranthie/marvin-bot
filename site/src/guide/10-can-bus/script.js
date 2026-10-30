const DLEN=[0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64];
const DEFAULTS={fdmode:0,rate:1000000,drate:5000000,ext:1,dbi:8,stuff:100,motors:3,fpm:2,hz:500,lim:60};
const state={...DEFAULTS};
const $=id=>document.getElementById(id);
const PRESETS={
  hip:{...DEFAULTS},
  rarm:{...DEFAULTS,motors:6,hz:250},
  six:{...DEFAULTS,motors:6,hz:500},
  fd:{...DEFAULTS,fdmode:1,motors:6,hz:500,drate:5000000},
  larm:{...DEFAULTS,motors:5,hz:250}
};
const FMT={fdmode:v=>+v?'FD':'classic',rate:v=>(v/1e6>=1?v/1e6+' Mbit/s':v/1e3+' kbit/s'),drate:v=>v/1e6+' Mbit/s',ext:v=>+v?'29-bit':'11-bit',
  dbi:v=>DLEN[v]+' B',stuff:v=>v+'%',motors:v=>v,fpm:v=>v,hz:v=>v+' Hz',lim:v=>v+'%'};
function frameBits(s){
  // returns {total, nom, dat, stuff, fields:[[name,bits,phase]]}
  const n=DLEN[s.dbi]; const f=s.stuff/100;
  if(!+s.fdmode){
    const nn=Math.min(n,8);
    const g=+s.ext?54:34; const worst=Math.floor((g+8*nn-1)/4); const st=f*worst;
    const fields=+s.ext?[['SOF',1],['ID A',11],['SRR',1],['IDE',1],['ID B',18],['RTR',1],['r1 r0',2],['DLC',4],['data',8*nn],['CRC',15],['stuff',st],['CRC del, ACK',3],['EOF',7],['IFS',3]]
                       :[['SOF',1],['ID',11],['RTR',1],['IDE',1],['r0',1],['DLC',4],['data',8*nn],['CRC',15],['stuff',st],['CRC del, ACK',3],['EOF',7],['IFS',3]];
    const total=g+8*nn+13+st;
    return {total,nom:total,dat:0,stuff:st,worst,fields,n:nn};
  }
  // CAN FD (counted as in Linux include/linux/can/length.h)
  const arb=+s.ext?36:17;               // SOF..BRS
  const hdr=arb+5;                      // + ESI + DLC  (41 ext / 22 std)
  const dyn=hdr+8*n;
  const stTot=Math.floor((dyn-1)/4), stArb=Math.floor((arb-1)/4);
  const crc=n<=16?17:21, fixed=n<=16?6:7;
  const crcData=4+crc+fixed;            // stuff count + CRC + fixed stuff bits
  const nom=arb+f*stArb+13;             // + CRC delimiter, ACK 2, EOF 7, IFS 3
  const dat=5+8*n+f*(stTot-stArb)+crcData;
  const fields=+s.ext?[['SOF',1,0],['ID A',11,0],['SRR IDE',2,0],['ID B',18,0],['RRS FDF res BRS',4,0],['stuff',f*stArb,0],['ESI DLC',5,1],['data',8*n,1],['stuff',f*(stTot-stArb),1],['SBC CRC',crcData,1],['CRC del ACK',3,0],['EOF',7,0],['IFS',3,0]]
                     :[['SOF',1,0],['ID',11,0],['RRS IDE FDF res BRS',5,0],['stuff',f*stArb,0],['ESI DLC',5,1],['data',8*n,1],['stuff',f*(stTot-stArb),1],['SBC CRC',crcData,1],['CRC del ACK',3,0],['EOF',7,0],['IFS',3,0]];
  return {total:nom+dat,nom,dat,stuff:f*stTot,worst:stTot,fields,n};
}
function frameTime(s,fb){ return fb.nom/s.rate + (+s.fdmode? fb.dat/s.drate:0); } // seconds
function bind(id){ const el=$(id), out=$('o_'+id);
  const upd=()=>{ state[id]= el.type==='checkbox'? el.checked : parseFloat(el.value); if(out) out.textContent=FMT[id]?FMT[id](state[id]):state[id]; render(); };
  el.addEventListener('input',upd); el.addEventListener('change',upd); if(out) out.textContent=FMT[id]?FMT[id](el.value):el.value; }
function setState(s){ Object.assign(state,s); for(const k in s){ const el=$(k); if(!el) continue; if(el.type==='checkbox') el.checked=!!s[k]; else el.value=s[k]; const o=$('o_'+k); if(o) o.textContent=FMT[k]?FMT[k](s[k]):s[k]; } render(); }
function ro(id,txt,cls){ const e=$(id); e.querySelector('.v').textContent=txt; e.className='ro'+(cls?' '+cls:''); }
const us=x=>(x*1e6).toFixed(x*1e6<100?1:0)+' µs';
function render(){
  const s=state;
  $('dratebox').style.display=+s.fdmode?'':'none';
  // classic limits data to 8 bytes
  const dbiEl=$('dbi'); dbiEl.max=+s.fdmode?15:8; if(!+s.fdmode && s.dbi>8){ s.dbi=8; dbiEl.value=8; $('o_dbi').textContent='8 B'; }
  const fb=frameBits(s); const ft=frameTime(s,fb);
  const T=1/s.hz; const nF=s.motors*s.fpm; const busT=nF*ft; const load=busT/T;
  const maxM=Math.floor((s.lim/100)*T/(s.fpm*ft));
  ro('r_bits',(Math.round(fb.total*10)/10)+(+s.fdmode?` (${Math.round(fb.nom)}+${Math.round(fb.dat)})`:''));
  ro('r_ft',us(ft));
  ro('r_cyc',us(busT)+' / '+us(T));
  ro('r_load',(load*100).toFixed(1)+'%', load*100<=s.lim? 'ok' : load<1?'warn':'bad');
  ro('r_max',maxM+(maxM<s.motors?' (fewer than you have)':''), maxM>=s.motors?'ok':'bad');
  ro('r_lat',us(busT)+(load>1?' (misses the cycle)':''), load>1?'bad':'');
  drawTimeline(s,ft,T,nF,fb);
  drawAnatomy(s,fb);
  busMap(s);
  updateNote(fb,ft,load,maxM);
}
function drawTimeline(s,ft,T,nF){
  const W=640,x0=20,x1=620; const span=Math.max(T,nF*ft)*1.02; const sx=t=>x0+(x1-x0)*t/span;
  let g=`<text x="${x0}" y="18" fill="#9aa7b4" font-size="12">One control cycle: ${us(T)} at ${s.hz} Hz · ${nF} frames packed back to back (best case: no reply delay, no errors)</text>`;
  g+=`<rect x="${sx(0)}" y="40" width="${sx(T)-sx(0)}" height="60" fill="#121820" stroke="#2a3340"/>`;
  g+=`<rect x="${sx(0)}" y="40" width="${sx(T*s.lim/100)-sx(0)}" height="60" fill="none" stroke="#4cc38a" stroke-dasharray="4 3"/>`;
  g+=`<text x="${sx(T*s.lim/100)}" y="114" fill="#4cc38a" font-size="11" text-anchor="middle">${s.lim}% limit</text>`;
  g+=`<text x="${sx(T)}" y="34" fill="#9aa7b4" font-size="11" text-anchor="end">cycle ends (next commands due)</text>`;
  for(let i=0;i<nF;i++){
    const t0=i*ft, t1=t0+ft; const over=t1>T*s.lim/100; const isCmd=(i%s.fpm)===0;
    const fill= isCmd?'#4fb3ff':'#ffb454';
    const w=Math.max(0.6,sx(t1)-sx(t0)-0.6);
    g+=`<rect x="${sx(t0)}" y="48" width="${w}" height="44" fill="${fill}" opacity="${over?0.45:0.9}" ${over?'stroke="#ff6b6b" stroke-width="1.5"':''}/>`;
    if(w>22) g+=`<text x="${sx(t0)+w/2}" y="74" fill="#0f1216" font-size="10" text-anchor="middle">${isCmd?'cmd':'rep'} ${Math.floor(i/s.fpm)+1}</text>`;
  }
  if(nF*ft>T){ g+=`<rect x="${sx(T)}" y="40" width="${sx(nF*ft)-sx(T)}" height="60" fill="#ff6b6b" opacity="0.15"/>`;
    g+=`<text x="${(sx(T)+sx(nF*ft))/2}" y="132" fill="#ff6b6b" font-size="12" text-anchor="middle">overflows into the next cycle</text>`; }
  // axis
  const step=T>=0.01?0.002:T>=0.004?0.0005:0.0002;
  for(let t=0;t<=span+1e-9;t+=step){ g+=`<line x1="${sx(t)}" x2="${sx(t)}" y1="100" y2="105" stroke="#56606c"/><text x="${sx(t)}" y="150" fill="#9aa7b4" font-size="10" text-anchor="middle">${Math.round(t*1e6)}</text>`; }
  g+=`<text x="${x1}" y="166" fill="#9aa7b4" font-size="10" text-anchor="end">time in µs</text>`;
  g+=`<text x="${x0}" y="192" fill="#e6edf3" font-size="12">Bus idle ${us(Math.max(0,T-nF*ft))} per cycle · a newly queued frame can wait up to ${us(ft)} behind a frame already on the wire</text>`;
  $('tl').innerHTML=g;
}
function drawAnatomy(s,fb){
  let g=`<text x="10" y="16" fill="#9aa7b4" font-size="12">One frame to scale (${+s.fdmode?'CAN FD; bright = nominal rate, darker = fast data phase, width in bits':'classic CAN; width in bit-times'})</text>`;
  const tot=fb.fields.reduce((a,f)=>a+f[1],0); let x=10; const k=620/tot;
  const col={SOF:'#9aa7b4',data:'#4cc38a',CRC:'#ff6b6b','SBC CRC':'#ff6b6b',stuff:'#b48cff',EOF:'#56606c',IFS:'#2a3340'};
  fb.fields.forEach((f,i)=>{ const w=f[1]*k; if(w<=0) return; const c=col[f[0]]||'#4fb3ff'; const dim=(+s.fdmode&&f[2]===1);
    g+=`<rect x="${x}" y="30" width="${Math.max(0.5,w-0.5)}" height="40" fill="${c}" opacity="${dim?0.45:0.85}"/>`;
    if(w>26) g+=`<text x="${x+w/2}" y="${i%2?86:100}" fill="#cfd8e2" font-size="10" text-anchor="middle">${f[0]} ${Math.round(f[1]*10)/10}</text>`;
    x+=w; });
  g+=`<text x="630" y="116" fill="#9aa7b4" font-size="10" text-anchor="end">total ${Math.round(fb.total*10)/10} bits${fb.worst!==undefined?' · stuff '+(Math.round(fb.stuff*10)/10)+' of worst '+fb.worst:''}</text>`;
  $('anat').innerHTML=g;
}
const BUSES=[['left_hip / right_hip','hip roll, hip pitch, hip yaw',3,500],['left_shank / right_shank','knee, two ankle motors',3,500],['left_arm','shoulder pitch, roll, yaw, elbow, wrist yaw',5,250],['right_arm','5 arm motors + waist',6,250],['all six leg motors (rejected)','one bus per leg',6,500]];
function busMap(s){
  const ex={...s,fdmode:0,ext:1,dbi:8}; const fb=frameBits(ex); const ft=frameTime(ex,fb);
  let h=`<tr><th>Bus</th><th>Motors</th><th class="num">n</th><th class="num">Rate</th><th class="num">Load, 160 bits (repo)</th><th class="num">Load, ${Math.round(fb.total*10)/10} bits (stuffing ${s.stuff}%)</th></tr>`;
  BUSES.forEach(b=>{ const lrepo=b[2]*2*b[3]*160/1e6, lx=b[2]*2*b[3]*ft; const c=v=>v<=0.6?'ok':v<1?'warn':'bad';
    h+=`<tr><td>${b[0]}</td><td>${b[1]}</td><td class="num">${b[2]}</td><td class="num">${b[3]} Hz</td><td class="num ${c(lrepo)}">${(lrepo*100).toFixed(1)}%</td><td class="num ${c(lx)}">${(lx*100).toFixed(1)}%</td></tr>`; });
  h+=`<tr><td>battery</td><td>BMS telemetry</td><td class="num">–</td><td class="num">10 Hz</td><td class="num ok" colspan="2">well under 1% (a few frames per cycle)</td></tr>`;
  $('busmap').innerHTML=h;
}
function updateNote(fb,ft,load,maxM){
  const s=state, parts=[];
  for(const k of ['fdmode','rate','drate','ext','dbi','stuff','motors','fpm','hz','lim']){
    if(k==='drate' && !+s.fdmode) continue;
    if(String(s[k])!==String(DEFAULTS[k])) parts.push(`${({fdmode:'protocol',rate:'nominal bit rate',drate:'FD data rate',ext:'identifier',dbi:'payload',stuff:'stuffing',motors:'motors on the bus',fpm:'frames per motor per cycle',hz:'loop rate',lim:'load limit'})[k]} ${FMT[k](s[k])}`);
  }
  $('note').textContent = parts.length
   ? `For Marvin Bot (1.44 m, 41 kg humanoid; RobStride motors on classic CAN 2.0B, 1 Mbit/s, 29-bit IDs, one command + one reply per motor per cycle; buses in description/robot.yaml), I explored a bus with ${parts.join(', ')}. Result: ${Math.round(fb.total*10)/10} bits per frame (${(ft*1e6).toFixed(1)} µs), ${s.motors*s.fpm} frames per ${(1e3/s.hz).toFixed(2)} ms cycle, bus load ${(load*100).toFixed(1)}% against a ${s.lim}% limit; at most ${maxM} motors fit at this rate. Exact worst-case classic frame (Davis et al. 2007) is 160 bits, the value robot.yaml budgets. Next step: confirm with the phase 2 bench timing test (measured reply turnaround and USB-CAN jitter).`
   : 'Change a control above to build a note. (Defaults: Marvin Bot\'s hip bus, 3 RS04/RS03 motors at 500 Hz, exact worst-case stuffing: 48% load.)';
}
$('copy').addEventListener('click',()=>{ navigator.clipboard.writeText($('note').textContent).then(()=>{ const b=$('copy'); b.textContent='Copied!'; setTimeout(()=>b.textContent='Copy',1200); }); });
document.querySelectorAll('[data-preset]').forEach(b=>b.addEventListener('click',()=>{ setState(PRESETS[b.dataset.preset]); document.querySelectorAll('[data-preset]').forEach(x=>x.classList.toggle('on',x===b)); }));
['fdmode','rate','drate','ext','dbi','stuff','motors','fpm','hz','lim'].forEach(bind);
render();
