const $=id=>document.getElementById(id);
const NS='http://www.w3.org/2000/svg';
function el(tag,a,p,txt){const e=document.createElementNS(NS,tag);for(const k in a)e.setAttribute(k,a[k]);if(txt!=null)e.textContent=txt;if(p)p.appendChild(e);return e;}
function ro(id,v,c){const r=$(id);r.querySelector('.v').textContent=v;r.className='ro '+(c||'');}
const G=9.81, D2R=Math.PI/180;
/* Arm from the URDF (description/urdf/marvin_bot.urdf), distances along each segment from its proximal joint */
const UA=[{m:0.832,r:0.098,k:'shoulder roll link + RS03'},{m:0.870,r:0.214,k:'upper arm + RS06s'}], L_UA=0.225; // shoulder pitch axis to elbow
const FA=[{m:0.522,r:0.089,k:'forearm + RS00'},{m:0.115,r:0.179,k:'wrist link + STS3250'}], L_FA=0.188; // elbow to wrist pitch
const ACT={wr:{n:'Wrist pitch (STS3250)',rated:1.57,peak:4.9},elb:{n:'Elbow (RS06)',rated:11,peak:36},sh:{n:'Shoulder pitch (RS03)',rated:20,peak:60}};
const DEF={sp:90,el:0,wp:0,pm:0.5,pd:0.17,hm:0.418,hc:0.086};
const state={...DEF};
const FMT={sp:v=>v+'°',el:v=>v+'°',wp:v=>v+'°',pm:v=>(+v).toFixed(2)+' kg',pd:v=>(+v).toFixed(3)+' m',hm:v=>(+v).toFixed(3)+' kg',hc:v=>(+v).toFixed(3)+' m'};
const PRESETS={today:{...DEF},r20:{...DEF,pm:1},close:{...DEF,sp:0,el:90,pm:2},level:{...DEF,pm:0},tray:{...DEF,sp:30,el:60,pm:1}};
function bind(id){const e=$(id),o=$('o_'+id);e.addEventListener('input',()=>{state[id]=parseFloat(e.value);o.textContent=FMT[id](state[id]);render();});o.textContent=FMT[id](e.value);}
Object.keys(FMT).forEach(bind);
function setState(s){Object.assign(state,s);for(const k in FMT){$(k).value=state[k];$('o_'+k).textContent=FMT[k](state[k]);}render();}
document.querySelectorAll('[data-preset]').forEach(b=>b.addEventListener('click',()=>setState(PRESETS[b.dataset.preset])));
const dir=phi=>[Math.sin(phi*D2R),-Math.cos(phi*D2R)]; // forward x, up y
let R={};
function solve(){
  const p1=state.sp, p2=state.sp+state.el, p3=p2+state.wp;
  const d1=dir(p1),d2=dir(p2),d3=dir(p3);
  const S=[0,0], E=[d1[0]*L_UA,d1[1]*L_UA], W=[E[0]+d2[0]*L_FA,E[1]+d2[1]*L_FA];
  const pts=[];
  UA.forEach(s=>pts.push({m:s.m,x:d1[0]*s.r,y:d1[1]*s.r,seg:0,k:s.k}));
  FA.forEach(s=>pts.push({m:s.m,x:E[0]+d2[0]*s.r,y:E[1]+d2[1]*s.r,seg:1,k:s.k}));
  pts.push({m:state.hm,x:W[0]+d3[0]*state.hc,y:W[1]+d3[1]*state.hc,seg:2,k:'hand + mount'});
  const P={m:state.pm,x:W[0]+d3[0]*state.pd,y:W[1]+d3[1]*state.pd,seg:2,k:'payload'}; pts.push(P);
  const tq=(jx,segMin)=>G*pts.filter(p=>p.seg>=segMin).reduce((a,p)=>a+p.m*(p.x-jx),0);
  const T={sh:tq(S[0],0),elb:tq(E[0],1),wr:tq(W[0],2)};
  const tip=[W[0]+d3[0]*0.181,W[1]+d3[1]*0.181];
  // payload wrist can hold at rated in this pose
  const hx=state.hc*d3[0], px=state.pd*d3[0];
  const mmax = Math.abs(px)<1e-4?Infinity:(ACT.wr.rated/G - Math.abs(state.hm*hx))/Math.abs(px);
  R={S,E,W,tip,pts,P,T,mmax};
}
function cls(t,a){const v=Math.abs(t);return v<=a.rated?'ok':v<=0.7*a.peak?'warn':'bad';}
function render(){
  solve();
  const sv=$('arm'); sv.innerHTML='';
  const sc=360, ox=210, oy=250; const X=x=>ox+x*sc, Y=y=>oy-y*sc;
  // floor ref & torso
  el('rect',{x:ox-120,y:oy-40,width:110,height:260,rx:12,fill:'#1c232c',stroke:'#2a3340'},sv);
  el('text',{x:ox-112,y:oy-48},sv,'torso (behind)');
  el('text',{x:12,y:18,class:'ttl'},sv,'Side view, one arm (forward is to the right)');
  // moment arm at shoulder: horizontal from shoulder to payload
  el('line',{x1:X(0),y1:Y(R.P.y),x2:X(R.P.x),y2:Y(R.P.y),stroke:'#ffb454','stroke-dasharray':'4 4'},sv);
  el('line',{x1:X(0),y1:Y(-0.6),x2:X(0),y2:Y(0.62),stroke:'#2a3340','stroke-dasharray':'2 4'},sv);
  el('text',{x:(X(0)+X(R.P.x))/2,y:Y(R.P.y)+(R.P.y>0?-24:52),'text-anchor':'middle',fill:'#ffb454'},sv,'payload moment arm at the shoulder '+Math.abs(R.P.x).toFixed(3)+' m');
  // segments
  const seg=(a,b,w,c)=>el('line',{x1:X(a[0]),y1:Y(a[1]),x2:X(b[0]),y2:Y(b[1]),stroke:c,'stroke-width':w,'stroke-linecap':'round'},sv);
  seg(R.S,R.E,16,'#3a4656'); seg(R.E,R.W,13,'#3a4656'); seg(R.W,R.tip,18,'#4a5a6c');
  // joints
  [[R.S,'shoulder pitch (RS03)',-1],[R.E,'elbow (RS06)',1],[R.W,'wrist pitch (STS3250)',-1]].forEach(([p,t,u],i)=>{ el('circle',{cx:X(p[0]),cy:Y(p[1]),r:9,fill:'#0c1015',stroke:'#4fb3ff','stroke-width':2.5},sv); el('text',{x:X(p[0])+(i===0?-12:10),y:Y(p[1])+(u<0?-16:30),'text-anchor':i===0?'end':'start',fill:'#cfe3f5'},sv,t); });
  // CoMs
  R.pts.forEach(p=>{ if(p.m<=0) return; const isP=p.k==='payload';
    if(isP){ el('rect',{x:X(p.x)-11,y:Y(p.y)-11,width:22,height:22,rx:3,fill:'#ffb454',opacity:.9},sv); }
    else el('circle',{cx:X(p.x),cy:Y(p.y),r:3+6*Math.sqrt(p.m),fill:'#4cc38a',opacity:.8},sv);
    el('line',{x1:X(p.x),y1:Y(p.y),x2:X(p.x),y2:Y(p.y)+12+18*p.m,stroke:isP?'#ffb454':'#4cc38a','stroke-width':1.5,'marker-end':'url(#g)'},sv); });
  // (weight arrows)
  const defs=el('defs',{},sv); const mk=el('marker',{id:'g',viewBox:'0 0 10 10',refX:5,refY:9,markerWidth:8,markerHeight:8,markerUnits:'userSpaceOnUse',orient:'0'},defs); el('path',{d:'M0,0 L10,0 L5,10 z',fill:'#9aa7b4'},mk);
  el('text',{x:12,y:490},sv,'Green dots: link masses from the URDF (size ∝ √mass); orange: payload; arrows: weight.');
  // meters
  const m=$('meters'); m.innerHTML='';
  ['wr','elb','sh'].forEach(k=>{ const a=ACT[k], t=Math.abs(R.T[k]), f=t/a.peak; const c=cls(t,a);
    const col=c==='ok'?'var(--good)':c==='warn'?'var(--warn)':'var(--bad)';
    const d=document.createElement('div'); d.className='meter';
    d.innerHTML=`<span>${a.n}</span><div class="trk"><span style="width:${Math.min(100,f*100)}%;background:${col}"></span><i style="left:${a.rated/a.peak*100}%;background:#e6edf3"></i><i style="left:70%;background:#ff6b6b;opacity:.6"></i></div><b>${t.toFixed(2)} N·m</b>`;
    m.appendChild(d); });
  ro('r_reach',R.P.x.toFixed(3)+' m');
  const pr=(k)=>{const a=ACT[k],t=Math.abs(R.T[k]);return t.toFixed(2)+' N·m · '+(t/a.rated*100).toFixed(0)+'% rated';};
  ro('r_wr',pr('wr'),cls(R.T.wr,ACT.wr)); ro('r_elb',pr('elb'),cls(R.T.elb,ACT.elb)); ro('r_sh',pr('sh'),cls(R.T.sh,ACT.sh));
  ro('r_max',isFinite(R.mmax)?(R.mmax>0?R.mmax.toFixed(2)+' kg':'0 kg'):'any (no lever)', R.mmax>=1?'ok':R.mmax>=0.5?'warn':'bad');
  updateNote();
}
/* fingers */
const SV={scs:{stall:0.226,rated:0.074,n:'SCS0009'},sts:{stall:0.441,rated:0.147,n:'STS3032'}};
const FD={sv:'scs',tu:33,L:0.05,eta:0.8,mu:0.5,sf:2};
const fs={...FD};
const FF={tu:v=>v+'%',L:v=>(+v).toFixed(3)+' m',eta:v=>(+v).toFixed(2),mu:v=>(+v).toFixed(2),sf:v=>(+v).toFixed(1)};
const FP={today:{...FD},stall:{...FD,tu:90},pads:{...FD,mu:0.9},sts:{...FD,sv:'sts'}};
Object.keys(FF).forEach(id=>{const e=$(id),o=$('o_'+id);e.addEventListener('input',()=>{fs[id]=parseFloat(e.value);o.textContent=FF[id](fs[id]);frender();});o.textContent=FF[id](e.value);});
$('sv').addEventListener('change',()=>{fs.sv=$('sv').value;frender();});
document.querySelectorAll('[data-fp]').forEach(b=>b.addEventListener('click',()=>{Object.assign(fs,FP[b.dataset.fp]);$('sv').value=fs.sv;for(const k in FF){$(k).value=fs[k];$('o_'+k).textContent=FF[k](fs[k]);}frender();}));
let FR={};
function frender(){
  const s=SV[fs.sv], tau=2*s.stall*fs.tu/100, F=tau*fs.eta/fs.L, m=2*fs.mu*F/G, msf=m/fs.sf;
  FR={tau,F,m,msf,s};
  ro('f_tau',tau.toFixed(3)+' N·m'); ro('f_F',F.toFixed(1)+' N'); ro('f_m',m.toFixed(2)+' kg'); ro('f_ms',msf.toFixed(2)+' kg',msf>=0.5?'ok':msf>=0.25?'warn':'bad');
  const sv=$('fing'); sv.innerHTML='';
  const defs=el('defs',{},sv); const mk=el('marker',{id:'fa',viewBox:'0 0 10 10',refX:9,refY:5,markerWidth:10,markerHeight:10,markerUnits:'userSpaceOnUse',orient:'auto-start-reverse'},defs); el('path',{d:'M0,0 L10,5 L0,10 z',fill:'#e6edf3'},mk);
  el('text',{x:12,y:18,class:'ttl'},sv,'Pinch: squeeze N on both faces, friction μN on each holds the weight');
  const cx=300, cy=140, ow=70, oh=90;
  el('rect',{x:cx-ow/2,y:cy-oh/2,width:ow,height:oh,rx:8,fill:'#2c3a4a',stroke:'#4fb3ff'},sv);
  el('text',{x:cx,y:cy+4,'text-anchor':'middle',fill:'#cfe3f5'},sv,'object');
  // fingers
  el('path',{d:`M${cx-ow/2-130},${cy-30} q60,-10 ${120},${30}`,stroke:'#5d6b7a','stroke-width':22,fill:'none','stroke-linecap':'round'},sv);
  el('path',{d:`M${cx+ow/2+130},${cy-30} q-60,-10 ${-120},${30}`,stroke:'#5d6b7a','stroke-width':22,fill:'none','stroke-linecap':'round'},sv);
  el('text',{x:cx-200,y:cy-50},sv,'thumb'); el('text',{x:cx+150,y:cy-50},sv,'finger');
  const sN=Math.min(90,10+F*6);
  el('line',{x1:cx-ow/2-sN,y1:cy,x2:cx-ow/2-2,y2:cy,stroke:'#ffb454','stroke-width':3,'marker-end':'url(#fa)'},sv);
  el('line',{x1:cx+ow/2+sN,y1:cy,x2:cx+ow/2+2,y2:cy,stroke:'#ffb454','stroke-width':3,'marker-end':'url(#fa)'},sv);
  el('text',{x:cx-ow/2-sN-4,y:cy-10,'text-anchor':'end',fill:'#ffb454'},sv,'N = '+F.toFixed(1)+' N');
  const fr=Math.min(70,8+fs.mu*F*6);
  [-1,1].forEach(sd=>el('line',{x1:cx+sd*(ow/2+14),y1:cy+44,x2:cx+sd*(ow/2+14),y2:cy+44-fr,stroke:'#4cc38a','stroke-width':3,'marker-end':'url(#fa)'},sv));
  el('text',{x:cx+ow/2+22,y:cy+40},sv,'μN = '+(fs.mu*F).toFixed(1)+' N each');
  el('line',{x1:cx,y1:cy+oh/2,x2:cx,y2:cy+oh/2+Math.min(60,10+m*G*3),stroke:'#ff6b6b','stroke-width':3,'marker-end':'url(#fa)'},sv);
  el('text',{x:cx+8,y:cy+oh/2+30,fill:'#ff6b6b'},sv,'max weight '+(m*G).toFixed(1)+' N ('+m.toFixed(2)+' kg)');
  updateNote();
}
function updateNote(){
  if(!R.T||!FR.s) return;
  const parts=[];
  for(const k in DEF) if(state[k]!==DEF[k]) parts.push(k);
  let s='';
  if(parts.length){ s+=`Arm pose shoulder pitch ${state.sp}°, elbow ${state.el}°, wrist pitch ${state.wp}°, payload ${state.pm} kg at ${state.pd} m from the wrist pitch axis, hand ${state.hm} kg with its centre ${state.hc} m out: wrist pitch STS3250 ${Math.abs(R.T.wr).toFixed(2)} N·m (${(Math.abs(R.T.wr)/1.57*100).toFixed(0)}% of 1.57 rated), elbow RS06 ${Math.abs(R.T.elb).toFixed(2)} N·m (${(Math.abs(R.T.elb)/11*100).toFixed(0)}% of 11), shoulder pitch RS03 ${Math.abs(R.T.sh).toFixed(2)} N·m (${(Math.abs(R.T.sh)/20*100).toFixed(0)}% of 20). In this pose the wrist holds ${isFinite(R.mmax)?Math.max(0,R.mmax).toFixed(2)+' kg':'any payload'} at its continuous rating. `; }
  const fp=[]; for(const k in FD) if(fs[k]!==FD[k]) fp.push(k);
  if(fp.length){ s+=`Fingers: ${FR.s.n} at ${fs.tu}% of stall, two servos per finger, ${fs.L} m lever, efficiency ${fs.eta}, μ = ${fs.mu}: about ${FR.F.toFixed(1)} N at the fingertip and ${FR.m.toFixed(2)} kg in a pinch (${FR.msf.toFixed(2)} kg with a safety factor of ${fs.sf}); the lever and efficiency are estimates, as Pollen publishes no grip force. `; }
  $('note').textContent = s ? `For Marvin Bot (1.44 m, 41 kg humanoid; AmazingHand kits on an RS00 wrist yaw and STS3250 wrist pitch; requirement R20 asks 1 kg per hand at full reach, v1 deviation ~0.5 kg): ${s}Static holding only, no accelerations; check against docs/decisions.md 2026-10-27.` : 'Change a control above to build a note.';
}
$('copy').addEventListener('click',()=>{navigator.clipboard.writeText($('note').textContent).then(()=>{const b=$('copy');b.textContent='Copied!';setTimeout(()=>b.textContent='Copy',1200);});});
render(); frender();
