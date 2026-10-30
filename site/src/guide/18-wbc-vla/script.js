"use strict";
const $=id=>document.getElementById(id);
const NS='http://www.w3.org/2000/svg';
function el(tag,a,p){const e=document.createElementNS(NS,tag);for(const k in a)e.setAttribute(k,a[k]);if(p)p.appendChild(e);return e;}

/* ================= Playground A ================= */
const ARCH={
 dec:{name:'Decoupled: RL legs + IK arms',
  boxes:[
   {c:0,r:0,t:'Head camera',s:'depth + colour',rate:'30 Hz',data:''},
   {c:0,r:1,t:'Instruction',s:'"put the cup in the bin"',rate:'per task',data:''},
   {c:1,r:0.5,t:'VLA (GR00T N1.7)',s:'System 2 + System 1',rate:'chunk every 0.15–0.8 s',data:'teleop demos of arm tasks (+ Mimic, Cosmos)',vla:true},
   {c:2,r:0,t:'Arm/hand targets',s:'end-effector poses',rate:'20–30 Hz actions',data:''},
   {c:2,r:1,t:'Walk command',s:'velocity, turn, height',rate:'20–30 Hz actions',data:''},
   {c:3,r:0,t:'IK solver (arms)',s:'+ hand servos',rate:'50 Hz',data:'robot model only'},
   {c:3,r:1,t:'RL walking policy',s:'legs, balance',rate:'50 Hz',data:'simulation (Isaac Lab)'},
   {c:4,r:0.5,t:'Motor drivers',s:'PD on CAN commands',rate:'500 Hz legs, ≥250 Hz arms',data:''}],
  links:[[0,2],[1,2],[2,3],[2,4],[3,5],[4,6],[5,7],[6,7]],
  r:{lat:5,rob:4,data:4,breadth:2,train:5,now:4},
  pros:['Reuses the existing walking/stair policies','VLA only outputs arm targets: matches table-top demonstrations','A late or wrong VLA output cannot knock the robot over','Each part testable on its own'],
  cons:['Legs and arms do not cooperate: arm motions disturb balance unless the leg policy was trained with random arm poses','No whole-body skills (lunge, squat-and-reach) without extra commands','Teleoperation is arms only plus a joystick for walking']},
 trk:{name:'General motion tracker',
  boxes:[
   {c:0,r:0,t:'Head camera',s:'depth + colour',rate:'30 Hz',data:''},
   {c:0,r:1,t:'VR teleop',s:'headset + hand tracking',rate:'50–90 Hz',data:''},
   {c:1,r:0,t:'VLA (or planner)',s:'outputs whole-body poses / tokens',rate:'chunk every 0.15–0.8 s',data:'whole-body teleop demos',vla:true},
   {c:2,r:0.5,t:'Whole-body reference',s:'future poses, 80–200 ms ahead',rate:'50 Hz',data:''},
   {c:3,r:0.5,t:'Motion tracker policy',s:'all 27 joints, balance',rate:'50 Hz',data:'large motion dataset (SONIC: ~700 h, 21k GPU h)'},
   {c:4,r:0.5,t:'Motor drivers',s:'PD on CAN commands',rate:'500 Hz legs, ≥250 Hz arms',data:''}],
  links:[[0,2],[2,3],[1,3],[3,4],[4,5]],
  r:{lat:4,rob:3,data:2,breadth:5,train:2,now:2},
  pros:['Whole-body skills from human motion: squat to reach, lean, step while reaching','The same policy serves teleoperation (to record demonstrations) and autonomy','A new skill is a new motion, not new reward code'],
  cons:['Needs a lot of motion data and compute to generalise to unseen motions','Retargeting limits (page 17) appear as errors in every task','Locomotion on stairs and rough ground usually weaker than a dedicated walking policy']},
 e2e:{name:'Single end-to-end policy',
  boxes:[
   {c:0,r:0,t:'Head camera',s:'depth + colour',rate:'30 Hz',data:''},
   {c:0,r:1,t:'Instruction + state',s:'text, IMU, joints',rate:'50 Hz',data:''},
   {c:2,r:0.5,t:'One large policy',s:'pixels + language → 27 joint targets',rate:'must answer every 20 ms',data:'demos that include balance, falls, pushes',vla:true},
   {c:4,r:0.5,t:'Motor drivers',s:'PD on CAN commands',rate:'500 Hz legs, ≥250 Hz arms',data:''}],
  links:[[0,2],[1,2],[2,3]],
  r:{lat:1,rob:1,data:1,breadth:4,train:1,now:1},
  pros:['No hand-designed interface between layers','In principle can learn any coordination the data shows'],
  cons:['A 3B-parameter VLA takes 35–360 ms per call (GR00T N1.7 benchmark): far over the 20 ms budget','Balance must be learned from demonstrations, which rarely contain recoveries','A model error goes straight to the legs: unsafe on a 41 kg robot']}
};
const RNAMES=[['lat','Latency tolerance'],['rob','Balance robustness'],['data','Low data cost'],['breadth','Skill breadth'],['train','Low training compute'],['now','Ready for Marvin Bot']];
const VERDICT={
 table:{dec:'Good fit. The legs stand still under the walking policy; the VLA moves the arm through IK. This is what GR00T demonstrations usually show.',trk:'Works, but the tracker is more machinery than a standing table-top task needs.',e2e:'Possible only if the policy can run at 50 Hz and has learned to stand; a stalled inference stalls the legs too.'},
 carry:{dec:'Works if the walking policy was trained with loads and random arm poses; the arms hold the box by IK while the VLA sends walking commands.',trk:'Natural: the reference contains walking and holding together; the tracker balances the load as part of the motion.',e2e:'Very hard: needs many demonstrations of walking with loads, all at 50 Hz.'},
 floor:{dec:'Limited. Marvin Bot has no waist pitch, so the hands reach the floor only in a squat. The leg policy needs a height command (as in HOMIE), and the pick-up clip on page 17 still stopped 27 cm above the floor.',trk:'Best fit: a human squat-and-pick motion drives knees, hips and arms together. Still limited by the knee (130°) and the missing waist pitch.',e2e:'Would need demonstrations of squatting with balance; impractical.'},
 wave:{dec:'Easy: arms by IK (or a recorded gesture), legs by the walking policy.',trk:'Easy and more natural (the gesture and the gait come from one human motion), and Marvin Bot\'s 25-clip tracker already does a standing wave in simulation.',e2e:'Overkill for a gesture.'}
};
const A={arch:'dec',task:'table',showData:true,showRates:true};
function drawArch(){
  const svg=$('arch');svg.innerHTML='';const a=ARCH[A.arch];
  el('rect',{x:0,y:0,width:760,height:380,fill:'#0c1015'},svg);
  const defs=el('defs',{},svg);const m=el('marker',{id:'ah',viewBox:'0 0 10 10',refX:9,refY:5,markerWidth:7,markerHeight:7,orient:'auto-start-reverse'},defs);el('path',{d:'M0,0 L10,5 L0,10 z',fill:'#56636f'},m);
  const cols=['Sense','Decide (slow)','Interface','Control (fast)','Actuate'];
  const cx=c=>20+c*148, bw=132, bh=A.showData?114:90, ry=r=>48+r*156;
  cols.forEach((t,i)=>el('text',{x:cx(i)+bw/2,y:26,fill:'#9aa7b4','font-size':12,'text-anchor':'middle'},svg).textContent=t);
  // fast/slow band
  el('rect',{x:cx(3)-8,y:38,width:bw*2+30,height:330,fill:'rgba(76,195,138,.05)',stroke:'rgba(76,195,138,.25)','stroke-dasharray':'4 4'},svg);
  el('text',{x:cx(3),y:376,fill:'#4cc38a','font-size':11},svg).textContent='keeps balance even if the layers above stall';
  const pos=a.boxes.map(b=>({x:cx(b.c),y:ry(b.r),b}));
  a.links.forEach(([i,j])=>{const p=pos[i],q=pos[j];el('line',{x1:p.x+bw,y1:p.y+bh/2,x2:q.x-2,y2:q.y+bh/2,stroke:'#56636f','stroke-width':1.5,'marker-end':'url(#ah)'},svg);});
  pos.forEach(({x,y,b})=>{
    el('rect',{x,y,width:bw,height:bh,rx:8,fill:b.vla?'#1d2738':'#161b22',stroke:b.vla?'#4fb3ff':'#2a3340','stroke-width':b.vla?2:1},svg);
    const tx=(t,dy,col,sz,w)=>{const e=el('text',{x:x+8,y:y+dy,fill:col,'font-size':sz,'font-weight':w||400},svg);e.textContent=t;return e;};
    tx(b.t,18,'#e6edf3',12.5,600);
    wrap(b.s,x+8,y+34,bw-14,11,'#9aa7b4',svg,2);
    if(A.showRates&&b.rate)wrap('⏱ '+b.rate,x+8,y+64,bw-14,11,'#ffb454',svg,2);
    if(A.showData&&b.data)wrap('▤ '+b.data,x+8,y+(A.showRates?92:64),bw-14,10.5,'#b48cff',svg,2);
  });
  if(A.arch==='dec'||A.arch==='trk'){el('rect',{x:20,y:350,width:212,height:22,rx:11,fill:'none',stroke:'#ffb454'},svg);
    el('text',{x:126,y:365,fill:'#ffb454','font-size':12,'text-anchor':'middle'},svg).textContent=A.arch==='dec'?'Marvin Bot plan: first step':'Marvin Bot plan: later';}
  // ratings
  $('ratings').innerHTML=RNAMES.map(([k,n])=>`<span>${n}</span><span class="rb"><i style="width:${a.r[k]*20}%"></i></span><span class="mono">${a.r[k]}/5</span>`).join('');
  $('pros').innerHTML=a.pros.map(x=>`<li>${x}</li>`).join('');$('cons').innerHTML=a.cons.map(x=>`<li>${x}</li>`).join('');
  $('verdict').innerHTML=`<b>${$('task').selectedOptions[0].textContent}:</b> ${VERDICT[A.task][A.arch]}`;
  document.querySelectorAll('[data-arch]').forEach(b=>b.classList.toggle('on',b.dataset.arch===A.arch));
}
function wrap(text,x,y,w,size,col,svg,maxLines){const words=text.split(' ');let line='',n=0;const cw=size*0.55;
  for(let i=0;i<words.length;i++){const t=line?line+' '+words[i]:words[i];if(t.length*cw>w&&line){el('text',{x,y:y+n*(size+2),fill:col,'font-size':size},svg).textContent=line;n++;line=words[i];if(n>=maxLines){line=null;break;}}else line=t;}
  if(line&&n<maxLines)el('text',{x,y:y+n*(size+2),fill:col,'font-size':size},svg).textContent=line;}
document.querySelectorAll('[data-arch]').forEach(b=>b.addEventListener('click',()=>{A.arch=b.dataset.arch;drawArch();updateNote();}));
$('task').addEventListener('change',e=>{A.task=e.target.value;drawArch();updateNote();});
$('showData').onchange=e=>{A.showData=e.target.checked;drawArch();};
$('showRates').onchange=e=>{A.showRates=e.target.checked;drawArch();};

/* ================= Playground B ================= */
const DEFAULTS={lat:50,rtt:15,jit:40,H:40,fa:20,E:16,mode:'async',skip:true};
const state={...DEFAULTS};
const PRESETS={
 desk:{lat:50,rtt:15,jit:40,H:40,fa:20,E:16,mode:'async',skip:true},
 orin:{lat:149,rtt:0,jit:15,H:40,fa:20,E:16,mode:'async',skip:true},
 eager:{lat:358,rtt:0,jit:20,H:40,fa:20,E:16,mode:'async',skip:true},
 sync:{lat:149,rtt:0,jit:15,H:40,fa:20,E:16,mode:'sync',skip:true},
 nochunk:{lat:50,rtt:15,jit:40,H:1,fa:20,E:1,mode:'async',skip:true}
};
function rng(seed){let s=seed>>>0;return()=>{s=(s*1664525+1013904223)>>>0;return s/4294967296;};}
const TEND=3.0;
function simulate(){
  const L=state.lat/1000,N=state.rtt/1000,J=state.jit/1000,H=state.H,f=state.fa,E=Math.min(state.E,state.H),dt=1/f;
  const R=rng(12345);const calls=[];const ticks=[];const ages=[];let lagSum=0,lagN=0;
  const launch=to=>{const j=R()*J;const c={tObs:to,L,N,j,tArr:to+L+N+j,id:calls.length};calls.push(c);return c;};
  const nT=Math.ceil(TEND/dt)+1;
  if(state.mode==='sync'){
    let c=launch(0);let t=0;
    while(t<TEND){ // wait, then play E actions, then observe again
      while(t<c.tArr-1e-9&&t<TEND){ticks.push({t,type:'hold'});t+=dt;}
      for(let i=0;i<E&&t<TEND;i++){ticks.push({t,type:'run',id:c.id});ages.push([t,t-c.tObs]);t+=dt;}
      c=launch(t);}
  }else{
    let cur=null,nextLaunch=Infinity;const pending=[launch(0)];
    for(let i=0;i<nT;i++){const t=i*dt;if(t>TEND)break;
      pending.sort((x,y)=>x.tArr-y.tArr);
      while(pending.length&&pending[0].tArr<=t+1e-9){const c=pending.shift();
        if(!cur||c.id>cur.c.id){const late=t-c.tObs;cur={c,idx:state.skip?Math.min(H-1,Math.round(late*f)):0};
          if(!state.skip){lagSum+=late;lagN++;}
          nextLaunch=t+Math.max(0,E*dt-(L+N));}}
      if(nextLaunch<=t+1e-9){pending.push(launch(t));nextLaunch=Infinity;}
      if(cur&&cur.idx<H){ticks.push({t,type:'run',id:cur.c.id});ages.push([t,t-cur.c.tObs]);cur.idx++;}
      else ticks.push({t,type:'hold'});}
  }
  const exec=[];ticks.forEach(k=>{const l=exec[exec.length-1];if(l&&l.type===k.type&&l.id===k.id)l.t1=k.t+dt;else exec.push({t0:k.t,t1:k.t+dt,type:k.type,id:k.id});});
  const firstRun=ticks.findIndex(k=>k.type==='run');const after=firstRun<0?[]:ticks.slice(firstRun);
  const holdFrac=after.length?after.filter(k=>k.type==='hold').length/after.length:1;
  const ag=ages.map(a=>a[1]);const mean=ag.length?ag.reduce((x,y)=>x+y,0)/ag.length:0,max=ag.length?Math.max(...ag):0;
  return {calls,exec,ages,holdFrac,mean,max,lag:lagN?lagSum/lagN:0,keep:(L+N+J)<=E*dt,E,dt};
}
function setRO(id,txt,cls){const e=$(id);e.querySelector('.v').textContent=txt;e.className='ro'+(cls?' '+cls:'');}
let SIM=null;
function drawTL(){
  SIM=simulate();const s=SIM;const svg=$('tl');svg.innerHTML='';
  el('rect',{x:0,y:0,width:760,height:330,fill:'#0c1015'},svg);
  const x0=120,x1=748;const px=t=>x0+t/TEND*(x1-x0);
  const lanes=[['Inference',30],['Executing',90],['Action age',150],['50 Hz control',268]];
  lanes.forEach(([n,y])=>el('text',{x:8,y:y+14,fill:'#9aa7b4','font-size':12},svg).textContent=n);
  for(let t=0;t<=TEND+1e-9;t+=0.5){el('line',{x1:px(t),x2:px(t),y1:24,y2:300,stroke:'#1a222c'},svg);el('text',{x:px(t),y:316,fill:'#56636f','font-size':11,'text-anchor':'middle'},svg).textContent=t.toFixed(1)+' s';}
  // inference lane: stack overlapping calls on 2 rows
  s.calls.forEach((c,i)=>{if(c.tObs>TEND)return;const y=24+(i%2)*20;
    const seg=(a,b,col)=>{if(b>a)el('rect',{x:px(Math.min(a,TEND)),y,width:Math.max(1,px(Math.min(b,TEND))-px(Math.min(a,TEND))),height:14,fill:col,rx:2},svg);};
    seg(c.tObs,c.tObs+c.L,'#4fb3ff');seg(c.tObs+c.L,c.tObs+c.L+c.N,'#b48cff');seg(c.tObs+c.L+c.N,c.tArr,'#ffb454');
    el('line',{x1:px(c.tObs),x2:px(c.tObs),y1:y-3,y2:y+17,stroke:'#e6edf3','stroke-width':1},svg);});
  // executing lane
  const cols=['#4cc38a','#2f9e6b'];
  s.exec.forEach(e=>{const a=Math.min(e.t0,TEND),b=Math.min(e.t1,TEND);if(b<=a)return;
    el('rect',{x:px(a),y:88,width:Math.max(1,px(b)-px(a)),height:22,fill:e.type==='hold'?'#ff6b6b':cols[(e.id||0)%2],rx:2},svg);});
  // ages
  const ay0=150,ay1=250;const amax=Math.max(0.2,Math.ceil(s.max*10)/10);
  const py=v=>ay1-v/amax*(ay1-ay0);
  [0,amax/2,amax].forEach(v=>{el('line',{x1:x0,x2:x1,y1:py(v),y2:py(v),stroke:'#1f2933'},svg);el('text',{x:x0-4,y:py(v)+4,fill:'#56636f','font-size':10,'text-anchor':'end'},svg).textContent=(v*1000).toFixed(0)+' ms';});
  s.ages.forEach(([t,a])=>{if(t<TEND)el('circle',{cx:px(t),cy:py(a),r:2.2,fill:'#e6edf3'},svg);});
  el('text',{x:x0+4,y:ay0-4,fill:'#9aa7b4','font-size':10.5},svg).textContent='time since the image each executed action was planned from';
  // 50 Hz ticks
  for(let t=0;t<TEND;t+=0.02){const hold=s.exec.some(e=>e.type==='hold'&&t>=e.t0&&t<e.t1);
    el('line',{x1:px(t),x2:px(t),y1:268,y2:284,stroke:hold?'#ff6b6b':'#56636f','stroke-width':1},svg);}
  el('text',{x:x0,y:298,fill:'#56636f','font-size':10.5},svg).textContent='legs balance on every tick; red = arms hold their last target';
  // readouts
  setRO('r_hold',(s.holdFrac*100).toFixed(0)+'%',s.holdFrac>0.1?'bad':s.holdFrac>0.01?'warn':'ok');
  setRO('r_age',`${(s.mean*1000).toFixed(0)} / ${(s.max*1000).toFixed(0)} ms`,s.max>1.0?'bad':s.max>0.4?'warn':'ok');
  setRO('r_lag',state.mode==='async'&&!state.skip?`${(s.lag*1000).toFixed(0)} ms behind`:(state.skip||state.mode==='sync')?'0 (aligned)':'–',state.mode==='async'&&!state.skip&&s.lag>0.1?'warn':'ok');
  setRO('r_calls',`${(s.calls.filter(c=>c.tObs<TEND).length/TEND).toFixed(1)} per s`);
  setRO('r_keep',s.keep?`yes (${((state.lat+state.rtt+state.jit)).toFixed(0)} < ${(s.E*s.dt*1000).toFixed(0)} ms)`:`no (${((state.lat+state.rtt+state.jit)).toFixed(0)} > ${(s.E*s.dt*1000).toFixed(0)} ms)`,s.keep?'ok':'bad');
  setRO('r_ctl',(50/state.fa).toFixed(1));
}
function bind(id,fmt){const e=$(id),o=$('o_'+id);e.addEventListener('input',()=>{state[id]=parseFloat(e.value);if(id==='H'&&state.E>state.H){state.E=state.H;$('E').value=state.E;$('o_E').textContent=state.E;}o.textContent=fmt(state[id]);drawTL();updateNote();});o.textContent=fmt(state[id]);}
const FMT={lat:v=>v+' ms',rtt:v=>v+' ms',jit:v=>v+' ms',H:v=>v+' actions',fa:v=>v+' Hz',E:v=>v+' actions'};
Object.keys(FMT).forEach(k=>bind(k,FMT[k]));
function sync(){for(const k in FMT){$(k).value=state[k];$('o_'+k).textContent=FMT[k](state[k]);}
  $('m_async').classList.toggle('on',state.mode==='async');$('m_sync').classList.toggle('on',state.mode==='sync');$('skip').checked=state.skip;}
$('m_async').onclick=()=>{state.mode='async';sync();drawTL();updateNote();};
$('m_sync').onclick=()=>{state.mode='sync';sync();drawTL();updateNote();};
$('skip').onchange=e=>{state.skip=e.target.checked;drawTL();updateNote();};
document.querySelectorAll('[data-preset]').forEach(b=>b.addEventListener('click',()=>{Object.assign(state,PRESETS[b.dataset.preset]);sync();drawTL();updateNote();}));

function updateNote(){
  const parts=[];
  for(const k in DEFAULTS)if(state[k]!==DEFAULTS[k])parts.push(k);
  if(A.arch==='dec'&&A.task==='table'&&!parts.length){$('note').textContent='Change a control above to build a note.';return;}
  const s=SIM;const a=ARCH[A.arch];
  let t=`Marvin Bot (1.44 m, 41 kg, 27-DOF humanoid; Jetson AGX Orin 64 GB, 50 Hz policy, CAN to RobStride actuators at 500 Hz legs / 250 Hz arms). Plan (docs/decisions.md 2026-10-25): decoupled RL legs + IK arms first, GR00T N1.7 served from a desktop GPU then moved to the Orin with TensorRT, motion-tracking controller later.\n`;
  t+=`Control split I looked at: ${a.name}, for the task "${$('task').selectedOptions[0].textContent}". ${VERDICT[A.task][A.arch]}\n`;
  t+=`VLA timing I explored: ${state.lat} ms model + ${state.rtt} ms network + up to ${state.jit} ms jitter; chunks of ${state.H} actions at ${state.fa} Hz, ${Math.min(state.E,state.H)} executed per chunk, ${state.mode==='async'?'inference while executing':'waiting for each chunk'}${state.mode==='async'?(state.skip?', stale actions skipped':', chunks played from their first action'):''}.\n`;
  t+=`Result: arms hold their last target ${(s.holdFrac*100).toFixed(0)}% of the time; executed actions are planned from images ${(s.mean*1000).toFixed(0)} ms old on average (max ${(s.max*1000).toFixed(0)} ms); ${s.keep?'the pipeline keeps up':'the pipeline does not keep up'}.\n`;
  t+=`Latency figures are from NVIDIA's Isaac-GR00T N1.7 benchmark (Orin TensorRT 149 ms, L40 TensorRT 35 ms); network figures are my estimates. Question: what chunk/horizon settings and where to run the model for roadmap open item 10 (N1.7 closed-loop latency on the Orin with the 50 Hz controller running)?`;
  $('note').textContent=t;}
$('copy').addEventListener('click',()=>{navigator.clipboard.writeText($('note').textContent).then(()=>{const b=$('copy');b.textContent='Copied!';setTimeout(()=>b.textContent='Copy',1200);});});
drawArch();sync();drawTL();updateNote();
