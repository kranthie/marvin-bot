const $ = id => document.getElementById(id);
const NS='http://www.w3.org/2000/svg';
function el(tag, attrs, parent, text){ const e=document.createElementNS(NS,tag); for(const k in attrs) e.setAttribute(k,attrs[k]); if(text!==undefined) e.textContent=text; if(parent) parent.appendChild(e); return e; }
const fmt=(v,d=2)=>Number(v).toFixed(d);

/* ================= Playground 1: plate strength ================= */
const MATS={
 al6061r:{n:'6061-T6, repo design value',y:240,rho:2.70,E:68900},
 al6061:{n:'6061-T6, datasheet typical',y:276,rho:2.70,E:68900},
 al7075r:{n:'7075-T6, repo design value',y:480,rho:2.81,E:71700},
 al7075:{n:'7075-T6, datasheet typical',y:503,rho:2.81,E:71700},
 pacfxy:{n:'PA6-CF printed, in layer (datasheet)',y:82,rho:1.17,E:5670},
 pacfz:{n:'PA6-CF printed, across layers (repo, conservative)',y:25,rho:1.10,E:4710},
 petg:{n:'PETG printed (datasheet)',y:47,rho:1.27,E:2000},
};
const sel=$('mat'); Object.entries(MATS).forEach(([k,v])=>{ const o=document.createElement('option'); o.value=k; o.textContent=v.n; sel.appendChild(o); });
const DEF={shape:'pocketed',b:59,t:9,f:2,rib:26,d:20,mat:'al6061r',mult:1,mass:40.3,L:115,bend:'out',Kt:1};
const st={...DEF};
const PRESETS={
 thigh:{...DEF},
 thigh8:{...DEF,t:8},
 drill6:{...DEF,shape:'drilled',t:6,d:20,Kt:2.5},
 al7075:{...DEF,shape:'solid',t:6,mat:'al7075r'},
 pacf:{...DEF,mat:'pacfxy'},
};
const UNITS={b:' mm',t:' mm',f:' mm',rib:' mm',d:' mm',mult:' ×',mass:' kg',L:' mm',Kt:''};
function sync(){ for(const k in st){ const e=$(k); if(e){ e.value=st[k]; const o=$('o_'+k); if(o) o.textContent=st[k]+(UNITS[k]||''); } } }
['b','t','f','rib','d','mult','mass','L','Kt'].forEach(k=>$(k).addEventListener('input',e=>{ st[k]=parseFloat(e.target.value); $('o_'+k).textContent=st[k]+UNITS[k]; renderBeam(); }));
['shape','mat','bend'].forEach(k=>$(k).addEventListener('change',e=>{ st[k]=e.target.value; renderBeam(); }));
document.querySelectorAll('[data-preset]').forEach(b=>b.addEventListener('click',()=>{ Object.assign(st,PRESETS[b.dataset.preset]); sync(); renderBeam(); }));
function rects(){ // [y0,y1,z0,z1] in mm; y across width, z across thickness
  const {b,t}=st; const f=Math.min(st.f,t), rib=Math.min(st.rib,b), d=Math.min(st.d,b-2);
  if(st.shape==='solid') return [[0,b,0,t]];
  if(st.shape==='drilled') return [[0,(b-d)/2,0,t],[(b+d)/2,b,0,t]];
  return [[0,rib/2,0,t],[rib/2,b-rib/2,0,f],[b-rib/2,b,0,t]];
}
function sectionProps(R, axis){ // axis 'out': stress varies with z; 'in': with y
  let A=0,Sy=0,Sz=0; R.forEach(([y0,y1,z0,z1])=>{ const a=(y1-y0)*(z1-z0); A+=a; Sy+=a*(y0+y1)/2; Sz+=a*(z0+z1)/2; });
  const yc=Sy/A, zc=Sz/A; let I=0;
  R.forEach(([y0,y1,z0,z1])=>{ const w=y1-y0,h=z1-z0,a=w*h;
    if(axis==='out') I+=w*h*h*h/12 + a*Math.pow((z0+z1)/2-zc,2);
    else I+=h*w*w*w/12 + a*Math.pow((y0+y1)/2-yc,2); });
  const c= axis==='out' ? Math.max(zc, st.t-zc) : Math.max(yc, st.b-yc);
  return {A,I,c,yc,zc};
}
let beamRes={};
function renderBeam(){
  ['c_f','c_rib'].forEach(id=>$(id).style.display=st.shape==='pocketed'?'':'none'); $('c_d').style.display=st.shape==='drilled'?'':'none';
  const M0=MATS[st.mat]; $('matHint').textContent=`yield ${M0.y} MPa, density ${M0.rho} g/cm³, E ${fmt(M0.E/1000,1)} GPa`;
  const R=rects(); const S=sectionProps(R, st.bend);
  const F=st.mult*st.mass*9.81, M=F*st.L; // N, N·mm
  const sig=st.Kt*M*S.c/S.I, sf=M0.y/sig;
  const defl=F*Math.pow(st.L,3)/(3*M0.E*S.I); // mm
  const g100=S.A*100/1000*M0.rho; // mm² × 100 mm = mm³; /1000 = cm³; × g/cm³
  const set=(id,v,cls)=>{const r=$(id); r.querySelector('.v').textContent=v; r.className='ro '+(cls||'');};
  set('r_M',fmt(M/1000,1)+' N·m'); set('r_I',fmt(S.I,0)+' mm⁴'); set('r_s',fmt(sig,0)+' MPa');
  set('r_sf',fmt(sf,2), sf>=2?'ok':(sf>=1?'warn':'bad')); set('r_def',fmt(defl,2)+' mm'); set('r_m',fmt(g100,0)+' g');
  $('beamNote').textContent=`F = ${fmt(st.mult,2)} × ${fmt(st.mass,1)} kg × 9.81 = ${fmt(F,0)} N; M = F·L = ${fmt(M/1000,1)} N·m; section area ${fmt(S.A,0)} mm², c = ${fmt(S.c,2)} mm; σ = ${st.Kt!==1?fmt(st.Kt,2)+' × ':''}M·c/I. Deflection = F·L³/(3·E·I) for a uniform cantilever. Mass is for this section over 100 mm (pockets and holes do not run the full length, so the real part is a little heavier).`;
  // drawing
  const svg=$('beam'); svg.innerHTML=''; el('rect',{x:0,y:0,width:640,height:360,fill:'#0c1015'},svg);
  // side view cantilever
  const wx=40, wy=40; el('rect',{x:wx-14,y:wy,width:14,height:150,fill:'#3a4654'},svg);
  for(let i=0;i<8;i++) el('line',{x1:wx-14,y1:wy+10+i*18,x2:wx-26,y2:wy+22+i*18,stroke:'#56616d'},svg);
  const Lpx=Math.min(300,40+st.L*0.9), th=Math.max(4,Math.min(40,(st.bend==='out'?st.t:st.b)*1.2));
  const dpx=Math.min(40,defl*4);
  el('path',{d:`M${wx},${wy+75-th/2} Q${wx+Lpx*0.6},${wy+75-th/2} ${wx+Lpx},${wy+75-th/2+dpx} L${wx+Lpx},${wy+75+th/2+dpx} Q${wx+Lpx*0.6},${wy+75+th/2} ${wx},${wy+75+th/2} Z`,fill:'#4fb3ff','fill-opacity':.35,stroke:'#4fb3ff'},svg);
  el('line',{x1:wx+Lpx,y1:wy+5,x2:wx+Lpx,y2:wy+75-th/2+dpx-2,stroke:'#ff6b6b','stroke-width':3,'marker-end':'url(#ar)'},svg);
  const defs=el('defs',{},svg); const mk=el('marker',{id:'ar',markerWidth:8,markerHeight:8,refX:4,refY:4,orient:'auto'},defs); el('path',{d:'M0,0 L8,4 L0,8 Z',fill:'#ff6b6b'},mk);
  el('text',{x:wx+Lpx+6,y:wy+18,'font-size':12,fill:'#ff6b6b'},svg,`F = ${fmt(F,0)} N`);
  el('line',{x1:wx,y1:wy+130,x2:wx+Lpx,y2:wy+130,stroke:'#9aa7b4'},svg); el('text',{x:wx+Lpx/2,y:wy+146,'font-size':12,fill:'#9aa7b4','text-anchor':'middle'},svg,`L = ${st.L} mm`);
  el('text',{x:wx,y:22,'font-size':12,fill:'#9aa7b4'},svg,'Side view (deflection × 4)');
  // cross-section
  const cx0=370, cy0=60, sc=Math.min(150/st.b, 90/Math.max(st.t,1));
  el('text',{x:cx0,y:22,'font-size':12,fill:'#9aa7b4'},svg,'Cross-section and bending stress');
  const ox=cx0, oy=cy0+40;
  R.forEach(([y0,y1,z0,z1])=>el('rect',{x:ox+y0*sc,y:oy+(st.t-z1)*sc,width:(y1-y0)*sc,height:(z1-z0)*sc,fill:'#4fb3ff','fill-opacity':.55,stroke:'#4fb3ff'},svg));
  if(st.bend==='out'){ const ny=oy+(st.t-S.zc)*sc; el('line',{x1:ox-10,x2:ox+st.b*sc+10,y1:ny,y2:ny,stroke:'#ffb454','stroke-dasharray':'4 3'},svg); el('text',{x:ox-14,y:ny+4,'font-size':10,fill:'#ffb454','text-anchor':'end'},svg,'neutral');
    // stress profile to the right
    const px=ox+st.b*sc+50, smax=sig; const top=oy, bot=oy+st.t*sc;
    const sTop=st.Kt*M*(st.t-S.zc)/S.I, sBot=st.Kt*M*S.zc/S.I, k=60/Math.max(smax,1);
    el('line',{x1:px,x2:px,y1:top-5,y2:bot+5,stroke:'#56616d'},svg);
    el('path',{d:`M${px},${top} L${px+sTop*k},${top} L${px-sBot*k},${bot} L${px},${bot} Z`,fill:'#ff6b6b','fill-opacity':.35,stroke:'#ff6b6b'},svg);
    el('text',{x:px+4,y:top-8,'font-size':10.5,fill:'#ff9b9b'},svg,`${fmt(sTop,0)} MPa`); el('text',{x:px-4,y:bot+16,'font-size':10.5,fill:'#ff9b9b','text-anchor':'end'},svg,`${fmt(sBot,0)} MPa`);
  } else { const nx=ox+S.yc*sc; el('line',{x1:nx,x2:nx,y1:oy-10,y2:oy+st.t*sc+10,stroke:'#ffb454','stroke-dasharray':'4 3'},svg); el('text',{x:nx+4,y:oy-14,'font-size':11,fill:'#ffb454'},svg,'neutral axis');
    const py=oy+st.t*sc+30, k=60/Math.max(sig,1); el('path',{d:`M${ox},${py} L${ox},${py-sig*k*0.6} L${ox+st.b*sc},${py+sig*k*0.6} L${ox+st.b*sc},${py} Z`,fill:'#ff6b6b','fill-opacity':.35,stroke:'#ff6b6b'},svg);
    el('text',{x:ox,y:py+sig*k*0.6+16,'font-size':10.5,fill:'#ff9b9b'},svg,`±${fmt(sig,0)} MPa at the edges`); }
  el('text',{x:ox,y:oy+st.t*sc+ (st.bend==='out'?44:120),'font-size':11,fill:'#9aa7b4'},svg,`b = ${st.b} mm, t = ${st.t} mm${st.shape==='pocketed'?`, floor ${st.f} mm`:st.shape==='drilled'?`, hole Ø${st.d}`:''}`);
  // SF bar
  const by=300; el('text',{x:40,y:by-8,'font-size':12,fill:'#9aa7b4'},svg,'Safety factor = yield / stress');
  const X=v=>40+Math.min(v,5)/5*560; el('rect',{x:40,y:by,width:560,height:16,fill:'#1c232c',rx:3},svg);
  el('rect',{x:40,y:by,width:X(sf)-40,height:16,fill:sf>=2?'#4cc38a':sf>=1?'#ffb454':'#ff6b6b',rx:3},svg);
  [1,2,3,4,5].forEach(v=>{ el('line',{x1:X(v),x2:X(v),y1:by-3,y2:by+19,stroke:v===2?'#e6edf3':'#56616d','stroke-dasharray':v===2?'':'2 2'},svg); el('text',{x:X(v),y:by+32,'font-size':10,fill:'#9aa7b4','text-anchor':'middle'},svg,v===2?'2 (target)':(v===5?'5+':v)); });
  beamRes={sig,sf,g100,defl,M,I:S.I}; updateNote();
}

/* ================= Playground 2: joint sweep ================= */
const CDEF={phi:0,th:8,a:55,Rc:90,mR:42.5,gap:4};
const cs={...CDEF};
const CPRE={today:{...CDEF},narrow:{...CDEF,Rc:30},out:{...CDEF,a:75},motor:{...CDEF,mR:55}};
const FIX={armT:0.07,L:0.2,mC:0.06,mLen:0.05,W:0.12,Dh:0.09,ztop:0.06,zbot:-0.45};
const REPO=[[20,0],[45,-10],[75,-10],[90,-20],[135,-20],[150,0]];
const D2R=Math.PI/180;
function geo(){ return {...FIX,a:cs.a/1000,Rc:Math.min(cs.Rc/1000,FIX.Dh),mR:cs.mR/1000,gap:cs.gap/1000}; }
function armPts(p){ const pts=[], h=p.armT/2;
  for(let z=0;z>=-p.L-1e-9;z-=0.01) for(const x of [-h,0,h]) for(const y of [-h,0,h]) pts.push([x,y,z]);
  for(let z=-p.mC+p.mLen/2;z>=-p.mC-p.mLen/2-1e-9;z-=0.0125) for(let i=0;i<16;i++){ const t=i/16*2*Math.PI; pts.push([p.mR*Math.cos(t),p.mR*Math.sin(t),z]); }
  return pts; }
function world(q,phi,th,p){ const [px,py,pz]=q; const yp=py*Math.cos(th)-pz*Math.sin(th), zp=py*Math.sin(th)+pz*Math.cos(th);
  return [px*Math.cos(phi)-zp*Math.sin(phi), p.a+yp, px*Math.sin(phi)+zp*Math.cos(phi)]; }
function inTorso(w,p){ const [x,y,z]=w; if(z>p.ztop||z<p.zbot) return false; const g=p.gap; const X=Math.abs(x), Y=Math.abs(y+p.W); const hx=p.Dh+g, hy=p.W+g, r=p.Rc+g;
  if(X>hx||Y>hy) return false; const dx=X-(hx-r), dy=Y-(hy-r); if(dx>0&&dy>0) return dx*dx+dy*dy<r*r; return true; }
function inHead(w,p){ const [x,y,z]=w; const g=p.gap; return Math.abs(x)<0.08+g && Math.abs(y+p.W)<0.071+g && z>0.12-g && z<0.30; }
function contacts(phi,th,p,pts){ const out=[]; for(const q of pts){ const w=world(q,phi,th,p); if(inTorso(w,p)||inHead(w,p)) out.push(w);} return out; }
function firstContact(phiDeg,p,pts){ for(let t=Math.max(cs.th,0);t>=-40;t-=0.5){ if(contacts(phiDeg*D2R,t*D2R,p,pts).length) return t+0.5; } return -40; }
function minRollCurve(p,pts){ const c=[]; for(let f=-60;f<=180;f+=5){ let r=-40; for(let t=0;t>=-40;t-=0.5){ if(contacts(f*D2R,t*D2R,p,pts).length){ r=t+0.5; break; } } c.push([f,r]); } return c; }
function repoMin(f){ if(f<=REPO[0][0]||f>=REPO[REPO.length-1][0]) return 0; for(let i=0;i<REPO.length-1;i++){ const [a,qa]=REPO[i],[b,qb]=REPO[i+1]; if(f>=a&&f<=b) return qa+(qb-qa)*(f-a)/(b-a);} return 0; }
let curveCache=null, curveKey='';
function renderSweep(){
  const p=geo(), pts=armPts(p), phi=cs.phi*D2R, th=cs.th*D2R;
  const hit=contacts(phi,th,p,pts);
  const svg=$('sweep'); svg.innerHTML=''; el('rect',{x:0,y:0,width:640,height:320,fill:'#0c1015'},svg);
  const sc=420;
  // front view (y right = outward, z up). origin = roll axis
  const fsc=360, fx=200, fz=36+0.30*fsc; const FY=y=>fx+y*fsc, FZ=z=>fz-z*fsc;
  el('text',{x:10,y:16,'font-size':12,fill:'#9aa7b4'},svg,'Front view (looking at the robot)');
  el('rect',{x:FY(-2*p.W),y:FZ(p.ztop),width:2*p.W*fsc,height:(p.ztop-p.zbot)*fsc,fill:'#ffb454','fill-opacity':.18,stroke:'#ffb454'},svg);
  el('rect',{x:FY(-p.W-0.071),y:FZ(0.30),width:0.142*fsc,height:0.18*fsc,rx:20,fill:'#ffb454','fill-opacity':.12,stroke:'#ffb454'},svg);
  el('text',{x:FY(-p.W),y:FZ(-0.2),'font-size':11,fill:'#ffb454','text-anchor':'middle'},svg,'torso');
  // top view (x forward = down the page? use: x up the page, y right)
  const tx=470, ty=170; const TY=y=>tx+y*sc*0.9, TX=x=>ty-x*sc*0.9;
  el('text',{x:330,y:16,'font-size':12,fill:'#9aa7b4'},svg,'Top view (front of robot up)');
  const r=p.Rc*sc*0.9; el('rect',{x:TY(-2*p.W),y:TX(p.Dh),width:2*p.W*sc*0.9,height:2*p.Dh*sc*0.9,rx:r,fill:'#ffb454','fill-opacity':.18,stroke:'#ffb454'},svg);
  el('text',{x:TY(-p.W),y:TX(0)+4,'font-size':11,fill:'#ffb454','text-anchor':'middle'},svg,'chest section');
  // draw arm samples projected, coloured by contact
  const drawSet=(q,col,op)=>{ const w=world(q,phi,th,p); el('circle',{cx:FY(w[1]),cy:FZ(w[2]),r:1.6,fill:col,'fill-opacity':op},svg); el('circle',{cx:TY(w[1]),cy:TX(w[0]),r:1.6,fill:col,'fill-opacity':op},svg); };
  pts.forEach(q=>drawSet(q,'#4fb3ff',.55));
  hit.forEach(w=>{ el('circle',{cx:FY(w[1]),cy:FZ(w[2]),r:3,fill:'#ff6b6b'},svg); el('circle',{cx:TY(w[1]),cy:TX(w[0]),r:3,fill:'#ff6b6b'},svg); });
  el('circle',{cx:FY(p.a),cy:FZ(0),r:4,fill:'#e6edf3'},svg); el('circle',{cx:TY(p.a),cy:TX(0),r:4,fill:'#e6edf3'},svg);
  el('text',{x:FY(p.a)+8,y:FZ(0)-6,'font-size':10.5,fill:'#e6edf3'},svg,'shoulder');
  // curve
  const key=JSON.stringify(p);
  if(key!==curveKey){ curveCache=minRollCurve(p,pts); curveKey=key; }
  const cv=$('curve'); cv.innerHTML=''; el('rect',{x:0,y:0,width:640,height:230,fill:'#0c1015'},cv);
  const CX=f=>60+(f+60)/240*560, CY=q=>30+(10-q)/50*160;
  for(let q=0;q>=-40;q-=10){ el('line',{x1:60,x2:620,y1:CY(q),y2:CY(q),stroke:'#1f2730'},cv); el('text',{x:54,y:CY(q)+4,'font-size':10,fill:'#6b7785','text-anchor':'end'},cv,q+'°'); }
  for(let f=-60;f<=180;f+=30){ el('text',{x:CX(f),y:206,'font-size':10,fill:'#6b7785','text-anchor':'middle'},cv,f+'°'); }
  el('text',{x:340,y:224,'font-size':11,fill:'#9aa7b4','text-anchor':'middle'},cv,'shoulder pitch (arm forward)');
  el('text',{x:60,y:16,'font-size':11,fill:'#9aa7b4'},cv,'lowest shoulder roll before contact (allowed region is above the line)');
  el('line',{x1:60,x2:620,y1:CY(-20),y2:CY(-20),stroke:'#56616d','stroke-dasharray':'2 3'},cv); el('text',{x:618,y:CY(-20)-4,'font-size':9.5,fill:'#6b7785','text-anchor':'end'},cv,'joint range −20°');
  el('polyline',{points:curveCache.map(([f,q])=>`${CX(f)},${CY(q)}`).join(' '),fill:'none',stroke:'#4fb3ff','stroke-width':2},cv);
  const rp=[]; for(let f=-60;f<=180;f+=1) rp.push(`${CX(f)},${CY(repoMin(f))}`); el('polyline',{points:rp.join(' '),fill:'none',stroke:'#ffb454','stroke-width':2,'stroke-dasharray':'5 3'},cv);
  el('circle',{cx:CX(cs.phi),cy:CY(cs.th),r:5,fill:hit.length?'#ff6b6b':'#4cc38a',stroke:'#e6edf3'},cv);
  const set=(id,v,cls)=>{const r=$(id); r.querySelector('.v').textContent=v; r.className='ro '+(cls||'');};
  set('r_hit',hit.length?`contact (${hit.length} pts)`:'clear', hit.length?'bad':'ok');
  const fc=firstContact(cs.phi,p,pts); set('r_first',fc<=-40?'none above −40°':fmt(fc,1)+'°');
  const at=f=>curveCache.find(c=>c[0]===f)[1];
  set('r_hang',fmt(at(0),1)+'°'); set('r_fwd',fmt(at(90),1)+'°');
  updateNote();
}
['phi','th','a','Rc','mR','gap'].forEach(k=>{ const e=$(k), o=$('o_'+k); const f=v=>(k==='phi'||k==='th')?v+'°':v+' mm';
  e.addEventListener('input',()=>{ cs[k]=parseFloat(e.value); o.textContent=f(cs[k]); renderSweep(); }); o.textContent=f(e.value); });
document.querySelectorAll('[data-cpreset]').forEach(b=>b.addEventListener('click',()=>{ const p=CPRE[b.dataset.cpreset]; Object.assign(cs,p); for(const k in p){ $(k).value=p[k]; $('o_'+k).textContent=p[k]+((k==='phi'||k==='th')?'°':' mm'); } renderSweep(); }));

/* ================= decision note ================= */
function updateNote(){
  const parts=[];
  const ch=Object.keys(DEF).filter(k=>st[k]!==DEF[k]);
  if(ch.length && beamRes.sf!==undefined){
    const sh={solid:'a solid rectangle',drilled:`a drilled section (Ø${st.d} mm hole)`,pocketed:`pocketed to a ${st.f} mm floor with ${st.rib} mm of full-depth ribs`}[st.shape];
    parts.push(`Plate check: ${st.b} mm wide, ${st.t} mm stock, ${sh}, in ${MATS[st.mat].n} (yield ${MATS[st.mat].y} MPa). Load ${st.mult} × body weight of ${st.mass} kg at a ${st.L} mm lever arm, bent ${st.bend==='out'?'out of plane':'in plane'}${st.Kt!==1?', with a stress concentration factor of '+st.Kt:''}: M = ${fmt(beamRes.M/1000,1)} N·m, I = ${fmt(beamRes.I,0)} mm⁴, σ = ${fmt(beamRes.sig,0)} MPa, SF = ${fmt(beamRes.sf,2)} (target 2), tip deflection ${fmt(beamRes.defl,2)} mm, ${fmt(beamRes.g100,0)} g per 100 mm. (Today's thigh plate: 59 × 9 mm pocketed to 2 mm, 6061, side fall 1 × BW at 115 mm, SF ≈ 2.2.)`);
  }
  const cch=['a','Rc','mR','gap'].filter(k=>cs[k]!==CDEF[k]);
  if((cch.length || cs.phi!==CDEF.phi || cs.th!==CDEF.th) && curveCache){
    const at=f=>curveCache.find(c=>c[0]===f)[1];
    parts.push(`Shoulder sweep (2-D toy, not the CAD): roll axis ${cs.a} mm outboard of the torso side, chest corner radius ${cs.Rc} mm, shoulder yaw motor radius ${cs.mR} mm, ${cs.gap} mm clearance. Lowest roll before contact: ${fmt(at(0),1)}° with the arm hanging, ${fmt(at(45),1)}° at 45° pitch, ${fmt(at(90),1)}° at 90° pitch (repo coupled limit: 0°, −10°, −20°). Pose checked: pitch ${cs.phi}°, roll ${cs.th}°. Confirm with python -m cad.covers --check and tools/check_packaging.py.`);
  }
  $('note').textContent = parts.length ? `For Marvin Bot (1.44 m, 40.3 kg bare, machined 6061 cores and printed PA-CF shells; strength target SF 2 by cad/strength.py, landing 3 × BW up, 0.5 fore-aft, 0.3 side):\n\n${parts.join('\n\n')}`
                                        : 'Change a control above to build a note.';
}
$('copy').addEventListener('click',()=>{ navigator.clipboard.writeText($('note').textContent).then(()=>{ const b=$('copy'); b.textContent='Copied!'; setTimeout(()=>b.textContent='Copy',1200); }); });
sync(); renderBeam(); renderSweep();
