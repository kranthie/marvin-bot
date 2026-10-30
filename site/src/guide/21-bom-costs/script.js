const BOM = [{"id":"sts3250","g":"act","item":"Feetech STS3250 serial-bus servo","qty":4,"sp":0,"unit":97.0,"line":388.0,"basis":"2026-10-29: $97 (AIFit Lab, ST-3250-C001, backordered); $125 elsewhere (sold out)","src":"https://aifitlab.com/products/feetech-sts3250-servo-motor","note":"neck yaw/pitch and wrist pitch; check the variant (C001 or C002) before ordering","ph":{"5":4},"cn":true,"tier":null},{"id":"rs00","g":"act","item":"RobStride RS00 14 N·m actuator","qty":2,"sp":0,"unit":138.99,"line":277.98,"basis":"Seeed 2026-10-29: $138.99 each, $131.99 at 10+, in stock (RobStride list $125)","src":"https://www.seeedstudio.com/Robostride-00-Actuator-p-6664.html","note":"wrist yaw","ph":{"5":2},"cn":true,"tier":131.99},{"id":"rs03","g":"act","item":"RobStride RS03 60 N·m actuator","qty":11,"sp":1,"unit":225.0,"line":2700.0,"basis":"Seeed 2026-10-29: $225 each, $214 at 10+, in stock (RobStride list $225)","src":"https://www.seeedstudio.com/Robostride-03-Actuator-p-6774.html","note":"hip yaw and the two ankle motors, both legs; waist yaw; shoulder pitch and roll. Buy one first for the actuator bench test; 10 or more in one order get Seeed's $214 price","ph":{"2":1,"3":11},"cn":true,"tier":214},{"id":"rs04","g":"act","item":"RobStride RS04 120 N·m actuator","qty":6,"sp":1,"unit":255.0,"line":1785.0,"basis":"Seeed 2026-10-29: $255 each, $242 at 10+, in stock (RobStride list $255)","src":"https://www.seeedstudio.com/Robostride-04-Actuator-p-6775.html","note":"hip pitch, hip roll and knee, both legs; buy one first for the actuator bench test","ph":{"2":1,"3":2,"4":4},"cn":true,"tier":242},{"id":"rs06","g":"act","item":"RobStride RS06 36 N·m actuator","qty":4,"sp":1,"unit":210.0,"line":1050.0,"basis":"Seeed 2026-10-29: $210 each, $200 at 10+, in stock (RobStride list $210)","src":"https://www.seeedstudio.com/Robostride-06-Actuator-p-6668.html","note":"shoulder yaw and elbow","ph":{"5":5},"cn":true,"tier":200},{"id":"hand-kit","g":"hands","item":"AmazingHand open-source 8-DOF four-finger hand kit, left and right (8 Feetech SCS0009 servos each, 5 V)","qty":2,"sp":0,"unit":99.0,"line":198.0,"basis":"Seeed 2026-10-29: $99 each, $97 at 10+, in stock; SKUs 100062181 (left), 100063642 (right)","src":"https://www.seeedstudio.com/Amazing-Hand-Open-Source-3D-Printable-Robotic-Hand-Kit.html","note":"the listing does not say what is in the box, so check that servos are included. 0.40 kg each as published (weigh); its design is CC-BY-4.0, see ACKNOWLEDGMENTS.md","ph":{"5":2},"cn":true,"tier":97},{"id":"dcdc-hand","g":"hands","item":"DC-DC 12 V to 5 V, 3 A, small (in the wrist)","qty":2,"sp":0,"unit":8.0,"line":16.0,"basis":"estimate","src":"","note":"one per hand: 8 SCS0009 servos at up to 0.3 A each","ph":{"5":2},"cn":false,"tier":null},{"id":"servo-adapter-hand","g":"hands","item":"Feetech serial-bus USB adapter (Waveshare Bus Servo Adapter (A) class)","qty":2,"sp":0,"unit":5.0,"line":10.0,"basis":"Waveshare 2026-10-29: $4.99 (SKU 25514)","src":"https://www.waveshare.com/bus-servo-adapter-a.htm","note":"one bus per hand: SCS0009 servos use Feetech's SCS protocol, the STS3250s its STS protocol; the hand kit's own driver board may do instead","ph":{"5":2},"cn":false,"tier":null},{"id":"compute","g":"comp","item":"Jetson AGX Orin 64 GB developer kit","qty":1,"sp":0,"unit":3499.0,"line":3499.0,"basis":"NVIDIA list price since July 2026 (was $1,999); SparkFun 2026-10-29: $3,499, backorder","src":"https://marketplace.nvidia.com/en-us/enterprise/robotics-edge/jetson-agx-orin-developer-kit/","note":"","ph":{"2":1},"cn":false,"tier":null},{"id":"imu-backup","g":"comp","item":"Low-cost USB AHRS IMU","qty":1,"sp":0,"unit":35.0,"line":35.0,"basis":"estimate","src":"","note":"backup IMU and bench use","ph":{"2":1},"cn":false,"tier":null},{"id":"nvme","g":"comp","item":"NVMe SSD, 1 TB, M.2 2280","qty":1,"sp":0,"unit":100.0,"line":100.0,"basis":"estimate","src":"","note":"the dev kit's 64 GB eMMC is too small for logs and datasets","ph":{"2":1},"cn":false,"tier":null},{"id":"camera","g":"comp","item":"Orbbec Gemini 335L depth camera","qty":1,"sp":0,"unit":359.0,"line":359.0,"basis":"Orbbec store list $359 (store page rate-limited 2026-10-29); RobotShop US $413 with tariffs","src":"https://store.orbbec.com/products/gemini-335l","note":"check the JetPack 7.2 driver before ordering","ph":{"5":1},"cn":false,"tier":null},{"id":"imu","g":"comp","item":"Primary IMU (industrial AHRS; or BMI088 on a microcontroller)","qty":1,"sp":0,"unit":730.0,"line":730.0,"basis":"distributor 2026-10","src":"","note":"decide after bench test; the existing USB AHRS IMU is the backup","ph":{"4":1},"cn":false,"tier":null},{"id":"charger","g":"power","item":"13S charger adjustable to 4.1 V per cell","qty":1,"sp":0,"unit":300.0,"line":300.0,"basis":"reseller 2026-10","src":"","note":"daily charging at 4.1 V per cell","ph":{"4":1},"cn":false,"tier":null},{"id":"bms","g":"power","item":"13S smart BMS 150 A with CAN telemetry","qty":1,"sp":0,"unit":145.0,"line":145.0,"basis":"2026-10-29: JK 8-24S 150 A $145 (18650 Battery Store, sold out); Daly 13S 150 A with CAN also sold","src":"https://www.18650batterystore.com/products/jk-b2a24s15p-bms","note":"check its CAN protocol and precharge support","ph":{"4":1},"cn":false,"tier":null},{"id":"psu-bench","g":"power","item":"48 V 20 A bench power supply","qty":1,"sp":0,"unit":250.0,"line":250.0,"basis":"estimate","src":"","note":"for bench and gantry tests","ph":{"2":1},"cn":false,"tier":null},{"id":"batt-conn","g":"power","item":"Anti-spark blind-mate battery connector (AS150 class)","qty":2,"sp":0,"unit":15.0,"line":30.0,"basis":"estimate","src":"","note":"one on the pack and one on the cradle","ph":{"4":2},"cn":false,"tier":null},{"id":"fuse","g":"power","item":"Class-T fuse 100-125 A with holder; limb branch fuses","qty":1,"sp":0,"unit":60.0,"line":60.0,"basis":"estimate","src":"","note":"","ph":{"4":1},"cn":false,"tier":null},{"id":"dcdc-servo","g":"power","item":"DC-DC 48 V to 12 V, 10 A, compact (at most 60 x 34 x 25 mm)","qty":1,"sp":0,"unit":60.0,"line":60.0,"basis":"estimate","src":"","note":"the pelvis channel under the waist motor has room for 60 x 34 x 25 mm; a chassis-mount 150 W module does not fit","ph":{"5":1},"cn":false,"tier":null},{"id":"dcdc-compute","g":"power","item":"DC-DC 75 V-input to 19-20 V; at least 120 W (computer)","qty":1,"sp":0,"unit":80.0,"line":80.0,"basis":"estimate","src":"","note":"upstream of the contactor","ph":{"4":1},"cn":false,"tier":null},{"id":"ideal-diode","g":"power","item":"Ideal-diode OR circuit (LTC4359 class) for the computer rail","qty":1,"sp":0,"unit":40.0,"line":40.0,"basis":"estimate","src":"","note":"","ph":{"4":1},"cn":false,"tier":null},{"id":"keepalive","g":"power","item":"Keep-alive battery 50-100 Wh for the computer","qty":1,"sp":0,"unit":80.0,"line":80.0,"basis":"estimate","src":"","note":"lets the computer stay up during a pack swap","ph":{"4":1},"cn":false,"tier":null},{"id":"contactor","g":"power","item":"Main contactor (48 V DC rated)","qty":1,"sp":0,"unit":60.0,"line":60.0,"basis":"estimate","src":"","note":"opened by the e-stop chain","ph":{"4":1},"cn":false,"tier":null},{"id":"cells","g":"power","item":"Molicel P45B 21700 cells","qty":39,"sp":0,"unit":9.95,"line":388.05,"basis":"2026-10-29: $9.95 (Batteries in a Flash, in stock); 18650 Battery Store $9.99 regular","src":"https://www.18650batterystore.com/products/molicel-p45b","note":"13S3P pack ~630 Wh; buy from an authorised dealer","ph":{"4":39},"cn":false,"tier":null},{"id":"pack-hw","g":"power","item":"Pack housing; nickel strip; quick-release cradle","qty":1,"sp":0,"unit":80.0,"line":80.0,"basis":"estimate","src":"","note":"","ph":{"4":1},"cn":false,"tier":null},{"id":"precharge","g":"power","item":"Precharge resistor and relay","qty":1,"sp":0,"unit":20.0,"line":20.0,"basis":"estimate","src":"","note":"","ph":{"4":1},"cn":false,"tier":null},{"id":"regen-clamp","g":"power","item":"Regen clamp (brake resistor chopper or TVS ~56-58 V)","qty":1,"sp":0,"unit":40.0,"line":40.0,"basis":"estimate","src":"","note":"needed whenever a bench supply is used","ph":{"2":1},"cn":false,"tier":null},{"id":"can-xcvr","g":"elec","item":"3.3 V CAN transceiver boards for the Orin's on-board CAN","qty":2,"sp":0,"unit":10.0,"line":20.0,"basis":"estimate","src":"","note":"arm buses","ph":{"2":2},"cn":false,"tier":null},{"id":"usb-hub","g":"elec","item":"Compact USB 3 hub, 4-port","qty":1,"sp":0,"unit":30.0,"line":30.0,"basis":"estimate","src":"","note":"the dev kit has 4 USB-A and 2 USB-C ports; the robot uses 3 CAN adapters, 3 servo adapters, the camera, the IMU and the gamepad receiver","ph":{"4":1},"cn":false,"tier":null},{"id":"servo-adapter","g":"elec","item":"Feetech serial-bus USB adapter (Waveshare Bus Servo Adapter (A) class)","qty":1,"sp":0,"unit":5.0,"line":5.0,"basis":"Waveshare 2026-10-29: $4.99 (SKU 25514)","src":"https://www.waveshare.com/bus-servo-adapter-a.htm","note":"STS3250 bus (neck and wrist pitch)","ph":{"5":1},"cn":false,"tier":null},{"id":"can-usb-dual","g":"elec","item":"Isolated USB-CAN adapter, dual channel (InnoMaker USB2CAN-X2 class; SocketCAN)","qty":2,"sp":0,"unit":130.0,"line":260.0,"basis":"unconfirmed 2026-10-29: no live US listing found; ~$126 (AED 462) at a UAE marketplace","src":"https://github.com/INNO-MAKER/usb2can","note":"one per leg (hip and shank buses), 3 kV isolation per its manual; with can-usb-batt and the Orin's two on-board ports (arm buses) this covers all 7 CAN buses in robot.yaml in the 3 adapter outlines of the electronics bay","ph":{"3":1,"4":1},"cn":false,"tier":null},{"id":"can-usb-single","g":"elec","item":"Isolated USB-CAN adapter, single channel (CANable Pro class; candleLight / gs_usb)","qty":2,"sp":0,"unit":60.0,"line":120.0,"basis":"reseller listing 2026-10-29, unverified: CANable Pro $60, Pro 1.1 (5 kV) $70","src":"https://reach.dog/shop/openlight-labs/products/canable-pro-isolated-usb-to-can-adapter","note":"one for the battery (BMS) bus, one for the actuator bench rig","ph":{"2":1,"4":1},"cn":false,"tier":null},{"id":"limb-conn","g":"elec","item":"Limb-root connectors: XT60-class power pair and latching signal pair, per limb","qty":4,"sp":0,"unit":12.0,"line":48.0,"basis":"estimate","src":"","note":"on the bulkheads beside each limb's first flange","ph":{"4":2,"5":2},"cn":false,"tier":null},{"id":"can-robstride-dbg","g":"elec","item":"RobStride USB-to-CAN debug adapter (official GUI tool)","qty":1,"sp":0,"unit":15.0,"line":15.0,"basis":"Seeed listing 2026-10, unverified: $15 each, $14 at 10+","src":"https://www.seeedstudio.com/Robostride-CAN-USB-Driver-Board-p-6708.html","note":"for RobStride's configuration GUI (IDs, firmware); check whether it works as a SocketCAN bus before using it on the robot","ph":{"2":1},"cn":false,"tier":14},{"id":"harness","g":"elec","item":"Silicone wire; connectors; strain relief; sleeving","qty":1,"sp":0,"unit":200.0,"line":200.0,"basis":"estimate","src":"","note":"","ph":{"3":0.25,"4":0.5,"5":0.25},"cn":false,"tier":null},{"id":"can-cable","g":"elec","item":"Twisted-pair CAN cable, connectors and 120 Ω terminations","qty":1,"sp":0,"unit":50.0,"line":50.0,"basis":"estimate","src":"","note":"","ph":{"2":0.25,"3":0.25,"4":0.25,"5":0.25},"cn":false,"tier":null},{"id":"gamepad","g":"elec","item":"Wireless gamepad","qty":1,"sp":0,"unit":60.0,"line":60.0,"basis":"estimate","src":"","note":"","ph":{"4":1},"cn":false,"tier":null},{"id":"alu-6061","g":"struct","item":"6061-T6 aluminium plate and bar stock","qty":1,"sp":0,"unit":300.0,"line":300.0,"basis":"estimate","src":"","note":"leg cores and brackets for both legs; about 6 kg of finished CNC 6061 across the robot (build_parts.csv)","ph":{"3":0.5,"4":0.5},"cn":false,"tier":null},{"id":"bearing-6806","g":"struct","item":"6806-2RS thin-section ball bearing (30 x 42 x 7 mm) for far-side joint support","qty":6,"sp":0,"unit":10.0,"line":60.0,"basis":"2026-10-29: $6.45 (Zoro, 61806-2RS) to CA$25.68 (VXB); budget grades ~$8","src":"https://www.zoro.com/ezo-deep-groove-30mm-bore-42mm-od-sealed-6806-2rsc0srl/i/G2080051","note":"hip roll, hip pitch and knee, both legs","ph":{"3":3,"4":3},"cn":false,"tier":null},{"id":"alu-7075","g":"struct","item":"7075-T6 bar for highly loaded flanges and ankle cranks","qty":1,"sp":0,"unit":100.0,"line":100.0,"basis":"estimate","src":"","note":"","ph":{"3":1},"cn":false,"tier":null},{"id":"materials-upper","g":"struct","item":"Aluminium and filament for torso; arms; head","qty":1,"sp":0,"unit":250.0,"line":250.0,"basis":"estimate","src":"","note":"","ph":{"5":1},"cn":false,"tier":null},{"id":"push-rods","g":"struct","item":"Ankle push rods (steel tube with threaded inserts)","qty":4,"sp":0,"unit":10.0,"line":40.0,"basis":"estimate","src":"","note":"two per ankle","ph":{"3":2,"4":2},"cn":false,"tier":null},{"id":"fil-cfnylon","g":"struct","item":"Carbon-fibre nylon filament (kg)","qty":4,"sp":0,"unit":95.0,"line":380.0,"basis":"typical first-party price 2026-10","src":"","note":"structural shells; needs a hardened nozzle and dry storage","ph":{"3":1,"4":1,"5":1,"6":1},"cn":false,"tier":null},{"id":"crossed-roller","g":"struct","item":"Crossed-roller bearing for the waist","qty":1,"sp":0,"unit":80.0,"line":80.0,"basis":"reseller range $20-80","src":"","note":"","ph":{"5":1},"cn":false,"tier":null},{"id":"rod-ends","g":"struct","item":"M6 steel-ball rod ends for ankle push rods","qty":8,"sp":0,"unit":8.0,"line":64.0,"basis":"estimate","src":"","note":"two per push rod; size from simulated linkage loads","ph":{"3":4,"4":4},"cn":false,"tier":null},{"id":"machining-pelvis","g":"struct","item":"Online machining: pelvis back (20 mm, 3D), pelvis top and two gussets (6 mm plates)","qty":1,"sp":0,"unit":400.0,"line":400.0,"basis":"estimate, not quoted (typical online CNC price)","src":"","note":"too large for a desktop mill; process \"service\" in build_parts.csv","ph":{"4":1},"cn":false,"tier":null},{"id":"machining-legs","g":"struct","item":"Online machining: shin inner and outer plates, foot bases (6 pocketed 4-5 mm 6061 plates)","qty":1,"sp":0,"unit":450.0,"line":450.0,"basis":"estimate, not quoted (typical online CNC price for pocketed plates)","src":"","note":"too large for a desktop mill; process \"service\" in build_parts.csv","ph":{"3":0.5,"4":0.5},"cn":false,"tier":null},{"id":"machining-torso","g":"struct","item":"Online machining: two torso uprights (25 mm 6061; 3D)","qty":1,"sp":0,"unit":350.0,"line":350.0,"basis":"estimate, not quoted (typical online CNC price)","src":"","note":"too large for a desktop mill; process \"service\" in build_parts.csv","ph":{"5":1},"cn":false,"tier":null},{"id":"fil-petgcf","g":"struct","item":"PETG-CF filament (kg)","qty":2,"sp":0,"unit":33.0,"line":66.0,"basis":"typical price 2026-10","src":"","note":"jigs and non-structural parts","ph":{"3":2},"cn":false,"tier":null},{"id":"fil-tpu","g":"struct","item":"TPU 95A filament (kg)","qty":1,"sp":0,"unit":42.0,"line":42.0,"basis":"typical price 2026-10","src":"","note":"soles and bumpers","ph":{"3":1},"cn":false,"tier":null},{"id":"fasteners","g":"fast","item":"Metric fasteners; heat-set inserts; dowel pins; threadlocker","qty":1,"sp":0,"unit":180.0,"line":180.0,"basis":"estimate","src":"","note":"includes the 348 actuator bolts in build_fasteners.csv (12.9 socket head, M3-M5)","ph":{"3":0.25,"4":0.4,"5":0.35},"cn":false,"tier":null},{"id":"spot-welder","g":"tool","item":"Battery spot welder and pack supplies (fish paper, insulating rings, Kapton)","qty":1,"sp":0,"unit":150.0,"line":150.0,"basis":"estimate","src":"","note":"or have the pack built by a battery shop","ph":{"4":1},"cn":false,"tier":null},{"id":"endmills","g":"tool","item":"Carbide 3-flute end mills for aluminium (1/8 and 1/4 in)","qty":8,"sp":0,"unit":20.0,"line":160.0,"basis":"reseller range $14-33","src":"","note":"","ph":{"3":8},"cn":false,"tier":null},{"id":"hand-tools","g":"tool","item":"Hand tools: torque screwdriver and hex bits, crimp tools, multimeter, digital scale (5 kg x 1 g)","qty":1,"sp":0,"unit":200.0,"line":200.0,"basis":"estimate","src":"","note":"the scale weighs parts against the mass budget; a desktop CNC mill and 3D printer are assumed and not counted","ph":{"2":1},"cn":false,"tier":null},{"id":"print-tools","g":"tool","item":"Hardened-steel nozzle and filament dryer","qty":1,"sp":0,"unit":80.0,"line":80.0,"basis":"estimate","src":"","note":"carbon-fibre nylon wears brass nozzles and must be printed dry","ph":{"3":1},"cn":false,"tier":null},{"id":"leg-jig","g":"tool","item":"Single-leg test jig (aluminium extrusion)","qty":1,"sp":0,"unit":120.0,"line":120.0,"basis":"estimate","src":"","note":"","ph":{"3":1},"cn":false,"tier":null},{"id":"padding","g":"safety","item":"Fall padding foam","qty":1,"sp":0,"unit":50.0,"line":50.0,"basis":"estimate","src":"","note":"","ph":{"4":1},"cn":false,"tier":null},{"id":"gantry","g":"safety","item":"Gantry frame; harness and hoist","qty":1,"sp":0,"unit":300.0,"line":300.0,"basis":"estimate","src":"","note":"","ph":{"4":1},"cn":false,"tier":null},{"id":"estop-wired","g":"safety","item":"Latching e-stop button and contactor driver","qty":1,"sp":0,"unit":40.0,"line":40.0,"basis":"estimate","src":"","note":"","ph":{"2":1},"cn":false,"tier":null},{"id":"estop-rf","g":"safety","item":"Wireless e-stop: handheld transmitter; receiver and watchdog relay","qty":1,"sp":0,"unit":80.0,"line":80.0,"basis":"estimate","src":"","note":"in series with the wired e-stop","ph":{"4":1},"cn":false,"tier":null}];
const GROUPS = {act:'Actuators',comp:'Compute & sensors',struct:'Structure & materials',power:'Power',elec:'Electronics & wiring',tool:'Tooling',safety:'Safety',hands:'Hands',fast:'Fasteners'};
const PHASES = ['0 Foundations','1 Robot in simulation','2 Actuator bench','3 One leg','4 Lower body','5 Torso, arms, head','6 Perception and intelligence'];
const CUR = {USD:['$',1],EUR:['€',0.92],GBP:['£',0.78],INR:['₹',84],AUD:['A$',1.52]};
const DEFAULTS = {view:'group', tier:'single', owned:[], duty:0, ship:0, cur:'USD'};
const state = JSON.parse(JSON.stringify(DEFAULTS));
const $ = id => document.getElementById(id);
const BOMTOTAL = BOM.reduce((s,r)=>s+r.line,0);

function fmt(usd, dec){ const [sym,k]=CUR[state.cur]; const v=usd*k;
  const d = dec!=null?dec:(Math.abs(v)<100?2:0);
  return (v<0?'−':'')+sym+Math.abs(v).toLocaleString('en-GB',{minimumFractionDigits:d,maximumFractionDigits:d}); }
function isOwned(r){ return state.owned.some(o=> o===r.id || o==='g:'+r.g); }
// price per unit for a given order size, under the current tier rule
function unitPrice(r, orderUnits){
  if(!r.tier) return r.unit;
  const n = r.qty + r.sp;
  if(state.tier==='single') return r.unit;
  if(state.tier==='bulk') return n>=10 ? r.tier : r.unit;
  if(state.tier==='second') return 2*n>=10 ? r.tier : r.unit;
  if(state.tier==='staged') return (n>1 && orderUnits>=10) ? r.tier : r.unit;
  return r.unit;
}
// cost of a line split by phase: {phase: usd}
function upFront(r){ const n=r.qty+r.sp; return !!r.tier && ((state.tier==='bulk'&&n>=10)||(state.tier==='second'&&2*n>=10)); }
function linePhases(r){
  const out={};
  for(const p in r.ph){ const u=r.ph[p]; out[p]=u*unitPrice(r, u); }
  return out;
}
function compute(){
  const duty = state.duty/100;
  let full=0, spend=0, dutyTot=0, act=0, firstCommit=0;
  const byG={}, byP={};
  for(const g in GROUPS) byG[g]={buy:0,own:0,duty:0};
  for(let p=0;p<7;p++) byP[p]={buy:0,own:0,duty:0};
  BOM.forEach(r=>{
    const lp=linePhases(r); let eff=0;
    const own=isOwned(r);
    for(const p in lp){ const c=lp[p]; eff+=c; const d= r.cn? c*duty:0;
      if(own){ byP[p].own+=c; } else { byP[p].buy+=c; byP[p].duty+=d; }
    }
    r.eff=eff; r.own=own;
    const d = r.cn? eff*duty:0;
    full+=eff; if(r.g==='act') act+=eff;
    if(own) byG[r.g].own+=eff; else { byG[r.g].buy+=eff; byG[r.g].duty+=d; spend+=eff+d; dutyTot+=d; }
  });
  // committed before the one-leg test: phases 0-2, plus any line ordered whole up front for its tier
  firstCommit = BOM.filter(r=>!r.own).reduce((s,r)=>{
    const lp=linePhases(r); let c=0; const up=upFront(r);
    for(const p in lp){ if(+p<=2 || up) c+=lp[p]; }
    return s + c*(r.cn?1+duty:1); },0);
  const shipAmt = spend*state.ship/100; spend+=shipAmt;
  return {full,spend,dutyTot,shipAmt,act,firstCommit,byG,byP};
}

function svgEl(tag, attrs, text){ const e=document.createElementNS('http://www.w3.org/2000/svg',tag); for(const k in attrs) e.setAttribute(k,attrs[k]); if(text!=null) e.textContent=text; return e; }

function drawBars(svg, rows, opts){
  // rows: [{label, buy, own, duty}] horizontal stacked bars
  svg.innerHTML='';
  const W=640, top=10, rowH=opts.rowH||36, left=opts.left||170, right=110;
  const H = top + rows.length*rowH + 30; svg.setAttribute('viewBox',`0 0 ${W} ${H}`);
  const max = Math.max(...rows.map(r=>r.buy+r.own+r.duty), opts.minMax||1);
  const nice = niceMax(max); const sx = v => (W-left-right)*v/nice;
  for(let i=0;i<=4;i++){ const v=nice*i/4, x=left+sx(v);
    svg.appendChild(svgEl('line',{x1:x,x2:x,y1:top,y2:H-24,stroke:'#2a3340','stroke-width':1}));
    svg.appendChild(svgEl('text',{x:x,y:H-8,fill:'#9aa7b4','font-size':11,'text-anchor':'middle'}, fmtShort(v)));
  }
  rows.forEach((r,i)=>{
    const y=top+i*rowH+6, h=rowH-14;
    svg.appendChild(svgEl('text',{x:left-8,y:y+h/2+4,fill:'#e6edf3','font-size':13,'text-anchor':'end'}, r.label));
    let x=left;
    if(r.buy>0){ svg.appendChild(svgEl('rect',{x,y,width:Math.max(sx(r.buy),1),height:h,rx:3,fill:'#4fb3ff'})); x+=sx(r.buy); }
    if(r.duty>0){ svg.appendChild(svgEl('rect',{x,y,width:sx(r.duty),height:h,fill:'#ff6b6b'})); x+=sx(r.duty); }
    if(r.own>0){ svg.appendChild(svgEl('rect',{x,y,width:sx(r.own),height:h,rx:3,fill:'#3a4757'})); x+=sx(r.own); }
    const tot=r.buy+r.own+r.duty;
    svg.appendChild(svgEl('text',{x:x+6,y:y+h/2+4,fill:'#e6edf3','font-size':12,'font-family':'ui-monospace,monospace'}, tot>0?fmt(tot,0):'$0 (simulation)'.replace('$',CUR[state.cur][0])));
  });
  return {nice};
}
function niceMax(v){ const p=Math.pow(10,Math.floor(Math.log10(v))); for(const m of [1,1.5,2,2.5,3,4,5,6,8,10]){ if(m*p>=v) return m*p; } return 10*p; }
function fmtShort(usd){ const [sym,k]=CUR[state.cur]; const v=usd*k; if(v>=1e6) return sym+(v/1e6).toFixed(1)+'M'; if(v>=1000) return sym+(v/1000).toFixed(v>=10000?0:1)+'k'; return sym+Math.round(v); }

function render(){
  const R=compute();
  const svg=$('view');
  if(state.view==='group'){
    $('charttitle').textContent='Cost by group, largest first';
    const rows=Object.keys(GROUPS).map(g=>({label:GROUPS[g],...R.byG[g]})).sort((a,b)=>(b.buy+b.own+b.duty)-(a.buy+a.own+a.duty));
    drawBars(svg, rows, {rowH:36,left:170});
    $('leg_cum').style.display='none';
  } else {
    $('charttitle').textContent='Spend per build phase (phase mapping is this guide\'s; see "Spend per build phase")';
    let c=0; const rows=PHASES.map((l,p)=>{ const b=R.byP[p]; c+=b.buy+b.own+b.duty; return {label:l.length>22?l.slice(0,21)+'…':l, ...b, cum:c}; });
    drawBars(svg, rows, {rowH:40,left:190});
    // cumulative labels at right edge
    rows.forEach((r,i)=>{ svg.appendChild(svgEl('text',{x:636,y:10+i*40+6+13+4,fill:'#ffb454','font-size':11,'text-anchor':'end','font-family':'ui-monospace,monospace'}, 'Σ '+fmtShort(r.cum))); });
    $('leg_cum').style.display='';
  }
  setRo('r_full', fmt(R.full,0), '');
  setRo('r_spend', fmt(R.spend,0), R.spend<R.full?'ok':(R.spend>R.full?'warn':''));
  const delta = R.spend - BOMTOTAL;
  setRo('r_save', (delta>0?'+':'')+fmt(delta,0), delta<0?'ok':(delta>0?'bad':''));
  setRo('r_act', Math.round(100*R.act/R.full)+'%', '');
  setRo('r_first', fmt(R.firstCommit,0), '');
  renderTable(); renderPhaseTable(R); updateNote(R);
}
function setRo(id, v, cls){ const el=$(id); el.className='ro'+(cls?' '+cls:''); el.querySelector('.v').textContent=v; }

// ---------- table ----------
let sortK='line', sortDir=-1;
function phaseText(r){ return Object.keys(r.ph).map(p=>{ const u=r.ph[p]; const n=r.qty+r.sp; return (n>1 && Number.isInteger(u))? `${p} (${u})` : p; }).join(', '); }
function renderTable(){
  const fg=$('f_group').value, fp=$('f_phase').value, fk=$('f_kind').value, ft=$('f_text').value.trim().toLowerCase();
  let rows=BOM.filter(r=> (!fg||r.g===fg) && (!fp|| r.ph[fp]!=null) && (!fk || (fk==='est'? /estimate/i.test(r.basis) : fk==='cn'? r.cn : !/estimate/i.test(r.basis))) && (!ft || (r.item+' '+r.note+' '+r.basis+' '+r.id).toLowerCase().includes(ft)));
  rows.sort((a,b)=>{ let x=a[sortK], y=b[sortK]; if(sortK==='g'){x=GROUPS[a.g];y=GROUPS[b.g];} if(sortK==='p'){x=+Object.keys(a.ph)[0];y=+Object.keys(b.ph)[0];}
    return (typeof x==='string'? x.localeCompare(y) : x-y)*sortDir; });
  const tb=document.querySelector('#bomtable tbody'); tb.innerHTML='';
  let sl=0, se=0;
  rows.forEach(r=>{ sl+=r.line; se+=r.eff;
    const tr=document.createElement('tr'); if(r.own) tr.className='owned';
    const tags=(/estimate/i.test(r.basis)?'<span class="tag est">estimate</span>':'')+(r.cn?'<span class="tag cn">ships from China</span>':'')+(r.own?'<span class="tag">owned</span>':'');
    tr.innerHTML=`<td>${GROUPS[r.g]}</td><td>${esc(r.item)}<br>${tags}</td><td class="num">${r.qty}</td><td class="num">${r.sp||''}</td><td class="num">${r.unit.toLocaleString('en-US',{minimumFractionDigits:2})}</td><td class="num">${r.line.toLocaleString('en-US',{minimumFractionDigits:2})}</td><td class="num">${fmt(r.eff,2)}</td><td>${phaseText(r)}</td><td class="basis">${esc(r.basis)}</td>`;
    tb.appendChild(tr); });
  $('t_line').textContent='$'+sl.toLocaleString('en-US',{minimumFractionDigits:2,maximumFractionDigits:2});
  $('t_eff').textContent=fmt(se,2);
  $('f_count').textContent=`${rows.length} of ${BOM.length} lines`;
  document.querySelectorAll('#bomtable th').forEach(th=>th.classList.toggle('sorted', th.dataset.k===sortK));
}
function esc(s){ return String(s).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c])); }
const PHASE_NOTES={
  0:['nothing: repo, requirements, v0 layout','plan'],
  1:['nothing in parts: simulation runs on a training computer (not counted)','plan'],
  2:['one RS04 and one RS03 bought singly, bench supply, regen clamp, single-channel CAN adapter, RobStride debug adapter, wired e-stop, hand tools; the Orin, NVMe and CAN transceivers for the CAN-timing check','plan + decisions; Orin placement is an inference'],
  3:['the other 11 RS03 in one 10+ order, two more RS04, one dual-channel CAN adapter, 7075 bar, half the leg stock and service machining, bearings, push rods, rod ends, jig, end mills, printing tools and filament','plan + decisions; splits are inferences'],
  4:['second leg (three RS04 + spare), pelvis machining, battery cells, BMS, charger, power distribution, contactor, IMU, gantry, wireless e-stop, padding, spot welder','plan; splits are inferences'],
  5:['RS06 ×5, RS00 ×2, four STS3250 servos, hand kits and their converters, camera, crossed-roller waist bearing, torso machining, upper-body materials','plan'],
  6:['the last kilogram of carbon-fibre nylon to finish the shells','plan (“finish the shells”); inference'],
};
function renderPhaseTable(R){
  const tb=$('phasetable'); tb.innerHTML='';
  for(let p=0;p<7;p++){ const b=R.byP[p]; const tr=document.createElement('tr');
    tr.innerHTML=`<td>${PHASES[p]}</td><td class="num">${fmt(b.buy+b.own+b.duty,0)}</td><td>${PHASE_NOTES[p][0]}</td><td style="color:var(--muted)">${PHASE_NOTES[p][1]}</td>`; tb.appendChild(tr); }
}

// ---------- cost per N·m ----------
const ACT=[
  {m:'RS04',peak:120,rated:40,p:255,t:242,n:7},
  {m:'RS03',peak:60,rated:20,p:225,t:214,n:12},
  {m:'RS06',peak:36,rated:11,p:210,t:200,n:5},
  {m:'RS00',peak:14,rated:5,p:138.99,t:131.99,n:2},
  {m:'STS3250 servo',peak:4.9,rated:1.57,p:97,t:97,n:4},
  {m:'Damiao DM-J10010L',peak:120,rated:40,p:230,t:230,alt:true},
  {m:'CubeMars AK80-64',peak:120,rated:48,p:890,t:890,alt:true},
];
const nm={by:'peak',price:'single',alt:false};
function renderNm(){
  const svg=$('nmview'); svg.innerHTML='';
  const list=ACT.filter(a=>nm.alt||!a.alt).map(a=>({...a,v:(nm.price==='tier'?a.t:a.p)/a[nm.by]}));
  const W=640,left=150,right=90,rowH=34,top=8,H=top+list.length*rowH+28; svg.setAttribute('viewBox',`0 0 ${W} ${H}`);
  const nice=niceMax(Math.max(...list.map(a=>a.v))); const sx=v=>(W-left-right)*v/nice;
  for(let i=0;i<=4;i++){ const v=nice*i/4,x=left+sx(v);
    svg.appendChild(svgEl('line',{x1:x,x2:x,y1:top,y2:H-22,stroke:'#2a3340'}));
    svg.appendChild(svgEl('text',{x,y:H-6,fill:'#9aa7b4','font-size':11,'text-anchor':'middle'},'$'+(+v.toFixed(1))));
  }
  list.forEach((a,i)=>{ const y=top+i*rowH+5,h=rowH-12;
    svg.appendChild(svgEl('text',{x:left-8,y:y+h/2+4,fill:a.alt?'#9aa7b4':'#e6edf3','font-size':13,'text-anchor':'end'},a.m));
    svg.appendChild(svgEl('rect',{x:left,y,width:sx(a.v),height:h,rx:3,fill:a.alt?'#b48cff':'#4fb3ff'}));
    svg.appendChild(svgEl('text',{x:left+sx(a.v)+6,y:y+h/2+4,fill:'#e6edf3','font-size':12,'font-family':'ui-monospace,monospace'},'$'+a.v.toFixed(2)+'/N·m'));
  });
  $('nmtitle').textContent=`US dollars per N·m of ${nm.by==='peak'?'peak':'continuous (rated)'} torque, ${nm.price==='tier'?'10+ tier':'single-unit'} price`+(nm.alt?' (purple: alternatives at reseller prices)':'');
  const bom=list.filter(a=>!a.alt); const best=bom.reduce((a,b)=>a.v<b.v?a:b);
  setRo('n_best', `${best.m} $${best.v.toFixed(2)}`, 'ok');
  // installed: total RobStride + servo spend (used units only) / total torque installed (used units)
  const used={RS04:6,RS03:11,RS06:4,RS00:2,'STS3250 servo':4};
  let $s=0,tq=0; ACT.filter(a=>!a.alt).forEach(a=>{ $s+=used[a.m]*(nm.price==='tier'?a.t:a.p); tq+=used[a.m]*a[nm.by]; });
  setRo('n_robot', `$${($s/tq).toFixed(2)}/N·m (${Math.round(tq)} N·m)`, '');
}
function nmBtn(id,k,v,other){ $(id).addEventListener('click',()=>{ nm[k]=v; $(id).classList.add('on'); $(other).classList.remove('on'); renderNm(); }); }
nmBtn('t_peak','by','peak','t_rated'); nmBtn('t_rated','by','rated','t_peak');
nmBtn('t_single','price','single','t_tier'); nmBtn('t_tier','price','tier','t_single');
$('alt').addEventListener('change',e=>{ nm.alt=e.target.checked; renderNm(); });

// ---------- note ----------
function updateNote(R){
  const parts=[];
  const tierTxt={staged:'staged buying (one order per build phase, taking the 10+ price wherever a single order reaches ten units)',bulk:'ordering everything in one order now to get the 10+ tier where a model reaches ten units',second:'ordering for two robots at once (cost shown per robot)'};
  if(state.tier!=='single') parts.push(tierTxt[state.tier]);
  const ownNames=[]; document.querySelectorAll('#owned input:checked').forEach(c=>ownNames.push(c.parentElement.textContent.trim()));
  if(ownNames.length) parts.push('already owning: '+ownNames.join('; '));
  if(state.duty) parts.push(`a hypothetical ${state.duty}% duty on the parts shipped from China (actuators, servos, hand kits)`);
  if(state.ship) parts.push(`a ${state.ship}% shipping allowance`);
  if(state.cur!=='USD') parts.push(`amounts shown in ${state.cur} at an illustrative fixed rate`);
  if(!parts.length){ $('note').textContent='Change a control above to build a note.'; return; }
  const delta=R.spend-BOMTOTAL;
  $('note').textContent=`For Marvin Bot (1.44 m, 41 kg, 27-DOF humanoid; bom/bom.xlsx totals $17,531.03 for 60 lines at October 2026 prices), I costed the build assuming ${parts.join(', ')}.
Result: full cost to replicate ${fmt(R.full,0)}; still to spend ${fmt(R.spend,0)} (${delta<=0?'':'+'}${fmt(delta,0)} vs the BOM total), of which ${fmt(R.dutyTot,0)} hypothetical duty and ${fmt(R.shipAmt,0)} shipping. About ${fmt(R.firstCommit,0)} is committed before the one-leg test (phases 0-2${state.tier==='bulk'||state.tier==='second'?', plus the actuator models ordered whole for the 10+ tier':''}).
Next steps: mark owned lines in my copy of bom/bom.xlsx; get a direct RobStride quote and current import costs (roadmap open item 1); price the "estimate" lines before each phase's order. Duty and exchange rates here are illustrative, not quotes.`;
}

// ---------- wiring ----------
function syncControls(){
  document.querySelectorAll('input[name=tier]').forEach(i=>i.checked=(i.value===state.tier));
  document.querySelectorAll('#owned input').forEach(i=>i.checked=i.dataset.own.split(',').every(o=>state.owned.includes(o)));
  $('duty').value=state.duty; $('o_duty').textContent=state.duty+'%';
  $('ship').value=state.ship; $('o_ship').textContent=state.ship+'%';
  $('cur').value=state.cur;
  $('v_group').classList.toggle('on',state.view==='group'); $('v_phase').classList.toggle('on',state.view==='phase');
}
function readOwned(){ const o=[]; document.querySelectorAll('#owned input:checked').forEach(c=>c.dataset.own.split(',').forEach(x=>o.push(x))); state.owned=o; }
document.querySelectorAll('input[name=tier]').forEach(i=>i.addEventListener('change',()=>{ state.tier=i.value; render(); }));
document.querySelectorAll('#owned input').forEach(i=>i.addEventListener('change',()=>{ readOwned(); render(); }));
$('duty').addEventListener('input',e=>{ state.duty=+e.target.value; $('o_duty').textContent=state.duty+'%'; render(); });
$('ship').addEventListener('input',e=>{ state.ship=+e.target.value; $('o_ship').textContent=state.ship+'%'; render(); });
$('cur').addEventListener('change',e=>{ state.cur=e.target.value; render(); renderNm(); });
$('v_group').addEventListener('click',()=>{ state.view='group'; syncControls(); render(); });
$('v_phase').addEventListener('click',()=>{ state.view='phase'; syncControls(); render(); });
['f_group','f_phase','f_kind'].forEach(id=>$(id).addEventListener('change',renderTable));
$('f_text').addEventListener('input',renderTable);
for(const g in GROUPS){ const o=document.createElement('option'); o.value=g; o.textContent=GROUPS[g]; $('f_group').appendChild(o); }
document.querySelectorAll('#bomtable th[data-k]').forEach(th=>th.addEventListener('click',()=>{ const k=th.dataset.k; if(sortK===k) sortDir=-sortDir; else { sortK=k; sortDir=(k==='item'||k==='g'||k==='basis'||k==='p')?1:-1; } renderTable(); }));
const PRESETS={
  today:{...DEFAULTS},
  staged:{...DEFAULTS, tier:'staged', view:'phase'},
  owner:{...DEFAULTS, owned:['compute','hand-kit'], tier:'staged'},
  second:{...DEFAULTS, tier:'second'},
  shock:{...DEFAULTS, duty:25, ship:5},
};
document.querySelectorAll('[data-preset]').forEach(b=>b.addEventListener('click',()=>{ Object.assign(state, JSON.parse(JSON.stringify(PRESETS[b.dataset.preset]))); syncControls(); render(); renderNm(); }));
$('copy').addEventListener('click',()=>{ navigator.clipboard.writeText($('note').textContent).then(()=>{ const b=$('copy'); b.textContent='Copied!'; setTimeout(()=>b.textContent='Copy',1200); }); });
syncControls(); render(); renderNm();
