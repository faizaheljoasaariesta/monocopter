import * as THREE from "three";

const S = {
  t:0,
  roll:0, pitch:0, yaw:0,
  gx:0, gy:0, gz:0,
  ax:0, ay:0, az:1,
  kfRoll:0, kfPitch:0, kfGain:0.19,
  alt:0, vz:0, altRef:3.0,
  thr:0.767, s1:0, s2:0, s3:0, s4:0,
  pidRoll:0, pidPitch:0, pidYaw:0, pidAlt:0,
  vbat:11.8, pct:78, hz:0, armed:false, mode:0,
  escUs:1000, escOk:null,
  gpsFix:false, gpsSat:0, gpsLat:0, gpsLng:0, gpsX:0, gpsY:0,
  navActive:false, navIdx:0, navCount:0, navDist:0,
  trimYawFF:0.108, trimS:[72,72,72,81], trimRoll:0, trimPitch:0
};

const $ = id => document.getElementById(id);
const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
const DEG=Math.PI/180;

function cssv(name){
  return getComputedStyle(document.body).getPropertyValue(name).trim();
}
function applyTheme(light){
  document.body.classList.toggle("light", light);
  const btn=$("themeBtn"); if(btn) btn.textContent = light ? "☀️" : "🌙";
  try{ localStorage.setItem("mc_theme", light?"light":"dark"); }catch(e){}
  updateThreeTheme();
}
function initTheme(){
  let light=false;
  try{ light = localStorage.getItem("mc_theme")==="light"; }catch(e){}
  applyTheme(light);
  const btn=$("themeBtn");
  if(btn) btn.onclick=()=>applyTheme(!document.body.classList.contains("light"));
}

const adi=$("adi"), adiCtx=adi.getContext("2d");
function drawADI(){
  const w=adi.width,h=adi.height,cx=w/2,cy=h/2,R=w/2-6;
  const c=adiCtx; c.clearRect(0,0,w,h);
  c.save(); c.translate(cx,cy);
  c.beginPath(); c.arc(0,0,R,0,7); c.clip();
  c.save();
  c.rotate(-S.roll*DEG);
  const pxPerDeg=R/45;
  const off=clamp(S.pitch,-45,45)*pxPerDeg;
  c.translate(0,off);
  c.fillStyle="#2487c9"; c.fillRect(-R*2,-R*3,R*4,R*3);
  c.fillStyle="#7a5a32"; c.fillRect(-R*2,0,R*4,R*3);
  c.strokeStyle="#fff"; c.lineWidth=2;
  c.beginPath(); c.moveTo(-R*2,0); c.lineTo(R*2,0); c.stroke();
  c.strokeStyle="rgba(255,255,255,.8)"; c.fillStyle="#fff";
  c.font="9px sans-serif"; c.textAlign="center"; c.lineWidth=1.5;
  for(let p=-40;p<=40;p+=10){ if(p===0)continue;
    const y=-p*pxPerDeg, len=(p%20===0)?34:20;
    c.beginPath(); c.moveTo(-len/2,y); c.lineTo(len/2,y); c.stroke();
    if(p%20===0){ c.fillText(Math.abs(p),-len/2-10,y+3);
      c.fillText(Math.abs(p),len/2+10,y+3); }
  }
  c.restore();
  c.strokeStyle="#fff"; c.lineWidth=1.5;
  for(let a=-60;a<=60;a+=15){
    c.save(); c.rotate(a*DEG);
    c.beginPath(); c.moveTo(0,-R); c.lineTo(0,-R+(a%45===0?10:6)); c.stroke();
    c.restore();
  }
  c.fillStyle="var(--amber)"; c.fillStyle="#ffb020";
  c.beginPath(); c.moveTo(0,-R+2); c.lineTo(-7,-R+13); c.lineTo(7,-R+13);
  c.closePath(); c.fill();
  c.restore();
  c.save(); c.translate(cx,cy);
  c.strokeStyle="#ffd000"; c.lineWidth=3;
  c.beginPath(); c.moveTo(-40,0); c.lineTo(-14,0);
  c.moveTo(14,0); c.lineTo(40,0);
  c.moveTo(0,-6); c.lineTo(0,6); c.stroke();
  c.fillStyle="#ffd000"; c.beginPath(); c.arc(0,0,2.5,0,7); c.fill();
  // bezel
  c.strokeStyle=cssv("--bezel"); c.lineWidth=5;
  c.beginPath(); c.arc(0,0,R+2,0,7); c.stroke();
  c.restore();
}

const pfd=$("pfd"); const pfdCtx=pfd.getContext("2d");
function sizePFD(){ const r=pfd.parentElement.getBoundingClientRect();
  pfd.width=r.width; pfd.height=r.height; }
function drawPFD(){
  const w=pfd.width,h=pfd.height; if(!w)return;
  const c=pfdCtx,cx=w/2,cy=h/2; c.clearRect(0,0,w,h);
  c.save(); c.translate(cx,cy); c.rotate(-S.roll*DEG);
  const pxPerDeg=h/70, off=clamp(S.pitch,-30,30)*pxPerDeg;
  c.translate(0,off);
  const big=Math.max(w,h)*2;
  c.fillStyle="#2a93d6"; c.fillRect(-big,-big,big*2,big);
  c.fillStyle="#9c6b32"; c.fillRect(-big,0,big*2,big);
  c.strokeStyle="#fff"; c.lineWidth=2;
  c.beginPath(); c.moveTo(-big,0); c.lineTo(big,0); c.stroke();
  c.fillStyle="#fff"; c.font="10px sans-serif"; c.textAlign="center";
  c.lineWidth=1.5; c.strokeStyle="#fff";
  for(let p=-30;p<=30;p+=10){ if(!p)continue; const y=-p*pxPerDeg;
    c.beginPath(); c.moveTo(-30,y); c.lineTo(30,y); c.stroke();
    c.fillText(Math.abs(p),-44,y+3); c.fillText(Math.abs(p),44,y+3);}
  c.restore();
  c.strokeStyle="#ffd000"; c.lineWidth=3;
  c.beginPath(); c.moveTo(cx-46,cy);c.lineTo(cx-16,cy);
  c.moveTo(cx+16,cy);c.lineTo(cx+46,cy);
  c.moveTo(cx,cy-5);c.lineTo(cx,cy+5);c.stroke();
  drawTape(c,w-46,cy,h,S.alt,"#19c3e6",1);
  drawTape(c,40,cy,h,S.vz*5,"#1fd65f",0.5,true);
}
function drawTape(c,x,cy,h,val,col,step,small){
  c.save();
  c.fillStyle=cssv("--tapebg");
  c.fillRect(x-22,cy-h/2+30,44,h-60);
  c.strokeStyle=cssv("--bezel"); c.strokeRect(x-22,cy-h/2+30,44,h-60);
  c.translate(x,cy); c.font="9px sans-serif"; c.textAlign="center";
  const ppu=18;
  for(let i=-4;i<=4;i++){ const v=Math.round(val)+i;
    const y=(val-v)*ppu; if(Math.abs(y)>h/2-32)continue;
    c.strokeStyle=cssv("--tick"); c.beginPath();
    c.moveTo(-18,y);c.lineTo(-12,y);c.stroke();
    c.fillStyle=cssv("--canvastext"); c.fillText(v.toFixed(small?1:0),2,y+3);
  }
  c.fillStyle=col; c.fillRect(-22,-9,44,18);
  c.fillStyle=cssv("--readtext"); c.font="bold 11px sans-serif";
  c.fillText(val.toFixed(small?2:2),0,4);
  c.restore();
}

function gauge(canvasId,val,min,max,unit,col){
  const cv=$(canvasId),c=cv.getContext("2d");
  const w=cv.width,h=cv.height,cx=w/2,cy=h/2,R=w/2-12;
  c.clearRect(0,0,w,h);
  const a0=Math.PI*0.75,a1=Math.PI*2.25;
  c.strokeStyle=cssv("--gaugetrack"); c.lineWidth=9; c.lineCap="round";
  c.beginPath(); c.arc(cx,cy,R,a0,a1); c.stroke();
  const f=clamp((val-min)/(max-min),0,1);
  c.strokeStyle=col; c.beginPath(); c.arc(cx,cy,R,a0,a0+(a1-a0)*f); c.stroke();
  c.strokeStyle=cssv("--tick"); c.lineWidth=1.5;
  for(let i=0;i<=10;i++){ const a=a0+(a1-a0)*i/10;
    c.beginPath();
    c.moveTo(cx+Math.cos(a)*(R-9),cy+Math.sin(a)*(R-9));
    c.lineTo(cx+Math.cos(a)*(R-4),cy+Math.sin(a)*(R-4)); c.stroke(); }
  const an=a0+(a1-a0)*f;
  c.strokeStyle=cssv("--needle"); c.lineWidth=2.5; c.lineCap="round";
  c.beginPath(); c.moveTo(cx,cy);
  c.lineTo(cx+Math.cos(an)*(R-14),cy+Math.sin(an)*(R-14)); c.stroke();
  c.fillStyle=col; c.beginPath(); c.arc(cx,cy,4,0,7); c.fill();
}

let renderer,scene,camera,rocket,finGroup=[];
function initThree(){
  const cv=$("three");
  renderer=new THREE.WebGLRenderer({canvas:cv,antialias:true,alpha:true});
  scene=new THREE.Scene();
  camera=new THREE.PerspectiveCamera(42,1,0.1,100);
  camera.position.set(3.2,1.8,3.6); camera.lookAt(0,0,0);

  scene.add(new THREE.AmbientLight(0x88aacc,0.7));
  const key=new THREE.DirectionalLight(0xffffff,1.1);
  key.position.set(4,6,3); scene.add(key);
  const rim=new THREE.DirectionalLight(0x19c3e6,0.6);
  rim.position.set(-3,1,-4); scene.add(rim);

  rocket=new THREE.Group();

  const bodyMat=new THREE.MeshStandardMaterial({color:0xff7a18,metalness:.3,roughness:.5});
  const body=new THREE.Mesh(new THREE.CylinderGeometry(.4,.4,1.6,40),bodyMat);
  rocket.add(body);
  const ctrlMat=new THREE.MeshStandardMaterial({color:0x222a33,metalness:.5,roughness:.4});
  const ctrl=new THREE.Mesh(new THREE.CylinderGeometry(.4,.4,.8,40),ctrlMat);
  ctrl.position.y=-1.2; rocket.add(ctrl);
  const noseMat=new THREE.MeshStandardMaterial({color:0x111418,metalness:.6,roughness:.3});
  const nose=new THREE.Mesh(new THREE.SphereGeometry(.4,32,16,0,6.3,0,Math.PI/2),noseMat);
  nose.position.y=.8; nose.scale.y=2; rocket.add(nose);

  const finMat=new THREE.MeshStandardMaterial({color:0x19c3e6,
    metalness:.2,roughness:.6,side:THREE.DoubleSide});
  for(let i=0;i<4;i++){
    const pivot=new THREE.Group();
    const fin=new THREE.Mesh(new THREE.BoxGeometry(.5,.5,.02),finMat);
    fin.position.set(.62,0,0);
    pivot.add(fin);
    pivot.position.y=-1.2;
    pivot.rotation.y=i*Math.PI/2;
    rocket.add(pivot); finGroup.push({pivot,fin});
  }

  const legMat=new THREE.MeshStandardMaterial({color:0x445566});
  for(let i=0;i<3;i++){
    const leg=new THREE.Mesh(new THREE.CylinderGeometry(.03,.03,.6,8),legMat);
    const a=i*2*Math.PI/3;
    leg.position.set(Math.cos(a)*.35,-1.75,Math.sin(a)*.35);
    leg.rotation.z=Math.cos(a)*.4; leg.rotation.x=-Math.sin(a)*.4;
    rocket.add(leg);
  }

  scene.add(rocket);

  makeGrid();

  resizeThree();
}
let grid=null;
function hex(name){
  const v=cssv(name); return parseInt(v.replace("#",""),16) || 0x1d2a3a;
}
function makeGrid(){
  if(grid){ scene.remove(grid); grid.geometry.dispose(); grid.material.dispose(); }
  grid=new THREE.GridHelper(10,20,hex("--grid3d1"),hex("--grid3d2"));
  grid.position.y=-2.1; scene.add(grid);
}
function updateThreeTheme(){ if(scene) makeGrid(); }
function resizeThree(){
  const cv=$("three"),r=cv.parentElement.getBoundingClientRect();
  renderer.setSize(r.width,r.height,false);
  camera.aspect=r.width/r.height; camera.updateProjectionMatrix();
}
function renderThree(){
  if(!rocket)return;
  rocket.rotation.set(S.pitch*DEG, S.yaw*DEG, -S.roll*DEG, "YXZ");
  const sv=[S.s1,S.s2,S.s3,S.s4];
  finGroup.forEach((f,i)=>{ f.fin.rotation.y=sv[i]*DEG; });
  renderer.render(scene,camera);
}

function buildTelem(){
  $("telemBody").innerHTML=`
    <div class="sec">ESTIMATION (KALMAN)</div>
    <div class="row hi"><span class="k">KF Roll</span><span class="v" id="t_kfr">0.0°</span></div>
    <div class="row hi"><span class="k">KF Pitch</span><span class="v" id="t_kfp">0.0°</span></div>
    <div class="row"><span class="k">Kalman Gain</span><span class="v" id="t_kfg">0.19</span></div>
    <div class="sec">GYRO / ACCEL</div>
    <div class="row"><span class="k">Gyro X / Y / Z</span><span class="v" id="t_gyro">0,0,0</span></div>
    <div class="row"><span class="k">Accel X / Y / Z</span><span class="v" id="t_acc">0,0,1</span></div>
    <div class="sec">ALTITUDE</div>
    <div class="row hi"><span class="k">Altitude</span><span class="v" id="t_alt">0.00 m</span></div>
    <div class="row"><span class="k">V-Speed</span><span class="v" id="t_vz">0.00 m/s</span></div>
    <div class="row"><span class="k">Alt Ref</span><span class="v" id="t_altref">3.00 m</span></div>
    <div class="sec">PID OUTPUT</div>
    <div class="row"><span class="k">Roll / Pitch</span><span class="v" id="t_pidrp">0,0</span></div>
    <div class="row"><span class="k">Yaw / Alt</span><span class="v" id="t_pidya">0,0</span></div>
    <div class="sec">SYSTEM</div>
    <div class="row"><span class="k">Battery</span><span class="v" id="t_bat">11.8 V</span></div>
    <div class="row"><span class="k">Loop Rate</span><span class="v" id="t_hz">0 Hz</span></div>
    <div class="sec">ESC / MOTOR</div>
    <div class="row hi"><span class="k">Pulsa ESC aktual</span><span class="v" id="t_escus">1000 µs</span></div>
    <div class="row"><span class="k">Attach PWM ESC</span><span class="v" id="t_escok">--</span></div>
    <div class="sec">GPS / NAVIGATION</div>
    <div class="row hi"><span class="k">GPS Status</span><span class="v" id="t_gps">NO FIX</span></div>
    <div class="row"><span class="k">Lat / Lng</span><span class="v" id="t_latlng">0.0000, 0.0000</span></div>
    <div class="row"><span class="k">Local X / Y</span><span class="v" id="t_localxy">0.0, 0.0 m</span></div>
    <div class="row hi"><span class="k">Waypoint</span><span class="v" id="t_wp">-- / --</span></div>
    <div class="row"><span class="k">WP Distance</span><span class="v" id="t_wpdist">0.0 m</span></div>
    <div class="sec">TRIM AKTIF <span style="font-weight:400;text-transform:none">(atur di halaman RC)</span></div>
    <div class="row"><span class="k">Yaw Anti-Torsi FF</span><span class="v" id="t_trimyaw">0.108 rad</span></div>
    <div class="row"><span class="k">Netral S1..S4</span><span class="v" id="t_trimneut">72, 72, 72, 81 °</span></div>
    <div class="row hi"><span class="k">Trim Roll / Pitch</span><span class="v" id="t_trimatt">0.0° / 0.0°</span></div>`;
}

function fmt(v,d=1){return (v>=0?"+":"")+v.toFixed(d);}
function updateDOM(){
  $("rRoll").textContent=S.roll.toFixed(1)+"°";
  $("rPitch").textContent=S.pitch.toFixed(1)+"°";
  $("rYaw").textContent=S.yaw.toFixed(1)+"°";
  $("hudR").textContent=S.roll.toFixed(0)+"°";
  $("hudP").textContent=S.pitch.toFixed(0)+"°";
  $("hudY").textContent=S.yaw.toFixed(0)+"°";
  const g=Math.sqrt(S.ax*S.ax+S.ay*S.ay+S.az*S.az);
  $("gforce").textContent=g.toFixed(2)+" G";
  $("pfdSpd").textContent=Math.abs(S.vz).toFixed(1);
  $("pfdAlt").textContent=S.alt.toFixed(2);
  $("vGRoll").textContent=fmt(S.gx,2);
  $("vGPitch").textContent=fmt(S.gy,2);
  $("vGYaw").textContent=fmt(S.gz,2);
  $("vGThr").textContent=Math.round(S.thr*100)+"%";
  [["sb1","sd1",S.s1],["sb2","sd2",S.s2],["sb3","sd3",S.s3],["sb4","sd4",S.s4]]
    .forEach(([bar,deg,v])=>{
      const f=clamp(v/30,-1,1); const el=$(bar);
      const half=30;
      el.style.top=(30 - f*half/30*30)+"%";
      el.style.height="2px";
      el.style.transform=`translateY(${-f*28}px)`;
      $(deg).textContent=v.toFixed(0);
    });
  setTxt("t_kfr",S.kfRoll.toFixed(1)+"°"); setTxt("t_kfp",S.kfPitch.toFixed(1)+"°");
  setTxt("t_kfg",S.kfGain.toFixed(2));
  setTxt("t_gyro",`${fmt(S.gx,2)} ${fmt(S.gy,2)} ${fmt(S.gz,2)}`);
  setTxt("t_acc",`${fmt(S.ax,2)} ${fmt(S.ay,2)} ${fmt(S.az,2)}`);
  setTxt("t_alt",S.alt.toFixed(2)+" m"); setTxt("t_vz",fmt(S.vz,2)+" m/s");
  setTxt("t_altref",S.altRef.toFixed(2)+" m");
  setTxt("t_pidrp",`${fmt(S.pidRoll,2)} ${fmt(S.pidPitch,2)}`);
  setTxt("t_pidya",`${fmt(S.pidYaw,2)} ${fmt(S.pidAlt,2)}`);
  setTxt("t_bat",S.vbat.toFixed(1)+" V");
  setTxt("t_hz",S.hz+" Hz");
  setTxt("t_gps",S.gpsFix?`FIX · ${S.gpsSat} sat`:"NO FIX");
  setTxt("t_latlng",S.gpsLat.toFixed(5)+", "+S.gpsLng.toFixed(5));
  setTxt("t_localxy",S.gpsX.toFixed(1)+", "+S.gpsY.toFixed(1)+" m");
  setTxt("t_wp",S.navCount>0?`${S.navActive?(S.navIdx+1):"–"} / ${S.navCount}`:"-- / --");
  setTxt("t_wpdist",S.navDist.toFixed(1)+" m");
  setTxt("t_trimyaw",S.trimYawFF.toFixed(3)+" rad");
  setTxt("t_trimneut",S.trimS.map(v=>v.toFixed(1)).join(", ")+" °");
  setTxt("t_trimatt",fmt(S.trimRoll,1)+"° / "+fmt(S.trimPitch,1)+"°");
  const hint=$("modelTrimHint");
  if(hint){
    hint.style.display=(S.trimRoll!==0||S.trimPitch!==0)?"none":"block";
  }
  $("tStamp").textContent="t="+S.t;
  setPill("pillArm",S.armed?"on":"err",$("armTxt"),S.armed?"ARMED":"DISARMED");
  setPill("pillMode","on",$("modeTxt"),["AUTO","SEMI","MANUAL"][S.mode]);
  const batCls=S.vbat<10.2?"err":(S.vbat<10.8?"warn":"on");
  setPill("pillBat",batCls,$("batTxt"),S.vbat.toFixed(1)+"V "+S.pct+"%");
  setPill("pillHz",S.hz>150?"on":"warn",$("hzTxt"),S.hz+" Hz");
  setTxt("t_escus",S.escUs+" µs");
  setTxt("t_escok",S.escOk===null?"--":(S.escOk?"OK":"GAGAL"));
  if(S.escOk===false) setPill("pillEsc","err",$("escTxt"),"ESC PWM GAGAL");
  else setPill("pillEsc",S.escUs>1020?"warn":"on",$("escTxt"),"ESC "+S.escUs+"µs");
}
function setTxt(id,v){const e=$(id);if(e)e.textContent=v;}
function setPill(pid,cls,txtEl,txt){
  const p=$(pid); p.className="pill "+cls; txtEl.textContent=txt;
}

function loop(){
  drawADI(); drawPFD();
  gauge("gRoll",S.gx,-3,3,"","#19c3e6");
  gauge("gPitch",S.gy,-3,3,"","#1fd65f");
  gauge("gYaw",S.gz,-3,3,"","#e64bd0");
  gauge("gThr",S.thr,0,1,"","#ffb020");
  renderThree();
  updateDOM();
  requestAnimationFrame(loop);
}

let ws=null, wsConnected=false;
const WS_URL = (location.protocol==="file:")
  ? "ws://192.168.4.1/ws"
  // : `ws://${location.hostname}/ws`;
  : `ws://192.168.4.1/ws`;

function connect(){
  try{ ws=new WebSocket(WS_URL); }
  catch(e){ scheduleReconnect(); return; }
  ws.onopen=()=>{ wsConnected=true;
    setPill("pillLink","on",$("linkTxt"),"CONNECTED");
    $("liveDot").style.background="var(--green)"; };
  ws.onclose=()=>{ wsConnected=false;
    setPill("pillLink","err",$("linkTxt"),"DISCONNECTED (mode DEMO)");
    scheduleReconnect(); };
  ws.onerror=()=>{ ws.close(); };
  ws.onmessage=ev=>{
    try{
      const d=JSON.parse(ev.data);
      if(d.cmd==="arm_rejected"){
        S.armed=false; $("btnDisarm").classList.add("active"); $("btnArm").classList.remove("active");
        alert("ARM ditolak firmware: throttle belum nol. Turunkan slider THROTTLE ke 0% lalu coba lagi.");
        return;
      }
      applyTelem(d);
    }catch(e){}
  };
}
function scheduleReconnect(){ setTimeout(connect,2000); }
function applyTelem(d){
  if(d.att){S.roll=d.att.roll;S.pitch=d.att.pitch;S.yaw=d.att.yaw;}
  if(d.rate){S.gx=d.rate.gx;S.gy=d.rate.gy;S.gz=d.rate.gz;}
  if(d.acc){S.ax=d.acc.ax;S.ay=d.acc.ay;S.az=d.acc.az;}
  if(d.kf){S.kfRoll=d.kf.roll;S.kfPitch=d.kf.pitch;S.kfGain=d.kf.gain;}
  if(d.alt){S.alt=d.alt.z;S.vz=d.alt.vz;S.altRef=d.alt.ref;}
  if(d.ctrl){S.thr=d.ctrl.thr;S.s1=d.ctrl.s1;S.s2=d.ctrl.s2;S.s3=d.ctrl.s3;S.s4=d.ctrl.s4;}
  if(d.pid){S.pidRoll=d.pid.roll;S.pidPitch=d.pid.pitch;S.pidYaw=d.pid.yaw;S.pidAlt=d.pid.alt;}
  if(d.sys){S.vbat=d.sys.vbat;S.pct=d.sys.pct;S.hz=d.sys.hz;S.armed=d.sys.armed;S.mode=d.sys.mode;
    if(d.sys.escUs!==undefined)S.escUs=d.sys.escUs;
    if(d.sys.escOk!==undefined)S.escOk=d.sys.escOk;}
  if(d.gps){S.gpsFix=d.gps.fix;S.gpsSat=d.gps.sat;
    S.gpsLat=d.gps.lat;S.gpsLng=d.gps.lng;S.gpsX=d.gps.x;S.gpsY=d.gps.y;}
  if(d.nav){S.navActive=d.nav.active;S.navIdx=d.nav.idx;
    S.navCount=d.nav.count;S.navDist=d.nav.dist;}
  if(d.trim){
    if(d.trim.yawFF!==undefined) S.trimYawFF=d.trim.yawFF;
    if([d.trim.s1,d.trim.s2,d.trim.s3,d.trim.s4].every(v=>v!==undefined))
      S.trimS=[d.trim.s1,d.trim.s2,d.trim.s3,d.trim.s4];
    if(d.trim.rollTrim!==undefined)  S.trimRoll =d.trim.rollTrim;
    if(d.trim.pitchTrim!==undefined) S.trimPitch=d.trim.pitchTrim;
  }
  if(d.t!==undefined)S.t=d.t;
}
function send(obj){ if(wsConnected&&ws.readyState===1) ws.send(JSON.stringify(obj)); }
let dashCtl=false;
setInterval(()=>{ if(dashCtl && wsConnected && S.armed) send({cmd:"rc"}); },250);

function bindControls(){
  function resetThrSlider(){
    const th=$("thrSet"); if(th){ th.value=0; $("thrSetVal").textContent="0%"; }
    S.thr=0; send({cmd:"rc",thr:0});
  }
  $("btnArm").onclick=()=>{
    const th=$("thrSet");
    if(th && +th.value > 3){
      alert("Turunkan slider THROTTLE ke 0% dulu sebelum ARM.\n\nPengaman ini mencegah motor langsung menyala mengejar nilai throttle yang tersisa dari sesi sebelumnya.");
      return;
    }
    dashCtl=true; send({cmd:"arm",armed:true});
    $("btnArm").classList.add("active");$("btnDisarm").classList.remove("active");
    if(!wsConnected){S.armed=true;} };
  $("btnDisarm").onclick=()=>{ dashCtl=false; send({cmd:"arm",armed:false});
    resetThrSlider();
    $("btnDisarm").classList.add("active");$("btnArm").classList.remove("active");
    if(!wsConnected){S.armed=false;} };
  document.querySelectorAll(".mode-btn").forEach(b=>{
    b.onclick=()=>{ const m=+b.dataset.mode;
      document.querySelectorAll(".mode-btn").forEach(x=>x.classList.remove("active"));
      b.classList.add("active"); send({cmd:"mode",mode:m});
      if(!wsConnected)S.mode=m; };
  });
  const ar=$("altRef");
  ar.oninput=()=>{ $("altRefVal").textContent=(+ar.value).toFixed(1)+" m";
    send({cmd:"alt_ref",value:+ar.value}); if(!wsConnected)S.altRef=+ar.value; };
  const th=$("thrSet");
  th.oninput=()=>{ dashCtl=true; $("thrSetVal").textContent=th.value+"%";
    send({cmd:"rc",thr:+th.value/100}); if(!wsConnected)S.thr=+th.value/100; };

  document.querySelectorAll(".pid-row").forEach(row=>{
    const loop=row.dataset.loop;
    const btn=row.querySelector(".pset");
    btn.onclick=()=>{
      const kp=+row.querySelector(".kp").value;
      const ki=+row.querySelector(".ki").value;
      const kd=+row.querySelector(".kd").value;
      send({cmd:"pid",loop:loop,kp:kp,ki:ki,kd:kd});
      btn.classList.add("ok"); btn.textContent="✓";
      const note=$("pidNote");
      if(note) note.textContent=`${loop}: Kp=${kp} Ki=${ki} Kd=${kd} terkirim`;
      setTimeout(()=>{ btn.classList.remove("ok"); btn.textContent="SET"; },1000);
    };
  });

  let loadedWp=[];
  $("btnWpLoad").onclick=()=>{
    const txt=$("wpInput").value.trim();
    const lines=txt.split("\n").map(l=>l.trim()).filter(l=>l.length>0);
    loadedWp=[];
    for(const ln of lines){
      const p=ln.split(",").map(s=>parseFloat(s.trim()));
      if(p.length>=2 && !isNaN(p[0]) && !isNaN(p[1])){
        loadedWp.push([p[0], p[1], isNaN(p[2])?3.0:p[2]]);
      }
    }
    if(loadedWp.length===0){
      $("wpStatus").textContent="⚠ Format salah — gunakan x,y,z per baris";
      return;
    }
    send({cmd:"wp_load", wp:loadedWp});
    $("wpStatus").textContent=`✓ ${loadedWp.length} waypoint dimuat (belum mulai)`;
    if(!wsConnected){ S.navCount=loadedWp.length; S.navIdx=0; S.navActive=false; }
  };
  $("btnWpStart").onclick=()=>{
    if(loadedWp.length===0){ $("wpStatus").textContent="⚠ Muat waypoint dulu"; return; }
    send({cmd:"wp_start"});
    $("wpStatus").textContent=`▶ Navigasi berjalan (${loadedWp.length} WP)`;
    if(!wsConnected){ S.navActive=true; S.navIdx=0; }
  };
  $("btnWpStop").onclick=()=>{
    send({cmd:"wp_stop"});
    $("wpStatus").textContent="■ Navigasi dihentikan";
    if(!wsConnected){ S.navActive=false; }
  };
}

let demo=true, dt=0;
function simulate(){
  if(wsConnected){demo=false;return;}
  dt+=0.03;
  S.t=Math.floor(performance.now());
  S.roll=12*Math.sin(dt*0.7);
  S.pitch=8*Math.sin(dt*0.5+1);
  S.yaw=(S.yaw+0.4)%360;
  S.gx=0.7*Math.cos(dt*0.7); S.gy=0.4*Math.cos(dt*0.5+1); S.gz=0.2*Math.sin(dt*0.3);
  S.ax=Math.sin(S.pitch*DEG); S.ay=-Math.sin(S.roll*DEG);
  S.az=Math.cos(S.roll*DEG)*Math.cos(S.pitch*DEG);
  S.kfRoll=S.roll+0.3*Math.sin(dt*4); S.kfPitch=S.pitch+0.3*Math.cos(dt*4);
  S.alt=3.0+0.4*Math.sin(dt*0.4); S.vz=0.16*Math.cos(dt*0.4);
  S.thr=0.767+0.04*Math.sin(dt*0.4);
  S.s1=10*Math.sin(dt*0.7); S.s3=-S.s1;
  S.s2=8*Math.sin(dt*0.5+1); S.s4=-S.s2;
  S.pidRoll=0.08*Math.sin(dt*0.7); S.pidPitch=0.05*Math.sin(dt*0.5);
  S.pidYaw=0.02; S.pidAlt=0.01*Math.cos(dt*0.4);
  S.vbat=11.8-0.0002*dt*10; S.pct=Math.max(0,Math.round((S.vbat-9.9)/(12.6-9.9)*100));
  S.hz=197+Math.round(Math.sin(dt)*2);
  S.kfGain=0.19;
  S.gpsFix=true; S.gpsSat=9+Math.round(Math.sin(dt*0.2)+1);
  S.gpsLat=-7.79560+0.00002*Math.sin(dt*0.3);
  S.gpsLng=110.36950+0.00002*Math.cos(dt*0.3);
  S.gpsX=2.2*Math.sin(dt*0.3); S.gpsY=2.2*Math.cos(dt*0.3);
  if(S.navActive && S.navCount>0){
    S.navDist=Math.max(0.3, 4*Math.abs(Math.cos(dt*0.5)));
    if(S.navDist<0.6 && S.navIdx<S.navCount-1) S.navIdx++;
  }
}
setInterval(simulate,30);

window.addEventListener("resize",()=>{ sizePFD(); resizeThree(); });
buildTelem();
sizePFD();
initThree();
initTheme();
bindControls();
connect();
loop();
