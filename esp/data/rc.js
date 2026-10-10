const $ = id => document.getElementById(id);
const clamp = (v,a,b) => Math.max(a, Math.min(b, v));

function applyTheme(light){
  document.body.classList.toggle('light', light);
  $('themeBtn').textContent = light ? '☀️' : '🌙';
  try{ localStorage.setItem('mc_theme', light?'light':'dark'); }catch(e){}
}
(function initTheme(){
  let light=false;
  try{ light = localStorage.getItem('mc_theme')==='light'; }catch(e){}
  applyTheme(light);
  $('themeBtn').onclick = ()=>applyTheme(!document.body.classList.contains('light'));
})();

let SERVO_NEUTRAL_JS = [72,72,72,81];

const RC = {
  roll: 0,
  pitch: 0,
  yaw: 0,
  thr: 0.0,
  armed: false,
  mode: 0
};

class Joystick {
  constructor(stickEl, knobEl, opts){
    this.stick = stickEl;
    this.knob  = knobEl;
    this.selfCenterX = opts.selfCenterX !== false;
    this.selfCenterY = opts.selfCenterY !== false;
    this.onMove = opts.onMove;
    this.x = 0; this.y = 0;
    this.active = false;
    this.touchId = null;
    this._bind();
  }
  _rect(){ return this.stick.getBoundingClientRect(); }
  _bind(){
    this.stick.addEventListener('pointerdown', e=>this._start(e), {passive:false});
    window.addEventListener('pointermove', e=>this._move(e), {passive:false});
    window.addEventListener('pointerup',   e=>this._end(e));
    window.addEventListener('pointercancel',e=>this._end(e));
  }
  _start(e){
    e.preventDefault();
    this.active = true; this.pid = e.pointerId;
    try{ this.stick.setPointerCapture(e.pointerId); }catch(err){}
    this._update(e.clientX, e.clientY);
  }
  _move(e){
    if(!this.active || e.pointerId!==this.pid) return;
    e.preventDefault();
    this._update(e.clientX, e.clientY);
  }
  _end(e){
    if(e.pointerId!==this.pid) return;
    this.active = false;
    if(this.selfCenterX) this.x = 0;
    if(this.selfCenterY) this.y = 0;
    this._render();
    if(this.onMove) this.onMove(this.x, this.y);
  }
  _update(cx, cy){
    const r = this._rect();
    const dx = (cx - (r.left + r.width/2)) / (r.width/2);
    const dy = (cy - (r.top  + r.height/2)) / (r.height/2);
    const mag = Math.hypot(dx, dy);
    const max = 0.82;
    let nx=dx, ny=dy;
    if(mag>max){ nx = dx/mag*max; ny = dy/mag*max; }
    this.x = clamp(nx/max, -1, 1);
    this.y = clamp(ny/max, -1, 1);
    this._render();
    if(this.onMove) this.onMove(this.x, this.y);
  }
  setY(val){ this.y = clamp(val,-1,1); this._render(); }
  _render(){
    const r = this._rect();
    const px = this.x * (r.width/2) * 0.82;
    const py = this.y * (r.height/2) * 0.82;
    this.knob.style.transform = `translate(calc(-50% + ${px}px), calc(-50% + ${py}px))`;
  }
}

const YAW_DEADZONE = 0.14;
const stickL = new Joystick($('stickL'), $('knobL'), {
  selfCenterX: true,
  selfCenterY: false,
  onMove: (x,y)=>{
    if(Math.abs(x) < YAW_DEADZONE){
      RC.yaw = 0;
    } else {
      const sign = x>0?1:-1;
      RC.yaw = sign * (Math.abs(x)-YAW_DEADZONE) / (1-YAW_DEADZONE);
    }
    RC.thr = clamp((-y+1)/2, 0, 1);
    updateLeftReadout();
  }
});
stickL.setY(1);

const stickR = new Joystick($('stickR'), $('knobR'), {
  selfCenterX: true,
  selfCenterY: true,
  onMove: (x,y)=>{
    RC.roll  = x;
    RC.pitch = -y;
    updateRightReadout();
  }
});

function updateLeftReadout(){
  $('vThr').textContent  = Math.round(RC.thr*100)+'%';
  $('vYaw').textContent  = (RC.yaw>=0?'+':'')+RC.yaw.toFixed(2);
  $('thrBig').textContent= Math.round(RC.thr*100);
}
function updateRightReadout(){
  $('vRoll').textContent  = (RC.roll>=0?'+':'')+RC.roll.toFixed(2);
  $('vPitch').textContent = (RC.pitch>=0?'+':'')+RC.pitch.toFixed(2);
}

/* ============================================================
   WEBSOCKET
   ============================================================ */
let ws=null, wsConnected=false;
const WS_URL = (location.protocol==="file:")
  ? "ws://192.168.4.1/ws"
  : `ws://${location.hostname}/ws`;

function connect(){
  try{ ws=new WebSocket(WS_URL); }catch(e){ scheduleReconnect(); return; }
  ws.onopen   = ()=>{ wsConnected=true;
    setPill('pillLink','on',$('linkTxt'),'LINK OK');
    $('liveDot').style.background='var(--green)'; };
  ws.onclose  = ()=>{ wsConnected=false;
    setPill('pillLink','err',$('linkTxt'),'NO LINK');
    $('liveDot').style.background='var(--red)';
    scheduleReconnect(); };
  ws.onerror  = ()=>{ ws.close(); };
  ws.onmessage= ev=>{
  try{
    const d = JSON.parse(ev.data);
    if(d.cmd==='arm_rejected'){
      RC.armed = false; renderArm();
      alert('ARM ditolak firmware: throttle belum nol. Turunkan stick kiri ke bawah lalu coba lagi.');
      return;
    }
    applyTelem(d);
  }catch(e){}
};
}
function scheduleReconnect(){ setTimeout(connect, 1500); }
function send(obj){ if(wsConnected && ws.readyState===1) ws.send(JSON.stringify(obj)); }

function applyTelem(d){
  if(d.att){
    $('tRoll').textContent  = d.att.roll.toFixed(0)+'°';
    $('tPitch').textContent = d.att.pitch.toFixed(0)+'°';
    $('tYaw').textContent   = d.att.yaw.toFixed(0)+'°';
  }
  if(d.alt) $('tAlt').textContent = d.alt.z.toFixed(1)+'m';
  if(d.gps){
    const fixCls = d.gps.fix ? 'on' : 'err';
    setPill('pillGps',fixCls,$('gpsTxt'),d.gps.fix?('GPS '+d.gps.sat):'GPS --');
  }
  if(d.nav){
    $('tWp').textContent = d.nav.active
      ? `${d.nav.idx+1}/${d.nav.count}·${d.nav.dist.toFixed(0)}m`
      : '--';
  }
  if(d.sys){
    $('tHz').textContent = d.sys.hz+'Hz';
    RC.armed = d.sys.armed;
    renderArm();
    const batCls = d.sys.vbat<10.2?'err':(d.sys.vbat<10.8?'warn':'on');
    setPill('pillBat',batCls,$('batTxt'),d.sys.vbat.toFixed(1)+'V');
    updateEscCalUi(d.sys.escCal);
    updateMotorTestUi(d.sys.motorTest);
    if(d.sys.escUs!==undefined) $('escLiveUs').textContent = d.sys.escUs+' µs';
    if(d.sys.escOk!==undefined){
      $('escLiveOk').textContent = d.sys.escOk ? 'OK' : 'GAGAL';
      $('escLiveOk').style.color = d.sys.escOk ? 'var(--green)' : 'var(--red)';
    }
    if(fsTestActive && d.sys.rcLost) fsObservedRcLost = true;
    if(failsafeOpen){
      $('fsLiveLink').textContent = d.sys.rcLost ? 'PUTUS (rcLost)' : 'OK';
      $('fsLiveLink').style.color = d.sys.rcLost ? 'var(--red)' : 'var(--green)';
    }
  }
  if(d.ctrl){
    $('tThr').textContent = Math.round(d.ctrl.thr*100)+'%';
    if(fsTestActive && d.ctrl.thr<=0.01) fsObservedThrZero = true;
  }
  if(d.trim) updateTrimUi(d.trim);
  if(d.led){
    $('physLedR').classList.toggle('on', !!d.led.r);
    $('physLedG').classList.toggle('on', !!d.led.g);
    $('physLedB').classList.toggle('on', !!d.led.b);
  }
}
function setPill(pid,cls,txtEl,txt){ $(pid).className='pill '+cls; txtEl.textContent=txt; }

/* ============================================================
   KIRIM PERINTAH RC SECARA KONTINU (50 Hz)
   ============================================================ */
let rcSendPaused = false;
setInterval(()=>{
  if(rcSendPaused) return;
  send({ cmd:"rc", roll:+RC.roll.toFixed(3), pitch:+RC.pitch.toFixed(3),
         yaw:+RC.yaw.toFixed(3), thr:+RC.thr.toFixed(3) });
}, 20);

/* ============================================================
   ARM / DISARM + MODE
   ============================================================ */
let armBtnBusy = false;
function resetThrottleStick(){
  RC.thr = 0;
  stickL.setY(1);
  updateLeftReadout();
}
$('armBtn').addEventListener('click', ()=>{
  if(armBtnBusy) return;

  if(typeof escCalOpen!=='undefined' && escCalOpen){
    alert('Tutup kalibrasi ESC terlebih dahulu sebelum ARM.'); return;
  }
  if(typeof servoTestOpen!=='undefined' && servoTestOpen){
    alert('Tutup test servo manual terlebih dahulu sebelum ARM.'); return;
  }

  const wantArm = !RC.armed;
  if(wantArm && RC.thr > 0.03){
    alert('Turunkan throttle ke posisi 0 (paling bawah) dulu sebelum ARM.\n\nIni pengaman standar: mencegah motor langsung menyala mengejar nilai throttle lama begitu di-ARM.');
    return;
  }

  armBtnBusy = true;
  setTimeout(()=>{ armBtnBusy=false; }, 400);
  RC.armed = wantArm;
  send({ cmd:"arm", armed:RC.armed });
  resetThrottleStick();
  renderArm();
});
function renderArm(){
  const b=$('armBtn');
  if(RC.armed){
    b.classList.add('armed'); b.textContent='DISARM';
    setPill('pillArm','err',$('armTxt'),'ARMED');
    $('warnBanner').classList.add('show');
  } else {
    b.classList.remove('armed'); b.textContent='ARM';
    setPill('pillArm','on',$('armTxt'),'SAFE');
    $('warnBanner').classList.remove('show');
  }
}
document.querySelectorAll('.mode-btn').forEach(btn=>{
  btn.onclick = ()=>{
    document.querySelectorAll('.mode-btn').forEach(x=>x.classList.remove('active'));
    btn.classList.add('active');
    RC.mode = +btn.dataset.mode;
    send({ cmd:"mode", mode:RC.mode });
  };
});

/* ============================================================
   INIT
   ============================================================ */
updateLeftReadout();
updateRightReadout();
renderArm();
connect();

document.addEventListener('touchmove', e=>{
  if(stickL.active || stickR.active) e.preventDefault();
}, {passive:false});

/* ============================================================
   KALIBRASI ESC
   ============================================================ */
let escCalOpen = false;

function openEscModal(){
  if(RC.armed){
    alert('DISARM dulu sebelum kalibrasi ESC. Kalibrasi tidak diizinkan saat armed.');
    return;
  }
  escCalOpen = true;
  $('preflightModal').classList.remove('show');
  $('escModal').classList.add('show');
  $('escMinBtn').disabled = true;
  $('escStepText').textContent =
    'Langkah 1 — Tekan "Kirim MAX", lalu nyalakan/hubungkan baterai ke ESC. '+
    'Tunggu bunyi beep konfirmasi dari ESC (menandakan batas atas terekam).';
  $('escStatusTxt').textContent = 'Status: idle';
}
function closeEscModal(){
  escCalOpen = false;
  $('escModal').classList.remove('show');
  send({ cmd:'esc_cal', stage:'cancel' });
}

$('hubEsc').onclick = openEscModal;

$('escMaxBtn').onclick = ()=>{
  send({ cmd:'esc_cal', stage:'max' });
  $('escMinBtn').disabled = false;
  $('escStepText').textContent =
    'Langkah 2 — Setelah ESC beep tanda batas atas terekam, tekan "Kirim MIN". '+
    'ESC akan beep lagi menandakan kalibrasi selesai.';
  $('escStatusTxt').textContent = 'Status: mengirim sinyal MAX (2000µs)…';
};

$('escMinBtn').onclick = ()=>{
  send({ cmd:'esc_cal', stage:'min' });
  $('escStepText').textContent =
    'Kalibrasi selesai. ESC sudah mengenali rentang throttle 1000–2000µs. '+
    'Tutup jendela ini lalu lanjutkan pre-flight check seperti biasa.';
  $('escStatusTxt').textContent = 'Status: mengirim sinyal MIN (1000µs) — selesai';
};

$('escCancelBtn').onclick = closeEscModal;

function updateEscCalUi(stage){
  if(!escCalOpen) return;
  if(stage===1) $('escStatusTxt').textContent = 'Status: mengirim sinyal MAX (2000µs)…';
  else if(stage===2) $('escStatusTxt').textContent = 'Status: mengirim sinyal MIN (1000µs) — selesai';
}

/* ============================================================
   TEST SERVO MANUAL (posisi fin satu per satu)
   ============================================================ */
let servoTestOpen   = false;
let servoTestRunning= false;
let servoActiveIdx  = 0;

const servoSelectBtns = document.querySelectorAll('#servoSelectRow button');
const servoAngleSlider= $('servoAngleSlider');
const servoAngleVal   = $('servoAngleVal');
const servoActiveLabel= $('servoActiveLabel');

function openServoModal(){
  if(RC.armed){
    alert('DISARM dulu sebelum test servo manual. Tidak diizinkan saat armed.');
    return;
  }
  servoTestOpen = true;
  $('preflightModal').classList.remove('show');
  $('servoModal').classList.add('show');
  $('servoStatusTxt').textContent = 'Status: idle (tekan "Mulai Mode Test")';
}
function closeServoModal(){
  if(servoTestRunning) stopServoTest();
  servoTestOpen = false;
  $('servoModal').classList.remove('show');
}

function setServoControlsEnabled(en){
  servoSelectBtns.forEach(b=>b.disabled=!en);
  servoAngleSlider.disabled = !en;
  $('servoQuick0').disabled = !en;
  $('servoQuick90').disabled = !en;
  $('servoQuick180').disabled = !en;
  $('servoStopBtn').disabled = !en;
  $('servoStartBtn').disabled = en;
}

function selectServo(idx){
  servoActiveIdx = idx;
  servoSelectBtns.forEach(b=>b.classList.toggle('active', +b.dataset.idx===idx));
  servoActiveLabel.textContent = 'S'+(idx+1);

  const neut = SERVO_NEUTRAL_JS[idx];
  servoAngleSlider.value = neut;
  servoAngleVal.textContent = neut.toFixed(0)+'°';
  sendServoAngle(neut);
}

function sendServoAngle(angle){
  send({ cmd:'servo_test', stage:'set', idx:servoActiveIdx, angle:+angle });
}

$('hubServo').onclick = openServoModal;

$('servoStartBtn').onclick = ()=>{
  send({ cmd:'servo_test', stage:'start' });
  servoTestRunning = true;
  setServoControlsEnabled(true);
  $('servoStatusTxt').textContent = 'Status: aktif — S1 dipilih, semua servo di netral kalibrasinya masing-masing';
  selectServo(0);
};

function stopServoTest(){
  send({ cmd:'servo_test', stage:'stop' });
  servoTestRunning = false;
  setServoControlsEnabled(false);
  $('servoStatusTxt').textContent = 'Status: idle (tekan "Mulai Mode Test")';
}

$('servoStopBtn').onclick = closeServoModal;

servoSelectBtns.forEach(btn=>{
  btn.onclick = ()=> selectServo(+btn.dataset.idx);
});

servoAngleSlider.addEventListener('input', ()=>{
  const v = +servoAngleSlider.value;
  servoAngleVal.textContent = v+'°';
  sendServoAngle(v);
});

$('servoQuick0').onclick   = ()=>{ servoAngleSlider.value=0;   servoAngleVal.textContent='0°';   sendServoAngle(0); };
$('servoQuick90').onclick  = ()=>{
  const neut = SERVO_NEUTRAL_JS[servoActiveIdx];
  servoAngleSlider.value=neut; servoAngleVal.textContent=neut.toFixed(0)+'°'; sendServoAngle(neut);
};
$('servoQuick180').onclick = ()=>{ servoAngleSlider.value=180; servoAngleVal.textContent='180°'; sendServoAngle(180); };

/* ============================================================
   TRIM & KALIBRASI — penyetelan live tanpa re-flash
   ============================================================ */
let trimOpen = false;

function openTrimModal(){
  if(RC.armed){
    alert('DISARM dulu sebelum menyetel trim. Firmware menolak perintah "trim" saat armed (keamanan).');
    return;
  }
  trimOpen = true;
  $('preflightModal').classList.remove('show');
  $('trimModal').classList.add('show');
  $('trimStatusTxt').textContent = 'Status: idle';
}
function closeTrimModal(){
  trimOpen = false;
  $('trimModal').classList.remove('show');
}
$('hubTrim').onclick = openTrimModal;
$('trimCloseBtn').onclick = closeTrimModal;

$('yawFfSlider').addEventListener('input', ()=>{
  $('yawFfVal').textContent = (+$('yawFfSlider').value).toFixed(3);
});
$('yawFfFlip').onclick = ()=>{
  const v = -(+$('yawFfSlider').value);
  $('yawFfSlider').value = v;
  $('yawFfVal').textContent = v.toFixed(3);
};
$('yawFfSend').onclick = ()=>{
  if(RC.armed){ alert('DISARM dulu.'); return; }
  const v = +$('yawFfSlider').value;
  send({ cmd:'trim', yaw_ff:v });
  $('trimStatusTxt').textContent = `Status: yaw_ff dikirim = ${v.toFixed(3)} rad — uji hover singkat, amati arah rotasi.`;
};

$('neutSend').onclick = ()=>{
  if(RC.armed){ alert('DISARM dulu.'); return; }
  const s1=+$('neutS1').value, s2=+$('neutS2').value, s3=+$('neutS3').value, s4=+$('neutS4').value;
  send({ cmd:'trim', s1, s2, s3, s4 });
  SERVO_NEUTRAL_JS = [s1,s2,s3,s4];
  $('trimStatusTxt').textContent = `Status: netral servo dikirim (${s1}, ${s2}, ${s3}, ${s4})°`;
};

$('attSend').onclick = ()=>{
  if(RC.armed){ alert('DISARM dulu.'); return; }
  const rt=+$('attRollTrim').value, pt=+$('attPitchTrim').value;
  send({ cmd:'trim', roll_trim:rt, pitch_trim:pt });
  $('trimStatusTxt').textContent = `Status: trim attitude dikirim (roll ${rt}°, pitch ${pt}°) — cek Flight Monitor, model 3D harus ikut tegak.`;
};

function updateTrimUi(tr){
  if(!tr) return;
  if(tr.yawFF!==undefined) $('curYawFf').textContent = tr.yawFF.toFixed(3);
  if([tr.s1,tr.s2,tr.s3,tr.s4].every(v=>v!==undefined)){
    $('curNeut').textContent = `${tr.s1.toFixed(1)}, ${tr.s2.toFixed(1)}, ${tr.s3.toFixed(1)}, ${tr.s4.toFixed(1)}`;
    SERVO_NEUTRAL_JS = [tr.s1,tr.s2,tr.s3,tr.s4];
  }
  if(tr.rollTrim!==undefined && tr.pitchTrim!==undefined){
    $('curAtt').textContent = `${tr.rollTrim.toFixed(1)}°, ${tr.pitchTrim.toFixed(1)}°`;
  }
}

function openPreflightHub(){ $('preflightModal').classList.add('show'); }
function closePreflightHub(){ $('preflightModal').classList.remove('show'); }
$('preflightOpenBtn').onclick = openPreflightHub;
$('preflightCloseBtn').onclick = closePreflightHub;

document.querySelectorAll('[data-back]').forEach(btn=>{
  btn.onclick = ()=>{
    const modal = btn.closest('.modal-overlay');
    if(!modal) return;
    switch(modal.id){
      case 'escModal':       closeEscModal();       break;
      case 'servoModal':     closeServoModal();      break;
      case 'trimModal':      closeTrimModal();       break;
      case 'motorTestModal': closeMotorTestModal();  break;
      case 'failsafeModal':
        if(fsTestActive){ rcSendPaused = false; fsTestActive = false; }
        failsafeOpen = false;
        $('failsafeModal').classList.remove('show');
        break;
    }
    openPreflightHub();
  };
});

$('hubReset').onclick = ()=>{
  send({ cmd:'esc_cal',    stage:'cancel' });
  send({ cmd:'servo_test', stage:'stop' });
  send({ cmd:'motor_test', stage:'stop' });
  if(escCalOpen)    closeEscModal();
  if(servoTestOpen) closeServoModal();
  if(typeof motorTestOpen!=='undefined' && motorTestOpen) closeMotorTestModal();
  if(typeof fsTestActive!=='undefined' && fsTestActive){ rcSendPaused=false; fsTestActive=false; }
  alert('Mode kalibrasi ESC, test servo, motor test & uji failsafe telah direset paksa ke kondisi aman.');
};

setInterval(()=>{
  if(escCalOpen)    send({ cmd:'esc_cal',    stage:'ping' });
  if(servoTestOpen) send({ cmd:'servo_test', stage:'ping' });
  if(motorTestOpen && motorTestRunning) send({ cmd:'motor_test', stage:'ping' });
}, 4000);

/* ============================================================
   ESC / MOTOR TEST (TANPA ARM)
   ============================================================ */
let motorTestOpen    = false;
let motorTestRunning = false;

function openMotorTestModal(){
  if(RC.armed){
    alert('Wahana sedang ARMED. DISARM dulu sebelum pakai fitur ini (fitur ini memang dirancang untuk uji tanpa ARM).');
    return;
  }
  motorTestOpen = true;
  $('preflightModal').classList.remove('show');
  $('motorTestModal').classList.add('show');
  $('motorTestStatusTxt').textContent = 'Status: idle';
}
function closeMotorTestModal(){
  if(motorTestRunning) stopMotorTest();
  motorTestOpen = false;
  $('motorTestModal').classList.remove('show');
}
function stopMotorTest(){
  send({ cmd:'motor_test', stage:'stop' });
  motorTestRunning = false;
  $('motorTestStartBtn').disabled = false;
  $('motorTestStopBtn').disabled = true;
  $('motorTestStatusTxt').textContent = 'Status: motor dihentikan';
}
function updateMotorTestUi(active){
  if(!motorTestOpen) return;
  if(active===0 && motorTestRunning){
    motorTestRunning = false;
    $('motorTestStartBtn').disabled = false;
    $('motorTestStopBtn').disabled = true;
    $('motorTestStatusTxt').textContent = 'Status: motor berhenti (timeout/dihentikan sistem)';
  }
}

$('hubMotor').onclick = openMotorTestModal;

$('motorTestSlider').addEventListener('input', ()=>{
  const p = +$('motorTestSlider').value;
  $('motorTestVal').textContent = p+'%';
  $('motorTestStatusTxt').textContent =
    `Target sinyal: ${1000+Math.round(p*10)} µs (mulai kecil, naikkan bertahap)`;
});

$('motorTestStartBtn').onclick = ()=>{
  const pct = (+$('motorTestSlider').value)/100;
  send({ cmd:'motor_test', stage:'start', pct });
  motorTestRunning = true;
  $('motorTestStartBtn').disabled = true;
  $('motorTestStopBtn').disabled = false;
  $('motorTestStatusTxt').textContent = `Status: motor berputar @ ${$('motorTestSlider').value}%…`;
};
$('motorTestStopBtn').onclick = stopMotorTest;

/* ============================================================
   UJI FAILSAFE
   ============================================================ */
let failsafeOpen     = false;
let fsTestActive     = false;
let fsObservedRcLost = false;
let fsObservedThrZero= false;

$('hubFailsafe').onclick = ()=>{
  failsafeOpen = true;
  $('preflightModal').classList.remove('show');
  $('failsafeModal').classList.add('show');
  $('fsResult').className = 'fs-result';
  $('fsResult').textContent = '';
  $('fsCountdown').textContent = '–';
  $('fsStatusTxt').textContent = RC.armed
    ? 'Status: siap — armed, throttle saat ini akan diamati.'
    : 'Status: siap — CATATAN: wahana belum ARMED, hasil kurang representatif untuk uji motor cut-off.';
};

$('fsStartBtn').onclick = ()=>{
  $('fsResult').className = 'fs-result';
  $('fsResult').textContent = '';
  fsTestActive = true;
  fsObservedRcLost = false;
  fsObservedThrZero = false;
  rcSendPaused = true;
  $('fsStatusTxt').textContent = 'Status: mensimulasikan link RC terputus…';
  $('fsStartBtn').disabled = true;

  let n = 3;
  $('fsCountdown').textContent = String(n);
  const iv = setInterval(()=>{
    n--;
    $('fsCountdown').textContent = String(Math.max(n,0));
    if(n<=0){
      clearInterval(iv);
      rcSendPaused = false;
      fsTestActive = false;
      $('fsStartBtn').disabled = false;
      const pass = fsObservedRcLost && fsObservedThrZero;
      $('fsResult').classList.add(pass ? 'pass' : 'fail');
      $('fsResult').textContent = pass
        ? '✅ LULUS — Sistem mendeteksi link putus (rcLost) DAN throttle otomatis turun ke 0%.'
        : '❌ GAGAL — ' +
          (!fsObservedRcLost ? 'Sistem tidak mendeteksi status rcLost. ' : '') +
          (!fsObservedThrZero ? 'Throttle tidak turun ke 0% selama simulasi.' : '');
      $('fsStatusTxt').textContent = 'Status: uji selesai, link RC dipulihkan otomatis.';
    }
  }, 1000);
};

$('fsCloseBtn').onclick = ()=>{
  if(fsTestActive){ rcSendPaused = false; fsTestActive = false; }
  failsafeOpen = false;
  $('failsafeModal').classList.remove('show');
};
