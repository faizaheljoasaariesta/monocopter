/* ============================================================
 *  Monocopter Flight Controller — ESP32 (Firmware Lengkap)
 *  Versi: parameter final + GPS NEO-M8N + navigasi waypoint
 *
 *  Melayani DUA antarmuka web dari LittleFS:
 *    http://192.168.4.1/         -> Flight Monitor (monitoring)
 *    http://192.168.4.1/rc       -> Remote Control (joystick)
 *  Keduanya berbagi WebSocket  ws://192.168.4.1/ws
 *
 *  Library (Arduino Library Manager):
 *    - ESPAsyncWebServer, AsyncTCP, ArduinoJson (v6), ESP32Servo
 *  Board: ESP32 Dev Module
 *
 *  Upload file web ke LittleFS via plugin "arduino-littlefs-upload"
 *    data/index.html              (monitor)
 *    data/app.js
 *    data/rc.html                 (remote control)
 *    data/rc.js
 *    data/three.module.min.js     (untuk 3D — unduh dulu, lihat PANDUAN_UPLOAD_LITTLEFS.md)
 *  Partition Scheme: "Default 4MB with spiffs (1.2MB APP/1.5MB SPIFFS)" (partisi sama, dipakai LittleFS)
 *
 *  Sensor:
 *    - GY-87  (MPU-6050 + HMC5883L + BMP180) via I2C  GPIO21/22
 *    - GPS u-blox NEO-M8N                    via UART2 GPIO16/17
 * ============================================================ */

#include <WiFi.h>
#include <Wire.h>
#include <LittleFS.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <math.h>

// ───────── KONFIGURASI WIFI (Access Point) ─────────
const char* AP_SSID = "Monochopter_FC";
const char* AP_PASS = "monochopter123";

// ───────── KONFIGURASI PIN (sesuai skematik final) ─────────
#define PIN_SERVO1   13   // sirip 0°   (roll +)
#define PIN_SERVO2   14   // sirip 90°  (pitch +)
#define PIN_SERVO3   27   // sirip 180° (roll -)
#define PIN_SERVO4   26   // sirip 270° (pitch -)
#define PIN_ESC      18   // motor BLDC via ESC
#define PIN_SDA      21   // GY-87 I2C data
#define PIN_SCL      22   // GY-87 I2C clock
#define PIN_GPS_RX   16   // GPS TX -> ESP32 RX (UART2)
#define PIN_GPS_TX   17   // GPS RX <- ESP32 TX (UART2)
#define PIN_VBAT     34   // ADC pembagi tegangan
#define PIN_LED_R    32   // LED merah  (daya/armed)
#define PIN_LED_G    15   // LED hijau  (arm-ready)
#define PIN_LED_B     4   // LED biru   (wifi/status)
#define PIN_BUZZER   33   // buzzer via NPN
#define PIN_ARM      35   // tombol ARM fisik (input)

// ───────── KONSTANTA SISTEM ─────────
#define USE_BARO_ALT 0
#define YAW_ANTITORQUE_FF 1

float YAW_TRIM_FF   = 0.108f;

#define WASH_GAIN_SCHED 1
#define THR_SLEW 1
const float THR_SLEW_RATE = 4.0f;
const float ARM_RAMP_S    = 0.4f;
const float TS         = 0.005f;    // 200 Hz
const float SERVO_MAX  = 0.5236f;   // 30° dalam rad
const float YAW_MAX    = 0.3491f;   // 20° dalam rad
const float THR_HOVER  = 0.767f;    // throttle hover (final, dari fisika)

float SERVO_NEUTRAL[4] = {72.0f, 72.0f, 72.0f, 81.0f};  // S1,S2,S3,S4 (derajat)

float ATT_TRIM_ROLL_DEG  = 0.0f;    // derajat — offset trim roll
float ATT_TRIM_PITCH_DEG = 0.0f;    // derajat — offset trim pitch

// PID gains (FINAL — pole-placement)
struct PID { float kp,ki,kd,outMax,outMin,integ,prevErr; };
PID pidRoll  = {0.5345f, 0.05f, 0.2596f,  SERVO_MAX, -SERVO_MAX, 0,0};
PID pidPitch = {0.5345f, 0.05f, 0.2596f,  SERVO_MAX, -SERVO_MAX, 0,0};
PID pidYaw   = {2.0f,    0.10f, 0.10f,    YAW_MAX,  -YAW_MAX,  0,0};
PID pidAlt   = {0.1349f, 0.02f, 0.1619f,  0.4f, -0.4f, 0,0};

// Kalman (per kanal: state=[angle,bias])
struct Kalman {
  float angle=0, bias=0, P[2][2]={{1,0},{0,1}};
  float Q_angle=0.001f, Q_bias=0.003f, R=0.03f;
};
Kalman kfRoll, kfPitch;

// ───────── NAVIGASI GPS / WAYPOINT ─────────
const float R_EARTH = 6371000.0f;   // jari-jari Bumi (m)
const float K_APPR  = 1.5f;         // approach gain (galat posisi -> v_target)
const float K_V     = 0.9f;         // velocity gain (v_err -> sudut ref)
const float V_MAX   = 1.5f;         // kecepatan jelajah maks (m/s)
const float WP_ACCEPT_R = 1.5f;     // radius penerimaan waypoint (m)
const int   MAX_WP   = 12;          // jumlah maksimum waypoint

struct GpsState {
  bool   fix=false;                 // ada fix valid?
  int    sat=0;                     // jumlah satelit
  double lat=0, lng=0;              // posisi sekarang (derajat)
  double homeLat=0, homeLng=0;      // titik home (di-set saat ARM)
  bool   homeSet=false;
  float  xE=0, yN=0;                // posisi lokal (m) relatif home
};
GpsState gps;

struct Waypoint { float x, y, z; }; // target lokal (m) + ketinggian
Waypoint wp[MAX_WP];
int  wpCount=0;                     // jumlah waypoint termuat
int  wpIndex=0;                     // waypoint aktif
bool navActive=false;               // navigasi waypoint sedang berjalan?
float vxE=0, vyN=0;
float prevXE=0, prevYN=0;
uint32_t lastNavMs=0;

// ───────── VARIABEL GLOBAL ─────────
Servo s1,s2,s3,s4,esc;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

float gyroBias[3]={0,0,0};
float roll=0,pitch=0,yaw=0;
float gx=0,gy=0,gz=0, ax=0,ay=0,az=1;
float rcRoll=0, rcPitch=0, rcYaw=0;                           // -1..1 dari joystick
float rollRef=0,pitchRef=0,yawRef=0,altRef=3.0f,thrCmd=0.0f;  // thrCmd wajib mulai 0 (safety-lock ARM)
float alt=0,vz=0;
float servoDeg[4]={0,0,0,0};
float pidOut[4]={0,0,0,0};
bool  armed=false; int mode=0;                                // 0=Auto 1=Semi 2=Manual
float vbat=11.8f; int loopHz=0;
uint32_t lastLoop=0,lastTelem=0,lastHzCalc=0; int loopCount=0;
uint32_t lastRcMs=0;                                          // untuk RC timeout failsafe
bool  rcLost=false;                                           // status failsafe (global agar bisa dikirim ke telemetry)
bool  ledR=false, ledG=false, ledB=false;                     // state LED fisik saat ini (utk dicerminkan ke RC UI)

// ─── Kalibrasi ESC ───
uint8_t escCalStage = 0;
uint32_t lastEscCalMs = 0;                                    // watchdog: waktu terakhir ada perintah esc_cal

// ─── Test servo manual (posisi fin satu per satu, mis. untuk alignment fisik) ───
uint8_t servoTestActive = 0;                                  // 0=off, 1=aktif (bypass servoMix)
float   servoTestAngle[4] = {72.0f,72.0f,72.0f,81.0f};        // sudut mentah tiap servo saat mode test (0-180°; netral = SERVO_NEUTRAL[i], BUKAN 90 seragam)
uint32_t lastServoTestMs = 0;                                 // watchdog: waktu terakhir ada perintah servo_test

// ─── ESC/Motor Test tanpa ARM (uji motor langsung di throttle rendah) ───
uint8_t motorTestActive = 0;                                  // 0=off, 1=aktif (bypass servoMix & armed)
float   motorTestPct = 0.08f;                                 // 0..1 (dibatasi keras 0.03..0.15 di handler)
uint32_t lastMotorTestMs = 0;                                 // watchdog: waktu terakhir ada perintah motor_test

#define TESTMODE_TIMEOUT_MS 15000                             // auto-reset mode test kalau tak ada perintah lanjutan >15s

// ─── Diagnostik ESC ───
bool escOk = false;
int  escUs = 1000;
float    thrOut    = 0.0f;          // throttle keluaran setelah slew-limit & ramp
uint32_t armT      = 0;             // waktu transisi ARM (untuk ramp spool-up)
bool     prevArmed = false;         // deteksi tepi ARM
void escWrite(int us){
  us = constrain(us, 1000, 2000);
  esc.writeMicroseconds(us);
  escUs = us;
}

// Set LED fisik sekaligus catat state-nya (dipakai untuk dicerminkan ke RC UI)
void setLed(uint8_t pin, bool on, bool &track){
  digitalWrite(pin, on?HIGH:LOW);
  track = on;
}

// ============================================================
//  GY-87 / MPU-6050
// ============================================================
#define MPU 0x68
void mpuWrite(uint8_t reg,uint8_t val){
  Wire.beginTransmission(MPU);Wire.write(reg);Wire.write(val);Wire.endTransmission();
}
void mpuInit(){
  Wire.begin(PIN_SDA,PIN_SCL,400000);
  mpuWrite(0x6B,0x01);              // wake, clock=PLL gyroX
  mpuWrite(0x19,0x04);              // sample-rate div -> 200 Hz
  mpuWrite(0x1A,0x03);              // DLPF 44 Hz
  mpuWrite(0x1B,0x00);              // gyro ±250°/s
  mpuWrite(0x1C,0x00);              // accel ±2g
  delay(50);
}
void mpuRead(float&axg,float&ayg,float&azg,float&gxr,float&gyr,float&gzr){
  Wire.beginTransmission(MPU);Wire.write(0x3B);Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU, (size_t)14, (bool)true);
  int16_t AX=Wire.read()<<8|Wire.read();
  int16_t AY=Wire.read()<<8|Wire.read();
  int16_t AZ=Wire.read()<<8|Wire.read();
  Wire.read();Wire.read();                    // temp
  int16_t GX=Wire.read()<<8|Wire.read();
  int16_t GY=Wire.read()<<8|Wire.read();
  int16_t GZ=Wire.read()<<8|Wire.read();
  axg=AX/16384.0f; ayg=AY/16384.0f; azg=AZ/16384.0f;
  gxr=(GX/131.0f)*DEG_TO_RAD - gyroBias[0];
  gyr=(GY/131.0f)*DEG_TO_RAD - gyroBias[1];
  gzr=(GZ/131.0f)*DEG_TO_RAD - gyroBias[2];
}
void calibrateGyro(){
  float s[3]={0,0,0}; float a,b,c,d,e,f;
  for(int i=0;i<500;i++){ mpuRead(a,b,c,d,e,f);
    s[0]+=d+gyroBias[0]; s[1]+=e+gyroBias[1]; s[2]+=f+gyroBias[2]; delay(3);}
  for(int k=0;k<3;k++) gyroBias[k]=s[k]/500.0f;
}

// ============================================================
//  GPS NEO-M8N — parser NMEA minimal (GGA + RMC)
// ============================================================
char nmea[100]; uint8_t nmeaIdx=0;

double nmeaToDeg(const char* s, char hemi){
  double raw = atof(s);
  int deg = (int)(raw/100);
  double minutes = raw - deg*100;
  double dec = deg + minutes/60.0;
  if(hemi=='S' || hemi=='W') dec = -dec;
  return dec;
}

void parseNMEA(char* line){
  if(!strncmp(line+3,"GGA",3)){
    char* tok; int field=0;
    char latS[16]="",lngS[16]=""; char latH=0,lngH=0; int fixq=0,nsat=0;
    tok=strtok(line,","); 
    while(tok){
      switch(field){
        case 2: strncpy(latS,tok,15); break;
        case 3: latH=tok[0]; break;
        case 4: strncpy(lngS,tok,15); break;
        case 5: lngH=tok[0]; break;
        case 6: fixq=atoi(tok); break;
        case 7: nsat=atoi(tok); break;
      }
      tok=strtok(NULL,","); field++;
    }
    gps.sat = nsat;
    gps.fix = (fixq>0);
    if(gps.fix && latS[0] && lngS[0]){
      gps.lat = nmeaToDeg(latS,latH);
      gps.lng = nmeaToDeg(lngS,lngH);
    }
  }
}

void readGPS(){
  while(Serial2.available()){
    char ch=Serial2.read();
    if(ch=='$'){ nmeaIdx=0; nmea[nmeaIdx++]='$'; }
    else if(ch=='\n' || ch=='\r'){
      if(nmeaIdx>6){ nmea[nmeaIdx]=0; parseNMEA(nmea); }
      nmeaIdx=0;
    } else if(nmeaIdx<sizeof(nmea)-1){
      nmea[nmeaIdx++]=ch;
    }
  }
}

void gpsToLocal(){
  if(!gps.fix || !gps.homeSet) return;
  double dLat = (gps.lat - gps.homeLat) * DEG_TO_RAD;
  double dLng = (gps.lng - gps.homeLng) * DEG_TO_RAD;
  gps.yN = (float)(R_EARTH * dLat);                                   // utara
  gps.xE = (float)(R_EARTH * dLng * cos(gps.homeLat*DEG_TO_RAD));     // timur
}

// ============================================================
//  KALMAN FILTER (1-D angle)
// ============================================================
float kalmanUpdate(Kalman&kf,float newAngle,float newRate,float dt){
  kf.angle += dt*(newRate - kf.bias);
  kf.P[0][0]+= dt*(dt*kf.P[1][1]-kf.P[0][1]-kf.P[1][0]+kf.Q_angle);
  kf.P[0][1]-= dt*kf.P[1][1];
  kf.P[1][0]-= dt*kf.P[1][1];
  kf.P[1][1]+= kf.Q_bias*dt;
  float S=kf.P[0][0]+kf.R;
  float K0=kf.P[0][0]/S, K1=kf.P[1][0]/S;
  float y=newAngle-kf.angle;
  kf.angle+=K0*y; kf.bias+=K1*y;
  float p00=kf.P[0][0],p01=kf.P[0][1];
  kf.P[0][0]-=K0*p00; kf.P[0][1]-=K0*p01;
  kf.P[1][0]-=K1*p00; kf.P[1][1]-=K1*p01;
  return kf.angle;
}

// ============================================================
//  PID
// ============================================================
float pidCompute(PID&p,float setp,float meas,float dt){
  float err=setp-meas;
  p.integ+=p.ki*err*dt;
  p.integ=constrain(p.integ,p.outMin,p.outMax);
  float deriv=(err-p.prevErr)/dt; p.prevErr=err;
  float out=p.kp*err+p.integ+p.kd*deriv;
  return constrain(out,p.outMin,p.outMax);
}

// ============================================================
//  NAVIGASI WAYPOINT
// ============================================================
void navUpdate(float dt){
  if(!navActive || wpCount==0 || !gps.fix || !gps.homeSet){
    return;
  }
  gpsToLocal();
  if(dt>1e-3f){
    vxE = 0.8f*vxE + 0.2f*((gps.xE-prevXE)/dt);
    vyN = 0.8f*vyN + 0.2f*((gps.yN-prevYN)/dt);
  }
  prevXE=gps.xE; prevYN=gps.yN;

  Waypoint& T = wp[wpIndex];
  float ex = T.x - gps.xE;                      // galat timur
  float ey = T.y - gps.yN;                      // galat utara
  float d  = sqrtf(ex*ex + ey*ey);

  // faktor pengereman halus saat dekat target
  float beta = constrain(d/0.5f, 0.0f, 1.0f);
  // target kecepatan (dibatasi)
  float vtx = constrain(K_APPR*ex, -V_MAX, V_MAX) * beta;
  float vty = constrain(K_APPR*ey, -V_MAX, V_MAX) * beta;
  // konversi ke referensi sikap (perhatikan tanda roll dibalik)
  pitchRef = constrain( K_V*(vtx - vxE), -SERVO_MAX, SERVO_MAX);
  rollRef  = constrain(-K_V*(vty - vyN), -SERVO_MAX, SERVO_MAX);
  // ketinggian referensi mengikuti waypoint
  altRef   = T.z;

  // cek pencapaian waypoint
  if(d <= WP_ACCEPT_R){
    if(wpIndex < wpCount-1) wpIndex++;
    else navActive=false;
  }
}

// ============================================================
//  SERVO MIXER
// ============================================================
void servoMix(float uR,float uP,float uY){
  float d[4];
  d[0]= uR + uY;   d[1]= uP + uY;
  d[2]=-uR + uY;   d[3]=-uP + uY;
  for(int i=0;i<4;i++){
    d[i]=constrain(d[i],-SERVO_MAX,SERVO_MAX);
    servoDeg[i]=d[i]*RAD_TO_DEG;
  }
  s1.write(SERVO_NEUTRAL[0]+servoDeg[0]); s2.write(SERVO_NEUTRAL[1]+servoDeg[1]);
  s3.write(SERVO_NEUTRAL[2]+servoDeg[2]); s4.write(SERVO_NEUTRAL[3]+servoDeg[3]);
}

// ============================================================
//  WEBSOCKET handler
// ============================================================
void onWsEvent(AsyncWebSocket*server,AsyncWebSocketClient*client,
               AwsEventType type,void*arg,uint8_t*data,size_t len){
  if(type==WS_EVT_CONNECT){
    Serial.printf("WS client #%u connected\n", client->id());
  } else if(type==WS_EVT_DISCONNECT){
    Serial.printf("WS client #%u disconnected\n", client->id());
  } else if(type==WS_EVT_DATA){
    StaticJsonDocument<512> doc;
    if(deserializeJson(doc,data,len))return;
    const char* cmd=doc["cmd"];
    if(!cmd)return;

    if(!strcmp(cmd,"rc")){
      lastRcMs=millis();
      if(doc.containsKey("roll"))  rcRoll =constrain(doc["roll"].as<float>(),-1,1);
      if(doc.containsKey("pitch")) rcPitch=constrain(doc["pitch"].as<float>(),-1,1);
      if(doc.containsKey("yaw"))   rcYaw  =constrain(doc["yaw"].as<float>(),-1,1);
      if(doc.containsKey("thr"))   thrCmd =constrain(doc["thr"].as<float>(),0,1);
    } else if(!strcmp(cmd,"arm")){
      // ARM diblokir selama kalibrasi ESC, test servo, atau motor test berlangsung (keamanan)
      if(escCalStage!=0 || servoTestActive!=0 || motorTestActive!=0) return;
      bool wantArm = doc["armed"].as<bool>();
      // ─── THROTTLE-ZERO SAFETY LOCK ───
      if(wantArm && thrCmd > 0.03f){
        armed = false;
        StaticJsonDocument<64> rej;
        rej["cmd"]="arm_rejected"; rej["reason"]="throttle_not_zero";
        String out; serializeJson(rej,out);
        ws.textAll(out);
        return;
      }
      armed=wantArm;
      thrCmd = 0.0f;                      // selalu nol-kan di titik transisi (kedua arah)
      thrOut = 0.0f;
      pidRoll.integ=pidPitch.integ=pidYaw.integ=pidAlt.integ=0;
      if(armed){
        if(gps.fix){ gps.homeLat=gps.lat; gps.homeLng=gps.lng; gps.homeSet=true;
          prevXE=prevYN=vxE=vyN=0; gps.xE=gps.yN=0; }
      }
    } else if(!strcmp(cmd,"esc_cal")){
      if(armed || servoTestActive!=0 || motorTestActive!=0) return;
      const char* stage = doc["stage"];   // "max" | "min" | "cancel"
      if(!stage) return;
      lastEscCalMs = millis();            // watchdog: tandai masih ada aktivitas
      if(!strcmp(stage,"max")){
        escCalStage = 1;
        escWrite(2000);                   // batas atas throttle
      } else if(!strcmp(stage,"min")){
        escCalStage = 2;
        escWrite(1000);                   // batas bawah throttle -> ESC selesai kalibrasi & beep
      } else if(!strcmp(stage,"cancel")){
        escCalStage = 0;
        escWrite(1000);                   // kembali ke idle aman
      }
    } else if(!strcmp(cmd,"servo_test")){
      if(armed || escCalStage!=0 || motorTestActive!=0) return;
      const char* stage = doc["stage"];   // "start" | "set" | "stop"
      if(!stage) return;
      lastServoTestMs = millis();         // watchdog: tandai masih ada aktivitas
      if(!strcmp(stage,"start")){
        servoTestActive = 1;
        for(int i=0;i<4;i++) servoTestAngle[i]=SERVO_NEUTRAL[i];
        s1.write(SERVO_NEUTRAL[0]); s2.write(SERVO_NEUTRAL[1]); s3.write(SERVO_NEUTRAL[2]); s4.write(SERVO_NEUTRAL[3]);
      } else if(!strcmp(stage,"set")){
        int idx = doc["idx"] | -1;        // 0..3 -> servo 1..4
        float defAng = (idx>=0 && idx<4) ? SERVO_NEUTRAL[idx] : 90.0f;
        float ang = doc["angle"] | defAng;
        ang = constrain(ang, 0, 180);
        if(servoTestActive!=0 && idx>=0 && idx<4){
          servoTestAngle[idx]=ang;
          switch(idx){
            case 0: s1.write(ang); break;
            case 1: s2.write(ang); break;
            case 2: s3.write(ang); break;
            case 3: s4.write(ang); break;
          }
        }
      } else if(!strcmp(stage,"stop")){
        servoTestActive = 0;
        s1.write(SERVO_NEUTRAL[0]); s2.write(SERVO_NEUTRAL[1]); s3.write(SERVO_NEUTRAL[2]); s4.write(SERVO_NEUTRAL[3]);
        for(int i=0;i<4;i++) servoTestAngle[i]=SERVO_NEUTRAL[i];
      }
    } else if(!strcmp(cmd,"motor_test")){
      if(armed || escCalStage!=0 || servoTestActive!=0) return;
      const char* stage = doc["stage"];   // "start" | "stop"
      if(!stage) return;
      lastMotorTestMs = millis();         // watchdog: tandai masih ada aktivitas
      if(!strcmp(stage,"start")){
        float pct = doc["pct"] | 0.08f;
        motorTestPct = constrain(pct, 0.03f, 0.30f);
        motorTestActive = 1;
      } else if(!strcmp(stage,"stop")){
        motorTestActive = 0;
        escWrite(1000);
      }
    } else if(!strcmp(cmd,"reset_test")){
      escCalStage = 0;
      servoTestActive = 0;
      motorTestActive = 0;
      for(int i=0;i<4;i++) servoTestAngle[i]=SERVO_NEUTRAL[i];
      escWrite(1000);
      s1.write(SERVO_NEUTRAL[0]); s2.write(SERVO_NEUTRAL[1]); s3.write(SERVO_NEUTRAL[2]); s4.write(SERVO_NEUTRAL[3]);
    } else if(!strcmp(cmd,"mode")){
      mode=doc["mode"].as<int>();
      // navigasi waypoint hanya pada mode Auto (0)
      if(mode!=0) navActive=false;
    } else if(!strcmp(cmd,"alt_ref")){
      altRef=doc["value"].as<float>();
    } else if(!strcmp(cmd,"pid")){
      const char* l=doc["loop"];
      PID* p = !strcmp(l,"roll")?&pidRoll:!strcmp(l,"pitch")?&pidPitch:
               !strcmp(l,"yaw")?&pidYaw:&pidAlt;
      p->kp=doc["kp"]|p->kp; p->ki=doc["ki"]|p->ki; p->kd=doc["kd"]|p->kd;
    } else if(!strcmp(cmd,"trim")){
      if(armed) return;
      YAW_TRIM_FF     = doc["yaw_ff"] | YAW_TRIM_FF;
      SERVO_NEUTRAL[0]= doc["s1"]     | SERVO_NEUTRAL[0];
      SERVO_NEUTRAL[1]= doc["s2"]     | SERVO_NEUTRAL[1];
      SERVO_NEUTRAL[2]= doc["s3"]     | SERVO_NEUTRAL[2];
      SERVO_NEUTRAL[3]= doc["s4"]     | SERVO_NEUTRAL[3];
      ATT_TRIM_ROLL_DEG = doc["roll_trim"]  | ATT_TRIM_ROLL_DEG;
      ATT_TRIM_PITCH_DEG= doc["pitch_trim"] | ATT_TRIM_PITCH_DEG;
      if(servoTestActive==0 && escCalStage==0 && motorTestActive==0){
        s1.write(SERVO_NEUTRAL[0]); s2.write(SERVO_NEUTRAL[1]);
        s3.write(SERVO_NEUTRAL[2]); s4.write(SERVO_NEUTRAL[3]);
      }
    } else if(!strcmp(cmd,"wp_load")){
      int i=0;
      for(JsonVariant v : doc["wp"].as<JsonArray>()){
        if(i>=MAX_WP) break;
        JsonArray xyz=v.as<JsonArray>();
        wp[i].x=xyz[0]|0.0f; wp[i].y=xyz[1]|0.0f; wp[i].z=xyz[2]|1.5f;
        i++;
      }
      wpCount=i; wpIndex=0; navActive=false;
    } else if(!strcmp(cmd,"wp_start")){
      if(wpCount>0 && gps.fix){ navActive=true; wpIndex=0;
        if(!gps.homeSet){ gps.homeLat=gps.lat; gps.homeLng=gps.lng; gps.homeSet=true; } }
    } else if(!strcmp(cmd,"wp_stop")){
      navActive=false;
    }
  }
}

void sendTelemetry(){
  StaticJsonDocument<1536> d;
  d["t"]=millis();
  JsonObject att=d.createNestedObject("att");
  att["roll"]=roll; att["pitch"]=pitch; att["yaw"]=yaw;
  JsonObject rate=d.createNestedObject("rate");
  rate["gx"]=gx; rate["gy"]=gy; rate["gz"]=gz;
  JsonObject acc=d.createNestedObject("acc");
  acc["ax"]=ax; acc["ay"]=ay; acc["az"]=az;
  JsonObject kf=d.createNestedObject("kf");
  kf["roll"]=kfRoll.angle*RAD_TO_DEG; kf["pitch"]=kfPitch.angle*RAD_TO_DEG;
  kf["gain"]=kfRoll.P[0][0]/(kfRoll.P[0][0]+kfRoll.R);
  JsonObject a=d.createNestedObject("alt");
  a["z"]=alt; a["vz"]=vz; a["ref"]=altRef;
  JsonObject c=d.createNestedObject("ctrl");
  c["thr"]=thrCmd; c["s1"]=servoDeg[0]; c["s2"]=servoDeg[1];
  c["s3"]=servoDeg[2]; c["s4"]=servoDeg[3];
  JsonObject p=d.createNestedObject("pid");
  p["roll"]=pidOut[0]; p["pitch"]=pidOut[1]; p["yaw"]=pidOut[2]; p["alt"]=pidOut[3];
  // ─── nilai trim aktif saat ini (untuk visibilitas saat penyetelan live) ───
  JsonObject tr=d.createNestedObject("trim");
  tr["yawFF"]=YAW_TRIM_FF;
  tr["s1"]=SERVO_NEUTRAL[0]; tr["s2"]=SERVO_NEUTRAL[1];
  tr["s3"]=SERVO_NEUTRAL[2]; tr["s4"]=SERVO_NEUTRAL[3];
  tr["rollTrim"]=ATT_TRIM_ROLL_DEG; tr["pitchTrim"]=ATT_TRIM_PITCH_DEG;
  JsonObject sy=d.createNestedObject("sys");
  sy["vbat"]=vbat; sy["pct"]=(int)constrain((vbat-9.9f)/(12.6f-9.9f)*100,0,100);
  sy["hz"]=loopHz; sy["armed"]=armed; sy["mode"]=mode;
  sy["rcLost"]=rcLost;
  sy["escUs"]=escUs;                          // pulsa aktual yang ditulis ke ESC (µs)
  sy["escOk"]=escOk;                          // false = attach ESC gagal, PWM tidak keluar
  sy["escCal"]=escCalStage;                   // 0=normal 1=kirim MAX 2=kirim MIN
  sy["servoTest"]=servoTestActive;            // 0=normal 1=mode test servo aktif
  sy["motorTest"]=motorTestActive;            // 0=normal 1=mode test motor aktif
  // ─── sudut servo saat mode test manual ───
  JsonObject svt=d.createNestedObject("svt");
  svt["active"]=servoTestActive;
  svt["a1"]=servoTestAngle[0]; svt["a2"]=servoTestAngle[1];
  svt["a3"]=servoTestAngle[2]; svt["a4"]=servoTestAngle[3];
  // ─── status LED fisik (dicerminkan ke RC UI) ───
  JsonObject led=d.createNestedObject("led");
  led["r"]=ledR; led["g"]=ledG; led["b"]=ledB;
  // ─── GPS + NAVIGASI ───
  JsonObject g=d.createNestedObject("gps");
  g["fix"]=gps.fix; g["sat"]=gps.sat;
  g["lat"]=gps.lat; g["lng"]=gps.lng;
  g["x"]=gps.xE; g["y"]=gps.yN;
  JsonObject nv=d.createNestedObject("nav");
  nv["active"]=navActive; nv["idx"]=wpIndex; nv["count"]=wpCount;
  if(navActive && wpCount>0){
    float ex=wp[wpIndex].x-gps.xE, ey=wp[wpIndex].y-gps.yN;
    nv["dist"]=sqrtf(ex*ex+ey*ey);
  } else nv["dist"]=0.0f;
  String out; serializeJson(d,out);
  ws.textAll(out);
}

float readVbat(){
  const float VBAT_BYPASS_CONST = 12.4f;      // asumsikan baterai 3S terisi penuh
  return VBAT_BYPASS_CONST;

  // ---- kode asli (nonaktif sementara) ----
  // int raw=analogRead(PIN_VBAT);
  // float v=raw/4095.0f*3.3f;
  // return v*(122.0f/22.0f);                 // pembagi 100k:22k
}

// ============================================================
//  SETUP
// ============================================================
void setup(){
  Serial.begin(115200);
  // GPS UART2
  Serial2.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);

  pinMode(PIN_LED_R,OUTPUT);pinMode(PIN_LED_G,OUTPUT);pinMode(PIN_LED_B,OUTPUT);
  pinMode(PIN_BUZZER,OUTPUT);
  pinMode(PIN_ARM,INPUT_PULLUP);
  analogReadResolution(12);

  pinMode(PIN_ESC, INPUT_PULLDOWN);
  esc.setPeriodHertz(50);
  esc.attach(PIN_ESC, 1000, 2000);
  escOk = esc.attached();
  escWrite(1000);
  Serial.printf("Attach ESC (kanal pertama): %s\n", escOk ? "OK" : "GAGAL");
  if(!escOk) Serial.println("!! ESC attach GAGAL — motor tidak akan menerima sinyal PWM");

  s1.setPeriodHertz(50); s2.setPeriodHertz(50);
  s3.setPeriodHertz(50); s4.setPeriodHertz(50);
  bool ok1 = s1.attach(PIN_SERVO1, 500, 2400) != 0;
  bool ok2 = s2.attach(PIN_SERVO2, 500, 2400) != 0;
  bool ok3 = s3.attach(PIN_SERVO3, 500, 2400) != 0;
  bool ok4 = s4.attach(PIN_SERVO4, 500, 2400) != 0;
  Serial.printf("Attach servo: S1=%d S2=%d S3=%d S4=%d\n", ok1, ok2, ok3, ok4);

  Serial.println("Arming ESC... (tahan 1000 us selama 5 detik)");
  {
    uint32_t t0 = millis();
    uint32_t lastPrint = 0;
    while(millis() - t0 < 5000){
      escWrite(1000);
      if(millis() - lastPrint >= 1000){
        lastPrint = millis();
        Serial.printf("  arming... %lus\n", (unsigned long)((millis()-t0)/1000 + 1));
      }
      delay(20);
    }
  }
  Serial.println("ESC Ready");
  digitalWrite(PIN_BUZZER, HIGH); delay(80); digitalWrite(PIN_BUZZER, LOW);

  mpuInit();
  setLed(PIN_LED_B,true,ledB);
  calibrateGyro();
  setLed(PIN_LED_B,false,ledB);

  // LittleFS untuk file web
  if(!LittleFS.begin(true)){
    Serial.println("LittleFS gagal di-mount!");
  }

  // WiFi Access Point
  WiFi.softAP(AP_SSID,AP_PASS);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  Serial.print("AP IP: ");Serial.println(WiFi.softAPIP());   // 192.168.4.1

  // WebSocket
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // ─── Routing dua halaman ───
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
  server.on("/rc", HTTP_GET, [](AsyncWebServerRequest*req){
    req->send(LittleFS, "/rc.html", "text/html");
  });
  server.onNotFound([](AsyncWebServerRequest*req){
    req->send(404, "text/plain", "404: Not Found");
  });

  server.begin();
  setLed(PIN_LED_G,true,ledG);
  lastLoop=micros(); lastHzCalc=millis(); lastRcMs=millis(); lastNavMs=millis();
}

// ============================================================
//  LOOP — control @200Hz, GPS + nav @ ~10Hz, telemetry @20Hz
// ============================================================
void loop(){
  readGPS();

  uint32_t now=micros();
  if(now-lastLoop >= (uint32_t)(TS*1e6)){
    float dt=(now-lastLoop)/1e6f; lastLoop=now;

    // 1. baca IMU
    mpuRead(ax,ay,az,gx,gy,gz);
    // 2. sudut dari akselerometer
    float accRoll  = atan2(ay,az);
    float accPitch = atan2(-ax,sqrt(ay*ay+az*az));
    // 3. Kalman
    float fRoll =kalmanUpdate(kfRoll, accRoll, gx,dt) + ATT_TRIM_ROLL_DEG *DEG_TO_RAD;
    float fPitch=kalmanUpdate(kfPitch,accPitch,gy,dt) + ATT_TRIM_PITCH_DEG*DEG_TO_RAD;

    roll =fRoll*RAD_TO_DEG; pitch=fPitch*RAD_TO_DEG;
    yaw += gz*dt*RAD_TO_DEG; if(yaw>180)yaw-=360; if(yaw<-180)yaw+=360;

    // 4. RC FAILSAFE: jika perintah RC putus > 500 ms -> auto-level
    rcLost = (millis()-lastRcMs > 500);
    if(rcLost){ rcRoll=0; rcPitch=0; rcYaw=0; }

    // 5. NAVIGASI (loop luar ~10 Hz) — hanya mode Auto
    if(millis()-lastNavMs >= 100){
      float ndt=(millis()-lastNavMs)/1000.0f; lastNavMs=millis();
      if(mode==0 && navActive){
        navUpdate(ndt);
      }
    }

    // 6. Konversi perintah RC -> setpoint (mode Semi/Manual) Mode Auto (0) + nav aktif: rollRef/pitchRef dari navUpdate
    if(!(mode==0 && navActive)){
      rollRef  = rcRoll  * SERVO_MAX;       // hingga ±30°
      pitchRef = rcPitch * SERVO_MAX;
    }
    yawRef  += rcYaw   * 0.02f;             // yaw rate command (integratif)
    // ─── Anti-windup ───
    {
      const float YAW_WINDOW_DEG = 45.0f;
      float yawActRad = yaw * DEG_TO_RAD;
      float yawWinRad = YAW_WINDOW_DEG * DEG_TO_RAD;
      if(yawRef > yawActRad + yawWinRad) yawRef = yawActRad + yawWinRad;
      if(yawRef < yawActRad - yawWinRad) yawRef = yawActRad - yawWinRad;
    }

    // 7. PID (mode 0/1 pakai PID; mode 2 Manual = bypass langsung) deteksi tepi ARM (untuk ramp spool-up lapisan fisika)
    bool justArmed = armed && !prevArmed;
    prevArmed = armed;
    if(justArmed){ armT = millis(); thrOut = 0.0f; }

    float uR=0,uP=0,uY=0,uZ=0;
    if(mode<2){
      uR=pidCompute(pidRoll, rollRef, fRoll, dt);
      uP=pidCompute(pidPitch,pitchRef,fPitch,dt);
      uY=pidCompute(pidYaw,  yawRef,  yaw*DEG_TO_RAD, dt);
      uZ=pidCompute(pidAlt,  altRef,  alt, dt);
    } else { uR=rollRef; uP=pitchRef; uY=rcYaw*YAW_MAX; }
    pidOut[0]=uR;pidOut[1]=uP;pidOut[2]=uY;pidOut[3]=uZ;

    // 7.5 WATCHDOG mode test
    if(escCalStage!=0 && millis()-lastEscCalMs > TESTMODE_TIMEOUT_MS){
      escCalStage = 0;
      escWrite(1000);
    }
    if(servoTestActive!=0 && millis()-lastServoTestMs > TESTMODE_TIMEOUT_MS){
      servoTestActive = 0;
      s1.write(SERVO_NEUTRAL[0]); s2.write(SERVO_NEUTRAL[1]); s3.write(SERVO_NEUTRAL[2]); s4.write(SERVO_NEUTRAL[3]);
      for(int i=0;i<4;i++) servoTestAngle[i]=SERVO_NEUTRAL[i];
    }
    if(motorTestActive!=0 && millis()-lastMotorTestMs > TESTMODE_TIMEOUT_MS){
      motorTestActive = 0;
      escWrite(1000);
    }

    // 8. Servo mixer + ESC
    if(servoTestActive!=0){
      // ─── Test servo manual sedang berlangsung ───
      escWrite(1000);                     // motor tetap idle/mati selama test
      bool blink = (millis()/300)%2;
      setLed(PIN_LED_G, blink, ledG);     // hijau berkedip = mode test servo
      setLed(PIN_LED_R, true,  ledR);     // tetap tunjukkan "tidak armed"
      setLed(PIN_LED_B, false, ledB);
    } else if(escCalStage!=0){
      // ─── Kalibrasi ESC sedang berlangsung ───
      servoMix(0,0,0);
      bool blink = (millis()/300)%2;
      setLed(PIN_LED_B, blink, ledB);
      setLed(PIN_LED_R, true,  ledR);     // tetap tunjukkan "tidak armed"
      setLed(PIN_LED_G, false, ledG);
    } else if(motorTestActive!=0){
      // ─── ESC/Motor Test sedang berlangsung (tanpa ARM) ───
      servoMix(0,0,0);
      escWrite(1000+(int)(motorTestPct*1000));
      bool blink = (millis()/300)%2;      // R & G berkedip bersamaan = peringatan
      setLed(PIN_LED_R, blink, ledR);
      setLed(PIN_LED_G, blink, ledG);
      setLed(PIN_LED_B, false, ledB);
    } else if(armed){
      // ── [FISIKA-2] gain scheduling prop-wash: otoritas fin ∝ thr² ──
      #if WASH_GAIN_SCHED
        float thrSched = max(thrOut, 0.30f);
        float sched = constrain((THR_HOVER*THR_HOVER)/(thrSched*thrSched),
                                0.6f, 1.8f);
      #else
        float sched = 1.0f;
      #endif
      // ── [FISIKA-1] feedforward anti-torsi: kolektif konstan 0,108 rad,
      //    fade-in mengikuti throttle (fin baru efektif saat ada wash) ──
      #if YAW_ANTITORQUE_FF
        float yawFF = YAW_TRIM_FF * constrain(thrOut/0.30f, 0.0f, 1.0f);
      #else
        float yawFF = 0.0f;
      #endif
      servoMix(uR*sched, uP*sched, uY*sched + yawFF);
      // ─── FAILSAFE ───
      #if USE_BARO_ALT
        float thrTgt = rcLost ? 0.0f : constrain(thrCmd+uZ,0,1);
      #else
        float thrTgt = rcLost ? 0.0f : constrain(thrCmd,0,1);
      #endif
      // ── [FISIKA-4] ramp spool-up selama ~1,2 s pertama setelah ARM ──
      float ramp = constrain((millis()-armT)/(ARM_RAMP_S*1000.0f), 0.0f, 1.0f);
      thrTgt *= ramp;
      // ── [FISIKA-3] slew-limit throttle: pangkas yaw kick I_p·dω/dt ──
      #if THR_SLEW
        float dMax = THR_SLEW_RATE * TS;
        thrOut += constrain(thrTgt - thrOut, -dMax, dMax);
      #else
        thrOut = thrTgt;
      #endif
      if(rcLost) thrOut = 0.0f;               // failsafe tetap instan
      escWrite(1000+thrOut*1000);
      setLed(PIN_LED_G, true,   ledG);
      setLed(PIN_LED_R, rcLost, ledR);
    } else {
      thrOut = 0.0f;                          // reset slew saat disarm
      servoMix(0,0,0);
      escWrite(1000);
      setLed(PIN_LED_R, true,  ledR);
      setLed(PIN_LED_G, false, ledG);
    }

    // 9. loop rate + baca baterai
    loopCount++;
    if(millis()-lastHzCalc>=1000){ loopHz=loopCount; loopCount=0;
      lastHzCalc=millis(); vbat=readVbat(); }
    // battery safety
    if(vbat<10.2f) armed=false;   // Li-Ion 3S: disarm di 3,4V/sel sebelum drop tajam
  }

  // telemetri 20 Hz
  if(millis()-lastTelem>=50){ lastTelem=millis(); sendTelemetry(); ws.cleanupClients(); }
}
