/* SCARA: 2 motores DC con PID + 2 servos controlados por Serial - ESP32

   M1 (q1): DC con encoder
     DIR   = 25
     PWM   = 26  (LEDC canal 0)
     ENC A = 34
     ENC B = 35

   M2 (q2): DC con encoder
     DIR   = 18
     PWM   = 19  (LEDC canal 1)
     ENC A = 32
     ENC B = 33

   q3: Servo SG90    -> pin 14
   q4: Servo HK15138 -> pin 27

   Serial 115200:
     ?                  -> estado
     z / z1 / z2        -> cero encoders
     s / s1 / s2        -> stop
     m1 <deg>, m2 <deg>, m <d1> <d2>
     q3 <deg>, q4 <deg>
*/

#include <Arduino.h>
#include <ESP32Servo.h>

// --------- Pines M1 (ESP32) ----------
const int M1_DIR   = 25;
const int M1_PWM   = 26;
const int M1_ENC_A = 34;
const int M1_ENC_B = 35;

// --------- Pines M2 (ESP32) ----------
const int M2_DIR   = 18;
const int M2_PWM   = 19;
const int M2_ENC_A = 32;
const int M2_ENC_B = 33;

// --------- Servos (q3, q4) ----------
const int SERVO_Q3_PIN = 14;
const int SERVO_Q4_PIN = 27;

Servo servoQ3;
Servo servoQ4;
int q3Pos = 0;
int q4Pos = 0;

// ===== PWM (LEDC) =====
const int CH_M1_PWM   = 0;
const int CH_M2_PWM   = 1;
const int PWM_FREQ    = 20000; // 20 kHz
const int PWM_RES_BIT = 8;     // 0..255

// ===== Mecánico / Encoder =====
const float PPR_MOTOR = 11.0f;
const float GEAR_RATIO = 110.0f;
const float CPR = PPR_MOTOR * GEAR_RATIO * 4.0f;

const int SIGN_DIR_M1 = 1;
const int SIGN_DIR_M2 = 1;

// ===== PID =====
float Kp = 0.088f, Ki = 0.156f, Kd = 0.0f;
float Ts = 0.01f;         // 10 ms
const float INTEG_LIM = 150.0f;

const float DEADZONE_DEG = 0.8f;
const int   PWM_MAX = 140;
const int   PWM_MIN = 45;

const int   PWM_STATIC = 18;
const float BOOST_OFF_DEG = 12.0f;

const float ALPHA_PWM = 0.25f;
const float ALPHA_D   = 0.25f;

// ===== Estado M1 =====
volatile long enc1Count = 0;
long   enc1Prev = 0;
float  integ1 = 0.0f;
float  pwmF1 = 0.0f;
float  wFilt1 = 0.0f;

// ===== Estado M2 =====
volatile long enc2Count = 0;
long   enc2Prev = 0;
float  integ2 = 0.0f;
float  pwmF2 = 0.0f;
float  wFilt2 = 0.0f;

// Setpoints
float target1Deg = 0.0f;
float target2Deg = 0.0f;

unsigned long tPrev = 0;

// ===== Utils =====
inline float wrap360(float a){
  while(a >= 360) a -= 360;
  while(a < 0)    a += 360;
  return a;
}

inline float cnt2deg(long c){
  return wrap360((float)c * 360.0f / CPR);
}

inline float angErr(float tgt,float act){
  float e = tgt - act;
  while(e > 180)  e -= 360;
  while(e < -180) e += 360;
  return e;
}

// ===== ISRs M1 (ENC A/B) =====
// (usamos dos interrupciones como en el UNO, pero ahora con attachInterrupt)
void IRAM_ATTR isr1A(){
  bool a = digitalRead(M1_ENC_A);
  bool b = digitalRead(M1_ENC_B);
  enc1Count += (a == b) ? SIGN_DIR_M1 : -SIGN_DIR_M1;
}

void IRAM_ATTR isr1B(){
  bool a = digitalRead(M1_ENC_A);
  bool b = digitalRead(M1_ENC_B);
  enc1Count += (a != b) ? SIGN_DIR_M1 : -SIGN_DIR_M1;
}

// ===== ISRs M2 (ENC A/B) =====
void IRAM_ATTR isr2A(){
  bool a = digitalRead(M2_ENC_A);
  bool b = digitalRead(M2_ENC_B);
  enc2Count += (a == b) ? SIGN_DIR_M2 : -SIGN_DIR_M2;
}

void IRAM_ATTR isr2B(){
  bool a = digitalRead(M2_ENC_A);
  bool b = digitalRead(M2_ENC_B);
  enc2Count += (a != b) ? SIGN_DIR_M2 : -SIGN_DIR_M2;
}

// ===== Control DC (adaptado a LEDC) =====
void controlUno(long encNow, long &encPrev, float &integ, float &wFilt, float &pwmFiltered,
                int pinDir, int pwmChannel, float targetDeg)
{
  float ang = cnt2deg(encNow);
  float e = angErr(targetDeg, ang);

  long dCounts = encNow - encPrev;
  float w = ((float)dCounts * 360.0f / CPR) / Ts;
  wFilt = ALPHA_D*w + (1.0f-ALPHA_D)*wFilt;

  if (fabs(e) < DEADZONE_DEG){
    integ = 0.0f;
    pwmFiltered = 0.0f;
    ledcWrite(pwmChannel, 0);
  } else {
    // Integral con anti-windup
    integ += Ki*Ts*e;
    if(integ > INTEG_LIM) integ = INTEG_LIM;
    if(integ < -INTEG_LIM) integ = -INTEG_LIM;

    float u = Kp*e + integ - Kd*wFilt;

    // Boost estático
    float boost = 0.0f;
    float ae = fabs(e);
    if(ae > DEADZONE_DEG){
      float k = ae>=BOOST_OFF_DEG?1.0f:(ae-DEADZONE_DEG)/(BOOST_OFF_DEG-DEADZONE_DEG);
      if(k<0) k=0;
      boost = PWM_STATIC*k;
    }

    int pwm = (int)(fabs(u)+boost);
    if(pwm > PWM_MAX) pwm = PWM_MAX;
    if(pwm < PWM_MIN) pwm = PWM_MIN;

    pwmFiltered = ALPHA_PWM*pwm + (1.0f-ALPHA_PWM)*pwmFiltered;

    digitalWrite(pinDir, (u>=0)?HIGH:LOW);
    ledcWrite(pwmChannel, (int)pwmFiltered); // 0..255 (8 bits)
  }

  encPrev = encNow;
}

// ===== Serial =====
void procesarLinea(const String &s);   // forward

void leerComandos(){
  static String buf="";
  while(Serial.available()){
    char c = Serial.read();
    if(c=='\r') continue;
    if(c=='\n'){
      buf.trim();
      if(buf.length()) procesarLinea(buf);
      buf="";
    } else buf += c;
  }
}

bool parse2floats(const String &s, float &a, float &b){
  int p1 = s.indexOf(' ');
  if(p1<0) return false;
  int p2 = s.indexOf(' ', p1+1);
  if(p2<0) return false;
  a = s.substring(p1+1,p2).toFloat();
  b = s.substring(p2+1).toFloat();
  return true;
}

void procesarLinea(const String &s){

  if(s=="?"){
    Serial.print("M1 ang="); Serial.print(cnt2deg(enc1Count));
    Serial.print(" tgt1="); Serial.print(target1Deg);

    Serial.print(" | M2 ang="); Serial.print(cnt2deg(enc2Count));
    Serial.print(" tgt2="); Serial.print(target2Deg);

    Serial.print(" | q3="); Serial.print(q3Pos);
    Serial.print(" q4="); Serial.println(q4Pos);
    return;
  }

  if(s=="z"){
    noInterrupts(); enc1Count=0; enc2Count=0; interrupts();
    target1Deg=target2Deg=0;
    integ1=integ2=0;
    pwmF1=pwmF2=0;
    Serial.println(">> Cero ambos");
    return;
  }

  if(s=="z1"){
    noInterrupts(); enc1Count=0; interrupts();
    target1Deg=0;
    integ1=pwmF1=0;
    Serial.println(">> Cero M1");
    return;
  }

  if(s=="z2"){
    noInterrupts(); enc2Count=0; interrupts();
    target2Deg=0;
    integ2=pwmF2=0;
    Serial.println(">> Cero M2");
    return;
  }

  if(s=="s"){
    target1Deg = cnt2deg(enc1Count);
    target2Deg = cnt2deg(enc2Count);
    integ1=integ2=0;
    pwmF1=pwmF2=0;
    Serial.println(">> Stop ambos");
    return;
  }

  if(s=="s1"){
    target1Deg=cnt2deg(enc1Count);
    integ1=pwmF1=0;
    Serial.println(">> Stop M1");
    return;
  }

  if(s=="s2"){
    target2Deg=cnt2deg(enc2Count);
    integ2=pwmF2=0;
    Serial.println(">> Stop M2");
    return;
  }

  if(s.startsWith("m1 ") || s.startsWith("q1 ")){
    float v=s.substring(3).toFloat();
    if(v>=0 && v<=360){
      target1Deg=wrap360(v);
      Serial.print(">> tgt1="); Serial.println(target1Deg);
    }
    return;
  }

  if(s.startsWith("m2 ") || s.startsWith("q2 ")){
    float v=s.substring(3).toFloat();
    if(v>=0 && v<=360){
      target2Deg=wrap360(v);
      Serial.print(">> tgt2="); Serial.println(target2Deg);
    }
    return;
  }

  if(s.startsWith("m ")){
    float a,b;
    if(parse2floats(s,a,b)){
      target1Deg=wrap360(a);
      target2Deg=wrap360(b);
      Serial.print(">> tgt1="); Serial.print(target1Deg);
      Serial.print(" tgt2="); Serial.println(target2Deg);
    }
    return;
  }

  if(s.startsWith("q3 ")){
    int v=s.substring(3).toInt();
    if(v>=0 && v<=180){
      q3Pos=v;
      servoQ3.write(v);
      Serial.print(">> q3="); Serial.println(v);
    }
    return;
  }

  if(s.startsWith("q4 ")){
    int v=s.substring(3).toInt();
    if(v>=0 && v<=180){
      q4Pos=v;
      servoQ4.write(v);
      Serial.print(">> q4="); Serial.println(v);
    }
    return;
  }

  Serial.println("Comando invalido");
}

// =======================================================
// ===================== SETUP ===========================
// =======================================================

void setup(){
  Serial.begin(115200);

  // Motores
  pinMode(M1_DIR,OUTPUT);
  pinMode(M2_DIR,OUTPUT);

  // PWM por LEDC
  ledcSetup(CH_M1_PWM, PWM_FREQ, PWM_RES_BIT);
  ledcAttachPin(M1_PWM, CH_M1_PWM);
  ledcSetup(CH_M2_PWM, PWM_FREQ, PWM_RES_BIT);
  ledcAttachPin(M2_PWM, CH_M2_PWM);

  digitalWrite(M1_DIR,LOW); ledcWrite(CH_M1_PWM,0);
  digitalWrite(M2_DIR,LOW); ledcWrite(CH_M2_PWM,0);

  // Encoders
  pinMode(M1_ENC_A,INPUT);
  pinMode(M1_ENC_B,INPUT);
  pinMode(M2_ENC_A,INPUT);
  pinMode(M2_ENC_B,INPUT);

  attachInterrupt(M1_ENC_A, isr1A, CHANGE);
  attachInterrupt(M1_ENC_B, isr1B, CHANGE);
  attachInterrupt(M2_ENC_A, isr2A, CHANGE);
  attachInterrupt(M2_ENC_B, isr2B, CHANGE);

  // Servos (rango típico 500–2400 us)
  servoQ3.attach(SERVO_Q3_PIN, 500, 2400);
  servoQ4.attach(SERVO_Q4_PIN, 500, 2400);
  servoQ3.write(0);
  servoQ4.write(0);

  enc1Prev = enc1Count;
  enc2Prev = enc2Count;

  Serial.println("SCARA ESP32 listo.");
}

// =======================================================
// ====================== LOOP ===========================
// =======================================================

void loop(){
  leerComandos();

  unsigned long t = millis();
  if(t - tPrev >= (unsigned long)(Ts*1000)){
    tPrev = t;

    long e1 = enc1Count;
    long e2 = enc2Count;

    controlUno(e1, enc1Prev, integ1, wFilt1, pwmF1, M1_DIR, CH_M1_PWM, target1Deg);
    controlUno(e2, enc2Prev, integ2, wFilt2, pwmF2, M2_DIR, CH_M2_PWM, target2Deg);
  }
}
