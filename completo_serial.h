/* SCARA: 2 motores DC con PID + 2 servos controlados por Serial
   -----------------------------------------
   M1 (q1): DC con encoder
     DIR = 8
     PWM = 11  <-- cambiado para no pelear con Servo (antes 9)
     ENC A = 2 (INT0)
     ENC B = 3 (INT1)

   M2 (q2): DC con encoder
     DIR = 7
     PWM = 6
     ENC A = 4  (PCINT20 / PORTD)
     ENC B = 12 (PCINT4  / PORTB)

   q3: Servo SG90  -> pin 10
   q4: Servo HK15138 -> pin 5

   Serial 115200:
     ?                  -> estado
     z / z1 / z2        -> cero encoders
     s / s1 / s2        -> stop
     m1 <deg>, m2 <deg>, m <d1> <d2>
     q3 <deg>, q4 <deg>
*/

#include <Arduino.h>
#include <Servo.h>

// --------- Pines M1 ----------
const int M1_DIR   = 8;
const int M1_PWM   = 11;
const int M1_ENC_A = 2;
const int M1_ENC_B = 3;

// --------- Pines M2 ----------
const int M2_DIR   = 7;
const int M2_PWM   = 6;
const int M2_ENC_A = 4;
const int M2_ENC_B = 12;

// --------- Servos (q3, q4) ----------
const int SERVO_Q3_PIN = 10;
const int SERVO_Q4_PIN = 5;

Servo servoQ3;
Servo servoQ4;
int q3Pos = 0;
int q4Pos = 0;

// ===== Mecánico / Encoder =====
const float PPR_MOTOR = 11.0f;
const float GEAR_RATIO = 110.0f;
const float CPR = PPR_MOTOR * GEAR_RATIO * 4.0f;

const int SIGN_DIR_M1 = 1;
const int SIGN_DIR_M2 = 1;

// ===== PID =====
float Kp = 0.088f, Ki = 0.156f, Kd = 0.0f;
float Ts = 0.01f;
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

volatile uint8_t m2_prevAB = 0;

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

// ===== ISRs M1 =====
void isr1A(){
  bool a = digitalRead(M1_ENC_A), b = digitalRead(M1_ENC_B);
  enc1Count += (a == b) ? SIGN_DIR_M1 : -SIGN_DIR_M1;
}
void isr1B(){
  bool a = digitalRead(M1_ENC_A), b = digitalRead(M1_ENC_B);
  enc1Count += (a != b) ? SIGN_DIR_M1 : -SIGN_DIR_M1;
}

// ===== LUT cuadratura =====
const int8_t qdecLUT[16] = {
  0,-1,+1, 0,
 +1, 0, 0,-1,
 -1, 0, 0,+1,
  0,+1,-1, 0
};

// ===== Handler M2 =====
inline void m2_pcint_update(){
  uint8_t a = (PIND >> 4) & 0x01;
  uint8_t b = (PINB >> 4) & 0x01;
  uint8_t curr = (b<<1) | a;
  uint8_t idx  = ((m2_prevAB & 0x03)<<2) | (curr & 0x03);
  int8_t step  = qdecLUT[idx];
  if(step!=0) enc2Count += step * SIGN_DIR_M2;
  m2_prevAB = curr;
}

ISR(PCINT2_vect){ m2_pcint_update(); }
ISR(PCINT0_vect){ m2_pcint_update(); }

// ===== Control DC =====
void controlUno(long encNow, long &encPrev, float &integ, float &wFilt, float &pwmFiltered,
                int pinDir, int pinPwm, float targetDeg)
{
  float ang = cnt2deg(encNow);
  float e = angErr(targetDeg, ang);

  long dCounts = encNow - encPrev;
  float w = ((float)dCounts * 360.0f / CPR) / Ts;
  wFilt = ALPHA_D*w + (1.0f-ALPHA_D)*wFilt;

  if (fabs(e) < DEADZONE_DEG){
    integ = 0.0f;
    pwmFiltered = 0.0f;
    analogWrite(pinPwm, 0);
  } else {
    integ += Ki*Ts*e;
    if(integ > INTEG_LIM) integ = INTEG_LIM;
    if(integ < -INTEG_LIM) integ = -INTEG_LIM;

    float u = Kp*e + integ - Kd*wFilt;

    float boost = 0.0f;
    float ae = fabs(e);
    if(ae > DEADZONE_DEG){
      float k = ae>=BOOST_OFF_DEG?1.0f:(ae-DEADZONE_DEG)/(BOOST_OFF_DEG-DEADZONE_DEG);
      if(k<0) k=0;
      boost = PWM_STATIC*k;
    }

    int pwm = (int)(fabs(u)+boost);
    if(pwm>PWM_MAX) pwm = PWM_MAX;
    if(pwm<PWM_MIN) pwm = PWM_MIN;

    pwmFiltered = ALPHA_PWM*pwm + (1.0f-ALPHA_PWM)*pwmFiltered;

    digitalWrite(pinDir, (u>=0)?HIGH:LOW);
    analogWrite(pinPwm, (int)pwmFiltered);
  }

  encPrev = encNow;
}

// ===== Serial =====
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

  pinMode(M1_DIR,OUTPUT); pinMode(M1_PWM,OUTPUT);
  pinMode(M2_DIR,OUTPUT); pinMode(M2_PWM,OUTPUT);

  digitalWrite(M1_DIR,LOW); analogWrite(M1_PWM,0);
  digitalWrite(M2_DIR,LOW); analogWrite(M2_PWM,0);

  pinMode(M1_ENC_A,INPUT);
  pinMode(M1_ENC_B,INPUT);
  attachInterrupt(digitalPinToInterrupt(M1_ENC_A), isr1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M1_ENC_B), isr1B, CHANGE);

  pinMode(M2_ENC_A,INPUT);
  pinMode(M2_ENC_B,INPUT);

  uint8_t a0=(PIND>>4)&0x01;
  uint8_t b0=(PINB>>4)&0x01;
  m2_prevAB=(b0<<1)|a0;

  PCICR |= (1<<PCIE2) | (1<<PCIE0);
  PCMSK2 |= (1<<PCINT20);
  PCMSK0 |= (1<<PCINT4);

  servoQ3.attach(SERVO_Q3_PIN);
  servoQ4.attach(SERVO_Q4_PIN);
  servoQ3.write(0);
  servoQ4.write(0);

  enc1Prev=enc1Count;
  enc2Prev=enc2Count;

  Serial.println("SCARA listo.");
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

    controlUno(e1, enc1Prev, integ1, wFilt1, pwmF1, M1_DIR, M1_PWM, target1Deg);
    controlUno(e2, enc2Prev, integ2, wFilt2, pwmF2, M2_DIR, M2_PWM, target2Deg);
  }
}
