/* Doble PID 0..360° en Arduino UNO con setpoints independientes
   M1: DIR=8, PWM=9, ENC A=2 (INT0), ENC B=3 (INT1)
   M2: DIR=7, PWM=6, ENC A=4 (PCINT20/PORTD), ENC B=12 (PCINT4/PORTB)  <-- cambiado
   Serial 115200 (NL):
     ?                -> estado
     z / z1 / z2      -> cero ambos / M1 / M2
     s / s1 / s2      -> parar ambos / M1 / M2 en su ángulo actual
     m1 <deg>         -> setpoint M1
     m2 <deg>         -> setpoint M2
     m <deg1> <deg2>  -> setpoints M1 y M2
*/

#include <Arduino.h>

// --------- Pines M1 ----------
const int M1_DIR   = 8;
const int M1_PWM   = 9;
const int M1_ENC_A = 2; // INT0
const int M1_ENC_B = 3; // INT1

// --------- Pines M2 ----------
const int M2_DIR   = 7;
const int M2_PWM   = 6;
const int M2_ENC_A = 4;   // PD4 (PCINT20) - queda igual
const int M2_ENC_B = 12;  // PB4 (PCINT4)  - ANTES era D5, ahora D12

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

// PCINT M2
volatile uint8_t m2_prevAB = 0;

// Setpoints independientes
float target1Deg = 0.0f;
float target2Deg = 0.0f;

unsigned long tPrev = 0;

// ===== Utils =====
inline float wrap360(float a){ while(a>=360)a-=360; while(a<0)a+=360; return a; }
inline float cnt2deg(long c){ return wrap360((float)c * 360.0f / CPR); }
inline float angErr(float tgt,float act){
  float e=tgt-act; while(e>180)e-=360; while(e<-180)e+=360; return e;
}

// ===== ISRs M1 (externas) =====
void isr1A(){ bool a=digitalRead(M1_ENC_A),b=digitalRead(M1_ENC_B); enc1Count += (a==b)? SIGN_DIR_M1 : -SIGN_DIR_M1; }
void isr1B(){ bool a=digitalRead(M1_ENC_A),b=digitalRead(M1_ENC_B); enc1Count += (a!=b)? SIGN_DIR_M1 : -SIGN_DIR_M1; }

// ===== LUT cuadratura (común) =====
const int8_t qdecLUT[16] = {
  0,-1,+1, 0,
 +1, 0, 0,-1,
 -1, 0, 0,+1,
  0,+1,-1, 0
};

// ===== Handler común M2: lee A(D4) y B(D12) y acumula =====
inline void m2_pcint_update(){
  uint8_t a = (PIND >> 4) & 0x01;  // D4 (PORTD4)
  uint8_t b = (PINB >> 4) & 0x01;  // D12 (PORTB4)
  uint8_t curr = (b<<1) | a;
  uint8_t idx  = ((m2_prevAB & 0x03) << 2) | (curr & 0x03);
  int8_t step  = qdecLUT[idx];
  if (step != 0) enc2Count += (long)(step * SIGN_DIR_M2);
  m2_prevAB = curr;
}

// ===== PCINT para M2: PORTD (A) y PORTB (B) =====
ISR(PCINT2_vect){ // cambios en PORTD (incluye D4)
  m2_pcint_update();
}
ISR(PCINT0_vect){ // cambios en PORTB (incluye D12)
  m2_pcint_update();
}

// ===== Control genérico =====
void controlUno(long encNow, long &encPrev, float &integ, float &wFilt, float &pwmFiltered,
                int pinDir, int pinPwm, float targetDeg)
{
  float ang = cnt2deg(encNow);
  float e   = angErr(targetDeg, ang);

  long dCounts = encNow - encPrev;
  float w = ( (float)dCounts * 360.0f / CPR ) / Ts;
  wFilt = ALPHA_D*w + (1.0f-ALPHA_D)*wFilt;

  if (fabs(e) < DEADZONE_DEG){
    integ = 0.0f; pwmFiltered = 0.0f; analogWrite(pinPwm, 0);
  } else {
    integ += Ki*Ts*e;
    if (integ >  INTEG_LIM) integ =  INTEG_LIM;
    if (integ < -INTEG_LIM) integ = -INTEG_LIM;

    float u = Kp*e + integ - Kd*wFilt;

    float boost = 0.0f;
    float ae = fabs(e);
    if (ae > DEADZONE_DEG){
      float k = ae >= BOOST_OFF_DEG ? 1.0f : (ae - DEADZONE_DEG) / (BOOST_OFF_DEG - DEADZONE_DEG);
      if (k < 0) k = 0;
      boost = PWM_STATIC * k;
    }

    int pwm = (int)(fabs(u) + boost);
    if (pwm > PWM_MAX) pwm = PWM_MAX;
    if (pwm < PWM_MIN) pwm = PWM_MIN;

    pwmFiltered = ALPHA_PWM*pwm + (1.0f-ALPHA_PWM)*pwmFiltered;

    digitalWrite(pinDir, (u>=0)?HIGH:LOW);
    analogWrite(pinPwm, (int)pwmFiltered);
  }

  encPrev = encNow;
}

// ===== Serial =====
void leerComandos(){
  static String buf;
  while(Serial.available()){
    char c=Serial.read();
    if(c=='\r') continue;
    if(c=='\n'){ buf.trim(); if(buf.length()) procesarLinea(buf); buf=""; }
    else buf+=c;
  }
}

bool parse2floats(const String& s, float &a, float &b){
  int sp = s.indexOf(' ');
  if (sp<0) return false;
  int sp2 = s.indexOf(' ', sp+1);
  if (sp2<0) return false;
  a = s.substring(sp+1, sp2).toFloat();
  b = s.substring(sp2+1).toFloat();
  return true;
}

void procesarLinea(const String& s){
  if(s=="?"){
    Serial.print(F("M1 ang=")); Serial.print(cnt2deg(enc1Count),2);
    Serial.print(F("  M2 ang=")); Serial.print(cnt2deg(enc2Count),2);
    Serial.print(F("  tgt1="));  Serial.print(target1Deg,1);
    Serial.print(F("  tgt2="));  Serial.print(target2Deg,1);
    Serial.print(F("  CPR="));   Serial.println(CPR,0);
    return;
  }
  if(s=="z"||s=="Z"){
    noInterrupts(); enc1Count=0; enc2Count=0; interrupts();
    target1Deg=0; target2Deg=0;
    integ1=integ2=0; pwmF1=pwmF2=0; wFilt1=wFilt2=0;
    Serial.println(F(">> Cero ambos y tgt=0°"));
    return;
  }
  if(s=="z1"||s=="Z1"){
    noInterrupts(); enc1Count=0; interrupts();
    target1Deg=0; integ1=0; pwmF1=0; wFilt1=0;
    Serial.println(F(">> Cero M1 y tgt1=0°"));
    return;
  }
  if(s=="z2"||s=="Z2"){
    noInterrupts(); enc2Count=0; interrupts();
    target2Deg=0; integ2=0; pwmF2=0; wFilt2=0;
    Serial.println(F(">> Cero M2 y tgt2=0°"));
    return;
  }
  if(s=="s"||s=="S"){
    target1Deg = wrap360(cnt2deg(enc1Count));
    target2Deg = wrap360(cnt2deg(enc2Count));
    integ1=integ2=0; pwmF1=pwmF2=0;
    analogWrite(M1_PWM,0); analogWrite(M2_PWM,0);
    Serial.println(F(">> Parados en su ángulo actual (ambos)"));
    return;
  }
  if(s=="s1"||s=="S1"){
    target1Deg = wrap360(cnt2deg(enc1Count));
    integ1=0; pwmF1=0; analogWrite(M1_PWM,0);
    Serial.println(F(">> M1 parado en su ángulo actual"));
    return;
  }
  if(s=="s2"||s=="S2"){
    target2Deg = wrap360(cnt2deg(enc2Count));
    integ2=0; pwmF2=0; analogWrite(M2_PWM,0);
    Serial.println(F(">> M2 parado en su ángulo actual"));
    return;
  }
  if(s.startsWith("m1 ")){
    float v = s.substring(3).toFloat();
    if(v>=0 && v<=360){ target1Deg=wrap360(v); Serial.print(F(">> tgt1=")); Serial.print(target1Deg,1); Serial.println(F("°")); }
    else Serial.println(F("m1 <0..360>"));
    return;
  }
  if(s.startsWith("m2 ")){
    float v = s.substring(3).toFloat();
    if(v>=0 && v<=360){ target2Deg=wrap360(v); Serial.print(F(">> tgt2=")); Serial.print(target2Deg,1); Serial.println(F("°")); }
    else Serial.println(F("m2 <0..360>"));
    return;
  }
  if(s.startsWith("m ")){
    float a,b;
    if(parse2floats(s, a, b) && a>=0 && a<=360 && b>=0 && b<=360){
      target1Deg=wrap360(a); target2Deg=wrap360(b);
      Serial.print(F(">> tgt1=")); Serial.print(target1Deg,1);
      Serial.print(F("°  tgt2=")); Serial.print(target2Deg,1); Serial.println(F("°"));
    } else {
      Serial.println(F("m <deg1 0..360> <deg2 0..360>"));
    }
    return;
  }

  Serial.println(F("Comando invalido. Usa ?: z/z1/z2, s/s1/s2, m1 <deg>, m2 <deg>, m <d1> <d2>"));
}

// ===== Setup / Loop =====
void setup(){
  Serial.begin(115200);

  pinMode(M1_DIR,OUTPUT); pinMode(M1_PWM,OUTPUT);
  pinMode(M2_DIR,OUTPUT); pinMode(M2_PWM,OUTPUT);
  analogWrite(M1_PWM,0); digitalWrite(M1_DIR,LOW);
  analogWrite(M2_PWM,0); digitalWrite(M2_DIR,LOW);

  // Encoders M1 (externas)
  pinMode(M1_ENC_A,INPUT); pinMode(M1_ENC_B,INPUT);
  attachInterrupt(digitalPinToInterrupt(M1_ENC_A), isr1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M1_ENC_B), isr1B, CHANGE);

  // Encoders M2 (PCINT en D4 y D12)
  pinMode(M2_ENC_A,INPUT);
  pinMode(M2_ENC_B,INPUT);

  // Estado inicial de A y B
  uint8_t a0 = (PIND >> 4) & 0x01;   // D4
  uint8_t b0 = (PINB >> 4) & 0x01;   // D12
  m2_prevAB = (b0<<1) | a0;

  // Habilitar interrupciones por cambio de pin para PORTD (A) y PORTB (B)
  PCICR  |= (1<<PCIE2) | (1<<PCIE0); // PORTD y PORTB
  PCMSK2 |= (1<<PCINT20);            // D4 (PD4)
  PCMSK0 |= (1<<PCINT4);             // D12 (PB4)

  noInterrupts(); enc1Count=0; enc2Count=0; interrupts();

  target1Deg=0; target2Deg=0;
  integ1=integ2=0; pwmF1=pwmF2=0; wFilt1=wFilt2=0;
  enc1Prev=enc1Count; enc2Prev=enc2Count;
  tPrev=millis();

  Serial.println(F("PID x2 (setpoints separados) listo ✅  (M2_ENC_B=D12)  ?: ayuda"));
}

void loop(){
  leerComandos();

  unsigned long t=millis();
  if(t - tPrev >= (unsigned long)(Ts*1000)){
    tPrev=t;

    long enc1Now = enc1Count;
    long enc2Now = enc2Count;

    controlUno(enc1Now, enc1Prev, integ1, wFilt1, pwmF1, M1_DIR, M1_PWM, target1Deg);
    controlUno(enc2Now, enc2Prev, integ2, wFilt2, pwmF2, M2_DIR, M2_PWM, target2Deg);
  }

  static unsigned long tlog=0;
  if(millis()-tlog >= 300){
    tlog=millis();
    Serial.print(F("M1 ang=")); Serial.print(cnt2deg(enc1Count),1);
    Serial.print(F(" (tgt="));  Serial.print(target1Deg,1); Serial.print(F(")"));
    Serial.print(F("  M2 ang=")); Serial.print(cnt2deg(enc2Count),1);
    Serial.print(F(" (tgt="));  Serial.print(target2Deg,1); Serial.print(F(")"));
    Serial.print(F("  pwm1="));  Serial.print(pwmF1,0);
    Serial.print(F("  pwm2="));  Serial.println(pwmF2,0);
  }
}
