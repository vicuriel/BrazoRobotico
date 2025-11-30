/* PID de posición 0..360° para MD13S + 5840WG-555PM-EN (encoder 11 PPR)
   Versión: ajuste de llegada precisa con compensación de fricción
   Serial 115200: 0..360 | z | s | ?
*/

const int PIN_DIR   = 8;
const int PIN_PWM   = 9;
const int PIN_ENC_A = 2;   // INT0
const int PIN_ENC_B = 3;   // INT1

// ===== Mecánico / Encoder =====
const float PPR_MOTOR = 11.0f;
// Si comprobaste otra relación, cámbiala aquí:
const float GEAR_RATIO = 100.0f;
const float CPR = PPR_MOTOR * GEAR_RATIO * 4.0f;

const int SIGN_DIR = -1;

// ===== PID (AJUSTADOS) =====
float Kp = 1.35f;    // ★ CHANGED (antes 1.0) más “firme” sin pasarse
float Ki = 0.80f;    // ★ CHANGED (antes 0.5) quita error residual
float Kd = 0.22f;    // ★ CHANGED (antes 0.15) freno suave

float Ts = 0.01f;    // 10 ms
float integ = 0.0f;
const float INTEG_LIM = 150.0f;  // ★ CHANGED (antes 120) más margen

// ===== Límites / zona muerta =====
const float DEADZONE_DEG = 0.8f;  // ★ CHANGED (antes 1.2) mayor precisión final
const int   PWM_MAX = 140;        // ★ CHANGED (antes 120) un poco más de “pulmón”
const int   PWM_MIN = 45;         // ★ CHANGED (antes 35) vence fricción

// ===== Compensación de fricción (NUEVO) =====
// Cuando hay error, añadimos un “offset” que se va apagando cerca del objetivo
const int   PWM_STATIC = 18;      // ★ NEW empujón fijo para arrancar
const float BOOST_OFF_DEG = 12.0; // ★ NEW por encima de este error aplica todo el boost

// Filtros
float pwmFiltered = 0.0f;
const float ALPHA_PWM = 0.25f;

float wFilt = 0.0f;
const float ALPHA_D = 0.25f;

volatile long encCount = 0;
long   encPrev = 0;
float  targetDeg = 0.0f;
unsigned long tPrev = 0;

// ===== Utilidades =====
inline float wrap360(float a){ while(a>=360)a-=360; while(a<0)a+=360; return a; }
inline float cnt2deg(long c){ return wrap360((float)c * 360.0f / CPR); }
inline float angErr(float tgt,float act){
  float e=tgt-act; while(e>180)e-=360; while(e<-180)e+=360; return e;
}

// ===== ISRs =====
void isrA(){ bool a=digitalRead(PIN_ENC_A),b=digitalRead(PIN_ENC_B); encCount += (a==b)? SIGN_DIR : -SIGN_DIR; }
void isrB(){ bool a=digitalRead(PIN_ENC_A),b=digitalRead(PIN_ENC_B); encCount += (a!=b)? SIGN_DIR : -SIGN_DIR; }

void setup(){
  Serial.begin(115200);
  pinMode(PIN_DIR,OUTPUT); pinMode(PIN_PWM,OUTPUT);
  pinMode(PIN_ENC_A,INPUT); pinMode(PIN_ENC_B,INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A),isrA,CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B),isrB,CHANGE);

  analogWrite(PIN_PWM,0); digitalWrite(PIN_DIR,LOW);
  noInterrupts(); encCount=0; interrupts();
  targetDeg=0; integ=0; pwmFiltered=0; wFilt=0; encPrev=encCount; tPrev=millis();

  Serial.println(F("PID ajustado listo ✅ 0..360 | z | s | ?"));
}

void loop(){
  leerComandos();

  unsigned long t=millis();
  if(t - tPrev >= (unsigned long)(Ts*1000)){
    tPrev=t;

    long  encNow = encCount;
    float ang    = cnt2deg(encNow);
    float e      = angErr(targetDeg, ang);

    // Velocidad angular (deg/s) + filtro
    long dCounts = encNow - encPrev;
    float w = ( (float)dCounts * 360.0f / CPR ) / Ts;
    wFilt = ALPHA_D*w + (1.0f-ALPHA_D)*wFilt;

    if (fabs(e) < DEADZONE_DEG){
      // Dentro de la ventana de precisión: paro y limpio integral
      integ = 0.0f;
      pwmFiltered = 0.0f;
      analogWrite(PIN_PWM, 0);
    } else {
      // PI con anti-windup (clamping)
      integ += Ki*Ts*e;
      if (integ >  INTEG_LIM) integ =  INTEG_LIM;
      if (integ < -INTEG_LIM) integ = -INTEG_LIM;

      // PID (derivada sobre la medida)
      float u = Kp*e + integ - Kd*wFilt;

      // ===== Compensación de fricción/estática (★ NEW) =====
      // Aplicamos un offset que disminuye linealmente a medida que |e| -> 0
      float boost = 0.0f;
      float ae = fabs(e);
      if (ae > DEADZONE_DEG){
        float k = ae >= BOOST_OFF_DEG ? 1.0f : (ae - DEADZONE_DEG) / (BOOST_OFF_DEG - DEADZONE_DEG);
        if (k < 0) k = 0;
        boost = PWM_STATIC * k;
      }

      // PWM y dirección
      int pwm = (int)(fabs(u) + boost);
      if (pwm > PWM_MAX) pwm = PWM_MAX;
      if (pwm < PWM_MIN) pwm = PWM_MIN;

      // Suavizado
      pwmFiltered = ALPHA_PWM*pwm + (1.0f-ALPHA_PWM)*pwmFiltered;

      digitalWrite(PIN_DIR, (u>=0)?HIGH:LOW);
      analogWrite(PIN_PWM, (int)pwmFiltered);
    }

    encPrev = encNow;
  }

  // Telemetría
  static unsigned long tlog=0;
  if(millis()-tlog >= 300){
    tlog=millis();
    Serial.print(F("ang=")); Serial.print(cnt2deg(encCount),1);
    Serial.print(F("  tgt=")); Serial.print(targetDeg,1);
    Serial.print(F("  pwmF=")); Serial.print(pwmFiltered,0);
    Serial.println();
  }
}

void leerComandos(){
  static String buf;
  while(Serial.available()){
    char c=Serial.read();
    if(c=='\r') continue;
    if(c=='\n'){ buf.trim(); if(buf.length()) procesarLinea(buf); buf=""; }
    else buf+=c;
  }
}

void procesarLinea(const String& s){
  if(s=="z"||s=="Z"){
    noInterrupts(); encCount=0; interrupts();
    targetDeg=0; integ=0; pwmFiltered=0; wFilt=0;
    Serial.println(F(">> Cero puesto (0°)"));
    return;
  }
  if(s=="s"||s=="S"){
    targetDeg=cnt2deg(encCount);
    integ=0; pwmFiltered=0; analogWrite(PIN_PWM,0);
    Serial.println(F(">> Parado en angulo actual"));
    return;
  }
  if(s=="?"){
    Serial.print(F("ang=")); Serial.print(cnt2deg(encCount),2);
    Serial.print(F(" tgt=")); Serial.print(targetDeg,2);
    Serial.print(F(" CPR=")); Serial.println(CPR,0);
    return;
  }
  float v=s.toFloat();
  if(v>=0.0f && v<=360.0f){
    targetDeg=wrap360(v);
    Serial.print(F(">> Nuevo setpoint: ")); Serial.print(targetDeg,1); Serial.println(F("°"));
  } else {
    Serial.println(F("Comando invalido. Usa 0..360, o z / s / ?"));
  }
}
