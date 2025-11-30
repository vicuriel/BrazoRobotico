import RPi.GPIO as GPIO
import time
import math

# ===== Configuración de Pines =====
PIN_DIR = 17    # Dirección (ejemplo de pin GPIO)
PIN_PWM = 18    # PWM (ejemplo de pin GPIO)
PIN_ENC_A = 23  # A del encoder (ejemplo de pin GPIO)
PIN_ENC_B = 24  # B del encoder (ejemplo de pin GPIO)

# ===== Mecánico / Encoder =====
PPR_MOTOR = 11.0
GEAR_RATIO = 110.0
CPR = PPR_MOTOR * GEAR_RATIO * 4.0

SIGN_DIR = -1

# ===== PID =====
Kp = 1.35
Ki = 0.80
Kd = 0.22
Ts = 0.01
integ = 0.0
INTEG_LIM = 150.0

DEADZONE_DEG = 0.8
PWM_MAX = 140
PWM_MIN = 45

PWM_STATIC = 18
BOOST_OFF_DEG = 12.0

# Filtros
pwmFiltered = 0.0
ALPHA_PWM = 0.25
wFilt = 0.0
ALPHA_D = 0.25

encCount = 0
encPrev = 0
targetDeg = 0.0
tPrev = 0

# ===== Funciones Utilitarias =====
def wrap360(a):
    while a >= 360:
        a -= 360
    while a < 0:
        a += 360
    return a

def cnt2deg(c):
    return wrap360((float(c) * 360.0) / CPR)

def angErr(tgt, act):
    e = tgt - act
    while e > 180:
        e -= 360
    while e < -180:
        e += 360
    return e

# ===== Interrupciones del Encoder =====
def isrA(channel):
    global encCount
    a = GPIO.input(PIN_ENC_A)
    b = GPIO.input(PIN_ENC_B)
    encCount += SIGN_DIR if a == b else -SIGN_DIR

def isrB(channel):
    global encCount
    a = GPIO.input(PIN_ENC_A)
    b = GPIO.input(PIN_ENC_B)
    encCount += SIGN_DIR if a != b else -SIGN_DIR

# ===== Configuración del GPIO =====
GPIO.setmode(GPIO.BCM)  # Usamos numeración BCM de pines
GPIO.setup(PIN_DIR, GPIO.OUT)
GPIO.setup(PIN_PWM, GPIO.OUT)
GPIO.setup(PIN_ENC_A, GPIO.IN)
GPIO.setup(PIN_ENC_B, GPIO.IN)

# Configuración de interrupciones para el encoder
GPIO.add_event_detect(PIN_ENC_A, GPIO.BOTH, callback=isrA)
GPIO.add_event_detect(PIN_ENC_B, GPIO.BOTH, callback=isrB)

# Función para controlar el motor
def control_motor():
    global encPrev, targetDeg, integ, pwmFiltered, wFilt

    t = time.time()
    if t - tPrev >= Ts:
        tPrev = t
        encNow = encCount
        ang = cnt2deg(encNow)
        e = angErr(targetDeg, ang)

        # Velocidad angular (deg/s) + filtro
        dCounts = encNow - encPrev
        w = ((float(dCounts) * 360.0) / CPR) / Ts
        wFilt = ALPHA_D * w + (1.0 - ALPHA_D) * wFilt

        if abs(e) < DEADZONE_DEG:
            integ = 0.0
            pwmFiltered = 0.0
            GPIO.output(PIN_DIR, GPIO.LOW)
            GPIO.output(PIN_PWM, 0)
        else:
            # PI con anti-windup (clamping)
            integ += Ki * Ts * e
            if integ > INTEG_LIM:
                integ = INTEG_LIM
            if integ < -INTEG_LIM:
                integ = -INTEG_LIM

            # PID (derivada sobre la medida)
            u = Kp * e + integ - Kd * wFilt

            # Compensación de fricción/estática
            boost = 0.0
            ae = abs(e)
            if ae > DEADZONE_DEG:
                k = 1.0 if ae >= BOOST_OFF_DEG else (ae - DEADZONE_DEG) / (BOOST_OFF_DEG - DEADZONE_DEG)
                if k < 0:
                    k = 0
                boost = PWM_STATIC * k

            pwm = int(abs(u) + boost)
            if pwm > PWM_MAX:
                pwm = PWM_MAX
            if pwm < PWM_MIN:
                pwm = PWM_MIN

            # Suavizado
            pwmFiltered = ALPHA_PWM * pwm + (1.0 - ALPHA_PWM) * pwmFiltered

            GPIO.output(PIN_DIR, GPIO.HIGH if u >= 0 else GPIO.LOW)
            GPIO.output(PIN_PWM, pwmFiltered)

        encPrev = encNow

# Función para leer comandos desde la consola (serial)
def leer_comandos():
    global targetDeg
    while True:
        comando = input()
        if comando == 'z' or comando == 'Z':
            encCount = 0
            targetDeg = 0
            integ = 0
            pwmFiltered = 0
            wFilt = 0
            print(">> Cero puesto (0°)")
        elif comando == 's' or comando == 'S':
            targetDeg = cnt2deg(encCount)
            integ = 0
            pwmFiltered = 0
            print(">> Parado en angulo actual")
        elif comando == '?':
            print(f"ang={cnt2deg(encCount)}  tgt={targetDeg}")
        else:
            try:
                v = float(comando)
                if 0.0 <= v <= 360.0:
                    targetDeg = wrap360(v)
                    print(f">> Nuevo setpoint: {targetDeg}°")
                else:
                    print("Comando invalido. Usa 0..360, o z / s / ?")
            except ValueError:
                print("Comando invalido. Usa 0..360, o z / s / ?")

# ===== Main Loop =====
if _name_ == "_main_":
    print("PID ajustado listo ✅ 0..360 | z | s | ?")
    while True:
        control_motor()
        leer_comandos()
