import math
import numpy as np
from typing import List, Tuple, Optional

# En cm
l1 = 9.97    # Longitud efectiva del primer eslabón  
l2 = 7.16    # Longitud efectiva del segundo eslabón
l3 = 14.46   # Longitud efectiva del tercer eslabón
l4 = 8.22    # Longitud del efector
a = 11.55    # Distancia vertical entre S0 y S1
b = 14.3     # Distancia vertical entre S1 y S2
c = 1.96     # Distancia horizontal entre S1 y S2 (direccion y0)
d = 1.55     # Distancia horizontal entre S2 y S3 (direccion y0)
e = 1.47     # Distancia horizontal entre S2 y S3 (direccion x0)

# DH parametros
# Si-1 -> Si |          thetai         |    di   |         ai       | alphai
#-----------------------------------------------------------------------------
#    0 -> 1  |            q1           |    a    |         l1       |   0
#    1 -> 2  |     q2 + atan(c/l2)     |    b    | (l2^2 + c^2)^1/2 |   pi
#    2 -> 3  | atan(c/l2) + atana(d/e) | q3 + l3 |  (d^2 + e^2)^1/2 |   0
#    3 -> 4  |  q4 + pi/2 - atan(d/e)  |    l4   |          0       |   0

def fk(q1_deg, q2_deg, q3, q4_deg):
    """
    Cinemática directa
    - q1_deg (float): rotación de la base en grados (sentido positivo antihorario).
    - q2_deg (float): rotación del segundo eje en grados.
    - q3 (float): desplazamiento lineal del tercer eslabón (en cm).
    - q4_deg (float): rotación final del efector en grados.

    Retorna:
    - (x, y, z, phi): posición (cm) y orientación `phi` en grados del efector respecto al marco base.
    Ejemplo:
    >>> fk(0, 0, 0, 0)
    (18.561, 0.024, 1.109, -46.705)
    """

    q1 = math.radians(q1_deg)
    q2 = math.radians(q2_deg)
    q4 = math.radians(q4_deg)

    k2 = math.atan2(c, l2)
    k3 = math.atan2(d, e)
    tita2 = q2 + k2 
    tita3 = k2 + k3
    tita4 = q4 + math.pi/2 - k3

    A01 = np.array([[math.cos(q1), -math.sin(q1), 0, l1*math.cos(q1)],
                    [math.sin(q1),  math.cos(q1), 0, l1*math.sin(q1)],
                    [      0,             0,      1,        a       ],
                    [      0,             0,      0,        1       ]])
    A12 = np.array([[math.cos(tita2),  math.sin(tita2), 0, math.cos(tita2)*math.sqrt(l2**2 + c**2)],
                    [math.sin(tita2), -math.cos(tita2), 0, math.sin(tita2)*math.sqrt(l2**2 + c**2)],
                    [      0,                0,        -1,                b                       ],
                    [      0,                0,         0,                1                       ]])
    A23 = np.array([[math.cos(tita3), -math.sin(tita3), 0, math.cos(tita3)*math.sqrt(d**2 + e**2)],
                    [math.sin(tita3),  math.cos(tita3), 0, math.sin(tita3)*math.sqrt(d**2 + e**2)],
                    [      0,                0,         1,             q3 + l3                   ],
                    [      0,                0,         0,               1                       ]])
    A34 = np.array([[math.cos(tita4), -math.sin(tita4), 0,  0],
                    [math.sin(tita4),  math.cos(tita4), 0,  0],
                    [     0,                 0,         1, l4],
                    [     0,                 0,         0,  1]])
    T04 = A01 @ A12 @ A23 @ A34
    R = T04[:3, :3]
    x = T04[0, 3]
    y = T04[1, 3]
    z = T04[2, 3]
    phi = math.atan2(R[1, 0], R[0, 0])

    return x, y, z, math.degrees(phi)

def print_fk(q1, q2, q3, q4):
    x, y, z, phi = fk(q1, q2, q3, q4)
    print(f"FK -> x: {x:.3f} cm, y: {y:.3f} cm, z: {z:.3f} cm, phi: {phi:.3f}°")


def ik(x, y, z, phi_deg, q_prev: Optional[List[float]] = None, limits: Optional[dict] = None, codo_estandar: Optional[str] = None):
    """
    Cinemática inversa
        - x, y, z: posición extremo en cm
        - phi_deg: orientación extremo en °
        - q_prev: vector articular previo [q1,q2,q3,q4]
        - limits: límites articulares
        - codo_estandar: 'up'|'down' preferencia de codo

    Retorna:
        (q1_deg, q2_deg, q3_cm, q4_deg) con la mejor solución seleccionada
        por `choose_solution`, o `None` si no hay solución válida.
    """

    A = l2 + e
    B = c - d
    R = math.hypot(A, B)

    sols = []

    # comprobar divisiones entre cero
    if R == 0:
        return sols

    K = (x**2 + y**2 - l1**2 - A**2 - B**2) / (2 * l1)
    val = K / R

    # val = K/R -> acos(val).
    # Si |val| > 1, la pose está fuera del alcance (no puede alcanzar esa distancia xy).
    # Permitir pequeña tolerancia numérica y redondear a [-1, 1] si esta se excede ligeramente.
    if abs(val) > 1.0 + 1e-9:
        return sols
    val = max(-1.0, min(1.0, val))

    delta = math.atan2(B, A)
    ang = math.acos(val)

    q2_candidatos = [delta + ang, delta - ang]

    for q2 in q2_candidatos:
        C = l1 + A * math.cos(q2) + B * math.sin(q2)
        D = A * math.sin(q2) - B * math.cos(q2)
        z0 = a + b - l3 - l4

        q1 = math.atan2(y, x) - math.atan2(D, C)

        q1_deg = math.degrees(q1)
        q2_deg = math.degrees(q2)
        q3_lineal = z0 - z
        q4_deg = q1_deg + q2_deg - phi_deg - 90.0

        # normalizar ángulos a [-180, 180]
        def norm(a):
            return (a + 180.0) % 360.0 - 180.0

        # q3 en cm (desplazamiento lineal, no ángulo de servo)
        sols.append((norm(q1_deg), norm(q2_deg), float(q3_lineal), norm(q4_deg)))

    if q_prev is None:
        q_prev = [0.0, 0.0, 0.0, 0.0]

    q_optima = choose_solution(sols, q_prev=q_prev, limits=limits, codo_estandar=codo_estandar)
    return q_optima

def choose_solution(sols: List[Tuple[float,float,float,float]], q_prev: List[float], limits: Optional[dict]=None, codo_estandar: Optional[str]=None) -> Optional[Tuple[float,float,float,float]]:
    """
    Selecciona la mejor solución entre candidatos IK.
    Segun (en orden de importancia):
    - Los limites de los GDL
    - Cambio articular total minimo desde q_prev
    - Penalizar soluciones cercanas a los limites articulares

    Parametros:
        sols: lista de candidatas (q1,q2,q3,q4)
        q_prev: vector articular previo [q1,q2,q3,q4]
        limits: {'q1':(min,max), ...}
        codo_estandar: opcional, 'up'|'down' o None. Si se establece, la función
            preferirá soluciones con la configuración de codo indicada
            (heurística basada en el signo/valor de q2).

    Retorna:
        Mejor solución tupla o None si no hay válida
    """
    if not sols:
        return None

    def in_limits(q):
        if not limits:
            return True
        keys = ['q1','q2','q3','q4']
        for val, k in zip(q, keys):
            if k in limits:
                mn, mx = limits[k]
                if val < mn - 1e-9 or val > mx + 1e-9:
                    return False
        return True

    candidatos = [tuple(map(float, s)) for s in sols if in_limits(s)]
    if not candidatos:
        return None
    q_optima = None
    mejor_score = float('inf')

    for q in candidatos:
        costo_cambio = sum(abs(qi - pi) for qi, pi in zip(q, q_prev))

        penalizacion = 0.0
        if limits:
            keys = ['q1','q2','q3','q4']
            for qi, k in zip(q, keys):
                if k in limits:
                    mn, mx = limits[k]
                    span = mx - mn
                    if span > 0:
                        dist = min(qi - mn, mx - qi)
                        threshold = 0.1 * span
                        if dist < threshold:
                            penalizacion += (threshold - dist) / threshold

        puntaje = costo_cambio + 10.0 * penalizacion

        if codo_estandar in ('up', 'down'):
            q2_deg = q[1]
            is_down = (q2_deg >= 0.0)
            matches = (is_down and codo_estandar == 'down') or (not is_down and codo_estandar == 'up')
            elbow_bias = 8.0
            if not matches:
                puntaje += elbow_bias
        if puntaje < mejor_score:
            mejor_score = puntaje
            q_optima = q

    return q_optima

if __name__ == "__main__":
    pass