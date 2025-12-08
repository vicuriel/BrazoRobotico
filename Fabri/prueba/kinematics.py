import math
import numpy as np

# En cm
l1 = 10.1    # Longitud efectiva del primer eslabón  
l2 = 6.929   # Longitud efectiva del segundo eslabón
l3 = 14.038  # Longitud efectiva del tercer eslabón
l4 = 8.853   # Longitud del efector
a = 12       # Distancia vertical entre S0 y S1
b = 12       # Distancia vertical entre S1 y S2
c = 1.65     # Distancia horizontal entre S1 y S2 (direccion y0)
d = 1.626    # Distancia horizontal entre S2 y S3 (direccion y0)
e = 1.532    # Distancia horizontal entre S2 y S3 (direccion x0)

# DH parametros
# Si-1 -> Si |          thetai         |    di   |         ai       | alphai
#-----------------------------------------------------------------------------
#    0 -> 1  |            q1           |    a    |         l1       |   0
#    1 -> 2  |     q2 + atan(c/l2)     |    b    | (l2^2 + c^2)^1/2 |   pi
#    2 -> 3  | atan(c/l2) + atana(d/e) | q3 + l3 |  (d^2 + e^2)^1/2 |   0
#    3 -> 4  |            q4           |    l4   |          0       |   0

def fk(q1_deg, q2_deg, q3, q4_deg):
    """
    Cinemática directa del robot SCARA.
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
    A34 = np.array([[math.cos(q4), -math.sin(q4), 0, 0],
                    [math.sin(q4),  math.cos(q4), 0, 0],
                    [     0,              0,      1, l4],
                    [     0,              0,      0, 1]])
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

def ik(x, y, z, phi_deg):
    """
    Cinemática inversa del robot SCARA.
    - x (float): posición x del efector en cm.
    - y (float): posición y del efector en cm.
    - z (float): posición z del efector en cm.
    - phi_deg (float): orientación `phi` del efector en grados.

    Retorna:
    - (q1_deg, q2_deg, q3, q4_deg): ángulos de las articulaciones en grados y desplazamiento lineal en cm.
    Ejemplo:
    >>> ik(18.561, 0.024, 1.109, -46.705)
    (0.0, 0.0, 0.0, 0.0)
    """
    phi = math.radians(phi_deg)

    A = l2 + e
    B = c - d
    R = math.hypot(A, B)

    sols = []

    # comprobar divisiones entre cero
    if R == 0:
        return sols

    K = (x**2 + y**2 - l1**2 - A**2 - B**2) / (2 * l1)
    val = K / R

    # val = K/R -> acos(val). Si |val| > 1, la localizacion está fuera del alcance en xy.
    # Permitir pequeña tolerancia numérica y redondear a [-1, 1] si esta se excede ligeramente.
    if abs(val) > 1.0 + 1e-9:
        return sols
    val = max(-1.0, min(1.0, val))

    delta = math.atan2(B, A)
    ang = math.acos(val)

    q2_candidatos = [delta + ang, delta - ang]

    # orientación de referencia con todas las juntas cero (para calcular q4)
    phi_offset = fk(0, 0, 0, 0)[3]

    for q2 in q2_candidatos:
        C = l1 + A * math.cos(q2) + B * math.sin(q2)
        D = A * math.sin(q2) - B * math.cos(q2)
        z0 = a + b - l3 - l4

        q1 = math.atan2(y, x) - math.atan2(D, C)

        q1_deg = math.degrees(q1)
        q2_deg = math.degrees(q2)
        q3_lineal = z0 - z
        q4_deg = q1_deg + q2_deg - (phi_deg - phi_offset)

        # normalizar ángulos a [-180, 180)
        def norm(a):
            return (a + 180.0) % 360.0 - 180.0

        # q3 en cm (desplazamiento lineal, no ángulo de servo)
        sols.append((norm(q1_deg), norm(q2_deg), float(q3_lineal), norm(q4_deg)))

    return sols

if __name__ == "__main__":
    print_fk(30, 30, 3, 90)
    ik_sols = ik(12.957, 12.389, -1.891, -76.705)
    for sol in ik_sols:
        print(f"IK sol: q1={sol[0]:.3f}°, q2={sol[1]:.3f}°, q3={sol[2]:.3f} cm, q4={sol[3]:.3f}°")