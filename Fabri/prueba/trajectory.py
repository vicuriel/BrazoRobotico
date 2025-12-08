import numpy as np

def interpolate_joint_space(q_i, q_f, T = 2.0, dt = 0.02):
    """
    Interpolación lineal por juntas.
    q_i: array con [q1, q2, q3, q4] iniciales
    q_f: array con [q1, q2, q3, q4] finales
    T: duración total en segundos
    dt: tiempo entre puntos (aprox 20 ms)

    Retorna:
        lista de q(t) intermedios
    """

    q_i = np.array(q_i, dtype = float)
    q_f = np.array(q_f, dtype = float)

    pasos = int(T / dt)             # número de pasos
    traj = []                       # lista de puntos de la trayectoria

    for i in range(pasos + 1):
        s = i / pasos               # discretización [0, 1] (inicio / final)
        q = q_i + s * (q_f - q_i)   # Ecuacion de interpolación lineal
        traj.append(q.tolist())     # Agregar punto a la trayectoria

    return traj
