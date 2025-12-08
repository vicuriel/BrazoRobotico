import time
from kinematics import ik
from trajectory import interpolate_joint_space
from comms import RobotComm
from ik_utils import choose_solution
from config_limits import LIMITS

Q3_SERVO_MAX = 4.39  # cm máximo

def servo_angle_to_linear(q3_servo_deg):
    return Q3_SERVO_MAX * (1.0 - q3_servo_deg / 180.0)

def linear_to_servo_angle(q3_linear_cm):
    return (1.0 - q3_linear_cm / Q3_SERVO_MAX) * 180.0

def get_initial_joint(comm):
    """
    Obtiene articulaciones actuales usando una conexion existente.
    Retorna [q1, q2, q3_lineal, q4] donde q3 es desplazamiento en cm.
    """
    q = comm.request_state()
    return q

def move_to(x, y, z, phi, comm, T = 2.0, dt = 0.02):
    """
    Mueve el robot a una localizacion cartesiana.
    
    Parámetros:
        x, y, z, phi: pose cartesiana destino
        comm: objeto RobotComm ya conectado al ESP32
        T: duración de la trayectoria [seg]
        dt: tiempo entre puntos [seg]
    """
    # 1) Lista de soluciones articulares IK 
    # 2) Obtener articulaciones actuales mediante ESP32
    # 3) Encontrar mejor solución IK dentro de límites y articulaciones actuales
    # 4) Generar trayectoria en espacio articular
    # 5) Enviar la trayectoria punto a punto usando la conexión existente

    sols = ik(x, y, z, phi)
    if not sols:
        raise ValueError(f"La localizacion ({x:.2f}, {y:.2f}, {z:.2f}, {phi:.2f}) no es alcanzable.")

    q_current = get_initial_joint(comm)
    q_target = choose_solution(sols, q_current, limits = LIMITS)
    if q_target is None:
        raise ValueError("No hay una solución IK válida dentro de los límites articulares.")

    traj = interpolate_joint_space(q_current, q_target, T = T, dt = dt)

    for q in traj:
        comm.send_joint_command(q)
        time.sleep(dt)
