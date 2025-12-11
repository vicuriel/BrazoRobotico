from kinematics import ik
from comms import RobotComm
from config_limits import LIMITS
import time

def move_to(x: float, y: float, z: float, phi: float, comm: RobotComm):
    # Mueve el robot a una pose cartesiana (Su extremo)
    # 1. Lee las articula actuales
    # 2. Determina la configuración de codo estándar basada en q2 actual
    # 3. Calcula la cinemática inversa para obtener la configuración articular
    # 4. Envía el comando articular al robot

    q_act = comm.request_state()
    codo_estandar = 'down' if q_act[1] >= 0.0 else 'up'
    q_target = ik(x, y, z, phi, q_prev=q_act, limits=LIMITS, codo_estandar=codo_estandar)
    if q_target is None:
        raise ValueError(f"Pose no alcanzable: ({x}, {y}, {z}, {phi})")
    comm.send_joint_command(q_target)
    time.sleep(2)

