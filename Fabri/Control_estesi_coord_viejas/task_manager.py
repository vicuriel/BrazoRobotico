from kinematics import ik
from comms import RobotComm
from limits_parameters import LIMITS
import time

class RobotAction:
    SAFE_Z = 3
    GRAB_Z = 0.3
    DROP_Z = 0.7

    def __init__(self, comm=None):
        self.comm = comm or RobotComm()

    def move_to(self, x: float, y: float, z: float, phi: float):
    # Mueve el extremo a una localizacion dada
    # 1. Lee las articula actuales
    # 2. Determina la configuración de codo estándar basada en q2 actual
    # 3. Calcula la cinemática inversa para obtener la configuración articular
    # 4. Envía el comando articular al robot
        
        q_act = self.comm.request_state()
        codo_estandar = 'down' if q_act[1] >= 0.0 else 'up'
        q_target = ik(x, y, z, phi, q_prev=q_act, limits=LIMITS, codo_estandar=codo_estandar)
        if q_target is None:
            raise ValueError(f"Pose no alcanzable: ({x}, {y}, {z}, {phi})")
        self.comm.send_joint_command(q_target)
        time.sleep(2)

    def go_home(self):
    # Envía el comando HOME nativo al robot
        self.comm.home()
        time.sleep(2)
        self.comm.stop()

    def search(self):
        # Envía el comando SEARCH nativo al robot (debe estar implementado en el ESP32)
        self.comm._send_line("SEARCH")
        time.sleep(2)
        self.comm.stop()

    def pick(self, x, y, phi):
    # Realiza la secuencia de pick
    # 1. Moverse sobre el objeto
    # 2. Bajar al nivel de agarre
    # 3. Activar la bomba de vacío
    # 4. Subir a posición segura

        self.move_to(x, y, self.SAFE_Z, phi)
        self.move_to(x, y, self.GRAB_Z, phi)
        self.comm.pump(True)
        time.sleep(1.5)
        self.move_to(x, y, self.SAFE_Z, phi)

    def place(self, x, y, phi):
    # Realiza la secuencia de place
    # 1. Moverse sobre la posición de clasificación
    # 2. Bajar al nivel de colocación
    # 3. Desactivar la bomba de vacío
    # 4. Subir a posición segura

        self.move_to(x, y, self.SAFE_Z, phi)
        self.comm.pump(False)
        self.move_to(x, y, self.DROP_Z, phi)
        self.move_to(x, y, self.SAFE_Z, phi)