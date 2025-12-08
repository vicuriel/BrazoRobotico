from move import move_to
from comms import RobotComm

SAFE_Z = 1.0      # altura donde nada se choca (dentro del alcance)
GRAB_Z = 0.5      # altura para agarrar
DROP_Z = 0.5      # altura para soltar

def go_home(comm):
    """Mueve a posición home. Requiere RobotComm conectado."""
    move_to(15, 0, SAFE_Z, 0, comm)


def pick(x, y, phi, comm):
    """Secuencia de agarre (pick). Requiere RobotComm conectado."""
    move_to(x, y, SAFE_Z, phi, comm)
    move_to(x, y, GRAB_Z, phi, comm)
    move_to(x, y, SAFE_Z, phi, comm)


def place(x, y, phi, comm):
    """Secuencia de deposito (place). Requiere RobotComm conectado."""
    move_to(x, y, SAFE_Z, phi, comm)
    move_to(x, y, DROP_Z, phi, comm)
    move_to(x, y, SAFE_Z, phi, comm)
