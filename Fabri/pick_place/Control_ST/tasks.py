from move import move_to
from comms import RobotComm

SAFE_Z = 3               # altura donde nada se choca (dentro del alcance)
GRAB_Z = 0.5             # altura para agarrar
DROP_Z = 1.0             # altura para soltar

def go_home(comm):
    """Mueve a posicion inicial predeterminada."""
    move_to(18.6, 0.410, SAFE_Z, -90, comm)

def pick(x, y, phi, comm):
    """Secuencia de agarre."""
    move_to(x, y, SAFE_Z, phi, comm)
    move_to(x, y, GRAB_Z, phi, comm)
    comm.pump(True)
    move_to(x, y, SAFE_Z, phi, comm)

def place(x, y, phi, comm):
    """Secuencia de deposito."""
    move_to(x, y, SAFE_Z, phi, comm)
    comm.pump(False)
    move_to(x, y, DROP_Z, phi, comm)
    move_to(x, y, SAFE_Z, phi, comm)
