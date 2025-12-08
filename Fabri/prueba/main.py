from vision import get_object_pose
from tasks import go_home, pick, place
from comms import RobotComm


def main():

    print("\n===== SISTEMA ROBÓTICO INICIADO =====\n")

    # Crear una ÚNICA conexión que persiste para toda la secuencia
    comm = RobotComm()
    comm.connect()
    try:
        # 1) Ir a home
        print("→ Moviendo a posición home...")
        go_home(comm)

        # 2) Detectar objeto con cámara
        print("→ Detectando objeto...")
        x, y, phi = get_object_pose()
        print(f"   Objeto encontrado en ({x:.2f}, {y:.2f}) con orientación {phi:.2f}°")

        # 3) Pick
        print("→ Agarrando objeto...")
        pick(x, y, phi, comm)

        # 4) Place (por ejemplo a una posición fija)
        print("→ Depositando objeto...")
        place(20.0, 0.0, 0.0, comm)   # posición destino

        print("\n===== TAREA COMPLETADA =====\n")
    finally:
        comm.close()


if __name__ == "__main__":
    main()
