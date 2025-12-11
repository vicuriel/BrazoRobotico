from comms import RobotComm
from kinematics import ik, fk
from tasks import go_home, pick, place
from vision_client import get_object_localization
from config_limits import LIMITS

comm: RobotComm | None = None

def verificar_camara():
    print("\n[Camara] Pidiendo detección...")
    coord = get_object_localization()

    if coord is None:
        print("No se recibió nada válido de la cámara.")
        return
    
    x, y, phi, tipo = coord
    print(f"Coordenadas recibidas: [{x:.2f}, {y:.2f}, {phi:.2f}] y tipo: {tipo}")

def verificar_cinematica():
    print("------------------------------------")
    print("\nVerificar cinemática")
    print("1. Probar FK (ingresar q1, q2, q3, q4)")
    print("2. Probar IK (ingresar x, y, z, phi)")
    opcion = input("-> ").strip()

    if opcion == "1":
        try:
            q1 = float(input("q1 [°]: "))
            q2 = float(input("q2 [°]: "))
            q3 = float(input("q3 [cm]: "))
            q4 = float(input("q4 [°]: "))
        except ValueError:
            print("Entrada inválida.")
            return
        
        x, y, z, phi = fk(q1, q2, q3, q4)
        print(f"\nFK(q) -> x={x:.2f} cm, y={y:.2f} cm, z={z:.2f} cm, phi={phi:.2f}°")

    elif opcion == "2":
        try:
            x   = float(input("x [cm]: "))
            y   = float(input("y [cm]: "))
            z   = float(input("z [cm]: "))
            phi = float(input("phi [°]: "))
        except ValueError:
            print("Entrada inválida.")
            return
        
        q_prev = comm.request_state() if comm is not None else [0.0, 0.0, 0.0, 0.0]
        q_sol = ik(x, y, z, phi, q_prev=q_prev, limits=LIMITS, codo_estandar=None)

        if q_sol is None:
            print("Localizacion inalcanzable según IK.")
        else:
            q1, q2, q3, q4 = q_sol
            print(f"\nIK(x,y,z,phi) -> q1={q1:.2f}°, q2={q2:.2f}°, q3={q3:.2f} cm, q4={q4:.2f}°")
    else:
        print("Opción inválida.")


def pick_and_place(comm: RobotComm):
    """
    Un ciclo de pick & place:
      1) Pide objeto a la cámara e imprime coordendas
      2) Muestra IK propuesta
      3) Hace pick
      4) Muestra la localizacion obtenida del objeto 
      4) Hace place en un lugar fijo
      5) Vuelve a HOME
    """
    print("\n------ JF XV INICIADO ------")

    #coord = get_object_localization()
    coord = (10.0, 10.0, -90.0, "tipo1")  # Para pruebas sin cámara
    if coord is not None:    
        x, y, phi, tipo = coord

        print("-----------------------------------------------------------------")
        print(f"Objeto detectado en P = ({x:.2f}, {y:.2f}, {phi:.2f}) [cm, cm, °]")
        print(f"Tipo detectado: {tipo}")
        print("-----------------------------------------------------------------")

        q_prev = comm.request_state()
        q_sol  = ik(x, y, 0.0, phi, q_prev=q_prev, limits=LIMITS, codo_estandar=None)
        print("-----------------------------------------------------------------")
        print(f"Solución IK para esa localizacion: {q_sol}")
        print("-----------------------------------------------------------------")

        pick(x, y, phi, comm)

        print("-> Realizando pick")
        print("-----------------------------------------------------------------------------------")
        fk_vals = fk(*comm.request_state())
        fk_fmt = tuple(float(f"{float(val):.2f}") for val in fk_vals)
        print(f"Solucion FK del efector con articulaciones actuales: {fk_fmt}")
        print("-----------------------------------------------------------------------------------")

        place(10.0, -10.0, 0.0, comm)
        print("-> Realizando place")

        go_home(comm)
    else:   
        print("No se detectó ningún objeto válido")

def main():
    global comm
    comm = RobotComm()
    comm.connect()
    comm.zero_all()

    print("-------------------------------------")
    print("-- Ctrl + C = parada de emergencia --")
    print("-------------------------------------")

    try:
        while True:
            print("\n--------------------------------")
            print("Seleccione una opción:")
            print("1. Verificar la cámara")
            print("2. Verificar cinemática")
            print("3. Consultar espacio articular")
            print("4. Comenzar Pick & Place")
            print("5. Zero de motores")
            print("6. Ir a HOME")
            print("7. Salir")
            print("-------------------------------------")

            opcion = input("-> ").strip()

            if opcion == "1":
                verificar_camara()
            elif opcion == "2":
                verificar_cinematica()
            elif opcion == "3":
                if comm is None:
                    print("Sin conexión al robot.")
                else:
                    q = comm.request_state()
                    print(
                        f"\nEstado actual de las articulaciones:\n"
                        f"q1 = {q[0]:.2f}°\n"
                        f"q2 = {q[1]:.2f}°\n"
                        f"q3 = {q[2]:.2f} cm\n"
                        f"q4 = {q[3]:.2f}°"
                    )
            elif opcion == "4":
                if comm is None:
                    print("Sin conexión al robot.")
                else:
                    pick_and_place(comm)
            elif opcion == "5":
                if comm is None:
                    print("Sin conexión al robot.")
                else:
                    comm.zero_all()
                    print("Motores llevados a posición cero.")
            elif opcion == "6":
                if comm is None:
                    print("Sin conexión al robot.")
                else:
                    go_home(comm)
                    print("Robot llevado a posición HOME.")
            elif opcion == "7":
                print("Saliendo...")
                break
            else:
                print("\nERROR. Intente de nuevo.\n")
    finally:
        if comm is not None:
            comm.close()
            print("Conexión con el ESP32 cerrada.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nForzando detención del sistema...")
        try:
            if comm is not None:
                comm.stop()
                comm.close()
        except Exception:
            pass
        print("Sistema detenido.")
