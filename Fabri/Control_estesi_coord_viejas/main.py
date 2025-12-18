from comms import RobotComm
from kinematics import ik, fk
from task_manager import RobotAction
from vision_client import get_object_localization
from limits_parameters import LIMITS
from simulation import simular_pick_and_place
from cinta import CintaTransportadora

comm: RobotComm | None = None
robot: RobotAction | None = None

def verificar_camara_y_simular():
    print("\n[Camara] Pidiendo detección...")
    try:
        coord = get_object_localization()
    except Exception as e:
        print("No se pudo conectar con la cámara.")
        return
    if coord is None:
        print("No se recibió nada válido de la cámara.")
        return
    x, y, phi, tipo = coord
    print(f"Coordenadas recibidas: [{x:.2f}, {y:.2f}, {phi:.2f}] y tipo: {tipo}")
    simular = input("¿Simular coordenadas? (s/n): ").strip().lower()
    if simular == 's':
        q_inicial = [18.6, 0.40999, 3.17, -90]
        # Se asume z_pick = 3.1 (ajustable)
        q_pick = [x, y, 3.1, phi]
        q_place = [-5, -15, 2.0, 0.0]
        q_home = [0.0, 0.0, 5.0, 0.0]
        simular_pick_and_place(q_inicial, q_pick, q_place, q_home)

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
    print("\n------------------- JF XV INICIADO --------------------------")
    
    #coord = (10, -10, 0, 0)
    coord = get_object_localization()
    if coord is not None:    
        x, y, phi, tipo = coord

        print("-----------------------------------------------------------------------------")
        print(f"Objeto detectado en P = ({x:.2f}, {y:.2f}, {phi:.2f}) [cm, cm, °]")
        print(f"Tipo detectado: {tipo}")
        print("-----------------------------------------------------------------------------")
        q_prev = comm.request_state()
        q_sol  = ik(x, y, 0.0, phi, q_prev=q_prev, limits=LIMITS, codo_estandar=None)

        print("-----------------------------------------------------------------------------")
        print(f"Solución IK para esa localizacion: {q_sol}")
        print("-----------------------------------------------------------------------------")

        if q_sol is None:
            print("Localización inalcanzable. Cancelando movimiento y volviendo a HOME.")
            if robot is not None:
                robot.go_home()
            return

        if robot is not None:
            robot.pick(x, y, phi)

        print("-> REALIZANDO PICK")
        fk_vals = fk(*comm.request_state())
        fk_fmt = tuple(float(f"{float(val):.2f}") for val in fk_vals)
        print("-----------------------------------------------------------------------------------")
        print(f"Las posiciones articulares alcanzadas son: {comm.request_state()}")
        print(f"Solucion FK del efector con articulaciones actuales: {fk_fmt}")
        print("-----------------------------------------------------------------------------------")

        print("-> REALIZANDO PLACE")
        if robot is not None:
            robot.place(-5, 15, 0.0)

        print("-> VOLVIENDO A SEARCH")
        if robot is not None:
            robot.search()
    else:   
        print("No se detectó ningún objeto válido")

def main():
    global comm, robot, cinta
    comm = RobotComm()
    comm.connect()

    #cinta = CintaTransportadora()
    #cinta.connect()

    robot = RobotAction(comm)
    robot.search()

    print("-------------------------------------")
    print("-- Ctrl + C = parada de emergencia --")
    print("-------------------------------------")

    try:
        while True:
            print("\n--------------------------------")
            print("Seleccione una opción:")
            print("1. Consultar espacio articular")
            print("2. Verificar cinemática")
            print("3. Verificar cámara y/o simular Pick & Place")
            print("4. Comenzar Pick & Place")
            print("5. Salir")
            print("-------------------------------------")

            opcion = input("-> ").strip()

            if opcion == "1":
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
            elif opcion == "2":
                verificar_cinematica()
            elif opcion == "3":
                verificar_camara_y_simular()
            elif opcion == "4":
                if comm is None:
                    print("Sin conexión al robot.")
                else:
                    pick_and_place(comm)
            elif opcion == "5":
                print("Saliendo...")
                break
            else:
                print("\nERROR. Intente de nuevo.\n")
    finally:
        if comm is not None:
            comm.close()
            print("Conexión con el ESP32 cerrada.")
        #if cinta is not None:
            #cinta.stop()
            #cinta.close()
            #print("Detección y conexión con la cinta transportadora cerrada.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nForzando detención del sistema...")
        try:
            if comm is not None:
                comm.stop()
                comm.pump(False)
                comm.close()
            #if cinta is not None:
                #cinta.stop()
                #cinta.close()
        except Exception:
            pass
        print("Sistema detenido.")