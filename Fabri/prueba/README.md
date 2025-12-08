CARPETA /PRUEBA - Sistema Robotico SCARA
========================================

Esta carpeta contiene una copia de trabajo del sistema robotico SCARA
para pruebas incremental es sin afectar los archivos originales.

---

CONTENIDO
=========

kinematics.py
  - Cinematica directa (FK) del robot SCARA
  - Cinematica inversa (IK) con 2 soluciones posibles
  - Mapeo de desplazamiento lineal q3 a angulo de servo
  - DH parameters y transformaciones homogeneas

comms.py
  - Clase RobotComm para comunicacion TCP con ESP32
  - Metodos: connect(), close(), request_state(), send_joint_command()
  - IP actual: 192.168.0.16, Puerto: 5000

trajectory.py
  - Interpolacion lineal en espacio articular
  - Genera trayectorias punto a punto entre poses
  - Compatible con ESP32 (comandos cada ~20ms)

move.py
  - Funcion move_to(x, y, z, phi, T, dt)
  - Realiza IK -> interpolation -> envio de comandos
  - Obtiene estado inicial via request_state()
  - NUEVO: move_to_with_comm(x, y, z, phi, comm, T, dt)
    Versión que reutiliza conexión TCP existente (RECOMENDADO para hardware)

tasks.py
  - Secuencias de tareas: go_home(), pick(), place()
  - NUEVO: go_home_with_comm(), pick_with_comm(), place_with_comm()
    Versiones optimizadas que reutilizan una conexión TCP
  - Alturas de seguridad: SAFE_Z=5.0, GRAB_Z=1.0, DROP_Z=1.0
  - NUEVO: run_full_sequence_optimized() - Ejemplo de uso optimizado

main.py
  - Programa principal que ejecuta la secuencia completa
  - Requiere: funcion get_object_pose() del modulo vision

test_simulation.py
  - Script de prueba SIN necesidad de ESP32 real
  - Usa MockRobotComm para simular comunicaciones
  - Prueba secuencia: HOME -> DETECT -> PICK -> PLACE
  - Ejecutar: python test_simulation.py

run_and_report.py
  - Ejecuta la simulación completa y genera un reporte automático
  - Salidas: `REPORTE_SIMULACION.txt` y `simulation_plot.png`
  - Ejecutar: python run_and_report.py
  - Nota: Para generar el gráfico se requiere `matplotlib`. Instala con:

    ```bash
    python -m pip install matplotlib
    ```
    Si no está instalado, el script generará sólo el reporte en texto.

ejemplo_conexion_persistente.py
  - NUEVO: Demuestra mejora de performance con conexion persistente
  - Compara: 6 conexiones TCP vs 1 conexion reutilizada
  - Muestra ganancia de 6x en eficiencia de red
  - Ejecutar: python ejemplo_conexion_persistente.py

REPORTE_SIMULACION.txt
  - Reporte detallado de la simulacion
  - Hallazgos, problemas encontrados, recomendaciones

---

WORKSPACE DEL ROBOT
===================

El robot tiene alcance limitado:
  X: [-8.46, 18.56] cm
  Y: [-18.56, 18.56] cm
  Z: [-8.89, 6.11] cm

Nota: Z muy restringido (max 6.11 cm) - no hay mucho espacio vertical

---

COMO USAR
=========

1. PRUEBA RAPIDA (sin ESP32):
   python test_simulation.py
   
   Esto ejecuta la simulacion completa:
   - HOME (0,0,0,0) -> (15,0,5,0)
   - PICK en (16,6,30deg)
   - PLACE en (12,-8,-30deg)
   
   Ver estado final y cantidad de comandos

2. PRUEBA CON ESP32 REAL:
   a) Cambiar IP en comms.py si es necesario:
      ESP_IP = "192.168.0.16"  # <-- Tu IP del ESP32
   
   b) Modificar main.py para tu logica de deteccion
   
   c) Ejecutar: python main.py

3. PRUEBAS INDIVIDUALES:
   >>> from kinematics import fk, ik
   >>> fk(0, 0, 0, 0)
   (18.561, 0.024, 1.109, -46.705)
   
   >>> ik(18.561, 0.024, 1.109, -46.705)
   [(0.0, 0.0, 0.0, 0.0), ...]

---

PROBLEMAS CONOCIDOS Y SOLUCIONES
=================================

PROBLEMA 1: Estado del robot siempre [0,0,0,0] en simulacion
CAUSA: MockRobotComm no persiste estado entre move_to() calls
SOLUCION: En hardware real, usar encoders reales del ESP32

PROBLEMA 2: Hay que reconectar TCP en cada move_to()
CAUSA: Arquitectura actual cierra conexion al terminar interpolacion
MEJORA: Mantener conexion persistente (pooling de conexion)

PROBLEMA 3: Seleccion de solucion IK siempre sols[0]
CAUSA: No implementado choose_solution()
MEJORA: Implementar seleccion inteligente basada en q_prev

---

PROXIMOS PASOS
==============

1. [ ] Agregar limites articulares en IK
2. [ ] Mejorar seleccion de solucion con choose_solution()
3. [ ] Mantener conexion TCP persistente
4. [ ] Implementar collision checking
5. [ ] Calibrar mapeo q3 <-> servo con hardware real
6. [ ] Pruebas en ESP32 fisico
7. [ ] Agregar feedback de encoders para verificacion

---

TESTING
=======

Ver REPORTE_SIMULACION.txt para:
- Resultados de pruebas FK/IK
- Round-trip validation (FK -> IK -> FK)
- Analisis de workspace
- Recomendaciones detalladas

---

STATUS: LISTO PARA PRUEBAS EN HARDWARE

Ultima actualizacion: Diciembre 8, 2025
