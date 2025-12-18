import socket
import time

#ESP_IP = "192.168.0.16"           # IP del ESP32 Judith
ESP_IP = "192.168.135.63"           # IP del LAR
ESP_PORT = 5000                     # Puerto TCP del ESP32
Q3_SERVO_MAX = 4.4                  # Desplazamiento maximo q3 en cm

class RobotComm:
    """
    Comunicación TCP con el ESP32 del SCARA.

    Comandos:
      - Q q1 q2 q3 q4      → setpoint de todas las articulaciones
      - STATE?             → pide estado; recibe: STATE q1 q2 q3 q4 y retorna [q1, q2, q3, q4]
      - STOP               → detiene el movimiento (congela estado actual)
      - ZERO               → pone a cero q1 y q2
      - ZERO 1 / ZERO 2    → pone a cero q1 o q2
      - HOME               → mueve a posición HOME definida en el firmware
      - PUMP 1 / PUMP 0    → enciende / apaga la bomba
    """

    def __init__(self, ip: str = ESP_IP, port: int = ESP_PORT):
        self.ip = ip
        self.port = port
        self.sock: socket.socket | None = None

    def connect(self):
        """Abre conexión TCP con el ESP32."""
        print("Conectando........")
        if self.sock is not None:
            return 
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)       # Crear socket TCP
        self.sock.connect((self.ip, self.port))                             # Conectar al ESP32
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)     # Desactivar Nagle (se mandan paquetes inmediatamente)
        print(f"Conectado al ESP32 en {self.ip}:{self.port}")

    def close(self):
        """Cierra la conexión TCP."""
        if self.sock is not None:               # Si está conectado
            self.sock.close()                   # Cerrar socket
            self.sock = None                    # Marcar como desconectado

    def _ensure_connected(self):
        if self.sock is None:                   # Se asgura que hay conexión, en caso contrario lanza error
            raise RuntimeError("No hay conexión con el ESP32. Llamar a connect().")

    def _send_line(self, line: str):
        """Envía una línea terminada al ESP32."""
        self._ensure_connected()                # Asegura conexión
        msg = (line + "\n").encode("ascii")     # Codifica línea recibida, agrega \n y convierte a bytes
        self.sock.sendall(msg)                  # Envía todos los bytes al ESP32
        time.sleep(0.005)                       # Pequeña espera para evitar saturar el buffer del ESP32

    def q3_lineal_a_ang(self, q3_cm: float) -> float:
        deg = (1.0 - (q3_cm / Q3_SERVO_MAX)) * 180.0
        return float(max(0.0, min(180.0, deg)))

    def q3_ang_a_lineal(self, q3_deg: float) -> float:
        cm = Q3_SERVO_MAX * (1.0 - (q3_deg / 180.0))
        return float(max(0.0, min(Q3_SERVO_MAX, cm)))

    # Comandos

    def send_joint_command(self, q):
        if len(q) != 4:
            raise ValueError("q debe tener 4 elementos: [q1, q2, q3, q4].")
        q1, q2, q3_cm, q4 = q
        q3_servo = self.q3_lineal_a_ang(q3_cm)
        line = f"Q {q1:.3f} {q2:.3f} {q3_servo:.3f} {q4:.3f}"
        self._send_line(line)

    def request_state(self):
        self._send_line("STATE")
        buf = b""
        while b"\n" not in buf:
            received = self.sock.recv(64)
            if not received:
                raise ConnectionError("Conexión cerrada por el ESP32 al solicitar STATE.")
            buf += received

        line = buf.decode("ascii").strip()
        parts = line.split()

        if len(parts) != 5 or parts[0] != "STATE":
            raise ValueError(f"Respuesta STATE inválida: '{line}'")

        q1 = float(parts[1])
        q2 = float(parts[2])
        q3_servo = float(parts[3])
        q4 = float(parts[4])

        q3_cm = self.q3_ang_a_lineal(q3_servo)
        return [q1, q2, q3_cm, q4]

    def stop(self):
        self._send_line("STOP")

    def zero_all(self):
        self._send_line("ZERO")

    def zero_joint(self, joint: int):
        if joint not in (1, 2):
            raise ValueError("joint debe ser 1 o 2.")
        self._send_line(f"ZERO {joint}")

    def home(self):
        self._send_line("HOME")

    def pump(self, on: bool):
        val = 1 if on else 0
        self._send_line(f"PUMP {val}")
