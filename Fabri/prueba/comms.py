import socket

ESP_IP = "192.168.0.16"           # IP del ESP32
ESP_PORT = 5000                  # puerto en el ESP32

class RobotComm:
    def __init__(self, ip = ESP_IP, port = ESP_PORT):
        self.ip = ip
        self.port = port
        self.sock = None

    def connect(self):
        """Abre conexión TCP con el ESP32."""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.ip, self.port))
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def close(self):
        """Cierra la conexión."""
        if self.sock is not None:
            self.sock.close()
            self.sock = None
    
    def request_state(self):
        self.sock.sendall(b"GET_STATE\n")
        data = self.sock.recv(64).decode("ascii")
        # data por ejemplo: "STATE 12.30 -5.10 3.00 45.00\n"
        parts = data.split()
        return [float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4])]


    def send_joint_command(self, q):
        """
        q: lista [q1, q2, q3, q4] en grados / cm.
        Envía: 'Q q1 q2 q3 q4\n'
        """
        if self.sock is None:
            raise RuntimeError("No hay conexión con el ESP32. Conectese primero.")

        q1, q2, q3, q4 = q
        msg = f"Q {q1:.3f} {q2:.3f} {q3:.3f} {q4:.3f}\n"
        self.sock.sendall(msg.encode("ascii"))
