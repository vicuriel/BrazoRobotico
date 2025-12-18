import time
import serial 

ARDUINO_PORT = "COM3"  
BAUDRATE = 9600

class CintaTransportadora:
    """
    Cinta transportadora controlada por Arduino.
    Comandos
      '1' -> motor adelante
      '2' -> motor atrás
      '0' -> motor detenido
      '3' a '9' -> cambiar velocidad
    """

    def __init__(self, port: str = ARDUINO_PORT, baudrate: int = BAUDRATE):
        self.port = port
        self.baudrate = baudrate
        self.ser: serial.Serial | None = None

    def connect(self):
        if self.ser is not None and self.ser.is_open:
            return

        print(f"Conectando cinta en {self.port} ...")
        self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
        time.sleep(2.0)
        print("Cinta conectada.")

    def close(self):
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
            self.ser = None
            print("Conexión con la cinta cerrada.")

    def _ensure_connected(self):
        if self.ser is None or not self.ser.is_open:
            raise RuntimeError("Cinta no conectada. Llamar a connect().")

    def _send_cmd(self, cmd: str):
        """
        Envía un caracter de comando ('0','1','2','3'..'9') al Arduino.
        """
        self._ensure_connected()
        if len(cmd) != 1:
            raise ValueError("El comando debe ser un solo carácter.")
        self.ser.write(cmd.encode("ascii"))
        self.ser.flush()
        time.sleep(0.01)

    # ---- API pública ----

    def forward(self):
        self._send_cmd('1')

    def backward(self):
        self._send_cmd('2')

    def stop(self):
        self._send_cmd('0')

    def set_speed(self, level: int):
        """
          3 -> más lenta
          9 -> más rápida
        """
        if level < 3 or level > 9:
            raise ValueError("El nivel de velocidad debe estar entre 3 y 9.")
        self._send_cmd(str(level))
