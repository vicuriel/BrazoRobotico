import socket

CAM_IP = "192.168.135.64"   # IP de la PC con cámara
#CAM_IP = "192.168.0.14"
CAM_PORT = 6001             # Puerto del servidor de visión

def get_object_localization():
    """
    Habla con la PC de la cámara por TCP:
      - manda: 'DETECT\n'
      - recibe: 'OBJ x y phi tipo\n'
    Devuelve:
      (x, y, phi, tipo)
      None              
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((CAM_IP, CAM_PORT))

    try:
        # Pedir detección
        sock.sendall(b"DETECT\n")

        # Leer una línea de respuesta
        buf = b""
        while b"\n" not in buf:
            chunk = sock.recv(64)
            if not chunk:
                # Se cerró la conexión antes de tiempo
                return None
            buf += chunk

        line = buf.decode("ascii").strip()
        parts = line.split()

        # Esperamos: OBJ x y phi tipo  -> 5 partes
        if len(parts) != 5 or parts[0] != "OBJ":
            # Respuesta rara → devolvemos None para que el caller lo maneje
            print(f"Respuesta de cámara inválida: '{line}'")
            return None

        x   = float(parts[1])/10
        y   = float(parts[2])/10
        phi = float(parts[3])
        tipo = parts[4]

        return (x, y, phi, tipo)

    finally:
        sock.close()
