import socket

IP = "192.168.0.16"   # <- PONE AQUÍ EL IP DE TU ESP32
PORT = 5000              # <- DEBE SER EL MISMO QUE EN EL ESP32

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

print(f"Conectando al ESP32 en {IP}:{PORT} ...")
sock.connect((IP, PORT))
print("🔥 Conectado al ESP32! Ahora podes enviar comandos.\n")

try:
    while True:
        cmd = input("Comando > ")

        if cmd.lower() in ["exit", "quit"]:
            print("Cerrando conexión...")
            break

        # Enviar comando (siempre terminar con \n)
        sock.sendall((cmd + "\n").encode())

        # Intentar leer la respuesta del ESP32
        sock.settimeout(0.2)
        try:
            resp = sock.recv(1024)
            if resp:
                print("ESP32 >", resp.decode().strip())
        except socket.timeout:
            pass

finally:
    sock.close()
    print("Conexión cerrada.")
