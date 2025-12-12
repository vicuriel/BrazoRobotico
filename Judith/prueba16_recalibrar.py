# ============================================================
#   YOLO + ArUco (API nueva) + Clasificación por color/sellado
#   + Grid del sistema de referencia (mm)
#   + Ángulo robusto en el mundo (PCA + coherencia)
#   + Suavizado temporal (historial + mediana)
#   + Servidor TCP:
#       Cliente manda:  "DETECT\n"
#       Servidor responde: "OBJ x y phi tipo\n"
#       (solo para blísteres SELLADOS)
# ============================================================

import cv2
import time
import argparse
import numpy as np
from pathlib import Path
from ultralytics import YOLO
import os
import warnings
import socket   # TCP

warnings.filterwarnings('ignore', category=UserWarning)
os.environ['CUDA_VISIBLE_DEVICES'] = '-1'

# ===================== CONFIG PLANO REAL (mm) =====================
WORLD_WIDTH_MM  = 358.1   # ancho total (distancia horizontal 1-0 y 4-3)
WORLD_HEIGHT_MM = 235.3   # alto total  (distancia vertical 4-0 y 3-1)

# IDs de ArUco que definen las esquinas (en orden que usaremos):
# Patrón físico:
#   1   0   (fila superior)
#   4   3   (fila inferior)
#
# Mapeo a coordenadas reales:
#   id 4 -> (-W/2, 0)    (abajo izquierda)
#   id 3 -> (+W/2, 0)    (abajo derecha)
#   id 0 -> (+W/2, H)    (arriba derecha)
#   id 1 -> (-W/2, H)    (arriba izquierda)
ARUCO_IDS_CORNERS = [3, 4, 1,0]

# Paso de grid en mm
GRID_STEP_MM = 20.0
# ===================== CONFIG TCP (SERVIDOR) =====================
TCP_PORT = 6001   # escuchamos en todas las interfaces con ""
# ================================================================


def parse_args():
    p = argparse.ArgumentParser(description="YOLO + ArUco + Clasif blister + ángulo robusto + TCP server")
    p.add_argument("--camera", type=int, default=1)
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--model", type=str, default="auto")
    p.add_argument("--conf", type=float, default=0.30)
    p.add_argument("--imgsz", type=int, default=640)
    return p.parse_args()


def auto_find_weights():
    root = Path(__file__).parent
    prefer = [root / "best.pt", root / "last.pt"]
    for p in prefer:
        if p.exists():
            print(">> Usando pesos:", p)
            return str(p)

    runs = root / "runs"
    pts = sorted(
        list(runs.glob("**/weights/best.pt")) +
        list(runs.glob("**/weights/last.pt")),
        key=lambda p: p.stat().st_mtime,
        reverse=True
    )
    if pts:
        print(">> Usando pesos:", pts[0])
        return str(pts[0])

    raise FileNotFoundError("No se encontraron pesos .pt")


# ================================================================
#            ARUCO API NUEVA (OpenCV moderno)
# ================================================================
def get_aruco_detector():
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    params = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(dictionary, params)
    return detector, dictionary


def compute_homography(frame, detector):
    """
    Si encuentra TODOS los ARUCO_IDS_CORNERS, arma la homografía
    pixeles -> mundo (mm) con el siguiente sistema:

    Patrón ArUco en la escena (vista cámara):
        1   0
        4   3

    Coordenadas reales:
        id 4 -> (-W/2, 0)    (abajo izquierda)
        id 3 -> (+W/2, 0)    (abajo derecha)
        id 0 -> (+W/2, H)    (arriba derecha)
        id 1 -> (-W/2, H)    (arriba izquierda)

    Donde:
        eje Y: 0 en la recta 4-3, positivo hacia 1-0
        eje X: negativo hacia 1-4, positivo hacia 0-3
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is None or len(ids) == 0:
        return None, None

    ids = ids.flatten()
    img_pts = []

    for target_id in ARUCO_IDS_CORNERS:
        if target_id not in ids:
            return None, (corners, ids)

        idx = np.where(ids == target_id)[0][0]
        c = corners[idx][0]    # (4, 2)
        cx = np.mean(c[:, 0])
        cy = np.mean(c[:, 1])
        img_pts.append([cx, cy])

    img_pts = np.array(img_pts, dtype=np.float32)

    W2 = WORLD_WIDTH_MM / 2.0
    H  = WORLD_HEIGHT_MM

    world_pts = np.array([
        [-W2, 0.0],  # id 4 (abajo izquierda)
        [ W2, 0.0],  # id 3 (abajo derecha)
        [ W2,   H],  # id 0 (arriba derecha)
        [-W2,   H],  # id 1 (arriba izquierda)
    ], dtype=np.float32)

    Hmat, _ = cv2.findHomography(img_pts, world_pts, 0)
    return Hmat, (corners, ids)


def project_pixel_to_world(H, x, y):
    pt = np.array([x, y, 1.0], dtype=np.float32)
    dst = H @ pt
    if dst[2] == 0:
        return None
    return float(dst[0] / dst[2]), float(dst[1] / dst[2])


def to_world_vector(H, p1, p2):
    w1 = project_pixel_to_world(H, p1[0], p1[1])
    w2 = project_pixel_to_world(H, p2[0], p2[1])
    if w1 is None or w2 is None:
        return None
    return np.array([w2[0] - w1[0], w2[1] - w1[1]], dtype=float)


# ================================================================
#    ÁNGULO ROBUSTO EN EL PLANO ARUCO USANDO PCA EN MUNDO
# ================================================================
def robust_angle_world(H, bbox, frame):
    """
    1) Saca contorno del blister dentro del bbox.
    2) Proyecta TODOS los puntos del contorno al plano (mm).
    3) Hace PCA en mundo -> vector principal.
    4) Fuerza que el vector apunte hacia +X.
    5) Devuelve (ángulo[0..180], es_confiable).
    """

    x1, y1, x2, y2 = map(int, bbox)
    h, w = frame.shape[:2]
    x1c = max(0, x1)
    y1c = max(0, y1)
    x2c = min(w, x2)
    y2c = min(h, y2)

    roi = frame[y1c:y2c, x1c:x2c]
    if roi.size == 0:
        return None, False

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5,5), 0)
    edges = cv2.Canny(gray, 60, 150)

    cnts, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None, False

    # Contorno más grande
    cnt = max(cnts, key=cv2.contourArea).reshape(-1, 2)
    # Pasar a coords globales
    cnt[:, 0] += x1c
    cnt[:, 1] += y1c

    # Proyectar todos los puntos a mm
    world_pts = []
    for (px, py) in cnt:
        p_world = project_pixel_to_world(H, float(px), float(py))
        if p_world is not None:
            world_pts.append(p_world)

    if len(world_pts) < 10:
        # Muy pocos puntos, mala base para PCA
        return None, False

    world_pts = np.array(world_pts, dtype=np.float32)
    # Centrar
    mean = np.mean(world_pts, axis=0)
    centered = world_pts - mean

    # Covarianza 2D
    cov = np.cov(centered.T)
    eigvals, eigvecs = np.linalg.eigh(cov)  # ascendentes

    # Eje mayor = autovector de mayor autovalor
    idx_max = np.argmax(eigvals)
    idx_min = 1 - idx_max

    lambda_max = eigvals[idx_max]
    lambda_min = eigvals[idx_min]

    # Ratio de alargamiento: 0 -> muy alargado, 1 -> redondo
    if lambda_max <= 1e-6:
        return None, False

    elong_ratio = lambda_min / lambda_max

    # Vector principal en mundo
    v = eigvecs[:, idx_max]   # [vx, vy]
    v = v / np.linalg.norm(v)

    # Forzar que mire hacia +X del sistema ArUco
    if v[0] < 0:
        v = -v

    # Ángulo con respecto al eje X del plano (mm)
    ang = np.degrees(np.arctan2(v[1], v[0]))  # -90..+90
    if ang < 0:
        ang += 180.0  # lo llevo a 0..180

    # Considerar "confiable" si está suficientemente alargado
    # Si elong_ratio ~ 1 => objeto casi redondo, orientación mala.
    is_reliable = elong_ratio < 0.5

    return float(ang), bool(is_reliable)


# ================================================================
#                CLASIFICADOR BLISTER (TU LÓGICA)
# ================================================================
def draw_box(img, xyxy, label, color=(0, 255, 0), thickness=2):
    x1, y1, x2, y2 = map(int, xyxy)
    cv2.rectangle(img, (x1, y1), (x2, y2), color, thickness)
    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    cv2.rectangle(img, (x1, y1 - th - 6), (x1 + tw + 6, y1), color, -1)
    cv2.putText(img, label, (x1 + 3, y1 - 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (0, 0, 0), 1, cv2.LINE_AA)


def hole_inside_blister(h_box, b_box):
    hx1, hy1, hx2, hy2 = h_box
    bx1, by1, bx2, by2 = b_box
    cx = (hx1 + hx2) / 2
    cy = (hy1 + hy2) / 2
    return (bx1 <= cx <= bx2) and (by1 <= cy <= by2)


# ================================================================
#                           MAIN
# ================================================================
def main():
    args = parse_args()

    # Cámara
    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, 30)

    if not cap.isOpened():
        print("No se pudo abrir la cámara")
        return

    # YOLO
    model_path = auto_find_weights() if args.model == "auto" else args.model
    model = YOLO(model_path)

    # Nombres
    class_names = model.names
    print("Clases YOLO:", class_names)

    # Lógica de colores + hueco
    color_ids = {}
    hueco_id = None

    for cid, name in class_names.items():
        n = name.lower()
        if "azul" in n:
            color_ids["Azul"] = cid
        elif "negro" in n:
            color_ids["Negro"] = cid
        elif "rojo" in n:
            color_ids["Rojo"] = cid
        elif "verde" in n:
            color_ids["Verde"] = cid
        elif "hueco" in n or "hole" in n or "agujero" in n:
            hueco_id = cid

    if not color_ids:
        color_ids = {"Azul": 0, "Negro": 1, "Rojo": 2, "Verde": 3}
        print("⚠️ No encontré clases de color por nombre, usando IDs por defecto")
    if hueco_id is None:
        hueco_id = 4
        print("⚠️ No encontré clase 'hueco', asumo id=4")

    print(f"Colores detectados: {color_ids}")
    print(f"ID de hueco: {hueco_id}")

    # ArUco
    detector, dictionary = get_aruco_detector()
    H = None
    H_ok = False
    force_recalib = False

    

    prev = time.time()

    # Historial de ángulos por blister
    angle_history = {}    # key -> [ángulos últimos]
    center_smooth = {}    # key -> (cx, cy)
    MAX_HISTORY = 7
    ALPHA_CENTER = 0.30

    # =============== Servidor TCP (no bloqueante) ===============
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind(("", TCP_PORT))   # "" => todas las interfaces
    server_sock.listen(1)
    server_sock.setblocking(False)
    print(f">> Servidor TCP escuchando en puerto {TCP_PORT}")
    client_sock = None
    client_addr = None
    recv_buffer = b""
    # ============================================================

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # ------------------- ARUCO (recalibración manual) -------------------
        ar_data = None

        if force_recalib:

        # Intentar recalibrar SOLO cuando tocás 'c'
            H_new, ar_data = compute_homography(frame, detector)

            if H_new is not None:
                H = H_new
                H_ok = True
                print(">> Calibración EXITOSA.")
            else:
                print(">> No se encontraron los 4 ArUco. Manteniendo calibración previa.")

            # Recalibrar solo una vez (hasta volver a tocar C)
            force_recalib = False

        else:
            # Si NO estamos recalibrando, solo dibujamos ArUco si aparecen,
            # pero NO tocamos la homografía.
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)


        if H_ok:
            cv2.putText(frame, "Plano calibrado", (10,60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,(255,0,0),2)

            W2 = WORLD_WIDTH_MM / 2.0
            Hh = WORLD_HEIGHT_MM

            # Bordes del rectángulo en coordenadas reales
            world_corners = np.array([
                [-W2, 0.0],
                [ W2, 0.0],
                [ W2, Hh ],
                [-W2, Hh ],
            ], dtype=np.float32)

            H_inv = np.linalg.inv(H)

            img_corners = []
            for X, Y in world_corners:
                pt_world = np.array([X, Y, 1.0], dtype=np.float32)
                dst = H_inv @ pt_world
                u, v = dst[0]/dst[2], dst[1]/dst[2]
                img_corners.append([int(u), int(v)])
            img_corners = np.array(img_corners, dtype=np.int32)
            cv2.polylines(frame, [img_corners], isClosed=True, color=(255,0,0), thickness=2)

            # GRID (cada GRID_STEP_MM) con X negativo y positivo y Y desde 0 a H
            x_vals = np.arange(-W2, W2 + 0.01, GRID_STEP_MM)
            for X in x_vals:
                p1_world = np.array([X, 0.0, 1.0], dtype=np.float32)
                p2_world = np.array([X, Hh, 1.0], dtype=np.float32)
                d1 = H_inv @ p1_world
                d2 = H_inv @ p2_world
                if d1[2] != 0 and d2[2] != 0:
                    u1, v1 = int(d1[0]/d1[2]), int(d1[1]/d1[2])
                    u2, v2 = int(d2[0]/d2[2]), int(d2[1]/d2[2])
                    cv2.line(frame, (u1, v1), (u2, v2), (100, 100, 255), 1)

            y_vals = np.arange(0.0, Hh + 0.01, GRID_STEP_MM)
            for Y in y_vals:
                p1_world = np.array([-W2, Y, 1.0], dtype=np.float32)
                p2_world = np.array([ W2, Y, 1.0], dtype=np.float32)
                d1 = H_inv @ p1_world
                d2 = H_inv @ p2_world
                if d1[2] != 0 and d2[2] != 0:
                    u1, v1 = int(d1[0]/d1[2]), int(d1[1]/d1[2])
                    u2, v2 = int(d2[0]/d2[2]), int(d2[1]/d2[2])
                    cv2.line(frame, (u1, v1), (u2, v2), (100, 100, 255), 1)

        # ------------------- YOLO -------------------
        res = model(frame, imgsz=args.imgsz, conf=args.conf, verbose=False)[0]

        boxes = res.boxes.xyxy.cpu().numpy() if res.boxes is not None else []
        cls_ids = res.boxes.cls.cpu().numpy().astype(int) if res.boxes is not None else []

        huecos = []
        blisters_por_color = {k: [] for k in color_ids.keys()}

        for xyxy, cid in zip(boxes, cls_ids):
            if cid == hueco_id:
                huecos.append(xyxy)
            else:
                for color, color_id in color_ids.items():
                    if cid == color_id:
                        blisters_por_color[color].append(xyxy)
                        break

        # SOLO vamos a guardar info para TCP de blisters SELLADOS
        best_target = None   # dict con {Xmm, Ymm, ang, color, area}

        # -------- CLASIFICAR + COORDENADAS + ÁNGULO ROBUSTO --------
        for color, blisters in blisters_por_color.items():
            for idx, b_box in enumerate(blisters):
                bx1, by1, bx2, by2 = b_box
                key = f"{color}_{idx}"

                cx_raw = (bx1 + bx2) / 2.0
                cy_raw = (by1 + by2) / 2.0

                # suavizado del centro
                if key not in center_smooth:
                    center_smooth[key] = (cx_raw, cy_raw)
                prev_cx, prev_cy = center_smooth[key]
                cx = ALPHA_CENTER * cx_raw + (1 - ALPHA_CENTER) * prev_cx
                cy = ALPHA_CENTER * cy_raw + (1 - ALPHA_CENTER) * prev_cy
                center_smooth[key] = (cx, cy)

                cx_i, cy_i = int(cx), int(cy)

                tiene_hueco = any(hole_inside_blister(h, b_box) for h in huecos)

                if tiene_hueco:
                    estado = "NO Sellado"
                    color_bgr = (0, 0, 255)
                else:
                    estado = "Sellado"
                    color_bgr = (0, 255, 0)

                # Siempre marcamos el bounding box y el texto de estado
                label = f"Blister {color} - {estado}"
                draw_box(frame, b_box, label, color=color_bgr)

                # >>> SOLO nos interesa centro/ángulo de los SELLADOS <<< 
                if (not tiene_hueco) and H_ok:
                    # Ángulo bruto + fiabilidad
                    ang_raw, is_reliable = robust_angle_world(H, b_box, frame)

                    if ang_raw is None:
                        # Si no pudimos calcular nada pero ya teníamos historia,
                        # usamos lo último que sabemos.
                        if key in angle_history and angle_history[key]:
                            ang_final = angle_history[key][-1]
                        else:
                            continue
                    else:
                        # Historial
                        if key not in angle_history:
                            angle_history[key] = [ang_raw]
                        else:
                            prev_ang = angle_history[key][-1]

                            # Desenrollar saltos: si de repente pasa de 10° a 170°,
                            # lo interpretamos como -10° -> sumamos/restamos 180.
                            diff = ang_raw - prev_ang
                            if diff > 90.0:
                                ang_raw -= 180.0
                            elif diff < -90.0:
                                ang_raw += 180.0

                            # Volvemos a rango 0..180
                            while ang_raw < 0.0:
                                ang_raw += 180.0
                            while ang_raw >= 180.0:
                                ang_raw -= 180.0

                            # Si NO es confiable (objeto muy "redondo"),
                            # preferimos usar el valor previo.
                            if not is_reliable:
                                ang_use = prev_ang
                            else:
                                ang_use = ang_raw

                            angle_history[key].append(ang_use)
                            if len(angle_history[key]) > MAX_HISTORY:
                                angle_history[key].pop(0)

                        # Ángulo final = mediana de la historia
                        ang_final = float(np.median(angle_history[key]))

                    # Centro respecto al sistema ArUco
                    world = project_pixel_to_world(H, cx, cy)
                    if world is not None:
                        Xmm, Ymm = world
                        text = f"{Xmm:.1f}mm {Ymm:.1f}mm {ang_final:.1f}°"
                        cv2.circle(frame, (cx_i, cy_i), 4, color_bgr, -1)
                        cv2.putText(frame, text, (cx_i+10, cy_i),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr, 2)

                        area = (bx2 - bx1) * (by2 - by1)
                        if best_target is None or area > best_target["area"]:
                            best_target = {
                                "Xmm": Xmm,
                                "Ymm": Ymm,
                                "ang": ang_final,
                                "color": color,
                                "area": area
                            }
                # Si es NO Sellado: no mostramos centro ni lo usamos para TCP

        # ================== TCP SERVER LOOP (no bloqueante) ==================
        if client_sock is None:
            try:
                client_sock, client_addr = server_sock.accept()
                client_sock.setblocking(False)
                recv_buffer = b""
                print(f">> Cliente conectado desde {client_addr}")
            except BlockingIOError:
                pass
        else:
            try:
                data = client_sock.recv(1024)
                if data:
                    recv_buffer += data
                    while b"\n" in recv_buffer:
                        line, recv_buffer = recv_buffer.split(b"\n", 1)
                        cmd = line.strip().upper()
                        if cmd == b"DETECT":
                            # Solo respondemos con SELLADO (best_target)
                            if best_target is not None:
                                msg = "OBJ {:.2f} {:.2f} {:.2f} {}\n".format(
                                    best_target["Xmm"],
                                    best_target["Ymm"],
                                    best_target["ang"],
                                    best_target["color"]
                                )
                            else:
                                msg = "OBJ 0.00 0.00 0.00 NONE\n"
                            try:
                                client_sock.sendall(msg.encode("ascii"))
                            except Exception as e:
                                print("⚠️ Error enviando al cliente:", e)
                                client_sock.close()
                                client_sock = None
                                client_addr = None
                                recv_buffer = b""
                else:
                    print(">> Cliente desconectado")
                    client_sock.close()
                    client_sock = None
                    client_addr = None
                    recv_buffer = b""
            except BlockingIOError:
                pass
            except ConnectionResetError:
                print("⚠️ Cliente reseteó la conexión")
                client_sock.close()
                client_sock = None
                client_addr = None
                recv_buffer = b""
        # ====================================================================

        # FPS
        now = time.time()
        fps = 1.0 / (now - prev)
        prev = now
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        cv2.imshow("YOLO + ArUco + Grid + Ángulo robusto + TCP", frame)
        k = cv2.waitKey(1) & 0xFF
        if k == 27 or k == ord('q'):
            break

        if k == ord('c'):
            print(">> Recalibrando plano...")
            force_recalib = True

    cap.release()
    if client_sock is not None:
        client_sock.close()
    server_sock.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
