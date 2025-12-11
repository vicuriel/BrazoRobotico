import math
from typing import List, Tuple, Optional
from kinematics import fk
from config_limits import LIMITS

try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

def convex_hull(points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    if len(points) <= 2:
        return sorted(points)
    sorted_pts = sorted(set(points))
    if len(sorted_pts) <= 2:
        return sorted_pts
    lower = []
    for p in sorted_pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(sorted_pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]

def cross(o: Tuple[float, float], a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

def plot_workspace(all_points: list, save_img: bool):
    if not HAS_MATPLOTLIB:
        print("⚠ matplotlib no disponible")
        return
    fig, ax = plt.subplots(figsize=(8, 8))
    xs = [p[0] for p in all_points]
    ys = [p[1] for p in all_points]
    ax.scatter(xs, ys, c='royalblue', s=2, alpha=0.7, label='Puntos alcanzables')
    # Contorno tipo semi-anillo (no convex hull):
    # Ordenar puntos por ángulo y radio, trazar borde exterior e interior
    import numpy as np
    points_np = np.array([(x, y) for x, y, _ in all_points])
    rs = np.linalg.norm(points_np, axis=1)
    thetas = np.arctan2(points_np[:,1], points_np[:,0])
    # Borde exterior: para cada ángulo, tomar el punto de mayor radio
    bins = np.linspace(-np.pi, np.pi, 360)
    idxs = np.digitize(thetas, bins)
    max_r_points = {}
    min_r_points = {}
    for i, idx in enumerate(idxs):
        th = bins[idx-1] if idx > 0 else bins[0]
        if th not in max_r_points or rs[i] > max_r_points[th][0]:
            max_r_points[th] = (rs[i], points_np[i])
        if th not in min_r_points or rs[i] < min_r_points[th][0]:
            min_r_points[th] = (rs[i], points_np[i])
    ext_pts = [v[1] for k,v in sorted(max_r_points.items())]
    int_pts = [v[1] for k,v in sorted(min_r_points.items(), reverse=True)]
    semi_annulus = np.vstack([ext_pts, int_pts, ext_pts[0:1]])
    ax.plot(semi_annulus[:,0], semi_annulus[:,1], 'r-', lw=2, label='Contorno')
    ax.set_xlabel('X (cm)', fontsize=12)
    ax.set_ylabel('Y (cm)', fontsize=12)
    ax.set_title(f'Área de Trabajo XY | Puntos alcanzados: {len(all_points)}', fontsize=14)
    ax.grid(True, which='both', linestyle='--', alpha=0.4)
    ax.axis('equal')
    ax.legend(fontsize=10)
    if save_img:
        try:
            plt.savefig('workspace_xy.png', dpi=150, bbox_inches='tight')
            print(f"✓ Gráfico XY guardado en 'workspace_xy.png'")
        except Exception as e:
            print(f"⚠ Error al guardar gráfico XY: {e}")
    plt.show()

def plot_robot_sideview(save_img: bool):
    """
    Dibuja los eslabones 1, 2, 3 y el rango de q3 en vista lateral.
    """
    if not HAS_MATPLOTLIB:
        print("⚠ matplotlib no disponible")
        return
    from kinematics import l1, l2, l3, l4, a, b
    fig, ax = plt.subplots(figsize=(8, 5))
    q3_min, q3_max = LIMITS['q3']
    # S0 (0,0), S1 (l1, a), S2 (l1+l2, a+b)
    # Ejes verticales (base a e1, e1 a e2)
    ax.plot([0, 0], [0, a], 'k:', lw=2, label='Eje base-e1')
    ax.plot([l1, l1], [a, a + b], 'k:', lw=2, label='Eje e1-e2')
    # Eslabón 1: S0 a S1 (horizontal, a)
    ax.plot([0, l1], [a, a], 'b-', lw=4, label='Eslabón 1')
    # Eslabón 2: S1 a S2 (horizontal, a+b)
    ax.plot([l1, l1 + l2], [a + b, a + b], 'g-', lw=4, label='Eslabón 2')
    # Prismatico q3: desde S2 (l1+l2, a+b) hacia abajo (q3=0 y q3=max)
    x2 = l1 + l2
    z2 = a + b
    z3_0 = z2 - q3_min
    z3_1 = z2 - q3_max
    # l3: desde extremo prismatico hacia abajo
    z4_0 = z3_0 - l3
    z4_1 = z3_1 - l3
    # l4: desde extremo l3 hacia abajo (efector)
    z5_0 = z4_0 - l4
    z5_1 = z4_1 - l4
    # Prismatico (q3=0 y q3=max), separando visualmente
    ax.plot([x2-0.5, x2-0.5], [z2, z3_0], 'r--', lw=2, label=f'Prismático (q3={q3_min} cm)')
    ax.plot([x2+0.5, x2+0.5], [z2, z3_1], 'r-', lw=4, label=f'Prismático (q3={q3_max} cm)')
    # l3 (q3=0 y q3=max)
    ax.plot([x2-0.5, x2-0.5], [z3_0, z4_0], 'm--', lw=2, label=f'l3 (q3={q3_min})')
    ax.plot([x2+0.5, x2+0.5], [z3_1, z4_1], 'm-', lw=4, label=f'l3 (q3={q3_max})')
    # l4 (efector, q3=0 y q3=max)
    ax.plot([x2-0.5, x2-0.5], [z4_0, z5_0], 'k--', lw=2, label=f'l4 (q3={q3_min})')
    ax.plot([x2+0.5, x2+0.5], [z4_1, z5_1], 'k-', lw=4, label=f'l4 (q3={q3_max})')
    # Marcar extremos
    ax.scatter([0, l1, x2, x2-0.5, x2+0.5, x2-0.5, x2+0.5], [a, a + b, z2, z3_0, z3_1, z5_0, z5_1], c='black')
    ax.set_xlabel('X (cm)', fontsize=12)
    ax.set_ylabel('Z (cm)', fontsize=12)
    ax.set_title('Vista lateral real: eslabones y extremos para q3 mínimo y máximo', fontsize=14)
    ax.grid(True, which='both', linestyle='--', alpha=0.4)
    ax.legend(fontsize=10, loc='best')
    ax.axis('equal')
    if save_img:
        try:
            plt.savefig('robot_sideview.png', dpi=150, bbox_inches='tight')
            print(f"✓ Gráfico lateral guardado en 'robot_sideview.png'")
        except Exception as e:
            print(f"⚠ Error al guardar gráfico lateral: {e}")
    plt.show()

def polygon_area(hull: List[Tuple[float, float]]) -> float:
    if len(hull) < 3:
        return 0.0
    area = 0.0
    for i in range(len(hull)):
        j = (i + 1) % len(hull)
        area += hull[i][0] * hull[j][1]
        area -= hull[j][0] * hull[i][1]
    return abs(area) / 2.0

def compute_workspace(
    step_deg: float = 5.0,
    save_csv: bool = False,
    plot: bool = False,
    save_img: bool = False
) -> Tuple[float, list, list]:
    q1_min, q1_max = LIMITS['q1']
    q2_min, q2_max = LIMITS['q2']
    q4_min, q4_max = LIMITS['q4']
    q3_fixed = LIMITS['q3'][0]  # Usar q3 mínimo
    all_points = []
    from kinematics import l1, l2
    q1 = q1_min
    while q1 <= q1_max + 1e-6:
        q2 = q2_min
        while q2 <= q2_max + 1e-6:
            q4 = q4_min
            while q4 <= q4_max + 1e-6:
                try:
                    x, y, z, phi = fk(q1, q2, q3_fixed, q4)
                    r = (x**2 + y**2)**0.5
                    min_r = abs(l1 - l2) * 0.95
                    max_r = (l1 + l2) * 1.05
                    if min_r < r < max_r and abs(q2) < 180:
                        all_points.append((x, y, z))
                except Exception:
                    pass
                q4 += step_deg
            q2 += step_deg
        q1 += step_deg
    if not all_points:
        return 0.0, [], []
    all_points_xy = [(x, y) for x, y, z in all_points]
    hull = convex_hull(all_points_xy)
    area = polygon_area(hull)
    if save_csv:
        try:
            with open('workspace_points.csv', 'w') as f:
                f.write("x,y,z\n")
                for x, y, z in all_points:
                    f.write(f"{x:.4f},{y:.4f},{z:.4f}\n")
            print(f"✓ Puntos guardados en 'workspace_points.csv' ({len(all_points)} puntos)")
        except Exception as e:
            print(f"⚠ Error al guardar CSV: {e}")
    if plot and HAS_MATPLOTLIB:
        plot_workspace(all_points, save_img)
        plot_robot_sideview(save_img)
    elif plot and not HAS_MATPLOTLIB:
        print("⚠ matplotlib no disponible; no se generará gráfico")
    return area, hull, all_points

if __name__ == "__main__":
    print("\n--- Workspace JF XV ---")
    try:
        step = float(input("Paso de muestreo para q1, q2, q3, q4 [°/cm] (default 5.0): ") or 5.0)
    except ValueError:
        step = 5.0
    save_csv = input("¿Guardar puntos alcanzables en CSV? (s/n, default n): ").strip().lower() in ("s", "si", "y", "yes", "1")
    save_img = input("¿Guardar imágenes generadas? (s/n, default n): ").strip().lower() in ("s", "si", "y", "yes", "1")

    print(f"\nCalculando workspace (paso={step}°/cm)...")
    area, hull, all_pts = compute_workspace(step_deg=step, save_csv=save_csv, plot=True, save_img=save_img)

    print(f"\nResultados:")
    print(f"  Puntos alcanzables: {len(all_pts)}")
    print(f"  Puntos en convex hull: {len(hull)}")
    print(f"  Área de trabajo (aprox convex hull): {area:.2f} cm²")

    print(f"\nContorno del área de trabajo (convex hull):")
    sample_step = max(1, len(hull) // 20)
    for i in range(0, len(hull), sample_step):
        x, y = hull[i]
        print(f"  [{i}] x={x:.2f}, y={y:.2f}")
    if len(hull) % sample_step != 0:
        x, y = hull[-1]
        print(f"  [{len(hull)-1}] x={x:.2f}, y={y:.2f}")
def convex_hull(points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
## --- Eliminar main duplicado y dejar solo el segundo (más completo) ---
    """
    Computa el envolvente convexo de puntos 2D usando monotone chain.
    Retorna lista de puntos en orden antihorario.
    """
    if len(points) <= 2:
        return sorted(points)

    sorted_pts = sorted(set(points))
    if len(sorted_pts) <= 2:
        return sorted_pts

    lower = []
    for p in sorted_pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)

    upper = []
    for p in reversed(sorted_pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)

    return lower[:-1] + upper[:-1]


def cross(o: Tuple[float, float], a: Tuple[float, float], b: Tuple[float, float]) -> float:
    """Producto cruz 2D."""
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def plot_workspace(all_points: list, save_img: bool):
    if not HAS_MATPLOTLIB:
        print("⚠ matplotlib no disponible")
        return
    fig, ax = plt.subplots(figsize=(8, 8))
    xs = [p[0] for p in all_points]
    ys = [p[1] for p in all_points]
    ax.scatter(xs, ys, c='royalblue', s=2, alpha=0.7, label='Puntos alcanzables')
    ax.set_xlabel('X (cm)', fontsize=12)
    ax.set_ylabel('Y (cm)', fontsize=12)
    ax.set_title(f'Área de Trabajo XY | Puntos alcanzados: {len(all_points)}', fontsize=14)
    ax.grid(True, which='both', linestyle='--', alpha=0.4)
    ax.axis('equal')
    ax.legend(fontsize=10)
    if save_img:
        try:
            plt.savefig('workspace_xy.png', dpi=150, bbox_inches='tight')
            print(f"✓ Gráfico XY guardado en 'workspace_xy.png'")
        except Exception as e:
            print(f"⚠ Error al guardar gráfico XY: {e}")
    plt.show()

def plot_workspace_xz(all_points: list, save_img: bool):
    if not HAS_MATPLOTLIB:
        print("⚠ matplotlib no disponible")
        return
    fig, ax = plt.subplots(figsize=(8, 8))
    xs = [p[0] for p in all_points]
    zs = [p[2] for p in all_points]
    ax.scatter(xs, zs, c='seagreen', s=2, alpha=0.7, label='Puntos alcanzables')
    ax.set_xlabel('X (cm)', fontsize=12)
    ax.set_ylabel('Z (cm)', fontsize=12)
    ax.set_title(f'Área de Trabajo XZ | Puntos alcanzados: {len(all_points)}', fontsize=14)
    ax.grid(True, which='both', linestyle='--', alpha=0.4)
    ax.axis('equal')
    ax.legend(fontsize=10)
    if save_img:
        try:
            plt.savefig('workspace_xz.png', dpi=150, bbox_inches='tight')
            print(f"✓ Gráfico XZ guardado en 'workspace_xz.png'")
        except Exception as e:
            print(f"⚠ Error al guardar gráfico XZ: {e}")
    plt.show()


def polygon_area(hull: List[Tuple[float, float]]) -> float:
    """
    Calcula el área de un polígono usando la fórmula del shoelace.
    """
    if len(hull) < 3:
        return 0.0
    area = 0.0
    for i in range(len(hull)):
        j = (i + 1) % len(hull)
        area += hull[i][0] * hull[j][1]
        area -= hull[j][0] * hull[i][1]
    return abs(area) / 2.0


def compute_workspace(
    step_deg: float = 1.0,
    q3_cm: float = 0.0,
    q4_deg: float = 0.0,
    save_csv: bool = False,
    plot: bool = False,
    save_img: bool = False
) -> Tuple[float, list, list]:
    """
    Calcula el área de trabajo del robot en el plano XY.

    Parámetros:
      - step_deg: paso de muestreo para q1 y q2 en grados (por defecto 1.0)
      - q3_cm: desplazamiento prismático fijo (por defecto 0.0, no afecta XY)
      - q4_deg: rotación del efector fija (por defecto 0.0)
      - save_csv: si True, guarda puntos en 'workspace_points.csv'
      - plot: si True, genera gráfico del área de trabajo

    Retorna:
      - (area_cm2, hull_points, all_points)
        * area_cm2: área del envolvente convexo en cm²
        * hull_points: lista de puntos del convex hull
        * all_points: todos los puntos alcanzables muestreados
    """
    q1_min, q1_max = LIMITS['q1']
    q2_min, q2_max = LIMITS['q2']
    from kinematics import l1, l2
    import numpy as np
    from multiprocessing import Pool, cpu_count
    q1_vals = np.arange(q1_min, q1_max + step_deg, step_deg)
    q2_vals = np.arange(q2_min, q2_max + step_deg, step_deg)
    q1_grid, q2_grid = np.meshgrid(q1_vals, q2_vals)
    q1_flat = q1_grid.flatten()
    q2_flat = q2_grid.flatten()
    q3_flat = np.full_like(q1_flat, q3_cm)
    q4_flat = np.full_like(q1_flat, q4_deg)
    def worker(args):
        q1, q2, q3, q4 = args
        try:
            x, y, z, phi = fk(q1, q2, q3, q4)
            r = (x**2 + y**2)**0.5
            min_r = abs(l1 - l2) * 0.95
            max_r = (l1 + l2) * 1.05
            if min_r < r < max_r and abs(q2) < 180:
                return (x, y, z)
        except Exception:
            pass
        return None
    args_list = list(zip(q1_flat, q2_flat, q3_flat, q4_flat))
    with Pool(cpu_count()) as pool:
        results = pool.map(worker, args_list)
    all_points = [pt for pt in results if pt is not None]

    if not all_points:
        return 0.0, [], []
    all_points_xy = [(x, y) for x, y, z in all_points]
    hull = convex_hull(all_points_xy)
    area = polygon_area(hull)

    if save_csv:
        try:
            with open('workspace_points.csv', 'w') as f:
                f.write("x,y,z\n")
                for x, y, z in all_points:
                    f.write(f"{x:.4f},{y:.4f},{z:.4f}\n")
            print(f"✓ Puntos guardados en 'workspace_points.csv' ({len(all_points)} puntos)")
        except Exception as e:
            print(f"⚠ Error al guardar CSV: {e}")

    if plot and HAS_MATPLOTLIB:
        plot_workspace(all_points, save_img)
        plot_workspace_xz(all_points, save_img)
    elif plot and not HAS_MATPLOTLIB:
        print("⚠ matplotlib no disponible; no se generará gráfico")

    return area, hull, all_points
