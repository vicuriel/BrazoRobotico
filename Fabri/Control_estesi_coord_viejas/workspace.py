import math
from typing import List, Tuple, Optional
try:
    from kinematics import fk, l1, l2, l3, l4, a, b, e
except ImportError:
    from kinematics import fk, l1, l2, l3, l4, a, b
    e = 1.47  # Valor por defecto igual al de kinematics.py
from config_limits import LIMITS

# Optional plotting imports
try:
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.patches import Rectangle, Arc
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

def _add_rectangle(ax, *args, **kwargs):
    rect = Rectangle(*args, **kwargs)
    ax.add_patch(rect)
    return rect

def _add_arc(ax, *args, **kwargs):
    arc = Arc(*args, **kwargs)
    ax.add_patch(arc)
    return arc

def plot_workspace(all_points: list, save_img: bool):
    if not HAS_MATPLOTLIB:
        print("⚠ matplotlib no disponible")
        return
    fig, ax = plt.subplots(figsize=(8, 8))
    xs = [p[0] for p in all_points]
    ys = [p[1] for p in all_points]
    ax.scatter(xs, ys, c='royalblue', s=2, alpha=0.7, label='Puntos alcanzables')
    # Dibuja los ejes X e Y como vectores
    max_x = max(xs) if xs else 10
    max_y = max(ys) if ys else 10
    eje_x_len = max_x * 0.5
    eje_y_len = max_y * 0.5
    ax.arrow(0, 0, eje_x_len, 0, head_width=max_y * 0.04, head_length=max_x * 0.08, fc='crimson', ec='crimson', lw=2, length_includes_head=True, label='Eje X')
    ax.arrow(0, 0, 0, eje_y_len, head_width=max_x * 0.04, head_length=max_y * 0.08, fc='seagreen', ec='seagreen', lw=2, length_includes_head=True, label='Eje Y')
    ax.text(eje_x_len * 1.04, 0, 'X', color='crimson', fontsize=13, fontweight='bold', va='center', ha='left')
    ax.text(0, eje_y_len * 1.04, 'Y', color='seagreen', fontsize=13, fontweight='bold', va='bottom', ha='center')
    # Dibuja la base como rectángulo (de x=-15 a x=0, ancho 12)
    base_x0, base_x1 = -15, 0
    base_w = 12
    _add_rectangle(ax, (base_x0, -base_w/2), base_x1-base_x0, base_w, color='dimgray', alpha=0.7)
    # Eslabón 1 y 2: solo contorno superior e inferior y extremos semicirculares, sin líneas verticales
    eslabon_w = 10
    # Eslabón 1
    ax.plot([0, l1], [eslabon_w/2, eslabon_w/2], color='black', lw=2, zorder=2, label='Robot')
    ax.plot([0, l1], [-eslabon_w/2, -eslabon_w/2], color='black', lw=2, zorder=2)
    _add_arc(ax, (0, 0), eslabon_w, eslabon_w, angle=0, theta1=90, theta2=270, edgecolor='black', lw=2, zorder=2)
    # Eslabón 2
    ax.plot([l1, l1+l2], [eslabon_w/2, eslabon_w/2], color='black', lw=2, zorder=3)
    ax.plot([l1, l1+l2], [-eslabon_w/2, -eslabon_w/2], color='black', lw=2, zorder=3)
    _add_arc(ax, (l1+l2, 0), eslabon_w, eslabon_w, angle=0, theta1=270, theta2=90, edgecolor='black', lw=2, zorder=3)
    _add_arc(ax, (l1, 0), eslabon_w, eslabon_w, angle=0, theta1=90, theta2=270, edgecolor='black', lw=2, zorder=3)
    # Contorno tipo semi-anillo (no convex hull):
    points_np = np.array([(x, y) for x, y, _ in all_points])
    rs = np.linalg.norm(points_np, axis=1)
    thetas = np.arctan2(points_np[:,1], points_np[:,0])
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
    ax.set_title('Workspace XY', fontsize=14)
    ax.grid(True, which='both', linestyle='--', alpha=0.25, linewidth=0.7)
    ax.set_xticks(np.arange(int(ax.get_xlim()[0]), int(ax.get_xlim()[1])+2, 2))
    ax.set_yticks(np.arange(int(ax.get_ylim()[0]), int(ax.get_ylim()[1])+2, 2))
    ax.axis('equal')
    # Marcar la posición del efector solo en la posición inicial XY (q1=0, q2=0)
    eff_x, eff_y, _, _ = fk(0, 0, 0, 0)
    ax.plot(eff_x, eff_y, 'o', markersize=13, markeredgecolor='orangered', markerfacecolor='orange', label='Extremo', zorder=10)
    handles, labels = ax.get_legend_handles_labels()
    # Reordenar leyenda: puntos, contorno, robot, extremo
    order = []
    for name in ['Puntos alcanzables', 'Contorno', 'Robot', 'Extremo']:
        for i, lab in enumerate(labels):
            if lab == name:
                order.append(i)
    if len(order) == 4:
        ax.legend([handles[i] for i in order], [labels[i] for i in order], fontsize=10)
    else:
        ax.legend(fontsize=10)
    if save_img:
        try:
            plt.savefig('workspace_xy.png', dpi=150, bbox_inches='tight')
            print(f"✓ Gráfico XY guardado en 'workspace_xy.png'")
        except Exception as e:
            print(f"⚠ Error al guardar gráfico XY: {e}")
    plt.show()

def plot_robot_sideview(save_img: bool):
    global e  # Asegura que 'e' sea la global importada
    """
    Dibuja los eslabones 1, 2, 3 y el rango de q3 en vista lateral.
    """
    if not HAS_MATPLOTLIB:
        print("⚠ matplotlib no disponible")
        return
    fig, ax = plt.subplots(figsize=(8, 5))
    q3_min, q3_max = LIMITS['q3']
    # BASE
    _add_rectangle(ax, (-15, 0), 18, a, edgecolor='black', facecolor='dimgray', lw=2, label='Robot', zorder=1, alpha=0.7)
    # ESLABÓN 1
    _add_rectangle(ax, (-1, a), l1+2, b, edgecolor='black', facecolor='none', lw=2, label='Eslabón 1', zorder=2)
    # ESLABÓN 2
    _add_rectangle(ax, (l1-1, a+b), l2+1.5, 1, edgecolor='black', facecolor='none', lw=2, label='Robot', zorder=3)
    # ESLABÓN 3
    x3 = l1 + l2
    z3_top = a + b
    _add_rectangle(ax, (x3, z3_top - l3), 1, l3, edgecolor='black', facecolor='none', lw=2, label='Eslabón 3', zorder=4)
    # Plataforma
    plat_x = x3 + e
    plat_z = z3_top - l3
    _add_rectangle(ax, (plat_x - 0.75, plat_z - 1), 1.5, 1, edgecolor='black', facecolor='none', lw=2, label=None, zorder=5)
    # Efector
    eff_x = plat_x
    eff_z_top = plat_z - 1
    _add_rectangle(ax, (eff_x, eff_z_top - l4), 1, l4, edgecolor='black', facecolor='none', lw=2, label=None, zorder=6)
    # Marcar extremos principales (base y articulaciones)
    ax.scatter([0, l1], [a, a + b], c='black', zorder=10)
    # Marcar extremos para q3=0 y q3=max usando fk para obtener la z real
    x_extremo = l1 + l2 + 0.5 + e
    z_extremo_0 = fk(0, 0, 0, 0)[2]
    z_extremo_1 = fk(0, 0, 4.4, 0)[2]
    ax.scatter([x_extremo], [z_extremo_0], c='orangered', s=80, zorder=20, label='Extremo q3 = 0')
    ax.scatter([x_extremo], [z_extremo_1], c='deepskyblue', s=80, zorder=20, label='Extremo q3 = 4.4')
    ax.text(x_extremo + 0.7, z_extremo_0, f"z={z_extremo_0:.2f}", color='orangered', fontsize=11, va='center', ha='left', fontweight='bold')
    ax.text(x_extremo + 0.7, z_extremo_1, f"z={z_extremo_1:.2f}", color='deepskyblue', fontsize=11, va='center', ha='left', fontweight='bold')
    # Dibujo validación: líneas originales
    ax.plot([0, 0], [0, a], 'k:', lw=2)
    ax.plot([l1, l1], [a, a + b], 'k:', lw=2)
    # ax.plot([0, l1], [a, a], 'b-', lw=4)  # Eliminada línea azul l1
    # ax.plot([l1, l1 + l2], [a + b, a + b], 'g-', lw=4)  # Eliminada línea verde l2
    ax.plot([x_extremo, x_extremo], [z_extremo_0, z_extremo_1], 'r-', lw=3, label='q3')
    # Contorno del workspace XZ (si all_points está disponible)
    try:
        from inspect import currentframe, getouterframes
        frame = currentframe()
        outer = getouterframes(frame)
        all_points = outer[1].frame.f_locals.get('all_points', None)
    except Exception:
        all_points = None
    if all_points is not None and len(all_points) > 0:
        xz_points = [(x, z) for x, _, z in all_points]
        hull = convex_hull(xz_points)
        if hull and len(hull) > 2:
            hull_np = np.array(hull + [hull[0]])
            ax.plot(hull_np[:,0], hull_np[:,1], 'r-', lw=2, label='Contorno')
    ax.set_xlabel('X (cm)', fontsize=12)
    ax.set_ylabel('Z (cm)', fontsize=12)
    ax.set_title('Workspace XZ', fontsize=14)
    ax.grid(True, which='both', linestyle='--', alpha=0.4)
    ax.set_xticks(np.arange(int(ax.get_xlim()[0]), int(ax.get_xlim()[1])+2, 2))
    ax.set_yticks(np.arange(int(ax.get_ylim()[0]), int(ax.get_ylim()[1])+2, 2))
    # Ejes X y Z como vectores (más cortos)
    xlim = ax.get_xlim()
    zlim = ax.get_ylim()
    eje_x_len = (xlim[1] - xlim[0]) * 0.18
    eje_z_len = (zlim[1] - zlim[0]) * 0.18
    ax.arrow(0, 0, eje_x_len, 0, head_width=(zlim[1]-zlim[0])*0.025, head_length=(xlim[1]-xlim[0])*0.045, fc='crimson', ec='crimson', lw=2, length_includes_head=True, label='Eje X')
    ax.arrow(0, 0, 0, eje_z_len, head_width=(xlim[1]-xlim[0])*0.025, head_length=(zlim[1]-zlim[0])*0.045, fc='seagreen', ec='seagreen', lw=2, length_includes_head=True, label='Eje Z')
    ax.text(eje_x_len * 1.08, 0, 'X', color='crimson', fontsize=13, fontweight='bold', va='center', ha='left')
    ax.text(0, eje_z_len * 1.08, 'Z', color='seagreen', fontsize=13, fontweight='bold', va='bottom', ha='center')
    # Leyenda ordenada (solo un 'Robot', el que NO tiene el rectángulo pintado)
    handles, labels = ax.get_legend_handles_labels()
    # Filtrar duplicados y dejar solo el 'Robot' de Eslabón 1 (sin relleno)
    legend_dict = {}
    for h, l in zip(handles, labels):
        if l == 'Robot':
            # Solo agregar el primer 'Robot' que NO es el rectángulo base (que tiene facecolor distinto de 'none')
            if not legend_dict.get('Robot'):
                # Si es un Rectangle y su facecolor es 'none', es el correcto
                if hasattr(h, 'get_facecolor') and (h.get_facecolor() == (0.0, 0.0, 0.0, 0.0) or h.get_facecolor() == (1.0, 1.0, 1.0, 0.0)):
                    legend_dict['Robot'] = h
            continue
        legend_dict[l] = h
    order = []
    for name in ['Robot', 'Extremo q3 = 0', 'Extremo q3 = 4.4']:
        if name in legend_dict:
            order.append(name)
    if order:
        ax.legend([legend_dict[n] for n in order], order, fontsize=10, loc='best')
    else:
        ax.legend(fontsize=10, loc='best')
    ax.axis('equal')
    if save_img:
        try:
            plt.savefig('workspace_xz.png', dpi=150, bbox_inches='tight')
            print(f"✓ Gráfico lateral guardado en 'workspace_xz.png'")
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
    plot: bool = False,
    save_img: bool = False
) -> Tuple[float, list, list]:
    q1_min, q1_max = LIMITS['q1']
    q2_min, q2_max = LIMITS['q2']
    q4_min, q4_max = LIMITS['q4']
    q3_fixed = LIMITS['q3'][0]  # Usar q3 mínimo
    all_points = []
    q1 = q1_min
    while q1 <= q1_max + 1e-6:
        q2 = q2_min
        while q2 <= q2_max + 1e-6:
            q4 = q4_min
            while q4 <= q4_max + 1e-6:
                try:
                    x, y, z, phi = fk(q1, q2, q3_fixed, q4)
                    if abs(q2) < 180:
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
    # save_csv = input("¿Guardar puntos alcanzables en CSV? (s/n, default n): ").strip().lower() in ("s", "si", "y", "yes", "1")
    save_img = input("¿Guardar imágenes generadas? (s/n, default n): ").strip().lower() in ("s", "si", "y", "yes", "1")

    print(f"\nCalculando workspace (paso={step}°/cm)...")
    area, hull, all_pts = compute_workspace(step_deg=step, plot=True, save_img=save_img)

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
    ax.set_xticks(np.arange(int(ax.get_xlim()[0]), int(ax.get_xlim()[1])+2, 2))
    ax.set_yticks(np.arange(int(ax.get_ylim()[0]), int(ax.get_ylim()[1])+2, 2))
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
    # Calcular y graficar el contorno (convex hull) en XZ solo si hay suficientes puntos
    xz_points = [(x, z) for x, _, z in all_points]
    hull = convex_hull(xz_points)
    if hull and len(hull) > 2:
        import numpy as np
        hull_np = np.array(hull + [hull[0]])
        ax.plot(hull_np[:,0], hull_np[:,1], 'r-', lw=2, label='Contorno')
    ax.set_xlabel('X (cm)', fontsize=12)
    ax.set_ylabel('Z (cm)', fontsize=12)
    ax.set_title(f'Área de Trabajo XZ | Puntos alcanzados: {len(all_points)}', fontsize=14)
    ax.grid(True, which='both', linestyle='--', alpha=0.4)
    ax.set_xticks(np.arange(int(ax.get_xlim()[0]), int(ax.get_xlim()[1])+2, 2))
    ax.set_yticks(np.arange(int(ax.get_ylim()[0]), int(ax.get_ylim()[1])+2, 2))
    ax.axis('equal')
    # Ejes X y Z como vectores (más cortos)
    xlim = ax.get_xlim()
    zlim = ax.get_ylim()
    eje_x_len = (xlim[1] - xlim[0]) * 0.18
    eje_z_len = (zlim[1] - zlim[0]) * 0.18
    ax.arrow(0, 0, eje_x_len, 0, head_width=(zlim[1]-zlim[0])*0.025, head_length=(xlim[1]-xlim[0])*0.045, fc='crimson', ec='crimson', lw=2, length_includes_head=True, label='Eje X')
    ax.arrow(0, 0, 0, eje_z_len, head_width=(xlim[1]-xlim[0])*0.025, head_length=(zlim[1]-zlim[0])*0.045, fc='seagreen', ec='seagreen', lw=2, length_includes_head=True, label='Eje Z')
    ax.text(eje_x_len * 1.08, 0, 'X', color='crimson', fontsize=13, fontweight='bold', va='center', ha='left')
    ax.text(0, eje_z_len * 1.08, 'Z', color='seagreen', fontsize=13, fontweight='bold', va='bottom', ha='center')
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
    plot: bool = False,
    save_img: bool = False
) -> Tuple[float, list, list]:
    """
    Calcula el área de trabajo del robot en el plano XY.

    Parámetros:
      - step_deg: paso de muestreo para q1 y q2 en grados (por defecto 1.0)
      - q3_cm: desplazamiento prismático fijo (por defecto 0.0, no afecta XY)
      - q4_deg: rotación del efector fija (por defecto 0.0)
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

    if plot and HAS_MATPLOTLIB:
        plot_workspace(all_points, save_img)
        plot_workspace_xz(all_points, save_img)
    elif plot and not HAS_MATPLOTLIB:
        print("⚠ matplotlib no disponible; no se generará gráfico")

    return area, hull, all_points