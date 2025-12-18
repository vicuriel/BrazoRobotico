import matplotlib.pyplot as plt
import numpy as np
import time
from matplotlib.patches import Arc
from kinematics import l1, l2

def simular_pick_and_place(q_inicial, q_pick, q_place, q_home):
    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(-l1-l2-5, l1+l2+5)
    ax.set_ylim(-l1-l2-5, l1+l2+5)
    ax.set_xlabel('Y (cm)')
    ax.set_ylabel('X (cm)')
    ax.set_title('Simulación Pick & Place (Vista Rotada 90°)')
    ax.grid(True, which='both', linestyle='--', alpha=0.4)
    # Ejes YX (rotados)
    ax.arrow(0, 0, 0, l1+l2+2, head_width=2, head_length=2, fc='crimson', ec='crimson', lw=2, length_includes_head=True)
    ax.arrow(0, 0, -(l1+l2+2), 0, head_width=2, head_length=2, fc='seagreen', ec='seagreen', lw=2, length_includes_head=True)
    ax.text(0, l1+l2+3, 'X', color='crimson', fontsize=13, fontweight='bold', va='center', ha='left')
    ax.text(-(l1+l2+3), 0, 'Y', color='seagreen', fontsize=13, fontweight='bold', va='bottom', ha='center')

    def plot_scara(q, color='b', label=None, annotate_angles=False):
        q1, q2, q3, q4 = q
        x0, y0 = 0, 0
        # Rotar 90° antihorario: (x, y) -> (-y, x)
        x1 = l1 * np.cos(np.radians(q1))
        y1 = l1 * np.sin(np.radians(q1))
        x2 = x1 + l2 * np.cos(np.radians(q1 + q2))
        y2 = y1 + l2 * np.sin(np.radians(q1 + q2))
        # Aplicar rotación a todos los puntos
        x0r, y0r = -y0, x0
        x1r, y1r = -y1, x1
        x2r, y2r = -y2, x2
        ax.plot([x0r, x1r], [y0r, y1r], '-o', color=color, lw=3, label=label)
        ax.plot([x1r, x2r], [y1r, y2r], '-o', color=color, lw=3)
        ax.plot(x2r, y2r, 'o', color=color, markersize=10)
        if annotate_angles:
            # q1 respecto a X (rotado)
            arc1 = Arc((0,0), 6, 6, angle=90, theta1=0, theta2=q1, color=color, lw=2)
            ax.add_patch(arc1)
            # El texto del ángulo q1 se coloca cerca del arco, no flotando
            ang1_x = -3 * np.sin(np.radians(q1/2))
            ang1_y = 3 * np.cos(np.radians(q1/2))
            ax.text(ang1_x, ang1_y, f"∠q₁ = {q1:.1f}°", color=color, fontsize=11, fontweight='bold', va='center', ha='center', bbox=dict(facecolor='white', alpha=0.7, edgecolor='none', boxstyle='round,pad=0.2'))
            # q2 respecto al eslabón 1 (rotado)
            arc2 = Arc((x1r, y1r), 4, 4, angle=90+q1, theta1=0, theta2=q2, color=color, lw=2)
            ax.add_patch(arc2)
            ang2_x = x1r - 2 * np.sin(np.radians(q1 + q2/2))
            ang2_y = y1r + 2 * np.cos(np.radians(q1 + q2/2))
            ax.text(ang2_x, ang2_y, f"∠q₂ = {q2:.1f}°", color=color, fontsize=11, fontweight='bold', va='center', ha='center', bbox=dict(facecolor='white', alpha=0.7, edgecolor='none', boxstyle='round,pad=0.2'))

    # Nueva secuencia: Search (q1=110), Pick (cámara), Place, Home
    q_search = [110.0, 0.0, 3.0, 0.0]  # q1=110, resto neutro
    tray = [q_search, q_pick, q_place, q_home]
    colores = ['c', 'orange', 'm', 'g']
    labels = ['Búsqueda', 'Pick', 'Place', 'Home']
    for i, q in enumerate(tray):
        ax.clear()
        ax.set_xlim(-l1-l2-5, l1+l2+5)
        ax.set_ylim(-l1-l2-5, l1+l2+5)
        ax.set_xlabel('Y (cm)')
        ax.set_ylabel('X (cm)')
        ax.grid(True, which='both', linestyle='--', alpha=0.4)
        # Ejes YX (rotados)
        ax.arrow(0, 0, 0, l1+l2+2, head_width=2, head_length=2, fc='crimson', ec='crimson', lw=2, length_includes_head=True)
        ax.arrow(0, 0, -(l1+l2+2), 0, head_width=2, head_length=2, fc='seagreen', ec='seagreen', lw=2, length_includes_head=True)
        ax.text(0, l1+l2+3, 'X', color='crimson', fontsize=13, fontweight='bold', va='center', ha='left')
        ax.text(-(l1+l2+3), 0, 'Y', color='seagreen', fontsize=13, fontweight='bold', va='bottom', ha='center')
        # Robot en color actual
        plot_scara(q, color=colores[i], label=labels[i], annotate_angles=True)
        ax.legend()
        # Mensaje de subida/bajada en z
        z_msg = ''
        if i == 1 and abs(q[2] - tray[0][2]) > 1e-2:
            z_dir = 'bajando' if q[2] < tray[0][2] else 'subiendo'
            z_msg = f'{z_dir.capitalize()} en z = {q[2]:.2f} cm'
        elif i == 2 and abs(q[2] - tray[1][2]) > 1e-2:
            z_dir = 'bajando' if q[2] < tray[1][2] else 'subiendo'
            z_msg = f'{z_dir.capitalize()} en z = {q[2]:.2f} cm'
        ax.set_title(f'Simulación Pick & Place: {labels[i]}' + (f'\n{z_msg}' if z_msg else ''))
        plt.draw()
        plt.pause(0.1)
        msg = f"Haz clic o presiona una tecla en la ventana para avanzar a la siguiente etapa: {labels[i+1] if i+1 < len(labels) else 'Fin'}..."
        print(msg)
        plt.waitforbuttonpress()
    plt.ioff()
    plt.show()

if __name__ == "__main__":
    pass