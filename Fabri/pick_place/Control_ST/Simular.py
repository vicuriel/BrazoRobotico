import matplotlib.pyplot as plt
import numpy as np
import time
import math
from matplotlib.patches import Arc
from kinematics import l1, l2

def simular_pick_and_place(q_inicial, q_pick, q_place, q_home):
    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(-l1-l2-5, l1+l2+5)
    ax.set_ylim(-l1-l2-5, l1+l2+5)
    ax.set_xlabel('X (cm)')
    ax.set_ylabel('Y (cm)')
    ax.set_title('Simulación Pick & Place')
    ax.grid(True, which='both', linestyle='--', alpha=0.4)
    # Ejes XY
    ax.arrow(0, 0, l1+l2+2, 0, head_width=2, head_length=2, fc='crimson', ec='crimson', lw=2, length_includes_head=True)
    ax.arrow(0, 0, 0, l1+l2+2, head_width=2, head_length=2, fc='seagreen', ec='seagreen', lw=2, length_includes_head=True)
    ax.text(l1+l2+3, 0, 'X', color='crimson', fontsize=13, fontweight='bold', va='center', ha='left')
    ax.text(0, l1+l2+3, 'Y', color='seagreen', fontsize=13, fontweight='bold', va='bottom', ha='center')

    def plot_scara(q, color='b', label=None, annotate_angles=False):
        q1, q2, q3, q4 = q
        x0, y0 = 0, 0
        x1 = l1 * np.cos(np.radians(q1))
        y1 = l1 * np.sin(np.radians(q1))
        x2 = x1 + l2 * np.cos(np.radians(q1 + q2))
        y2 = y1 + l2 * np.sin(np.radians(q1 + q2))
        ax.plot([x0, x1], [y0, y1], '-o', color=color, lw=3, label=label)
        ax.plot([x1, x2], [y1, y2], '-o', color=color, lw=3)
        ax.plot(x2, y2, 'o', color=color, markersize=10)
        if annotate_angles:
            # q1 respecto a X
            arc1 = Arc((0,0), 6, 6, angle=0, theta1=0, theta2=q1, color=color, lw=2)
            ax.add_patch(arc1)
            ang1_x = 3 * np.cos(np.radians(q1/2))
            ang1_y = 3 * np.sin(np.radians(q1/2))
            ax.text(ang1_x, ang1_y, f"∠q₁ = {q1:.1f}°", color=color, fontsize=11, fontweight='bold', va='bottom', ha='left')
            # q2 respecto al eslabón 1 (ángulo relativo)
            arc2 = Arc((x1, y1), 4, 4, angle=q1, theta1=0, theta2=q2, color=color, lw=2)
            ang2_x = x1 + 2 * np.cos(np.radians(q1 + q2/2))
            ang2_y = y1 + 2 * np.sin(np.radians(q1 + q2/2))
            ax.add_patch(arc2)
            ax.text(ang2_x, ang2_y, f"∠q₂ = {q2:.1f}°", color=color, fontsize=11, fontweight='bold', va='bottom', ha='left')

    tray = [q_inicial, q_pick, q_place, q_home]
    colores = ['b', 'orange', 'm', 'g']
    labels = ['Inicio', 'Pick', 'Place', 'Home']
    for i, q in enumerate(tray):
        ax.clear()
        ax.set_xlim(-l1-l2-5, l1+l2+5)
        ax.set_ylim(-l1-l2-5, l1+l2+5)
        ax.set_xlabel('X (cm)')
        ax.set_ylabel('Y (cm)')
        ax.grid(True, which='both', linestyle='--', alpha=0.4)
        # Ejes XY
        ax.arrow(0, 0, l1+l2+2, 0, head_width=2, head_length=2, fc='crimson', ec='crimson', lw=2, length_includes_head=True)
        ax.arrow(0, 0, 0, l1+l2+2, head_width=2, head_length=2, fc='seagreen', ec='seagreen', lw=2, length_includes_head=True)
        ax.text(l1+l2+3, 0, 'X', color='crimson', fontsize=13, fontweight='bold', va='center', ha='left')
        ax.text(0, l1+l2+3, 'Y', color='seagreen', fontsize=13, fontweight='bold', va='bottom', ha='center')
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
        time.sleep(3)
    plt.ioff()
    plt.show()

if __name__ == "__main__":
    # Ejemplo de uso sin robot ni cámara
    q_inicial = [0.0, 0.0, 5.0, 0.0]
    q_pick = [30.0, -45.0, 2.0, 0.0]
    q_place = [60.0, -30.0, 2.0, 0.0]
    q_home = [0.0, 0.0, 5.0, 0.0]
    simular_pick_and_place(q_inicial, q_pick, q_place, q_home)
