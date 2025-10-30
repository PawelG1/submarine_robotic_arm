import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import math

# === Macierze przekształceń ===
def Rx(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[1, 0, 0, 0],
                     [0, c, -s, 0],
                     [0, s,  c, 0],
                     [0, 0, 0, 1]])

def Rz(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0, 0],
                     [s,  c, 0, 0],
                     [0,  0, 1, 0],
                     [0,  0, 0, 1]])

def Tz(d):
    return np.array([[1,0,0,0],
                     [0,1,0,0],
                     [0,0,1,d],
                     [0,0,0,1]])

def Tx(a):
    return np.array([[1,0,0,a],
                     [0,1,0,0],
                     [0,0,1,0],
                     [0,0,0,1]])

# === Kinematyka prosta (Z–X–X–Z–X–Z + chwytak wzdłuż Z) ===
def forward_kinematics(theta, L=0.05, Lg=0.05):
    th0, th1, th2, th3, th4, th5 = theta
    T = np.eye(4)
    points = [T[0:3,3].copy()]

    # 0: obrót wokół Z
    T = T @ Rz(th0)
    points.append(T[0:3,3].copy())

    # 1: przesunięcie + obrót wokół X
    T = T @ Tz(L) @ Rx(th1)
    points.append(T[0:3,3].copy())

    # 2: przesunięcie + obrót wokół X
    T = T @ Tz(L) @ Rx(th2)
    points.append(T[0:3,3].copy())

    # 3: przesunięcie + obrót wokół Z
    T = T @ Tz(L) @ Rz(th3)
    points.append(T[0:3,3].copy())

    # 4: przesunięcie + obrót wokół X
    T = T @ Tz(L) @ Rx(th4)
    points.append(T[0:3,3].copy())

    # 5: przesunięcie + obrót wokół Z
    T = T @ Tz(L) @ Rz(th5)
    points.append(T[0:3,3].copy())

    # 6: chwytak – przedłużenie osi Z ostatniego przegubu
    T = T @ Tz(Lg)
    points.append(T[0:3,3].copy())

    pos = T[0:3,3]
    return np.array(points).T, pos, T

# === Rysowanie 3D ===
fig = plt.figure(figsize=(7,8))
ax = fig.add_subplot(111, projection='3d')
plt.subplots_adjust(bottom=0.42)

ax.set_xlim(-0.25, 0.25)
ax.set_ylim(-0.25, 0.25)
ax.set_zlim(0, 0.4)
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')
ax.set_title('6 DOF Robot – Z–X–X–Z–X–Z')
ax.set_box_aspect([1,1,1])

# Początkowe kąty
angles_deg = [0, 0, 0, 0, 0, 0]
angles = [math.radians(a) for a in angles_deg]
points, pos, T = forward_kinematics(angles)
[line] = ax.plot(points[0,:], points[1,:], points[2,:], '-o', linewidth=3, markersize=6)
[grip] = ax.plot([pos[0]], [pos[1]], [pos[2]], 'ro', markersize=8)

# === Suwaki ===
axcolor = 'lightgoldenrodyellow'
sliders = []
names = ['θ0 (Z)', 'θ1 (X)', 'θ2 (X)', 'θ3 (Z)', 'θ4 (X)', 'θ5 (Z)']

for i in range(6):
    ax_slider = plt.axes([0.15, 0.36 - i*0.035, 0.7, 0.02], facecolor=axcolor)
    slider = Slider(ax_slider, names[i], -180, 180, valinit=angles_deg[i])
    sliders.append(slider)

# === Tekst współrzędnych ===
text_ax = plt.axes([0.15, 0.07, 0.7, 0.1])
text_ax.axis("off")
coord_text = text_ax.text(0, 0.6, "", fontsize=11, fontfamily='monospace')

def update_coords(pos):
    coord_text.set_text(f"x = {pos[0]:.3f} m\n"
                        f"y = {pos[1]:.3f} m\n"
                        f"z = {pos[2]:.3f} m")

update_coords(pos)

# === Aktualizacja po zmianie suwaka ===
def update(val):
    degs = [s.val for s in sliders]
    th = [math.radians(d) for d in degs]
    pts, pos, T = forward_kinematics(th)
    line.set_data(pts[0,:], pts[1,:])
    line.set_3d_properties(pts[2,:])
    grip.set_data([pos[0]], [pos[1]])
    grip.set_3d_properties([pos[2]])
    update_coords(pos)
    fig.canvas.draw_idle()

for s in sliders:
    s.on_changed(update)

# plt.show()  # Usunięte, aby nie blokować w ROS2
