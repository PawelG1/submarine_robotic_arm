import numpy as np

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

# === Kinematyka prosta ===
def forward_kinematics(theta, L=0.05, Lg=0.05):
    th0, th1, th2, th3, th4, th5 = theta
    T = np.eye(4)
    points = [T[0:3,3].copy()]
    
    T = T @ Rz(th0)
    points.append(T[0:3,3].copy())
    
    T = T @ Tz(L) @ Rx(th1)
    points.append(T[0:3,3].copy())
    
    T = T @ Tz(L) @ Rx(th2)
    points.append(T[0:3,3].copy())
    
    T = T @ Tz(L) @ Rz(th3)
    points.append(T[0:3,3].copy())
    
    T = T @ Tz(L) @ Rx(th4)
    points.append(T[0:3,3].copy())
    
    T = T @ Tz(L) @ Rz(th5)
    points.append(T[0:3,3].copy())
    
    T = T @ Tz(Lg)
    points.append(T[0:3,3].copy())
    
    pos = T[0:3,3]
    # # Swap x and y to match RViz axes
    # pos = np.array([pos[1], pos[0], pos[2]])
    return np.array(points).T, pos, T

# === Kinematyka odwrotna (numeryczna - Jacobian) ===
def inverse_kinematics(target_pos, theta_init, L=0.05, Lg=0.05, max_iter=100, tol=1e-4):
    theta = np.array(theta_init, dtype=float)
    
    for iteration in range(max_iter):
        _, current_pos, _ = forward_kinematics(theta, L, Lg)
        error = target_pos - current_pos
        
        if np.linalg.norm(error) < tol:
            return theta, True
        
        # Oblicz Jacobian numerycznie
        J = np.zeros((3, 6))
        delta = 1e-6
        
        for i in range(6):
            theta_plus = theta.copy()
            theta_plus[i] += delta
            _, pos_plus, _ = forward_kinematics(theta_plus, L, Lg)
            J[:, i] = (pos_plus - current_pos) / delta
        
        # Damped least squares (aby uniknąć osobliwości)
        lambda_damping = 0.01
        delta_theta = J.T @ np.linalg.inv(J @ J.T + lambda_damping * np.eye(3)) @ error
        
        # Aktualizacja z krokiem adaptacyjnym
        alpha = 0.5
        theta += alpha * delta_theta
        
        # Ograniczenia kątów
        theta = np.clip(theta, -np.pi, np.pi)
    
    return theta, False