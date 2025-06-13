# pendulum_control.py
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# Физические параметры маятника
m = 1.0     # масса (кг)
l = 1.0     # длина (м)
g = 9.81    # ускорение свободного падения (м/с^2)

# Параметры управления
kp = 100
kv = 20

# Время моделирования
T = 4.0

# Начальное и конечное положения (в радианах)
theta_0 = 0.0
theta_f = np.pi / 2


def generate_5th_order_poly(t0, tf, q0, qf):
    """
    Возвращает коэффициенты полинома 5-го порядка, интерполирующего траекторию
    с нулевыми начальными и конечными скоростью и ускорением.
    """
    M = np.array([
        [1, t0, t0**2, t0**3, t0**4, t0**5],
        [0, 1, 2*t0, 3*t0**2, 4*t0**3, 5*t0**4],
        [0, 0, 2, 6*t0, 12*t0**2, 20*t0**3],
        [1, tf, tf**2, tf**3, tf**4, tf**5],
        [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
        [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3]
    ])
    b = np.array([q0, 0, 0, qf, 0, 0])
    return np.linalg.solve(M, b)


coeffs = generate_5th_order_poly(0, T, theta_0, theta_f)

def desired_trajectory(t):
    T_vec = np.array([1, t, t**2, t**3, t**4, t**5])
    dT_vec = np.array([0, 1, 2*t, 3*t**2, 4*t**3, 5*t**4])
    ddT_vec = np.array([0, 0, 2, 6*t, 12*t**2, 20*t**3])
    
    theta_d = np.dot(coeffs, T_vec)
    dtheta_d = np.dot(coeffs, dT_vec)
    ddtheta_d = np.dot(coeffs, ddT_vec)
    return theta_d, dtheta_d, ddtheta_d


def pendulum_ode(t, y):
    theta, dtheta = y
    theta_d, dtheta_d, ddtheta_d = desired_trajectory(t)
    
    # Линеаризация обратной связью
    u = m*l**2*(ddtheta_d + kv*(dtheta_d - dtheta) + kp*(theta_d - theta)) + m*g*l*np.sin(theta)
    ddtheta = (u - m*g*l*np.sin(theta)) / (m*l**2)
    return [dtheta, ddtheta]

#ОДУ
sol = solve_ivp(pendulum_ode, [0, T], [theta_0, 0], t_eval=np.linspace(0, T, 300))

# Получение траекторий
theta_traj = sol.y[0]
dtheta_traj = sol.y[1]
time = sol.t

# Желаемая траектория
theta_d_traj = []
dtheta_d_traj = []
for t in time:
    td, dtd, _ = desired_trajectory(t)
    theta_d_traj.append(td)
    dtheta_d_traj.append(dtd)

# Визуализация
plt.figure(figsize=(12, 5))
plt.subplot(1, 2, 1)
plt.plot(time, theta_traj, label='Actual')
plt.plot(time, theta_d_traj, '--', label='Desired')
plt.title('Position (theta)')
plt.xlabel('Time [s]')
plt.ylabel('Theta [rad]')
plt.legend()

plt.subplot(1, 2, 2)
plt.plot(time, dtheta_traj, label='Actual')
plt.plot(time, dtheta_d_traj, '--', label='Desired')
plt.title('Velocity (dtheta)')
plt.xlabel('Time [s]')
plt.ylabel('Angular Velocity [rad/s]')
plt.legend()

plt.tight_layout()
plt.show()
