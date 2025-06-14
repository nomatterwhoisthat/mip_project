import pybullet as p
import matplotlib.pyplot as plt
import numpy as np
import os
from config import PENDULUM

plt.style.use('ggplot')

def setup_simulation(urdf_path, gui=False):
    client = p.connect(p.GUI if gui else p.DIRECT)
    p.setGravity(0, 0, -PENDULUM['g'])
    plane = os.path.join(os.getcwd(), "plane.urdf")
    if not os.path.exists(plane):
        raise FileNotFoundError(f"plane.urdf not found: {plane}")
    p.loadURDF(plane)
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"{urdf_path} not found")
    pend = p.loadURDF(urdf_path, useFixedBase=True)
    for ji in range(p.getNumJoints(pend)):
        p.changeDynamics(pend, ji, linearDamping=0, angularDamping=0)
    return pend

def set_initial_state(pend_id, joint_idx, theta0):
    p.setJointMotorControl2(pend_id, joint_idx, controlMode=p.POSITION_CONTROL, targetPosition=theta0)
    for _ in range(1000): p.stepSimulation()
    p.setJointMotorControl2(pend_id, joint_idx, controlMode=p.VELOCITY_CONTROL, targetVelocity=0, force=0)

def plot_comparison(times, thetas, theta_ds, omegas, omega_ds):
    fig, axs = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

    axs[0].plot(times, thetas, label='θ (реальный)', linewidth=2)
    axs[0].plot(times, theta_ds, '--', label='θ (теоретический)', linewidth=2)
    axs[0].set_ylabel("Угол (рад)")
    axs[0].legend()
    axs[0].grid(True)

    axs[1].plot(times, omegas, label='ω (реальный)', linewidth=2)
    axs[1].plot(times, omega_ds, '--', label='ω (теоретический)', linewidth=2)
    axs[1].set_ylabel("Скорость (рад/с)")
    axs[1].set_xlabel("Время (с)")
    axs[1].legend()
    axs[1].grid(True)

    plt.tight_layout()
    plt.show()
    
def plot_trajectory(thetas, L):
    xs = L * np.sin(thetas)
    ys = -L * np.cos(thetas)

    plt.figure(figsize=(6, 6))
    plt.plot(xs, ys, '-o', markersize=2)
    plt.title("Траектория движения маятника")
    plt.xlabel("x (м)")
    plt.ylabel("y (м)")
    plt.axis('equal')
    plt.grid(True)
    plt.show()


