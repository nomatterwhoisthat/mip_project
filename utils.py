import pybullet as p
import matplotlib.pyplot as plt
import numpy as np
import os

plt.style.use('ggplot')

def setup_simulation(urdf_path, gui=False):
    client = p.connect(p.GUI if gui else p.DIRECT)
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

def plot_results(times, thetas, velocities, torques, pend):
    fig, axes = plt.subplots(3,1, figsize=(10,8), sharex=True)
    axes[0].plot(times, thetas, label='θ (рад)')
    axes[0].plot(times, np.interp(times, times, times*0 + np.pi/2), '--', label='θ_d = π/2')
    axes[0].set_ylabel('θ')
    axes[0].legend()
    axes[1].plot(times, velocities, label='ω (рад/с)'); axes[1].set_ylabel('ω'); axes[1].legend()
    axes[2].plot(times, torques, label='τ (Н·м)'); axes[2].set_ylabel('τ'); axes[2].legend()
    axes[2].set_xlabel('Time (s)')
    plt.tight_layout()
    plt.show()
