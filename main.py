import pybullet as p
import numpy as np
import time
from config import SIMULATION, PENDULUM
from utils import setup_simulation, set_initial_state
from pendulum_model import PendulumPolyController
import pandas as pd
import matplotlib.pyplot as plt

def plot_trajectory(thetas, L):
    xs = L * np.sin(thetas)
    ys = -L * np.cos(thetas)

    plt.figure(figsize=(6,6))
    plt.plot(xs, ys, '-o', markersize=2)
    plt.title("Траектория движения маятника")
    plt.xlabel("x (м)")
    plt.ylabel("y (м)")
    plt.axis('equal')
    plt.grid(True)
    plt.show()

def main():
    dt = SIMULATION['dt']
    T = SIMULATION['max_time']
    gui = SIMULATION['gui']
    θ0 = SIMULATION['theta0']

    pend = PendulumPolyController(
        m=PENDULUM['m'], L=PENDULUM['L'], g=PENDULUM['g'],
        T=T, theta0=θ0, thetaf=np.pi/2
    )

    pend_id = setup_simulation("simple.urdf.xml", gui=gui)
    joint = next(i for i in range(p.getNumJoints(pend_id))
                 if p.getJointInfo(pend_id, i)[2] == p.JOINT_REVOLUTE)

    set_initial_state(pend_id, joint, θ0)

    ts = np.arange(0, T, dt)
    thetas = np.zeros_like(ts)
    omegas = np.zeros_like(ts)
    taus = np.zeros_like(ts)

    for idx, t in enumerate(ts):
        θ, ω = p.getJointState(pend_id, joint)[:2]
        τ = pend.compute_control(θ, ω, t)
        p.setJointMotorControl2(pend_id, joint, p.TORQUE_CONTROL, force=τ)
        p.stepSimulation()
        if gui: time.sleep(dt)

        thetas[idx] = θ
        omegas[idx] = ω
        taus[idx] = τ

    p.disconnect()

    plot_trajectory(thetas, PENDULUM['L'])


if __name__=="__main__":
    main()
