import pybullet as p
import numpy as np
import time
from config import SIMULATION, PENDULUM
from utils import setup_simulation, set_initial_state, plot_comparison, plot_trajectory
from pendulum_model import PendulumPolyController
import matplotlib.pyplot as plt


def main():
    dt = SIMULATION['dt']
    T = SIMULATION['max_time']
    gui = SIMULATION['gui']
    θ0 = SIMULATION['theta0']

    pend = PendulumPolyController(
        m=PENDULUM['m'], L=PENDULUM['L'], g=PENDULUM['g'],
        T=T, theta0=θ0, thetaf=np.pi / 2
    )

    pend_id = setup_simulation("simple.urdf.xml", gui=gui)
    joint = next(i for i in range(p.getNumJoints(pend_id))
                 if p.getJointInfo(pend_id, i)[2] == p.JOINT_REVOLUTE)

    set_initial_state(pend_id, joint, θ0)

    ts = np.arange(0, T, dt)
    thetas = np.zeros_like(ts)
    omegas = np.zeros_like(ts)
    taus = np.zeros_like(ts)
    theta_ds = np.zeros_like(ts)  
    omega_ds = np.zeros_like(ts) 

    for idx, t in enumerate(ts):
        θ, ω = p.getJointState(pend_id, joint)[:2]
        θd, ωd, _ = pend.desired(t)
        τ = pend.compute_control(θ, ω, t)
        p.setJointMotorControl2(pend_id, joint, p.TORQUE_CONTROL, force=τ)
        p.stepSimulation()
        if gui:
            time.sleep(dt)

        thetas[idx] = θ
        omegas[idx] = ω
        taus[idx] = τ
        theta_ds[idx] = θd
        omega_ds[idx] = ωd

    p.disconnect()
    
    plot_comparison(ts, thetas, theta_ds, omegas, omega_ds)
    plot_trajectory(thetas, PENDULUM['L'])


if __name__ == "__main__":
    main()
