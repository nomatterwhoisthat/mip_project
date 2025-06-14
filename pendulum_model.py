import numpy as np

class PendulumPolyController:
    def __init__(self, m=1.0, L=1.0, g=9.81, T=4.0,
                 theta0=0.0, thetaf=np.pi/2,
                 kp=550, kv=20):
        self.m, self.L, self.g = m, L, g
        self.kp, self.kv = kp, kv
        self.T = T
        self.coeffs = self._poly5(0, T, theta0, thetaf)

    def _poly5(self, t0, tf, q0, qf):
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

    def desired(self, t):
        vec = [1, t, t**2, t**3, t**4, t**5]
        dvec = [0,1,2*t,3*t**2,4*t**3,5*t**4]
        ddvec = [0,0,2,6*t,12*t**2,20*t**3]
        θd = np.dot(self.coeffs, vec)
        ωd = np.dot(self.coeffs, dvec)
        αd = np.dot(self.coeffs, ddvec)
        return θd, ωd, αd

    def compute_control(self, θ, ω, t):
        θd, ωd, αd = self.desired(t)
        I = self.m*self.L**2
        u = I * (αd + self.kv*(ωd-ω) + self.kp*(θd-θ))+ self.m*self.g*self.L*np.sin(θ)
        return u