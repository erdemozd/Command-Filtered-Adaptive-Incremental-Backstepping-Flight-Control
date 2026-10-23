"""Independent NumPy/SciPy implementation of the MATLAB CABS/CIBS simulator.

Angles are radians internally; source physical units remain imperial.
Run directly to save the README's matched 30% uncertainty experiment as NPZ.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy.linalg import expm


@dataclass
class Config:
    ts: float = 0.01
    duration: float = 60.0
    mach: float = 2.2
    vt: float = 3109.3
    mass: float = 450 / 32.2
    qbar: float = 6132.8
    area: float = 0.44
    chord: float = 0.75
    iyy: float = 182.5
    amplitude: float = np.deg2rad(11)
    omega: float = 0.6
    delta_max: float = np.deg2rad(9.5)
    c1: float = 4.0
    c2: float = 8.0
    wn: float = 20.0
    zeta: float = 0.7
    gain_min: float = 20.0
    theta: np.ndarray = field(default_factory=lambda: np.array(
        [303.1, -246.3, -37.56, 71.51, 10.01, 12.0393, 48.2246]))
    rho_wrong: np.ndarray = field(default_factory=lambda: np.array(
        [0.80, 1.20, 0.75, 1.25, 0.85, 1.15, 0.80]))
    gamma: np.ndarray = field(default_factory=lambda: np.array(
        [0.02, 0.02, 0.02, 0.02, 0.02, 0.005, 0.005]))

    @property
    def kq(self):
        return self.qbar * self.area * self.chord / self.iyy


FULL_PATTERN = np.array([1, -0.7, 0.4, -1, 0.8, -0.6, -0.3])


def plant_outputs(x, delta, theta, p):
    """Return [alphaDot, qDot] and measured normal acceleration Az."""
    alpha, q = x
    aa = alpha * abs(alpha)
    cz = (-288.7 * alpha**3 + 50.32 * aa - 23.89 * alpha
          + p.mach * (-13.53 * aa + 4.185 * alpha))
    az = p.qbar * p.area / p.mass * (cz + (1.6238 * p.mach - 6.7240) * delta)
    cm = (theta[0] * alpha**3 + theta[1] * aa + theta[2] * alpha
          + theta[3] * p.mach * aa + theta[4] * p.mach * alpha)
    qdot = p.kq * (cm + (theta[5] * p.mach + theta[6]) * delta)
    return np.array([q + az / p.vt, qdot]), az


def rk4_step(x, delta, theta, p, step=None):
    """Held-input RK4; optional substep is used by validate_numerics.py."""
    h = p.ts if step is None else step
    k1 = plant_outputs(x, delta, theta, p)[0]
    k2 = plant_outputs(x + h * k1 / 2, delta, theta, p)[0]
    k3 = plant_outputs(x + h * k2 / 2, delta, theta, p)[0]
    k4 = plant_outputs(x + h * k3, delta, theta, p)[0]
    return x + h * (k1 + 2 * k2 + 2 * k3 + k4) / 6


def guarded_gain(g, minimum):
    return g if abs(g) >= minimum else np.copysign(minimum, g)


def simulate(controller, p=None, theta_plant=None, rho0=None, adaptation=True):
    """Mirror MATLAB's sample ordering, previous-input measurement, and logs.

    The default CABS estimate is rhoWrong, matching simulate_controller.m.
    Pass rho0=ones(7) for the matched-plant/README experiment.
    """
    p = Config() if p is None else p
    controller = controller.upper()
    if controller not in ('CABS', 'CIBS'):
        raise ValueError('controller must be CABS or CIBS')
    if p.ts <= 0 or p.duration <= 0 or not np.isfinite([p.ts, p.duration]).all():
        raise ValueError('ts and duration must be finite and positive')
    intervals = p.duration / p.ts
    if not np.isclose(intervals, round(intervals), rtol=0, atol=1e-9):
        raise ValueError('duration must be an integer multiple of ts')
    theta_plant = p.theta.copy() if theta_plant is None else np.asarray(theta_plant)
    rho = p.rho_wrong.copy() if rho0 is None else np.array(rho0, dtype=float, copy=True)
    if theta_plant.shape != (7,) or rho.shape != (7,):
        raise ValueError('theta_plant and rho0 must each have seven elements')
    augmented = np.array([[0, 1, 0], [-p.wn**2, -2*p.zeta*p.wn, p.wn**2], [0, 0, 0]])
    discrete = expm(augmented * p.ts)
    t = np.arange(round(intervals) + 1) * p.ts
    names = ('alpha q alphaRef alphaRefDot Az qDot qRaw qc qcDot z1 zbar1 '
             'z2 chi1 deltaRaw delta gInner fHat nu deltaInc').split()
    out = {name: np.full(len(t), np.nan) for name in names}
    out.update(t=t, rhoHat=np.full((len(t), 7), np.nan),
               thetaHat=np.full((len(t), 7), np.nan))
    x = np.zeros(2); cf = np.zeros(2); chi = 0.0; delta = 0.0
    for k, time in enumerate(t):
        dx, az = plant_outputs(x, delta, theta_plant, p)
        ar = p.amplitude * np.sin(p.omega * time)
        ardot = p.amplitude * p.omega * np.cos(p.omega * time)
        z1 = x[0] - ar
        qraw = -p.c1 * z1 - az / p.vt + ardot
        qc, qcdot = cf
        cf_next = discrete[:2, :2] @ cf + discrete[:2, 2] * qraw
        chidot = -p.c1 * chi + qc - qraw
        zbar = z1 - chi; z2 = x[1] - qc
        nu = np.nan; increment = np.nan; fhat = np.nan
        if controller == 'CABS':
            estimate = p.theta * rho
            yf = p.kq * np.array([x[0]**3, x[0]*abs(x[0]), x[0],
                                   p.mach*x[0]*abs(x[0]), p.mach*x[0]])
            yg = p.kq * np.array([p.mach, 1])
            fhat = yf @ estimate[:5]
            gain = guarded_gain(yg @ estimate[5:], p.gain_min)
            raw = (qcdot - p.c2*z2 - zbar - fhat) / gain
            applied = np.clip(raw, -p.delta_max, p.delta_max)
            out['rhoHat'][k] = rho
            out['thetaHat'][k] = estimate
            if adaptation and abs(applied - raw) <= 1e-12:
                psi = np.concatenate((yf, yg*applied)) * p.theta
                rhodot = p.gamma * psi * z2
                rhodot[((rho <= .4) & (rhodot < 0)) | ((rho >= 1.6) & (rhodot > 0))] = 0
                rho = np.clip(rho + p.ts*rhodot, .4, 1.6)
        else:
            gain = guarded_gain(p.kq*(p.theta[5]*p.mach + p.theta[6]), p.gain_min)
            nu = qcdot - p.c2*z2 - zbar
            increment = (nu - dx[1]) / gain
            raw = delta + increment
            applied = np.clip(raw, -p.delta_max, p.delta_max)
        values = (x[0], x[1], ar, ardot, az, dx[1], qraw, qc, qcdot, z1,
                  zbar, z2, chi, raw, applied, gain, fhat, nu, increment)
        for name, value in zip(names, values):
            out[name][k] = value
        chi += p.ts * chidot
        x = rk4_step(x, applied, theta_plant, p)
        delta = applied; cf = cf_next
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=60)
    parser.add_argument('--uncertainty', type=float, choices=(0, .1, .2, .3), default=.3)
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[1]/'results/python')
    args = parser.parse_args()
    p = Config(duration=args.duration)
    args.output.mkdir(parents=True, exist_ok=True)
    for controller in ('CABS', 'CIBS'):
        r = simulate(controller, p, p.theta*(1+args.uncertainty*FULL_PATTERN), np.ones(7))
        np.savez_compressed(args.output/f'{controller}.npz', **r)
        rmse = np.rad2deg(np.sqrt(np.mean((r['alpha']-r['alphaRef'])**2)))
        print(f'{controller}: {len(r["t"])} samples; alpha RMSE = {rmse:.8f} deg')


if __name__ == '__main__':
    main()
