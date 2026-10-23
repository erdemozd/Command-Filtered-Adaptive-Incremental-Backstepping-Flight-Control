"""Check held-elevator RK4 integration against SciPy's adaptive DOP853 solver.

This validates numerical propagation, not aircraft fidelity or stability proofs.
The controller sample period is unchanged; only a plant step is subdivided.
"""
import argparse

import numpy as np
from scipy.integrate import solve_ivp

from simulate_controllers import Config, plant_outputs, rk4_step


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cases', type=int, default=64)
    parser.add_argument('--seed', type=int, default=4311)
    args = parser.parse_args()
    if args.cases < 8:
        parser.error('--cases must be at least 8 for the convergence check')
    p = Config(); rng = np.random.default_rng(args.seed)
    errors = np.zeros((args.cases, 4))
    for case in range(args.cases):
        x = rng.uniform([-0.2, -0.3], [0.2, 0.3])
        delta = rng.uniform(-p.delta_max, p.delta_max)
        theta = p.theta * (1 + rng.uniform(-.3, .3, 7))
        reference = solve_ivp(lambda _, state: plant_outputs(state, delta, theta, p)[0],
                              (0, p.ts), x, method='DOP853', rtol=1e-12, atol=1e-14)
        if not reference.success:
            raise RuntimeError(reference.message)
        for level in range(4):
            steps = 2**level; state = x.copy()
            for _ in range(steps):
                state = rk4_step(state, delta, theta, p, step=p.ts/steps)
            errors[case, level] = np.max(np.abs(state-reference.y[:, -1]))
    if not np.isfinite(errors).all():
        raise AssertionError('Nonfinite integration error')
    # Fourth-order global convergence should approach 16x per halving.
    # Exclude values near adaptive-solver/float roundoff from the ratio check.
    valid = (errors[:, 1] > 1e-10) & (errors[:, 2] > 1e-12)
    if valid.sum() < args.cases//2:
        raise AssertionError('Too few resolved cases to establish convergence')
    ratio = np.median(errors[valid, 1]/errors[valid, 2])
    if ratio < 8 or errors[:, -1].max() > 1e-5:
        raise AssertionError(f'RK4 convergence failed: median ratio={ratio:.3f}, '
                             f'finest max error={errors[:, -1].max():.3e}')
    print(f'PASS: {args.cases} seeded held-input cases; reference = DOP853')
    for level in range(4):
        print(f'  {2**level} RK4 substep(s): max state error {errors[:, level].max():.3e}')
    print(f'  median refinement ratio: {ratio:.3f} (fourth-order target: 16)')
    print('  Errors use native state units: alpha [rad], q [rad/s].')


if __name__ == '__main__':
    main()
