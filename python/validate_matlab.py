"""Compare the independent Python implementation to tracked MATLAB histories.

Run from any directory. Exits nonzero for stale configuration, mismatched
histories/NaN masks, nonfinite physical states, or exceeded actuator bounds.
The fixtures are numerical regression references, not experimental flight data.
"""
import argparse
from pathlib import Path

import numpy as np
from scipy.io import loadmat

from simulate_controllers import Config, simulate

ROOT = Path(__file__).resolve().parents[1]


def compare(label, controller, reference, theta, p, rho0=None, adaptation=True):
    result = simulate(controller, p, theta, rho0, adaptation)
    worst = (0.0, '')
    for name, actual in result.items():
        expected = np.asarray(reference[name])
        if actual.shape != expected.shape:
            raise AssertionError(f'{label}/{name}: shape mismatch')
        if not np.array_equal(np.isnan(actual), np.isnan(expected)):
            raise AssertionError(f'{label}/{name}: NaN-mask mismatch')
        if not np.allclose(actual, expected, rtol=1e-7, atol=1e-8, equal_nan=True):
            raise AssertionError(f'{label}/{name}: histories disagree (rtol=1e-7, atol=1e-8)')
        finite = np.isfinite(actual) & np.isfinite(expected)
        error = np.max(np.abs(actual[finite]-expected[finite])) if finite.any() else 0
        if error > worst[0]:
            worst = (float(error), name)
    for name in ('alpha', 'q', 'delta'):
        if not np.isfinite(result[name]).all():
            raise AssertionError(f'{label}/{name}: nonfinite physical signal')
    if np.max(np.abs(result['delta'])) > p.delta_max+1e-12:
        raise AssertionError(f'{label}: actuator limit exceeded')
    if controller == 'CABS' and not np.all((result['rhoHat'] >= .4) & (result['rhoHat'] <= 1.6)):
        raise AssertionError(f'{label}: adaptive estimate outside projection bounds')
    print(f'PASS {label}: {len(result["t"])} samples; worst |difference| '
          f'{worst[0]:.3e} ({worst[1] or "all exact"}, native units)')


def check_fixture_config(data, p):
    """Fail explicitly if a reference uses settings this port does not implement."""
    plant = data['P']; ctrl = data['C']
    mapping = dict(Ts=p.ts, tEnd=p.duration, M=p.mach, VT=p.vt, m=p.mass,
                   qbar=p.qbar, S=p.area, cbar=p.chord, Iyy=p.iyy,
                   alphaRefAmp=p.amplitude, omegaRef=p.omega, deltaMax=p.delta_max,
                   deltaMin=-p.delta_max, alpha0=0, q0=0, delta0=0)
    for key, value in mapping.items():
        if not np.allclose(plant[key], value, rtol=1e-12, atol=1e-14):
            raise AssertionError(f'Unsupported fixture setting: P.{key}')
    for key, value in dict(c1=p.c1, c2=p.c2, wn=p.wn, zeta=p.zeta,
                           chi10=0, gHatMin=p.gain_min).items():
        if not np.isclose(ctrl[key], value):
            raise AssertionError(f'Unsupported fixture setting: C.{key}')
    for value in (plant['thetaTrue'], ctrl['CABS']['thetaNom'], ctrl['CIBS']['thetaCtrl']):
        if not np.allclose(value, p.theta):
            raise AssertionError('Unsupported nominal parameter vector')
    cabs = ctrl['CABS']
    if not (np.allclose(cabs['Gamma'], np.diag(p.gamma))
            and np.allclose(cabs['rhoWrong'], p.rho_wrong)
            and np.allclose(cabs['rhoPerfect'], 1)
            and np.allclose(cabs['rhoMin'], .4) and np.allclose(cabs['rhoMax'], 1.6)
            and cabs['freezeOnSaturation']):
        raise AssertionError('Unsupported adaptation configuration')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results-dir', type=Path, default=ROOT/'results')
    args = parser.parse_args(); p = Config(); count = 0
    # CABS perfect/wrong-model cases also exercise adaptation being disabled.
    data = loadmat(args.results_dir/'CABS_comparison.mat', simplify_cells=True)
    check_fixture_config(data, p)
    for key, rho, adaptation in [('perfect', np.ones(7), False),
                                  ('wrongOff', p.rho_wrong, False),
                                  ('wrongOn', p.rho_wrong, True)]:
        compare(f'CABS/{key}', 'CABS', data['CABS'][key], p.theta, p, rho, adaptation)
        count += 1
    data = loadmat(args.results_dir/'CABS_CIBS_matched_uncertainty_comparison.mat', simplify_cells=True)
    check_fixture_config(data, p)
    for controller in ('CABS', 'CIBS'):
        compare(f'matched/nominal/{controller}', controller, data['Nominal'][controller],
                p.theta, p, np.ones(7))
        count += 1
    for family in data['Results']:
        for i, level in enumerate(np.atleast_1d(family['levels'])):
            for controller in ('CABS', 'CIBS'):
                compare(f'matched/{family["name"]}/{level:.0%}/{controller}', controller,
                        family[controller][i], family['thetaPlant'][i], p, np.ones(7))
                count += 1
    print(f'Validated {count} MATLAB/Python cases across all scalar and vector histories.')


if __name__ == '__main__':
    main()
