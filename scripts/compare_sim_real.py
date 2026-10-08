"""Compare repeated measured trial summaries. Never infer real results from sim."""
import argparse
import json
import math
from pathlib import Path
from statistics import mean

METRICS = ('duration_s', 'placement_error_m', 'joint_tracking_rmse_rad', 'localization_final_error', 'maximum_carry_tilt_deg')


def summarize(reports):
    if not reports:
        raise ValueError('At least one measured trial is required')
    if any(type(r.get('passed')) is not bool for r in reports):
        raise ValueError('Every trial needs an explicit boolean passed result')
    result = {'trials': len(reports), 'success_rate': mean(float(r['passed']) for r in reports)}
    for key in METRICS:
        values = [r[key] for r in reports if r.get(key) is not None]
        if any(type(v) not in (int, float) or not math.isfinite(v) or v < 0 for v in values):
            raise ValueError('Invalid measured metric: ' + key)
        result[key] = {'measured_trials': len(values), 'mean': mean(values) if values else None}
    return result


def compare(sim, real):
    a, b = summarize(sim), summarize(real)
    delta = {'success_rate': b['success_rate'] - a['success_rate']}
    for key in METRICS:
        av, bv = a[key]['mean'], b[key]['mean']
        delta[key] = None if av is None or bv is None else bv - av
    return {'sim': a, 'real': b, 'real_minus_sim': delta,
            'note': 'Descriptive comparison only; match hardware, task, units and success criteria. Missing measurements remain null. No gap pass/fail threshold is assumed.'}


def read_trials(paths):
    trials = []
    for path in paths:
        value = json.loads(Path(path).read_text())
        trials.extend(value if isinstance(value, list) else [value])
    return trials


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sim', nargs='+', required=True)
    parser.add_argument('--real', nargs='+', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    result = compare(read_trials(args.sim), read_trials(args.real))
    Path(args.output).write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps(result, indent=2))
