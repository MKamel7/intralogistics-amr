#!/usr/bin/env python3
"""Run a configuration N times and report the distribution, not a number.

WHY THIS EXISTS, AND WHY IT IS NOT POLISH

Every measurement taken while debugging the second platform was a SINGLE RUN,
against a system whose distance per cycle varied by 35 m between identical
journeys. Two conclusions were drawn from single runs. One of them had to be
retracted the same evening, because the run that appeared to confirm a fix
turned out to be the only one of the day whose keepout mask never published: two
variables moved and the improvement was credited to the one being watched.

No amount of care fixes n=1. A tool that makes ten runs as easy as one is the
difference between debugging and guessing, and it is cheap.

WHAT IT DOES NOT DO

It does not decide anything. It runs the same command repeatedly, parses what
the transport task already prints, and reports the spread. Whether a difference
between two configurations is real is a judgement made by a person looking at
overlapping ranges, not a p-value produced here. Inventing a significance test
on four samples would be a new way to be confidently wrong.

RECORDING THE ENVIRONMENT IS PART OF THE MEASUREMENT. Each run's log directory
is kept and named in the output, and the keepout mask state is checked per run,
because that is the exact confound that cost a conclusion. A run whose safety
layers were not all up is reported separately rather than averaged in.

Usage:
    tools/experiment.py --runs 10 -- --test-track --cameras off --run mission
    tools/experiment.py --runs 5 --label baseline -- --cameras off --run mission
"""

import argparse
import json
import re
import statistics
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
RUN_STACK = REPO / 'tools' / 'run_stack.sh'
LOG_ROOT = Path('/tmp/amr-logs')

CYCLE = re.compile(
    r'cycle (\d+): complete in (\d+) s, ([\d.]+) m driven, (\d+) protective stop')
SUMMARY = re.compile(r'(\d+) of (\d+) cycle\(s\) completed')


def parse_run(log_dir):
    """What one run produced. Missing fields stay None rather than becoming 0."""
    mission = log_dir / 'mission.log'
    out = {
        'log_dir': str(log_dir),
        'completed': None,
        'attempted': None,
        'cycle_times': [],
        'cycle_distances': [],
        'protective_stops': [],
        'healthy': True,
        'notes': [],
    }
    if not mission.is_file():
        out['healthy'] = False
        out['notes'].append('no mission log; the run did not get that far')
        return out

    text = mission.read_text()
    for m in CYCLE.finditer(text):
        out['cycle_times'].append(int(m.group(2)))
        out['cycle_distances'].append(float(m.group(3)))
        out['protective_stops'].append(int(m.group(4)))
    s = SUMMARY.search(text)
    if s:
        out['completed'], out['attempted'] = int(s.group(1)), int(s.group(2))

    # THE CONFOUND CHECK. A run whose keepout mask never published is not a
    # sample of the same system, and averaging it in is how a retraction
    # happens. It is reported, not silently dropped.
    nav = log_dir / 'nav.log'
    if nav.is_file() and 'Filter mask was not received' in nav.read_text():
        out['healthy'] = False
        out['notes'].append('keepout mask never published; not the same system')

    stage = log_dir / 'stage.log'
    if stage.is_file() and 'preflight exit 0' not in stage.read_text():
        out['healthy'] = False
        out['notes'].append('preflight did not pass')
    return out


def spread(values):
    """Mean, min, max and stdev. Stdev needs two samples; say so if it cannot."""
    if not values:
        return None
    d = {'n': len(values), 'mean': statistics.fmean(values),
         'min': min(values), 'max': max(values)}
    d['stdev'] = statistics.stdev(values) if len(values) > 1 else None
    return d


def fmt(d, unit=''):
    if not d:
        return 'no samples'
    sd = f' sd {d["stdev"]:.1f}' if d['stdev'] is not None else ''
    return (f'{d["mean"]:.1f}{unit} '
            f'[{d["min"]:.1f} to {d["max"]:.1f}]{sd}  n={d["n"]}')


def main():
    ap = argparse.ArgumentParser(
        description='Run a stack configuration N times and report the spread.')
    ap.add_argument('--runs', type=int, default=5)
    ap.add_argument('--label', default='')
    ap.add_argument('--out', type=Path,
                    help='write the raw per-run results here as JSON')
    ap.add_argument('--settle', type=float, default=10.0,
                    help='seconds between runs, for the simulator to release')
    ap.add_argument('args', nargs=argparse.REMAINDER,
                    help='arguments for run_stack.sh, after --')
    a = ap.parse_args()

    stack_args = [x for x in a.args if x != '--']
    if not stack_args:
        sys.exit('nothing to run: pass run_stack.sh arguments after --')

    label = a.label or ' '.join(stack_args)
    print(f'{a.runs} runs of: run_stack.sh {" ".join(stack_args)}')
    print(f'label: {label}\n')

    results = []
    for i in range(1, a.runs + 1):
        before = set(LOG_ROOT.glob('2*')) if LOG_ROOT.is_dir() else set()
        t0 = time.time()
        print(f'  run {i}/{a.runs} ... ', end='', flush=True)
        subprocess.run([str(RUN_STACK)] + stack_args,
                       cwd=REPO, capture_output=True, text=True)
        subprocess.run([str(REPO / 'tools' / 'stop_all.sh')],
                       cwd=REPO, capture_output=True)
        after = set(LOG_ROOT.glob('2*')) if LOG_ROOT.is_dir() else set()
        fresh = sorted(after - before)
        if not fresh:
            print('no log directory appeared')
            results.append({'log_dir': None, 'healthy': False,
                            'notes': ['run produced no log directory'],
                            'cycle_times': [], 'cycle_distances': [],
                            'protective_stops': [], 'completed': None,
                            'attempted': None})
            continue
        r = parse_run(fresh[-1])
        r['wall_s'] = round(time.time() - t0, 1)
        results.append(r)
        state = 'ok' if r['healthy'] else 'EXCLUDED'
        done = f'{r["completed"]}/{r["attempted"]}' if r['attempted'] else '?'
        print(f'{done} cycles, {r["wall_s"]:.0f} s wall, {state}')
        for n in r['notes']:
            print(f'      {n}')
        time.sleep(a.settle)

    good = [r for r in results if r['healthy']]
    print(f'\n{"=" * 68}')
    print(f'{label}')
    print(f'{len(good)} of {len(results)} runs usable')
    if not good:
        print('nothing to report: no run was healthy')
        return 1

    times = [x for r in good for x in r['cycle_times']]
    dists = [x for r in good for x in r['cycle_distances']]
    stops = [x for r in good for x in r['protective_stops']]
    comp = sum(r['completed'] or 0 for r in good)
    att = sum(r['attempted'] or 0 for r in good)

    print(f'  cycles completed   {comp} of {att}')
    print(f'  cycle time         {fmt(spread(times), " s")}')
    print(f'  distance           {fmt(spread(dists), " m")}')
    print(f'  protective stops   {fmt(spread(stops))}')
    print(f'{"=" * 68}')
    print('Ranges, not means, decide whether two configurations differ. If the '
          'ranges overlap, run more.')

    if a.out:
        a.out.write_text(json.dumps(
            {'label': label, 'args': stack_args, 'runs': results}, indent=2))
        print(f'\nraw results: {a.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
