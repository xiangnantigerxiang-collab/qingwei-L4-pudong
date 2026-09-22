#!/usr/bin/env python3
"""D沿程曲率/电机参考/末停/液压释放回归，复用verify_forward_brake的ROS桩公共对象。"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess

HERE = Path(__file__).resolve().parent
PNC = HERE.parents[1]
CASES = ['spatial_preview', 'geometry', 'motor', 'terminal', 'manual_n_reset',
         'release', 'increase_release', 'planning_ripple', 'regeneration',
         'delayed_regeneration', 'terminal_callbacks']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--control-build', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    work = args.output.resolve()
    build = args.control_build.resolve()
    control = PNC / 'src/robot_control'
    work.mkdir(parents=True, exist_ok=True)
    tracked = list(control.rglob('*.cpp')) + list(control.rglob('*.h')) + list(control.rglob('*.inc'))
    tracked += [HERE / 'forward_speed_smooth.cpp', HERE / 'brake_state_revision.cpp']
    def hashes():
        return {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
    before = hashes()
    includes = re.findall(r'^#include.*$', (control / 'control_comply.h').read_text(), re.M)
    (work / 'forward_smooth_access.h').write_text('\n'.join(includes) +
        '\n#define private public\n#include "control_comply.h"\n#undef private\n')
    flags = ['g++', '-std=c++11', '-O2', '-Wall', '-Wextra', '-include', 'cstdint', '-include', 'array']
    flags += ['-I' + str(p) for p in (work, control, build / 'stubs', PNC / 'include', PNC / 'src', PNC / 'src/robot_path_plan')]
    common = [build / (name + '.o') for name in ('stanley_controller', 'lateral_reverse_control',
              'longitudinal_speed_control', 'pubalgor', 'Spline')]
    command = flags + [str(HERE / 'forward_speed_smooth.cpp'), str(control / 'control_comply.cpp')]
    binary = work / 'forward_speed_smooth'
    command += list(map(str, common)) + ['-o', str(binary)]
    with (work / 'compile.log').open('w') as log:
        subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
    results = []
    for case in CASES:
        run = subprocess.run([str(binary), case], text=True, capture_output=True)
        (work / (case + '.log')).write_text(run.stdout + run.stderr)
        summary = [s for s in run.stdout.splitlines() if s.startswith('RESULT ')]
        failures = [s for s in run.stderr.splitlines() if s.startswith('FAIL ')]
        result = dict(case=case, exit_code=run.returncode, summary=summary, failures=failures)
        results.append(result)
        print(summary[-1] if summary else result, flush=True)
    after = hashes()
    result = dict(cases=results, sources=after, source_unchanged=before == after,
                  failed_cases=sum(r['exit_code'] != 0 for r in results))
    (work / 'compile_commands.json').write_text(json.dumps([command], indent=2) + '\n')
    (work / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    if result['failed_cases'] or before != after:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
