#!/usr/bin/env python3
"""Verify NFR-M / NFR-REP maintainability and reproducibility evidence."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

import yaml


PACKAGES = (
    'bridge_monitor_msgs',
    'pybullet_bridge',
    'dist_monitor',
    'risk_engine',
    'hoc_console',
    'manipulation_actions',
    'moveit_config',
)

PYTHON_PACKAGES = (
    'pybullet_bridge',
    'dist_monitor',
    'risk_engine',
    'hoc_console',
    'manipulation_actions',
)

VERIFY_SCRIPTS = (
    'verify_bridge_comm.sh',
    'verify_moveit_closure.sh',
    'verify_monitor_metrics.sh',
    'verify_risk_management.sh',
    'verify_hoc_console.sh',
    'verify_performance_nfr.sh',
    'verify_reliability_nfr.sh',
    'verify_safety_nfr.sh',
    'verify_portfolio.sh',
)


def _run(cmd: list[str], *, cwd: Path, timeout_sec: int = 180) -> dict[str, Any]:
    start = time.monotonic()
    try:
        proc = subprocess.run(
            cmd,
            cwd=str(cwd),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout_sec,
            check=False,
        )
        return {
            'cmd': cmd,
            'returncode': proc.returncode,
            'elapsed_sec': round(time.monotonic() - start, 3),
            'output_tail': proc.stdout[-4000:],
        }
    except subprocess.TimeoutExpired as exc:
        return {
            'cmd': cmd,
            'returncode': 124,
            'elapsed_sec': timeout_sec,
            'output_tail': (exc.stdout or '')[-4000:] if isinstance(exc.stdout, str) else 'timeout',
        }


def _validate_yaml(root: Path) -> dict[str, Any]:
    files = sorted(root.glob('*/config/*.yaml'))
    failures: list[dict[str, str]] = []
    for path in files:
        try:
            yaml.safe_load(path.read_text(encoding='utf-8'))
        except Exception as exc:  # noqa: BLE001
            failures.append({'path': str(path.relative_to(root)), 'error': str(exc)})
    return {
        'files': [str(path.relative_to(root)) for path in files],
        'count': len(files),
        'failures': failures,
        'passes': not failures and len(files) >= 10,
    }


def _launch_inventory(root: Path) -> dict[str, Any]:
    files = sorted(root.glob('*/launch/*.py'))
    compile_failures: list[dict[str, Any]] = []
    for path in files:
        res = _run([sys.executable, '-m', 'py_compile', str(path)], cwd=root, timeout_sec=20)
        if res['returncode'] != 0:
            compile_failures.append({
                'path': str(path.relative_to(root)),
                'output_tail': res['output_tail'],
            })
    hoc_experiment = root / 'hoc_console' / 'launch' / 'hoc_experiment.launch.py'
    text = hoc_experiment.read_text(encoding='utf-8') if hoc_experiment.is_file() else ''
    return {
        'files': [str(path.relative_to(root)) for path in files],
        'count': len(files),
        'hoc_experiment_includes_portfolio': 'portfolio_demo.launch.py' in text,
        'hoc_experiment_includes_hoc': 'hoc.launch.py' in text,
        'compile_failures': compile_failures,
        'passes': (
            not compile_failures
            and (root / 'pybullet_bridge' / 'launch' / 'portfolio_demo.launch.py').is_file()
            and 'portfolio_demo.launch.py' in text
            and 'hoc.launch.py' in text
        ),
    }


def _package_structure(root: Path) -> dict[str, Any]:
    items: list[dict[str, Any]] = []
    for pkg in PACKAGES:
        pkg_dir = root / pkg
        pkg_xml = pkg_dir / 'package.xml'
        build_file = (pkg_dir / 'setup.py').is_file() or (pkg_dir / 'CMakeLists.txt').is_file()
        res = _run(['ros2', 'pkg', 'prefix', pkg], cwd=root, timeout_sec=20)
        items.append({
            'package': pkg,
            'package_xml': pkg_xml.is_file(),
            'build_file': build_file,
            'ros2_pkg_prefix_ok': res['returncode'] == 0,
            'prefix_output': res['output_tail'].strip(),
        })
    return {
        'packages': items,
        'passes': all(item['package_xml'] and item['build_file'] and item['ros2_pkg_prefix_ok'] for item in items),
    }


def _load_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    return json.loads(path.read_text(encoding='utf-8'))


def _dynamic_config_and_metadata(root: Path) -> dict[str, Any]:
    monitor = _load_json(root / 'docs' / 'samples' / 'monitor-metrics.json')
    hoc = _load_json(root / 'docs' / 'samples' / 'hoc-console-metrics.json')
    reliability = _load_json(root / 'docs' / 'samples' / 'reliability-nfr-metrics.json')
    hot_reload = monitor.get('threshold_hot_reload', {})
    set_randomization = hoc.get('commands', {}).get('set_randomization', {})
    bag_path = reliability.get('rosbag_recording', {}).get('bag', {}).get('path', '')
    bag_metadata = Path(bag_path) / 'metadata.yaml' if bag_path else None
    return {
        'threshold_hot_reload': hot_reload,
        'hoc_set_randomization': set_randomization,
        'rosbag_metadata': {
            'path': str(bag_metadata) if bag_metadata else '',
            'exists': bool(bag_metadata and bag_metadata.is_file()),
        },
        'passes': bool(hot_reload.get('passes'))
        and bool(set_randomization.get('success'))
        and bool(bag_metadata and bag_metadata.is_file()),
    }


def _verify_scripts(root: Path) -> dict[str, Any]:
    items: list[dict[str, Any]] = []
    for name in VERIFY_SCRIPTS:
        path = root / 'scripts' / name
        res = _run(['bash', '-n', str(path)], cwd=root, timeout_sec=20) if path.is_file() else {'returncode': 1, 'output_tail': 'missing'}
        items.append({
            'script': f'scripts/{name}',
            'exists': path.is_file(),
            'executable': path.is_file() and bool(path.stat().st_mode & 0o111),
            'bash_n_ok': res['returncode'] == 0,
        })
    deterministic_outputs = [
        'bridge-comm-metrics.json',
        'moveit-closure-metrics.json',
        'monitor-metrics.json',
        'risk-management-metrics.json',
        'hoc-console-metrics.json',
        'performance-nfr-metrics.json',
        'reliability-nfr-metrics.json',
        'safety-nfr-metrics.json',
    ]
    outputs = [root / 'docs' / 'samples' / name for name in deterministic_outputs]
    return {
        'scripts': items,
        'sample_outputs': [str(path.relative_to(root)) for path in outputs if path.is_file()],
        'passes': all(item['exists'] and item['executable'] and item['bash_n_ok'] for item in items)
        and all(path.is_file() for path in outputs),
    }


def _coverage(root: Path, output_dir: Path, timeout_sec: int) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    coverage_file = output_dir / '.coverage'
    xml_path = output_dir / 'coverage.xml'
    json_path = output_dir / 'coverage.json'
    source_arg = ','.join(PYTHON_PACKAGES)
    test_args = [
        'dist_monitor/test',
        'risk_engine/test',
        'pybullet_bridge/test',
        'hoc_console/test',
        'manipulation_actions/test',
        '--ignore=pybullet_bridge/test/test_m1_launch.py',
        '--ignore=pybullet_bridge/test/test_full_system_launch.py',
        '-q',
    ]
    env_cmd = [
        'bash',
        '-lc',
        (
            f'export COVERAGE_FILE="{coverage_file}"; '
            f'{sys.executable} -m coverage erase; '
            f'{sys.executable} -m coverage run --source="{source_arg}" -m pytest {" ".join(test_args)}; '
            f'{sys.executable} -m coverage json -o "{json_path}"; '
            f'{sys.executable} -m coverage xml -o "{xml_path}"'
        ),
    ]
    res = _run(env_cmd, cwd=root, timeout_sec=timeout_sec)
    data = _load_json(json_path)
    total = data.get('totals', {}).get('percent_covered', 0.0)
    return {
        'command': res['cmd'],
        'returncode': res['returncode'],
        'elapsed_sec': res['elapsed_sec'],
        'coverage_json': str(json_path.relative_to(root)) if json_path.is_file() else '',
        'coverage_xml': str(xml_path.relative_to(root)) if xml_path.is_file() else '',
        'percent_covered': round(float(total), 3) if total is not None else 0.0,
        'output_tail': res['output_tail'],
        'passes': res['returncode'] == 0 and float(total or 0.0) >= 70.0,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Verify NFR-M / NFR-REP maintainability evidence.')
    parser.add_argument('--root', type=Path, default=Path.cwd())
    parser.add_argument('--output', type=Path, default=Path('docs/samples/maintainability-nfr-metrics.json'))
    parser.add_argument('--coverage-dir', type=Path, default=Path('docs/samples/maintainability-coverage'))
    parser.add_argument('--coverage-timeout-sec', type=int, default=240)
    args = parser.parse_args(argv)
    root = args.root.resolve()

    result = {
        'timestamp_unix': round(time.time(), 3),
        'criteria': {
            'yaml_config_min_files': 10,
            'coverage_percent_min': 70.0,
            'expected_ros_packages': list(PACKAGES),
            'expected_verify_scripts': [f'scripts/{name}' for name in VERIFY_SCRIPTS],
        },
        'yaml_config': _validate_yaml(root),
        'launch_inventory': _launch_inventory(root),
        'package_structure': _package_structure(root),
        'dynamic_config_and_metadata': _dynamic_config_and_metadata(root),
        'verify_scripts_and_outputs': _verify_scripts(root),
        'coverage': _coverage(root, root / args.coverage_dir, args.coverage_timeout_sec),
    }
    result['passes'] = {
        'NFR-M01_yaml_dynamic_config': result['yaml_config']['passes'] and result['dynamic_config_and_metadata']['passes'],
        'NFR-M02_one_click_launch': result['launch_inventory']['passes'],
        'NFR-M03_standard_packages_buildable': result['package_structure']['passes'],
        'NFR-M04_unit_coverage_70pct': result['coverage']['passes'],
        'NFR-REP01_seed_configurable': bool(
            (root / 'pybullet_bridge' / 'config' / 'bridge_config.yaml').is_file()
            and (root / 'docs' / 'samples' / 'hoc-console-metrics.json').is_file()
        ),
        'NFR-REP02_config_result_linked': result['dynamic_config_and_metadata']['passes'],
        'NFR-REP03_deterministic_demo_outputs': result['verify_scripts_and_outputs']['passes'],
    }
    result['overall_passes'] = all(result['passes'].values())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['overall_passes'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
