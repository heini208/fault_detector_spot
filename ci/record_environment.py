#!/usr/bin/env python3
"""Record revision, working-tree identity and resolved dependencies for a run."""

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import subprocess
import sys


def command(*args, cwd=None):
    result = subprocess.run(args, cwd=cwd, text=True, capture_output=True, check=True)
    return result.stdout.strip()


def repository(path):
    """Identify local changes as well as HEAD; never label a dirty tree a commit."""
    path = path.resolve()
    files = command('git', 'ls-files', '-c', '-o', '--exclude-standard', '-z', cwd=path)
    digest = hashlib.sha256()
    for name in sorted(set(files.split('\0')) - {''}):
        file = path / name
        if file.is_symlink():
            content = os.readlink(file).encode()
        elif file.is_file():
            content = file.read_bytes()
        else:
            continue
        digest.update(name.encode() + b'\0' + hashlib.sha256(content).digest())
    return {
        'path': str(path),
        'commit': command('git', 'rev-parse', 'HEAD', cwd=path),
        'status': command('git', 'status', '--porcelain=v1', cwd=path).splitlines(),
        'tree_sha256': digest.hexdigest(),
        'diff_sha256': hashlib.sha256(
            command('git', 'diff', '--binary', 'HEAD', cwd=path).encode()
        ).hexdigest(),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--dependency', action='append', default=[], type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    args.output.mkdir(parents=True, exist_ok=True)
    packages = sorted(
        ({'name': dist.metadata['Name'], 'version': dist.version,
          'location': str(dist.locate_file(''))}
         for dist in importlib.metadata.distributions()),
        key=lambda item: (item['name'].lower(), item['location']),
    )
    data = {
        'recorded_utc': datetime.now(timezone.utc).isoformat(),
        'source': repository(root),
        'dependencies': [repository(path) for path in args.dependency],
        'python': sys.version,
        'python_executable': sys.executable,
        'platform': platform.platform(),
        'os_release': Path('/etc/os-release').read_text(),
        'ros_distro': os.environ.get('ROS_DISTRO'),
        'ament_prefix_path': os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep),
        'github_sha': os.environ.get('GITHUB_SHA'),
        'github_run_id': os.environ.get('GITHUB_RUN_ID'),
        'python_packages': packages,
    }
    (args.output / 'environment.json').write_text(json.dumps(data, indent=2) + '\n')
    (args.output / 'apt-packages.txt').write_text(command(
        'dpkg-query', '-W', '-f=${Package}\t${Version}\t${Architecture}\n',
    ) + '\n')
    print(f"Recorded {data['source']['commit']} in {args.output}")


if __name__ == '__main__':
    main()
