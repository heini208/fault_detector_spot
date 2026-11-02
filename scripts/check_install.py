#!/usr/bin/env python3
"""Check a copied install offline; no entry point is invoked and no node starts.

Source the install first, then pass its package prefix with --prefix. Optional
--source-tree checks installed data and console declarations against that tree.
This deliberately rejects editable installs and imports from another checkout.
"""

import argparse
import ast
import importlib.metadata
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET


PACKAGE = "fault_detector_spot"
RETIRED_ENTRY = "nav2_cmd_vel_gate"
IMPORT_CHECK = """
import importlib.metadata
from pathlib import Path
import sys
from ament_index_python.packages import get_package_prefix, get_package_share_directory
import xml.etree.ElementTree as ET

prefix, messages_prefix, entry_name = sys.argv[1:]
prefixes = {'fault_detector_spot': Path(prefix)}
if messages_prefix:
    prefixes['fault_detector_msgs'] = Path(messages_prefix)
for package, expected in prefixes.items():
    assert Path(get_package_prefix(package)).resolve() == expected, package
if entry_name:
    dist = importlib.metadata.distribution('fault_detector_spot')
    assert Path(dist.locate_file('')).resolve().is_relative_to(Path(prefix))
    entry = next(ep for ep in dist.entry_points
                 if ep.group == 'console_scripts' and ep.name == entry_name)
    assert callable(entry.load()), entry_name
else:
    for distribution in ('bosdyn-api', 'bosdyn-client', 'bosdyn-core'):
        assert importlib.metadata.version(distribution) == '5.0.1', distribution
    for package in ('bosdyn_api_msgs', 'bosdyn_spot_api_msgs'):
        manifest = Path(get_package_share_directory(package)) / 'package.xml'
        assert ET.parse(manifest).findtext('version') == '5.0.1', package
    from fault_detector_msgs.action import ExecuteOperation
    from fault_detector_msgs.msg import SensorHeadGeometry
    from fault_detector_msgs.srv import EmergencyStop
    from rclpy.type_support import check_for_type_support
    for interface in (SensorHeadGeometry, EmergencyStop, ExecuteOperation):
        check_for_type_support(interface)
for name, module in tuple(sys.modules.items()):
    package = name.split('.')[0]
    if package in prefixes and getattr(module, '__file__', None):
        path = Path(module.__file__).resolve()
        assert path.is_relative_to(prefixes[package]), f'{name}: {path}'
"""


def source_entries(source):
    """Read the literal console declaration without executing setup.py."""
    tree = ast.parse((source / "setup.py").read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
            if node.func.id == "setup":
                value = next(k.value for k in node.keywords if k.arg == "entry_points")
                entries = ast.literal_eval(value)["console_scripts"]
                return dict(tuple(part.strip() for part in item.split("=", 1))
                            for item in entries)
    raise AssertionError("No setup() console declaration found")


def check_data(source, share):
    paths = [Path("package.xml"), Path("LICENSE")]
    for directory, patterns in (
        ("launch", ("*launch.[pxy][yma]*",)),
        ("config", ("*.yaml", "*.rviz", "*.xml")),
    ):
        expected = {p.relative_to(source) for pattern in patterns
                    for p in (source / directory).glob(pattern)}
        actual = {p.relative_to(share) for pattern in patterns
                  for p in (share / directory).glob(pattern)}
        assert actual == expected, f"Installed {directory} differs: {actual ^ expected}"
        paths.extend(sorted(expected))
    for relative in paths:
        assert (share / relative).read_bytes() == (source / relative).read_bytes(), relative
    print(f"Verified {len(paths)} installed metadata, launch and configuration files")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prefix", required=True, type=Path)
    parser.add_argument("--source-tree", type=Path)
    parser.add_argument("--messages-prefix", type=Path)
    args = parser.parse_args()
    prefix = args.prefix.resolve()
    messages_prefix = str(args.messages_prefix.resolve()) if args.messages_prefix else ""
    share = prefix / "share" / PACKAGE
    assert (prefix / "share/ament_index/resource_index/packages" / PACKAGE).is_file()
    dist = importlib.metadata.distribution(PACKAGE)
    assert Path(dist.locate_file("")).resolve().is_relative_to(prefix), dist.locate_file("")
    entries = {ep.name: ep.value for ep in dist.entry_points if ep.group == "console_scripts"}
    assert entries and RETIRED_ENTRY not in entries, entries
    assert not (share / "launch/nav2_spot_launch.py").exists()
    assert not (prefix / "lib" / PACKAGE / RETIRED_ENTRY).exists()
    package_xml = ET.parse(share / "package.xml").getroot()
    assert package_xml.findtext("version") == dist.version
    assert package_xml.findtext("description") == dist.metadata["Summary"]
    assert package_xml.findtext("license") == dist.metadata["License"]
    assert (share / "LICENSE").is_file()
    if args.source_tree:
        source = args.source_tree.resolve()
        assert entries == source_entries(source), "Console declarations differ from source"
        check_data(source, share)
    environment = dict(os.environ, PYTHONDONTWRITEBYTECODE="1", PYTHONNOUSERSITE="1")
    with tempfile.TemporaryDirectory(prefix="fault-detector-import-check-") as directory:
        for name in sorted(entries):
            wrapper = prefix / "lib" / PACKAGE / name
            assert wrapper.is_file() and os.access(wrapper, os.X_OK), wrapper
            subprocess.run([sys.executable, "-c", IMPORT_CHECK, str(prefix),
                            messages_prefix, name], cwd=directory, env=environment,
                           check=True, timeout=60)
            print(f"Imported {name}", flush=True)
        subprocess.run([sys.executable, "-c", IMPORT_CHECK, str(prefix),
                        messages_prefix, ""], cwd=directory, env=environment,
                       check=True, timeout=60)
    print(f"Verified {len(entries)} console scripts and message/service/action type support")


if __name__ == "__main__":
    main()
