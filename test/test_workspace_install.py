"""Installer contracts exercised offline; package/network/build commands are faked."""

import difflib
import os
from pathlib import Path
import shlex
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[1]
HOTFIX = ROOT / 'scripts/install_moveit_hotfix.sh'


def shell(script, *, environment=None, cwd=None):
    return subprocess.run(['bash', '-c', script], cwd=cwd, text=True,
                          capture_output=True, timeout=20,
                          env={**os.environ, **(environment or {})})


@pytest.mark.parametrize('state', ['clean', 'partial', 'conflicting'])
def test_hotfix_preserves_changes_and_only_accepts_complete_patch(tmp_path, state):
    before = 'old_one\n' + ''.join(f'line_{n}\n' for n in range(12)) + 'old_two\n'
    after = before.replace('old_one', 'new_one').replace('old_two', 'new_two')
    actual = {'clean': before, 'partial': before.replace('old_one', 'new_one'),
              'conflicting': before.replace('old_one', 'user_edit')}[state]
    target = tmp_path / 'source.txt'
    target.write_text(actual)
    sentinel = tmp_path / 'my_notes.txt'
    sentinel.write_text('preserve me\n')
    patch = tmp_path / 'fix.patch'
    patch.write_text(''.join(difflib.unified_diff(
        before.splitlines(True), after.splitlines(True), 'a/source.txt', 'b/source.txt',
    )))
    command = f'source {shlex.quote(str(HOTFIX))}; apply_moveit_hotfix "$PWD" fix.patch'
    result = shell(command, cwd=tmp_path)
    if state == 'clean':
        assert result.returncode == 0, result.stderr
        assert target.read_text() == after
        again = shell(command, cwd=tmp_path)
        assert again.returncode == 0, again.stderr
        assert 'already applied' in again.stdout
        assert target.read_text() == after
    else:
        assert result.returncode != 0
        assert 'conflicts or is partially applied' in result.stderr
        assert target.read_text() == actual
    assert sentinel.read_text() == 'preserve me\n'
    assert not list(tmp_path.glob('*.rej'))


@pytest.mark.parametrize('version', ['2.5.10', '2.5.11'])
def test_hotfix_rejects_unsupported_installed_moveit_before_build(tmp_path, version):
    module = tmp_path / 'ament_index_python'
    module.mkdir()
    (module / '__init__.py').write_text('')
    (module / 'packages.py').write_text(
        'import os\ndef get_package_share_directory(package):\n'
        '    return os.path.join(os.environ["MOCK_MOVEIT_ROOT"], package)\n'
    )
    for package in ('moveit_core', 'moveit_ros_planning', 'moveit_ros_visualization'):
        directory = tmp_path / package
        directory.mkdir()
        (directory / 'package.xml').write_text(f'<package><version>{version}</version></package>')
    result = shell(f'source {shlex.quote(str(HOTFIX))}; check_moveit_versions', environment={
        'PYTHONPATH': str(tmp_path), 'MOCK_MOVEIT_ROOT': str(tmp_path),
    })
    assert (result.returncode == 0) is (version == '2.5.10')
    if version != '2.5.10':
        assert 'requires moveit_core 2.5.10; installed version is 2.5.11' in result.stderr


@pytest.mark.parametrize('ros_state', ['installed', 'missing', 'broken'])
def test_bootstrap_repairs_ros_then_builds_and_preserves_manual_config(tmp_path, ros_state):
    setup = tmp_path / 'ros_setup.bash'
    if ros_state != 'missing':
        setup.write_text('export ROS_DISTRO=humble\n' if ros_state == 'installed' else 'return 1\n')
    installer = tmp_path / 'bootstrap.sh'
    installer.write_text((ROOT / 'install.sh').read_text().replace('/opt/ros/humble/setup.bash', str(setup)))
    workspace = tmp_path / 'workspace'
    workspace.mkdir()
    log = tmp_path / 'commands.log'
    script = r'''
source "$INSTALLER"
source() { if [[ $1 == /etc/os-release ]]; then ID=ubuntu; VERSION_ID=22.04; else builtin source "$@"; fi; }
uname() { echo x86_64; }
ros2() { echo 'Unexpected live ROS command' >&2; return 99; }
function /usr/bin/python3() { echo python >> "$LOG"; }
install_humble() { echo install >> "$LOG"; echo 'export ROS_DISTRO=humble' > "$ROS_SETUP"; }
git() {
    echo "git $*" >> "$LOG"
    if [[ $1 == clone ]]; then mkdir -p "$3/ci"; touch "$3/ci/provision_humble.sh"; else echo "$2"; fi
}
bash() {
    echo "runtime $*" >> "$LOG"
    mkdir -p src/spot_ros2/spot_driver/config
    echo 'auto_power_on: false' > src/spot_ros2/spot_driver/config/spot_ros_example.yaml
}
main
'''
    environment = {'INSTALLER': str(installer), 'ROS_SETUP': str(setup), 'LOG': str(log)}
    result = shell(script, environment=environment, cwd=workspace)
    assert result.returncode == 0, result.stderr
    calls = log.read_text()
    assert ('install\n' in calls) is (ros_state != 'installed')
    assert calls.index('git clone') < calls.index('runtime ')
    assert '--runtime' in calls
    assert 'username, password, and hostname' in result.stdout
    assert str(workspace / 'setup.bash') in result.stdout
    assert 'ros2 launch spot_driver' in result.stdout and 'fault_detector_launch.py' in result.stdout
    config = workspace / 'config/spot_ros.yaml'
    assert config.stat().st_mode & 0o777 == 0o600
    config.write_text('user credentials remain private\n')
    log.write_text('')
    repeated = shell(script, environment=environment, cwd=workspace)
    assert repeated.returncode == 0, repeated.stderr
    assert config.read_text() == 'user credentials remain private\n'
    assert 'git clone' not in log.read_text()


def test_unsupported_bootstrap_platform_stops_before_external_commands(tmp_path):
    result = shell(f'''
source {shlex.quote(str(ROOT / 'install.sh'))}
source() {{ ID=ubuntu; VERSION_ID=24.04; }}
uname() {{ echo x86_64; }}
install_humble() {{ echo UNEXPECTED; }}
prepare_application() {{ echo UNEXPECTED; }}
main
''', cwd=tmp_path)
    assert result.returncode != 0
    assert 'Ubuntu 22.04 amd64' in result.stderr
    assert 'UNEXPECTED' not in result.stdout
    assert not list(tmp_path.iterdir())


@pytest.fixture
def provision(tmp_path):
    import shutil

    workspace = tmp_path / 'workspace'
    app = workspace / 'src/fault_detector_spot'
    for directory in ('ci', 'patches', 'scripts'):
        shutil.copytree(ROOT / directory, app / directory)
    for name in ('requirements-base.txt', 'requirements-humble-common.txt',
                 'requirements-humble-sdk.txt', 'requirements-humble-runtime.txt'):
        shutil.copy(ROOT / name, app / name)
    ros_setup = tmp_path / 'ros_setup.bash'
    ros_setup.write_text('export ROS_DISTRO=humble\n')
    script = app / 'ci/provision_humble.sh'
    script.write_text(script.read_text().replace('/opt/ros/humble/setup.bash', str(ros_setup)))
    stubs = tmp_path / 'stubs.sh'
    stubs.write_text(r'''
log() { printf '%s\n' "$*" >> "$LOG"; }
source() {
    log source "$@"
    if [[ $1 == /etc/os-release ]]; then ID=ubuntu; VERSION_ID=22.04; else builtin source "$@"; fi
}
uname() { echo x86_64; }
sudo() { shift 2; "$@"; }
apt-get() { log apt-get "$@"; }
dpkg-query() { echo 5.0.1; }
test() { [[ $* == '-f /opt/spot-cpp-sdk/lib/cmake/bosdyn/bosdynConfig.cmake' ]] || builtin test "$@"; }
function /usr/bin/python3() { log python "$@"; if [[ $1 == - ]]; then cat >/dev/null; fi; }
curl() { log curl "$@"; touch "${@: -1}"; }
sha256sum() { log sha256sum "$@"; }
rosdep() { log rosdep "$@"; }
ros2() { log FORBIDDEN-ROS "$@"; return 99; }
vcs() {
    log vcs "$@"
    if [[ $1 == import ]]; then
        for name in spot_ros2 spot_wrapper fault_detector_msgs; do mkdir -p "$WORKSPACE/src/$name"; done
    fi
}
git() {
    log git "$@"
    if [[ $* == *'rev-parse HEAD' ]]; then echo 57d086216d01ec43121845d385894a25987f8a2c; return; fi
    if [[ $* == *'apply --reverse --check'* ]]; then [[ -f $2/.patched ]]; return; fi
    [[ ${FAIL_PATCH:-} != 1 ]] || return 1
    [[ $* == *'apply --check'* ]] || touch "$2/.patched"
}
colcon() {
    log colcon "$@"
    mkdir -p "$WORKSPACE/install"
    touch "$WORKSPACE/install/setup.bash" "$WORKSPACE/install/local_setup.bash"
}
bash() {
    [[ $1 == */install_moveit_hotfix.sh ]] || return 99
    log rviz-hotfix "$@"
    mkdir -p "$WORKSPACE/hotfixes/install"
    touch "$WORKSPACE/hotfixes/install/local_setup.bash"
}
''')
    log = tmp_path / 'calls.log'
    environment = {'BASH_ENV': str(stubs), 'WORKSPACE': str(workspace), 'LOG': str(log)}
    return workspace, script, environment, log


@pytest.mark.parametrize('failure', [False, True])
def test_runtime_patch_build_source_check_order_and_retry_preserves_files(provision, failure):
    workspace, script, environment, log = provision
    sentinel = workspace / 'src/my_custom_package.txt'
    sentinel.write_text('user source\n')
    result = shell(f'exec /bin/bash {shlex.quote(str(script))} "$WORKSPACE" --runtime',
                   environment={**environment, 'FAIL_PATCH': '1' if failure else ''})
    calls = log.read_text().splitlines()
    if failure:
        assert result.returncode != 0
        assert not any(line.startswith(('colcon ', 'rviz-hotfix ')) for line in calls)
    else:
        assert result.returncode == 0, result.stderr
        build = next(index for index, line in enumerate(calls) if line.startswith('colcon '))
        assert all(any(name in line and 'apply' in line for line in calls[:build])
                   for name in ('spot_wrapper', 'spot_ros2'))
        assert all((workspace / 'src' / name / '.patched').is_file()
                   for name in ('spot_wrapper', 'spot_ros2'))
        assert any(line.startswith('rosdep check ') for line in calls[:build])
        assert all(name in calls[build] for name in (
            '/spot_driver', '/spot_rviz_plugins', '/fault_detector_msgs', '/fault_detector_spot',
        ))
        assert '--symlink-install' not in calls[build]
        hotfix = next(index for index, line in enumerate(calls) if line.startswith('rviz-hotfix '))
        check = next(index for index, line in enumerate(calls) if 'check_install.py' in line)
        assert build < hotfix < check
        assert any('/install/setup.bash' in line for line in calls[build:hotfix])
        assert any('/hotfixes/install/local_setup.bash' in line for line in calls[hotfix:check])
        repeated = shell(f'exec /bin/bash {shlex.quote(str(script))} "$WORKSPACE" --runtime',
                         environment=environment)
        assert repeated.returncode == 0, repeated.stderr
        assert 'already applied' in repeated.stdout
        setup = workspace / 'setup.bash'
        customized = setup.read_text() + '# user customization\n'
        setup.write_text(customized)
        changed = shell(f'exec /bin/bash {shlex.quote(str(script))} "$WORKSPACE" --runtime',
                        environment=environment)
        assert changed.returncode != 0
        assert setup.read_text() == customized
        assert 'manual reconciliation' in changed.stderr
    assert sentinel.read_text() == 'user source\n'
    assert not any('reset ' in line or 'clean ' in line for line in calls)
    assert not any(line.startswith('FORBIDDEN-ROS') for line in calls)
