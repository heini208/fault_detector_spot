"""The test harness must not read or modify a user's persistent settings."""

import os
import shutil
import subprocess
import sys
import textwrap
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_runtime_settings_are_isolated_during_pytest(tmp_path):
    """Keep external and per-test settings apart, including in subprocesses."""
    external_home = tmp_path / 'external-ros'
    external_settings = (
        external_home / 'fault_detector_spot' / 'basic_settings.yaml'
    )
    external_settings.parent.mkdir(parents=True)
    external_settings.write_text(
        'version: 2\nsettings:\n  force_threshold_percent: 250.0\n',
        encoding='utf-8',
    )
    original_settings = external_settings.read_bytes()
    suite = tmp_path / 'suite'
    suite.mkdir()
    shutil.copyfile(ROOT / 'test' / 'conftest.py', suite / 'conftest.py')
    (suite / 'test_isolation.py').write_text(textwrap.dedent("""\
        import os
        from pathlib import Path
        import subprocess
        import sys

        from fault_detector_spot.shared.persistence.basic_settings import (
            BasicSettingsRepository, startup_settings,
        )

        COLLECTION_HOME = Path(os.environ["ROS_HOME"])
        Path(__file__).with_name("collection_home.txt").write_text(str(COLLECTION_HOME))
        assert startup_settings() == {}

        def test_first_can_save_its_own_settings():
            global first_home
            first_home = Path(os.environ["ROS_HOME"])
            assert first_home != COLLECTION_HOME
            BasicSettingsRepository().save({"force_threshold_percent": 175.0})
            assert startup_settings()["force_threshold_percent"] == 175.0
            subprocess.run([
                sys.executable, "-c",
                "import os, sys; "
                "from fault_detector_spot.shared.persistence.basic_settings "
                "import startup_settings; "
                "assert os.environ['ROS_HOME'] == sys.argv[1]; "
                "assert startup_settings().get('force_threshold_percent') "
                "== 175.0",
                str(first_home),
            ], check=True, timeout=15)

        def test_second_cannot_see_the_first_tests_settings():
            assert Path(os.environ["ROS_HOME"]) not in (
                first_home, COLLECTION_HOME,
            )
            assert not BasicSettingsRepository().path.exists()
            assert startup_settings() == {}
            subprocess.run([
                sys.executable, "-c",
                "from fault_detector_spot.shared.persistence.basic_settings "
                "import startup_settings; "
                "assert startup_settings() == {}",
            ], check=True, timeout=15)
        """), encoding='utf-8')
    environment = os.environ.copy()
    environment['ROS_HOME'] = str(external_home)
    environment['PYTEST_DISABLE_PLUGIN_AUTOLOAD'] = '1'
    environment.pop('PYTEST_ADDOPTS', None)
    environment['PYTHONPATH'] = os.pathsep.join(filter(None, (
        str(ROOT), environment.get('PYTHONPATH'),
    )))
    result = subprocess.run(
        [sys.executable, '-c', textwrap.dedent("""\
            import os
            from pathlib import Path
            import sys

            import pytest

            from fault_detector_spot.shared.persistence.basic_settings import (
                startup_settings,
            )

            original_home = os.environ["ROS_HOME"]
            assert startup_settings()["force_threshold_percent"] == 250.0
            result = pytest.main(["-q", "-p", "no:cacheprovider", sys.argv[1]])
            assert result == 0
            assert os.environ["ROS_HOME"] == original_home
            assert startup_settings()["force_threshold_percent"] == 250.0
            collection_marker = Path(sys.argv[1], "collection_home.txt")
            collection_home = Path(collection_marker.read_text())
            assert not collection_home.exists()
            """), str(suite)],
        cwd=suite,
        env=environment,
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert external_settings.read_bytes() == original_settings
