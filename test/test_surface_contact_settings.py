"""Contact-search speed editing and persistence without the ROS execution stack."""

import pytest
from PyQt5.QtWidgets import QApplication

from fault_detector_spot.shared.persistence.basic_settings import (
    BasicSettingsRepository, _startup_settings, parameter_override,
)
from fault_detector_spot.ui.settings.advanced_settings_page import AdvancedSettingsPage


@pytest.fixture
def app():
    return QApplication.instance() or QApplication([])


@pytest.mark.parametrize('speed_mm_s', [.5, 3.])
def test_contact_speed_roundtrips_units_and_only_applies_after_restart(app, speed_mm_s):
    key = 'close_surface.contact_search_speed_mps'
    repo = BasicSettingsRepository()
    repo.save({'arm_mode': 'precision'})
    page = AdvancedSettingsPage(repository=repo)
    assert page.fields[key].value() == 1.
    assert page.fields[key].suffix() == ' mm/s'
    page.save()
    assert repo.load() == {'arm_mode': 'precision'}
    assert parameter_override(key, .001) == .001

    page.fields[key].setValue(speed_mm_s)
    page.save()
    expected = {'arm_mode': 'precision', key: speed_mm_s / 1000.}
    assert repo.load() == expected
    assert parameter_override(key, .001) == .001
    page.close()

    _startup_settings.cache_clear()  # Simulate the next application start.
    assert parameter_override(key, .001) == pytest.approx(speed_mm_s / 1000.)
    restored = AdvancedSettingsPage(repository=repo)
    assert restored.fields[key].value() == speed_mm_s
    restored.reset_defaults()
    assert restored.fields[key].value() == 1.
    assert repo.load() == expected
    restored.save()
    assert repo.load() == {'arm_mode': 'precision'}
    restored.close()
