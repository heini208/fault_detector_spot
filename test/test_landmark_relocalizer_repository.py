"""Tests for landmark relocalizer repository ownership."""

from types import SimpleNamespace

from fault_detector_spot.navigation.behaviours.landmark_relocalizer import (
    LandmarkRelocalizer,
)


class FakeRepository:
    def __init__(self):
        self.loaded = []

    def load(self, map_name):
        self.loaded.append(map_name)
        return SimpleNamespace(localization_landmarks=[])


class FakeSlamHelper:
    maps_dir = "/unused"


def test_landmark_relocalizer_uses_its_repository_dependency():
    repository = FakeRepository()
    relocalizer = LandmarkRelocalizer(
        FakeSlamHelper(),
        map_repository=repository,
    )

    relocalizer._load_landmarks_for_map("plant")

    assert repository.loaded == ["plant"]
    assert relocalizer.landmark_map == {}
    assert relocalizer.feedback_message == (
        "No localization landmarks found"
    )


def test_landmark_relocalizer_does_not_require_repository_on_rtab_helper():
    repository = FakeRepository()
    helper = FakeSlamHelper()
    assert not hasattr(helper, "map_repository")

    relocalizer = LandmarkRelocalizer(
        helper,
        map_repository=repository,
    )

    relocalizer._load_landmarks_for_map("plant")

    assert repository.loaded == ["plant"]
