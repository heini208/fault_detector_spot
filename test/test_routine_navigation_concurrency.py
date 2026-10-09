"""Navigation assignments stay consistent across independent setup clients."""

from concurrent.futures import ThreadPoolExecutor
from threading import Event

from test_routine_navigation_setup import navigation_setup


def test_concurrent_map_change_cannot_commit_old_maps_waypoint(
    navigation_setup, monkeypatch,
):
    probe, commands, _, context = navigation_setup
    first = probe.save_routine_map(
        context, "motor", "magnetic_scan", "factory",
    )
    second = probe.select_routine(
        probe.open_context("other-probe-ui").context,
        "motor",
        "magnetic_scan",
    )
    lookup_entered = Event()
    release_lookup = Event()
    map_save_started = Event()
    map_save_finished = Event()
    persisted_associations = []
    original_lookup = probe.map_repository.get_waypoint
    original_save = probe.object_repository.save

    def paused_waypoint_lookup(map_id, waypoint_id):
        waypoint = original_lookup(map_id, waypoint_id)
        lookup_entered.set()
        assert release_lookup.wait(2), "Waypoint lookup was not released"
        return waypoint

    def capture_persisted_association(definition, **kwargs):
        result = original_save(definition, **kwargs)
        routine = definition.get_routine("magnetic_scan")
        persisted_associations.append((routine.map_id, routine.waypoint_id))
        return result

    def change_map():
        map_save_started.set()
        try:
            return probe.save_routine_map(
                second.context, "motor", "magnetic_scan", "workshop",
            )
        finally:
            map_save_finished.set()

    monkeypatch.setattr(probe.map_repository, "get_waypoint", paused_waypoint_lookup)
    monkeypatch.setattr(probe.object_repository, "save", capture_persisted_association)
    with ThreadPoolExecutor(max_workers=2) as workers:
        waypoint_save = workers.submit(
            probe.save_routine_waypoint,
            first.context, "motor", "magnetic_scan", "factory", "inspection",
        )
        try:
            assert lookup_entered.wait(2), "Waypoint lookup did not start"
            map_save = workers.submit(change_map)
            assert map_save_started.wait(2), "Other client's map save did not start"
            assert not map_save_finished.wait(0.25), (
                "Map changed while the previous map's waypoint was being validated"
            )
        finally:
            release_lookup.set()
        waypoint_save.result(timeout=2)
        map_save.result(timeout=2)

    assert persisted_associations == [
        ("factory", "inspection"),
        ("workshop", ""),
    ]
    routine = probe.object_repository.load("motor").get_routine("magnetic_scan")
    assert (routine.map_id, routine.waypoint_id) == ("workshop", "")
    assert commands.submitted == []
