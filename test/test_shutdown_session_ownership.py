"""A process exit is uncertainty only while that process owns unfinished work."""

import json
import os
from concurrent.futures import ThreadPoolExecutor
import subprocess
import sys

import pytest

from fault_detector_spot.shared.persistence.shutdown_state import StopUnconfirmedStore


def exited_session(tmp_path, *, unresolved=False):
    subprocess.run([
        sys.executable, "-c",
        "from fault_detector_spot.shared.persistence.shutdown_state import StopUnconfirmedStore\n"
        "import sys\n"
        "store = StopUnconfirmedStore(root=sys.argv[1])\n"
        "assert store.begin_session()\n"
        "store.set_session_work(sys.argv[2] == 'dirty')\n",
        str(tmp_path), "dirty" if unresolved else "idle",
    ], check=True)
    return StopUnconfirmedStore(root=tmp_path)


def test_exited_idle_session_restarts_without_recovery_or_stop(tmp_path):
    restarted = exited_session(tmp_path)
    old_token = json.loads(restarted.session_path.read_text())["token"]

    assert restarted.load() == {}
    assert StopUnconfirmedStore("application_api", tmp_path).load() == {}
    assert restarted.begin_session()
    assert json.loads(restarted.session_path.read_text())["token"] != old_token
    assert restarted.load() == {}


def test_concurrent_peer_startup_never_reads_partial_session_marker(tmp_path, monkeypatch):
    store = StopUnconfirmedStore(root=tmp_path)
    peer = StopUnconfirmedStore("application_api", tmp_path)
    fsync = os.fsync
    observations = []

    def observe_during_write(descriptor):
        observations.append(peer.load())
        fsync(descriptor)

    monkeypatch.setattr(os, "fsync", observe_during_write)
    assert store.begin_session()
    assert observations and all(not unresolved for unresolved in observations)


def test_exited_owned_work_still_requires_reconciliation(tmp_path):
    restarted = exited_session(tmp_path, unresolved=True)
    before = restarted.session_path.read_bytes()

    assert "bt_runner/active_session" in restarted.load()
    assert not restarted.begin_session()
    assert restarted.session_path.read_bytes() == before


def test_concurrent_restarts_cannot_replace_the_new_live_owner(tmp_path):
    exited_session(tmp_path)
    contenders = [StopUnconfirmedStore(root=tmp_path) for _ in range(4)]
    with ThreadPoolExecutor(max_workers=4) as pool:
        acquired = list(pool.map(lambda store: store.begin_session(), contenders))
    assert acquired.count(True) == 1
    winner = contenders[acquired.index(True)]
    winner.set_session_work(True)
    assert json.loads(winner.session_path.read_text())["unresolved_work"] is True


def test_idle_session_replacement_preserves_prior_resource_evidence(tmp_path):
    restarted = exited_session(tmp_path)
    restarted.record({"arm": "Previous action termination is unresolved"})

    assert restarted.begin_session()
    assert restarted.load() == {"bt_runner/arm": "Previous action termination is unresolved"}


@pytest.mark.parametrize("kind", ["legacy", "missing_flag", "invalid_flag", "invalid_process", "corrupt"])
def test_unknown_session_ownership_is_never_assumed_idle(tmp_path, kind):
    restarted = exited_session(tmp_path)
    data = json.loads(restarted.session_path.read_text())
    if kind == "legacy":
        data["version"] = 1
        data.pop("unresolved_work")
    elif kind == "missing_flag":
        data.pop("unresolved_work")
    elif kind == "invalid_flag":
        data["unresolved_work"] = "false"
    elif kind == "invalid_process":
        data["process"] = {"pid": -1}
    restarted.session_path.write_text("invalid" if kind == "corrupt" else json.dumps(data))

    assert "bt_runner/active_session" in restarted.load()
    assert not restarted.begin_session()


def test_live_idle_owner_cannot_be_replaced_or_cleared_by_another_owner(tmp_path):
    original = StopUnconfirmedStore(root=tmp_path)
    assert original.begin_session()
    duplicate = StopUnconfirmedStore(root=tmp_path)
    assert not duplicate.begin_session()
    with pytest.raises(RuntimeError, match="ownership was acquired"):
        duplicate.set_session_work(False)
    assert original.session_path.exists()


@pytest.mark.parametrize("field,value", [("pid", str), ("pid", lambda _: True),
                                         ("started", lambda _: "unknown"),
                                         ("boot", lambda _: "unknown")])
def test_malformed_process_identity_cannot_be_used_as_proof_of_exit(tmp_path, field, value):
    store = StopUnconfirmedStore(root=tmp_path)
    assert store.begin_session()
    data = json.loads(store.session_path.read_text())
    data["process"][field] = value(data["process"][field])
    store.session_path.write_text(json.dumps(data))
    duplicate = StopUnconfirmedStore(root=tmp_path)

    assert "bt_runner/active_session" in duplicate.load()
    assert not duplicate.begin_session()
    assert json.loads(store.session_path.read_text()) == data


def test_owned_work_can_only_be_changed_with_current_session_token(tmp_path):
    store = StopUnconfirmedStore(root=tmp_path)
    assert store.begin_session()
    store.set_session_work(True)
    assert json.loads(store.session_path.read_text())["unresolved_work"] is True
    store.set_session_work(False)
    assert json.loads(store.session_path.read_text())["unresolved_work"] is False
    data = json.loads(store.session_path.read_text())
    data["token"] = "replacement-owner"
    data["unresolved_work"] = True
    store.session_path.write_text(json.dumps(data))
    with pytest.raises(RuntimeError, match="ownership changed"):
        store.set_session_work(False)
    assert json.loads(store.session_path.read_text())["unresolved_work"] is True


def test_failed_resource_report_keeps_session_uncertain(tmp_path, monkeypatch):
    store = StopUnconfirmedStore(root=tmp_path)
    assert store.begin_session()
    monkeypatch.setattr(store, "_write", lambda _: (_ for _ in ()).throw(OSError("disk full")))
    with pytest.raises(OSError, match="disk full"):
        store.record({"arm": "Unresolved termination"})
    assert json.loads(store.session_path.read_text())["unresolved_work"] is True


def test_retry_finishes_directory_sync_after_atomic_ownership_write(tmp_path, monkeypatch):
    store = StopUnconfirmedStore(root=tmp_path)
    assert store.begin_session()
    sync = store._sync_directory
    attempts = []

    def interrupted_sync():
        attempts.append(True)
        if len(attempts) == 1:
            raise OSError("Directory sync interrupted")
        sync()

    monkeypatch.setattr(store, "_sync_directory", interrupted_sync)
    with pytest.raises(OSError, match="sync interrupted"):
        store.set_session_work(True)
    assert json.loads(store.session_path.read_text())["unresolved_work"] is True
    store.set_session_work(True)
    assert len(attempts) == 2
