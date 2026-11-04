"""Retain unresolved shutdown ownership across execution-process restarts."""

from contextlib import contextmanager
import fcntl
import json
import os
from pathlib import Path
import tempfile
from threading import RLock
from uuid import UUID, uuid4

from .file_storage import atomic_write_text, validate_storage_name
from .runtime_paths import fault_detector_runtime_root


class StopUnconfirmedStore:
    """Keep each process owner's stop uncertainty independently and durably.

    A fresh process being idle cannot reconcile work from a previous process.
    ``clear_confirmed`` is reserved for independently verified terminal action
    state and fresh physical-stop evidence for the named prior resources.
    """

    def __init__(self, owner="bt_runner", root=None):
        self.owner = validate_storage_name(owner, "Shutdown owner")
        self.root = (Path(root) if root is not None else
                     fault_detector_runtime_root() / "shutdown")
        self.path = self.root / f"{self.owner}.json"
        self.session_path = self.root / f"{self.owner}.active"
        self._lock = RLock()
        self._session_token = None

    @staticmethod
    def _process_identity(pid):
        stat = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
        fields = stat.rsplit(")", 1)[1].split()
        if fields[0] in {"Z", "X"}:
            raise ProcessLookupError("Session process has exited")
        return {
            "pid": pid,
            "started": fields[19],
            "boot": Path("/proc/sys/kernel/random/boot_id").read_text(encoding="utf-8").strip(),
        }

    def _read_session(self, path):
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            if not self._valid_session(data):
                raise ValueError("Invalid active-session schema")
            if path == self.session_path and data["token"] == self._session_token:
                return {}
            if path != self.session_path:
                process = data["process"]
                try:
                    # BT and API start independently. Their currently live
                    # sessions are not historical shutdown uncertainty.
                    if self._process_identity(int(process["pid"])) == process:
                        return {}
                except (OSError, ValueError, KeyError, TypeError, IndexError):
                    pass
            if self._idle_session_has_exited(data):
                return {}
            return {"active_session": "Prior session has no confirmed shutdown; reconciliation is required"}
        except FileNotFoundError:
            return {}
        except (OSError, ValueError, KeyError, TypeError, AttributeError) as error:
            return {"active_session": f"Active-session evidence cannot be read: {error}"}

    @staticmethod
    def _valid_session(data):
        return (isinstance(data, dict) and data.get("version") in (1, 2)
                and isinstance(data.get("token"), str) and bool(data["token"])
                and isinstance(data.get("process"), dict)
                and (data["version"] == 1
                     or isinstance(data.get("unresolved_work"), bool)))

    def _idle_session_has_exited(self, data):
        """A dead process with durable empty ownership left nothing to stop."""
        if data.get("version") != 2 or data.get("unresolved_work") is not False:
            return False
        process = data["process"]
        try:
            pid = process["pid"]
            if (type(pid) is not int or pid <= 0
                    or set(process) != {"pid", "started", "boot"}
                    or not isinstance(process["started"], str)
                    or not process["started"].isascii() or not process["started"].isdecimal()
                    or not isinstance(process["boot"], str)
                    or str(UUID(process["boot"])) != process["boot"]):
                return False
            return self._process_identity(pid) != process
        except (FileNotFoundError, ProcessLookupError):
            return True
        except (OSError, ValueError, KeyError, TypeError, IndexError):
            return False

    @contextmanager
    def _session_guard(self):
        """Serialize session replacement and ownership updates across processes."""
        self.root.mkdir(parents=True, exist_ok=True)
        descriptor = os.open(self.root / f"{self.owner}.lock", os.O_RDWR | os.O_CREAT, 0o600)
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX)
            yield
        finally:
            os.close(descriptor)

    def begin_session(self):
        """Acquire an idle session; retain any prior uncertain physical work."""
        with self._lock, self._session_guard():
            if self._session_token is not None:
                return True
            try:
                previous = json.loads(self.session_path.read_text(encoding="utf-8"))
            except FileNotFoundError:
                previous = None
            except (OSError, ValueError, TypeError):
                return False
            if previous is not None:
                if not self._valid_session(previous) or not self._idle_session_has_exited(previous):
                    return False
                self.session_path.unlink()
            token = str(uuid4())
            content = json.dumps({"version": 2, "owner": self.owner, "token": token,
                                  "process": self._process_identity(os.getpid()),
                                  "unresolved_work": False}) + "\n"
            # Publish a complete marker atomically. The other process owner
            # starts concurrently and must never see a partially written file.
            descriptor, staged = tempfile.mkstemp(prefix=f".{self.owner}.", suffix=".tmp", dir=self.root)
            try:
                with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                    if stream.write(content) != len(content):
                        raise OSError("Incomplete shutdown session write")
                    stream.flush()
                    os.fsync(stream.fileno())
                try:
                    os.link(staged, self.session_path)
                except FileExistsError:
                    return False
            finally:
                os.unlink(staged)
            self._sync_directory()
            self._session_token = token
            return True

    def set_session_work(self, unresolved):
        """Persist ownership before dispatch; clear only with terminal evidence."""
        if not isinstance(unresolved, bool):
            raise TypeError("Session work state must be a boolean")
        with self._lock, self._session_guard():
            if self._session_token is None:
                raise RuntimeError("No shutdown session ownership was acquired")
            data = json.loads(self.session_path.read_text(encoding="utf-8"))
            if not self._valid_session(data) or data["token"] != self._session_token:
                raise RuntimeError("Active shutdown session ownership changed")
            if data.get("unresolved_work") is unresolved:
                # A previous atomic replacement may have succeeded before its
                # directory sync failed. Retrying must finish that durability.
                self._sync_directory()
                return
            data["version"] = 2
            data["unresolved_work"] = unresolved
            atomic_write_text(self.session_path, json.dumps(data) + "\n")
            self._sync_directory()

    def confirm_session(self):
        """Remove this session's marker only after complete confirmed cleanup."""
        with self._lock, self._session_guard():
            if self._session_token is None:
                return True
            try:
                data = json.loads(self.session_path.read_text(encoding="utf-8"))
            except FileNotFoundError:
                self._sync_directory()
                self._session_token = None
                return True
            if data.get("token") != self._session_token:
                raise RuntimeError("Active shutdown session ownership changed")
            self.session_path.unlink()
            self._sync_directory()
            self._session_token = None
            return True

    def _sync_directory(self):
        descriptor = os.open(self.root, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)

    @staticmethod
    def _read(path):
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            unresolved = data["unresolved"]
            if (data.get("version") != 1 or not isinstance(unresolved, dict)
                    or any(not isinstance(key, str) or not key
                           or not isinstance(detail, str)
                           for key, detail in unresolved.items())):
                raise ValueError("Invalid shutdown-state schema")
            return unresolved
        except FileNotFoundError:
            return {}
        except (OSError, ValueError, KeyError, TypeError) as exception:
            # Corrupt or unreadable evidence is uncertainty, never idle proof.
            return {"state_read_error": f"Shutdown evidence cannot be read: {exception}"}

    def load(self):
        """Return unresolved resources from every process owner for admission."""
        with self._lock:
            try:
                paths = sorted(path for path in self.root.iterdir()
                               if path.suffix in {".json", ".active"})
            except FileNotFoundError:
                return {}
            except OSError as exception:
                return {"shutdown/state_read_error": str(exception)}
            return {
                f"{path.stem}/{resource}": detail
                for path in paths
                for resource, detail in (self._read_session(path) if path.suffix == ".active"
                                         else self._read(path)).items()
            }

    def record(self, unresolved):
        """Merge uncertainty for this owner; never erase retained prior work."""
        unresolved = dict(unresolved)
        if any(not isinstance(key, str) or not key or not isinstance(detail, str)
               for key, detail in unresolved.items()):
            raise ValueError("Shutdown resources and details must be nonempty names and strings")
        if not unresolved:
            return
        with self._lock:
            if self._session_token is not None:
                self.set_session_work(True)
            retained = self._read(self.path)
            retained.update(unresolved)
            self._write(retained)

    def clear_confirmed(self, resources):
        """Remove only individually reconciled prior resources for this owner.

        Callers must establish prior action termination and fresh physical stop
        evidence independently; new idle executors or a software reset are not
        sufficient. Ordinary startup and shutdown never call this operation.
        """
        with self._lock:
            retained = self._read(self.path)
            for resource in resources:
                retained.pop(resource, None)
                if resource == "active_session":
                    self.session_path.unlink(missing_ok=True)
                    self._session_token = None
            self._write(retained)

    def _write(self, unresolved):
        atomic_write_text(self.path, json.dumps({
            "version": 1, "owner": self.owner, "unresolved": unresolved,
        }, indent=2, sort_keys=True) + "\n")
