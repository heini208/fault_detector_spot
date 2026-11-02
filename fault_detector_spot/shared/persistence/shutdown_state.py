"""Retain unresolved shutdown ownership across execution-process restarts."""

import json
import os
from pathlib import Path
from threading import RLock
from uuid import uuid4

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
            if (data.get("version") != 1 or not isinstance(data.get("token"), str)
                    or not data["token"] or not isinstance(data.get("process"), dict)):
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
            return {"active_session": "Prior session has no confirmed shutdown; reconciliation is required"}
        except FileNotFoundError:
            return {}
        except (OSError, ValueError, KeyError, TypeError, AttributeError) as error:
            return {"active_session": f"Active-session evidence cannot be read: {error}"}

    def begin_session(self):
        """Persist a dirty marker before admission; never replace prior work."""
        with self._lock:
            if self._session_token is not None:
                return True
            self.root.mkdir(parents=True, exist_ok=True)
            token = str(uuid4())
            content = json.dumps({"version": 1, "owner": self.owner, "token": token,
                                  "process": self._process_identity(os.getpid())}) + "\n"
            try:
                descriptor = os.open(self.session_path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            except FileExistsError:
                return False
            # Leave a partial marker on write failure: unreadable evidence must
            # fence the next startup too. No motion has been admitted yet.
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                stream.write(content)
                stream.flush()
                os.fsync(stream.fileno())
            self._sync_directory()
            self._session_token = token
            return True

    def confirm_session(self):
        """Remove this session's marker only after complete confirmed cleanup."""
        with self._lock:
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
