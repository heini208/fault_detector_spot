"""Interrupted replacement must never publish a partial persisted contract."""

import pytest

from fault_detector_spot.shared.persistence import file_storage


@pytest.mark.parametrize("failure", ["partial_write", "interrupt", "fsync"])
def test_failed_atomic_write_preserves_previous_file(tmp_path, monkeypatch, failure):
    path = tmp_path / "recording.json"
    original = '{"commands": []}\n'
    file_storage.atomic_write_text(path, original)
    fdopen = file_storage.os.fdopen

    def faulty_stream(*args, **kwargs):
        stream = fdopen(*args, **kwargs)
        write = stream.write

        def partial_write(text):
            write(text[:3])
            if failure == "interrupt":
                raise KeyboardInterrupt()
            return 3

        monkeypatch.setattr(stream, "write", partial_write)
        return stream

    with monkeypatch.context() as patch:
        if failure == "fsync":
            def fail_sync(_descriptor):
                raise OSError("storage unavailable")
            patch.setattr(file_storage.os, "fsync", fail_sync)
        else:
            patch.setattr(file_storage.os, "fdopen", faulty_stream)
        with pytest.raises(KeyboardInterrupt if failure == "interrupt" else OSError):
            file_storage.atomic_write_text(path, '{"commands": ["replacement"]}\n')

    assert path.read_text() == original
    assert list(tmp_path.iterdir()) == [path]
    file_storage.atomic_write_text(path, '{"commands": ["retry"]}\n')
    assert path.read_text() == '{"commands": ["retry"]}\n'
