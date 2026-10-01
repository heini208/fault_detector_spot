"""Retain one fresh surface observation while capture-time TF catches up."""

from tf2_ros import TransformException

from fault_detector_spot.shared.geometry.movement_geometry import MovementGeometryUnavailable


class SurfaceOrientationTarget:
    """Callable target builder with separate sensing and TF wait budgets."""

    def __init__(self, surface_source, resolve_target, sensor_id,
                 receipt_not_before, clock, sensing_timeout_sec, tf_timeout_sec):
        self._source = surface_source
        self._resolve_target = resolve_target
        self._sensor_id = sensor_id
        self._receipt_not_before = receipt_not_before
        self._clock = clock
        self._sensing_timeout = sensing_timeout_sec
        self._tf_timeout = tf_timeout_sec
        self._estimate = None
        self._deadline = None

    def __call__(self):
        if self._deadline is None:
            self._deadline = self._clock() + self._sensing_timeout
        if self._estimate is None:
            self._acquire_estimate()
        try:
            return self._resolve_target(self._sensor_id, self._estimate)
        except TransformException as exception:
            self._wait_or_raise(
                exception,
                "Waiting for surface orientation capture-time TF "
                f"at {self._estimate.stamp_nanoseconds / 1e9:.9f}: {exception}",
                "Surface orientation TF synchronization timed out "
                f"after {self._tf_timeout:.1f}s; ",
            )

    def _acquire_estimate(self):
        try:
            self._estimate = self._source.surface_normal(
                receipt_not_before=self._receipt_not_before,
            )
        except ValueError as exception:
            self._wait_or_raise(
                exception,
                f"Waiting for a fresh reliable surface normal: {exception}",
                "Surface orientation sensing timed out after "
                f"{self._sensing_timeout:.1f}s; ",
            )
        self._deadline = self._clock() + self._tf_timeout

    def _wait_or_raise(self, exception, detail, timeout_prefix):
        if self._clock() >= self._deadline:
            raise RuntimeError(timeout_prefix + detail) from exception
        raise MovementGeometryUnavailable(detail) from exception
