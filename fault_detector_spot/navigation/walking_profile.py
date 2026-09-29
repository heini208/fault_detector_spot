"""Configurable native Spot walking settings for planar base goals."""

from dataclasses import dataclass, field
import math

from bosdyn.api.spot import robot_command_pb2


GAITS = {
    "auto": robot_command_pb2.HINT_AUTO,
    "trot": robot_command_pb2.HINT_TROT,
    "speed_select_trot": robot_command_pb2.HINT_SPEED_SELECT_TROT,
    "crawl": robot_command_pb2.HINT_CRAWL,
    "speed_select_crawl": robot_command_pb2.HINT_SPEED_SELECT_CRAWL,
}


@dataclass(frozen=True)
class WalkingProfile:
    relative_speed_mps: float = 0.10
    tag_speed_mps: float = 0.15
    angular_speed_rad_s: float = 0.20
    gait: str = "auto"

    def __post_init__(self):
        for value in (self.relative_speed_mps, self.tag_speed_mps, self.angular_speed_rad_s):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("Walking profile speeds must be positive and finite")
        if self.gait not in GAITS:
            raise ValueError(f"Unsupported walking gait: {self.gait}")


@dataclass(frozen=True)
class WalkingProfiles:
    normal: WalkingProfile = field(default_factory=WalkingProfile)
    precision: WalkingProfile = field(default_factory=lambda: WalkingProfile(
        relative_speed_mps=0.05, tag_speed_mps=0.05,
        angular_speed_rad_s=0.10, gait="speed_select_crawl",
    ))
    relative_profile: str = "normal"
    tag_profile: str = "normal"

    def __post_init__(self):
        for name in (self.relative_profile, self.tag_profile):
            if name not in ("normal", "precision"):
                raise ValueError(f"Unknown walking profile: {name}")

    def for_move(self, tag_relative=False):
        return getattr(self, self.tag_profile if tag_relative else self.relative_profile)

    @classmethod
    def from_node(cls, node):
        def parameter(key, default):
            name = f"base.walking.{key}"
            if not node.has_parameter(name):
                node.declare_parameter(name, default)
            return node.get_parameter(name).value

        defaults = cls()
        profiles = {}
        for name in ("normal", "precision"):
            profile = getattr(defaults, name)
            profiles[name] = WalkingProfile(
                **{key: float(parameter(f"{name}.{key}", getattr(profile, key)))
                   for key in ("relative_speed_mps", "tag_speed_mps", "angular_speed_rad_s")},
                gait=str(parameter(f"{name}.gait", profile.gait)),
            )
        return cls(
            **profiles,
            relative_profile=str(parameter("relative_profile", "normal")),
            tag_profile=str(parameter("tag_profile", "normal")),
        )
