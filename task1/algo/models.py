from dataclasses import dataclass, field
from typing import List


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    heading: float

    @property
    def theta(self) -> float:
        return self.heading


@dataclass(frozen=True)
class Obstacle:
    obstacle_id: int
    x: float
    y: float
    image_face: str

    @property
    def image_side(self) -> str:
        return self.image_face


@dataclass(frozen=True)
class MotionCommand:
    kind: str
    value: float


@dataclass
class PathResult:
    poses: List[Pose] = field(default_factory=list)
    commands: List[MotionCommand] = field(default_factory=list)
    distance_cm: float = 0.0
    estimated_time_s: float = 0.0
    is_valid: bool = False
