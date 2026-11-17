from dataclasses import dataclass


@dataclass(frozen=True)
class MotionCommand:
    vx_cm_s: float = 0.
    vy_cm_s: float = 0.
    w_rad_s: float = 0.


@dataclass(frozen=True)
class PersonObservation:
    track_id: int
    bbox: tuple[float, float, float, float]
    confidence: float
    timestamp: float


@dataclass(frozen=True)
class GestureObservation:
    name: str
    confidence: float
    target_id: int | None
    timestamp: float


@dataclass(frozen=True)
class RadarObservation:
    obstacle_detected: bool | None
    distance_m: float | None
    timestamp: float
    data_valid: bool


@dataclass(frozen=True)
class PerceptionFrame:
    timestamp: float
    width: int
    height: int
    people: tuple[PersonObservation, ...]
    gesture: GestureObservation | None = None
    camera_ok: bool = True


@dataclass(frozen=True)
class ControlDecision:
    state: str
    command: MotionCommand
    reason: str
    target_id: int | None
    horizontal_error: float = 0.
    relative_size: float | None = None
