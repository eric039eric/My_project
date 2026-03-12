# kinematics.py
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Dict, Tuple
import numpy as np # type: ignore


@dataclass
class ArmPose:
    base: int
    shoulder: int
    elbow: int
    claw: int
    name: str = ""

    def as_tuple(self):
        return self.base, self.shoulder, self.elbow, self.claw


@dataclass
class KinematicsConfig:
    work_width_mm: float = 300.0
    work_height_mm: float = 200.0

    base_range: Tuple[int, int] = (20, 160)
    shoulder_range: Tuple[int, int] = (45, 135)
    elbow_range: Tuple[int, int] = (30, 150)
    claw_range: Tuple[int, int] = (20, 120)

    claw_open: int = 95
    claw_closed: int = 55

    home_base: int = 90
    home_shoulder: int = 90
    home_elbow: int = 90
    home_claw: int = 95

    place_points: Dict[str, Tuple[float, float]] = field(default_factory=lambda: {
        "RED":   (260.0, 160.0),
        "GREEN": (260.0, 100.0),
        "BLUE":  (260.0,  40.0),
    })


# shoulder_offset, elbow_offset, claw_key ("open" / "closed")
PICK_OFFSETS: Dict[str, Tuple[int, int, str]] = {
    "above": (-6,   6, "open"),
    "down":  (10, -12, "open"),
    "lift":  (-4,   8, "closed"),
}

PLACE_OFFSETS: Dict[str, Tuple[int, int, str]] = {
    "above": (-8,  8, "closed"),
    "down":  ( 8, -8, "closed"),
}


class DemoKinematics:
    def __init__(self, cfg: KinematicsConfig):
        self.cfg = cfg

    def _clamp(self, val: int, lo: int, hi: int) -> int:
        return int(np.clip(val, lo, hi))

    def clamp_pose(self, pose: ArmPose) -> ArmPose:
        c = self.cfg
        return ArmPose(
            base     = self._clamp(pose.base,     *c.base_range),
            shoulder = self._clamp(pose.shoulder, *c.shoulder_range),
            elbow    = self._clamp(pose.elbow,    *c.elbow_range),
            claw     = self._clamp(pose.claw,     *c.claw_range),
            name     = pose.name,
        )

    def home(self) -> ArmPose:
        c = self.cfg
        return self.clamp_pose(ArmPose(
            c.home_base, c.home_shoulder, c.home_elbow, c.home_claw, name="home"
        ))

    def with_claw(self, pose: ArmPose, claw_value: int, name: str = "") -> ArmPose:
        return self.clamp_pose(ArmPose(
            pose.base, pose.shoulder, pose.elbow, claw_value,
            name=name or pose.name
        ))

    def _world_to_servo(self, wx: float, wy: float) -> Tuple[int, int, int]:
        c = self.cfg
        x = float(np.clip(wx, 0.0, c.work_width_mm))
        y = float(np.clip(wy, 0.0, c.work_height_mm))
        base     = int(np.interp(x, [0.0, c.work_width_mm],  [155, 25]))
        shoulder = int(np.interp(y, [0.0, c.work_height_mm], [74, 118]))
        elbow    = int(np.interp(y, [0.0, c.work_height_mm], [122, 72]))
        return base, shoulder, elbow

    def _resolve_claw(self, key: str) -> int:
        return self.cfg.claw_open if key == "open" else self.cfg.claw_closed

    def pick_pose(self, wx: float, wy: float, stage: str = "above") -> ArmPose:
        base, sh, el = self._world_to_servo(wx, wy)
        d_sh, d_el, claw_key = PICK_OFFSETS.get(stage, (0, 0, "open"))
        return self.clamp_pose(ArmPose(
            base, sh + d_sh, el + d_el, self._resolve_claw(claw_key),
            name=f"pick_{stage}"
        ))

    def place_pose(self, color: str, stage: str = "above") -> ArmPose:
        color = color.upper()
        if color not in self.cfg.place_points:
            raise KeyError(f"place_points 裡沒有顏色 {color}")
        wx, wy = self.cfg.place_points[color]
        base, sh, el = self._world_to_servo(wx, wy)
        d_sh, d_el, claw_key = PLACE_OFFSETS.get(stage, (0, 0, "closed"))
        return self.clamp_pose(ArmPose(
            base, sh + d_sh, el + d_el, self._resolve_claw(claw_key),
            name=f"place_{color}_{stage}"
        ))
