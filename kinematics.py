# kinematics.py
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Tuple
import numpy as np


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

    base_min: int = 20
    base_max: int = 160

    shoulder_min: int = 45
    shoulder_max: int = 135

    elbow_min: int = 30
    elbow_max: int = 150

    claw_min: int = 20
    claw_max: int = 120

    claw_open: int = 95
    claw_closed: int = 55

    home_base: int = 90
    home_shoulder: int = 90
    home_elbow: int = 90
    home_claw: int = 95

    # 每個顏色的放置點世界座標 (mm)
    place_points: Dict[str, Tuple[float, float]] = field(default_factory=lambda: {
        "RED": (260.0, 160.0),
        "GREEN": (260.0, 100.0),
        "BLUE": (260.0, 40.0),
    })


class DemoKinematics:
    def __init__(self, config: KinematicsConfig):
        self.cfg = config

    def clamp_pose(self, pose: ArmPose) -> ArmPose:
        return ArmPose(
            base=int(np.clip(pose.base, self.cfg.base_min, self.cfg.base_max)),
            shoulder=int(np.clip(pose.shoulder, self.cfg.shoulder_min, self.cfg.shoulder_max)),
            elbow=int(np.clip(pose.elbow, self.cfg.elbow_min, self.cfg.elbow_max)),
            claw=int(np.clip(pose.claw, self.cfg.claw_min, self.cfg.claw_max)),
            name=pose.name
        )

    def home(self) -> ArmPose:
        return self.clamp_pose(ArmPose(
            self.cfg.home_base,
            self.cfg.home_shoulder,
            self.cfg.home_elbow,
            self.cfg.home_claw,
            name="home"
        ))

    def with_claw(self, pose: ArmPose, claw_value: int, name: str = "") -> ArmPose:
        return self.clamp_pose(ArmPose(
            base=pose.base,
            shoulder=pose.shoulder,
            elbow=pose.elbow,
            claw=claw_value,
            name=name or pose.name
        ))

    def _map_world_to_arm(self, world_x: float, world_y: float):
        x = float(np.clip(world_x, 0.0, self.cfg.work_width_mm))
        y = float(np.clip(world_y, 0.0, self.cfg.work_height_mm))

        base = int(np.interp(x, [0.0, self.cfg.work_width_mm], [155, 25]))
        shoulder = int(np.interp(y, [0.0, self.cfg.work_height_mm], [74, 118]))
        elbow = int(np.interp(y, [0.0, self.cfg.work_height_mm], [122, 72]))

        return base, shoulder, elbow

    def pick_pose(self, world_x: float, world_y: float, stage: str = "above") -> ArmPose:
        base, shoulder, elbow = self._map_world_to_arm(world_x, world_y)

        if stage == "above":
            shoulder -= 6
            elbow += 6
            claw = self.cfg.claw_open
        elif stage == "down":
            shoulder += 10
            elbow -= 12
            claw = self.cfg.claw_open
        elif stage == "lift":
            shoulder -= 4
            elbow += 8
            claw = self.cfg.claw_closed
        else:
            claw = self.cfg.claw_open

        return self.clamp_pose(ArmPose(base, shoulder, elbow, claw, name=f"pick_{stage}"))

    def place_pose(self, color_name: str, stage: str = "above") -> ArmPose:
        color_name = color_name.upper()
        if color_name not in self.cfg.place_points:
            raise KeyError(f"place_points 裡沒有顏色 {color_name}")

        world_x, world_y = self.cfg.place_points[color_name]
        base, shoulder, elbow = self._map_world_to_arm(world_x, world_y)

        if stage == "above":
            shoulder -= 8
            elbow += 8
            claw = self.cfg.claw_closed
        elif stage == "down":
            shoulder += 8
            elbow -= 8
            claw = self.cfg.claw_closed
        else:
            claw = self.cfg.claw_closed

        return self.clamp_pose(ArmPose(base, shoulder, elbow, claw, name=f"place_{color_name}_{stage}"))
