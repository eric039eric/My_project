# planner.py
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional
import math
import time

from vision import TargetObservation # type: ignore
from kinematics import DemoKinematics, ArmPose # type: ignore


class PlannerState(Enum):
    IDLE = auto()
    SEARCH = auto()
    PICK_ABOVE = auto()
    PICK_DOWN = auto()
    GRAB = auto()
    LIFT = auto()
    PLACE_ABOVE = auto()
    PLACE_DOWN = auto()
    RELEASE = auto()
    RETURN_HOME = auto()


@dataclass
class PlannerConfig:
    stable_frames_required: int = 8
    stable_distance_mm: float = 12.0
    dwell_pick_above_sec: float = 0.9
    dwell_pick_down_sec: float = 0.9
    dwell_grab_sec: float = 0.8
    dwell_lift_sec: float = 0.9
    dwell_place_above_sec: float = 0.9
    dwell_place_down_sec: float = 0.9
    dwell_release_sec: float = 0.8
    dwell_return_sec: float = 1.0


class SortPlanner:
    def __init__(self, config: PlannerConfig):
        self.cfg = config
        self.state = PlannerState.IDLE
        self.enabled = False

        self.last_obs: Optional[TargetObservation] = None
        self.stable_count = 0
        self.locked_target: Optional[TargetObservation] = None

        self.pick_above_pose: Optional[ArmPose] = None
        self.pick_down_pose: Optional[ArmPose] = None
        self.grab_pose: Optional[ArmPose] = None
        self.lift_pose: Optional[ArmPose] = None
        self.place_above_pose: Optional[ArmPose] = None
        self.place_down_pose: Optional[ArmPose] = None
        self.release_pose: Optional[ArmPose] = None
        self.return_pose: Optional[ArmPose] = None

        self.state_enter_time = time.monotonic()
        self.status_text = "IDLE"

    def start(self):
        self.enabled = True
        self.state = PlannerState.SEARCH
        self.last_obs = None
        self.stable_count = 0
        self.locked_target = None
        self.state_enter_time = time.monotonic()
        self.status_text = "SEARCH"

    def stop(self):
        self.enabled = False
        self.state = PlannerState.IDLE
        self.last_obs = None
        self.stable_count = 0
        self.locked_target = None
        self.status_text = "IDLE"

    def _enter(self, state: PlannerState, text: str):
        self.state = state
        self.state_enter_time = time.monotonic()
        self.status_text = text

    def _elapsed(self) -> float:
        return time.monotonic() - self.state_enter_time

    def _is_stable(self, obs: TargetObservation) -> bool:
        if not obs.workspace_ready or obs.world_x is None or obs.world_y is None:
            self.last_obs = None
            self.stable_count = 0
            return False

        if self.last_obs is None:
            self.last_obs = obs
            self.stable_count = 1
            return False

        same_color = obs.color_name == self.last_obs.color_name
        dist = math.hypot(obs.world_x - self.last_obs.world_x, obs.world_y - self.last_obs.world_y)

        if same_color and dist <= self.cfg.stable_distance_mm:
            self.stable_count += 1
        else:
            self.stable_count = 1

        self.last_obs = obs
        return self.stable_count >= self.cfg.stable_frames_required

    def _build_cycle(self, obs: TargetObservation, kin: DemoKinematics):
        self.locked_target = obs

        self.pick_above_pose = kin.pick_pose(obs.world_x, obs.world_y, stage="above")
        self.pick_down_pose = kin.pick_pose(obs.world_x, obs.world_y, stage="down")
        self.grab_pose = kin.with_claw(self.pick_down_pose, kin.cfg.claw_closed, name="grab_close")
        self.lift_pose = kin.pick_pose(obs.world_x, obs.world_y, stage="lift")
        self.place_above_pose = kin.place_pose(obs.color_name, stage="above")
        self.place_down_pose = kin.place_pose(obs.color_name, stage="down")
        self.release_pose = kin.with_claw(self.place_down_pose, kin.cfg.claw_open, name="release_open")
        self.return_pose = kin.home()

    def update(self, obs: Optional[TargetObservation], kin: DemoKinematics) -> Optional[ArmPose]:
        if not self.enabled:
            return None

        if self.state == PlannerState.SEARCH:
            self.status_text = f"SEARCH stable={self.stable_count}/{self.cfg.stable_frames_required}"

            if obs is None:
                self.last_obs = None
                self.stable_count = 0
                return None

            if not self._is_stable(obs):
                return None

            self._build_cycle(obs, kin)
            self._enter(PlannerState.PICK_ABOVE, f"PICK_ABOVE {obs.color_name}")
            return self.pick_above_pose

        if self.state == PlannerState.PICK_ABOVE and self._elapsed() >= self.cfg.dwell_pick_above_sec:
            self._enter(PlannerState.PICK_DOWN, "PICK_DOWN")
            return self.pick_down_pose

        if self.state == PlannerState.PICK_DOWN and self._elapsed() >= self.cfg.dwell_pick_down_sec:
            self._enter(PlannerState.GRAB, "GRAB")
            return self.grab_pose

        if self.state == PlannerState.GRAB and self._elapsed() >= self.cfg.dwell_grab_sec:
            self._enter(PlannerState.LIFT, "LIFT")
            return self.lift_pose

        if self.state == PlannerState.LIFT and self._elapsed() >= self.cfg.dwell_lift_sec:
            self._enter(PlannerState.PLACE_ABOVE, "PLACE_ABOVE")
            return self.place_above_pose

        if self.state == PlannerState.PLACE_ABOVE and self._elapsed() >= self.cfg.dwell_place_above_sec:
            self._enter(PlannerState.PLACE_DOWN, "PLACE_DOWN")
            return self.place_down_pose

        if self.state == PlannerState.PLACE_DOWN and self._elapsed() >= self.cfg.dwell_place_down_sec:
            self._enter(PlannerState.RELEASE, "RELEASE")
            return self.release_pose

        if self.state == PlannerState.RELEASE and self._elapsed() >= self.cfg.dwell_release_sec:
            self._enter(PlannerState.RETURN_HOME, "RETURN_HOME")
            return self.return_pose

        if self.state == PlannerState.RETURN_HOME and self._elapsed() >= self.cfg.dwell_return_sec:
            self.locked_target = None
            self.last_obs = None
            self.stable_count = 0
            self._enter(PlannerState.SEARCH, "SEARCH")
            return None

        return None
