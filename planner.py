# planner.py
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional
import math, time

from vision import TargetObservation          # type: ignore
from kinematics import DemoKinematics, ArmPose  # type: ignore


class S(Enum):
    """Planner states (short alias keeps the transition table readable)."""
    IDLE        = auto()
    SEARCH      = auto()
    PICK_ABOVE  = auto()
    PICK_DOWN   = auto()
    GRAB        = auto()
    LIFT        = auto()
    PLACE_ABOVE = auto()
    PLACE_DOWN  = auto()
    RELEASE     = auto()
    RETURN_HOME = auto()


@dataclass
class PlannerConfig:
    stable_frames_required: int   = 8
    stable_distance_mm:     float = 12.0
    dwell_pick_above_sec:   float = 0.9
    dwell_pick_down_sec:    float = 0.9
    dwell_grab_sec:         float = 0.8
    dwell_lift_sec:         float = 0.9
    dwell_place_above_sec:  float = 0.9
    dwell_place_down_sec:   float = 0.9
    dwell_release_sec:      float = 0.8
    dwell_return_sec:       float = 1.0


# (current_state) -> (dwell_config_key, next_state, pose_key)
#   dwell_config_key : PlannerConfig 裡的屬性名稱 (字串)
#   next_state       : 轉換後的下一個狀態
#   pose_key         : self.poses dict 裡要送出的 key (字串)
TRANSITIONS = {
    S.PICK_ABOVE:  ("dwell_pick_above_sec",  S.PICK_DOWN,   "pick_down"),
    S.PICK_DOWN:   ("dwell_pick_down_sec",   S.GRAB,        "grab"),
    S.GRAB:        ("dwell_grab_sec",        S.LIFT,        "lift"),
    S.LIFT:        ("dwell_lift_sec",        S.PLACE_ABOVE, "place_above"),
    S.PLACE_ABOVE: ("dwell_place_above_sec", S.PLACE_DOWN,  "place_down"),
    S.PLACE_DOWN:  ("dwell_place_down_sec",  S.RELEASE,     "release"),
    S.RELEASE:     ("dwell_release_sec",     S.RETURN_HOME, "return_home"),
}


class SortPlanner:
    def __init__(self, cfg: PlannerConfig):
        self.cfg = cfg
        self.state = S.IDLE
        self.enabled = False
        self.status_text = "IDLE"

        self._last_obs: Optional[TargetObservation] = None
        self._stable_count = 0
        self.locked_target: Optional[TargetObservation] = None
        self.poses: dict[str, ArmPose] = {}
        self._enter_time = time.monotonic()

    # ---------- public controls ----------
    def start(self):
        self.enabled = True
        self._reset_search()
        self._enter(S.SEARCH, "SEARCH")

    def stop(self):
        self.enabled = False
        self._reset_search()
        self._enter(S.IDLE, "IDLE")

    # ---------- main tick ----------
    def update(self, obs: Optional[TargetObservation], kin: DemoKinematics) -> Optional[ArmPose]:
        if not self.enabled:
            return None

        # --- SEARCH 階段 ---
        if self.state == S.SEARCH:
            self.status_text = f"SEARCH stable={self._stable_count}/{self.cfg.stable_frames_required}"
            if obs is None:
                self._reset_search()
                return None
            if not self._is_stable(obs):
                return None
            self._build_cycle(obs, kin)
            self._enter(S.PICK_ABOVE, f"PICK_ABOVE {obs.color_name}")
            return self.poses["pick_above"]

        # --- RETURN_HOME 結束 → 回到 SEARCH ---
        if self.state == S.RETURN_HOME and self._elapsed() >= self.cfg.dwell_return_sec:
            self._reset_search()
            self._enter(S.SEARCH, "SEARCH")
            return None

        # --- 其餘所有狀態：查表轉換 ---
        tr = TRANSITIONS.get(self.state)
        if tr and self._elapsed() >= getattr(self.cfg, tr[0]):
            self._enter(tr[1], tr[1].name)
            return self.poses[tr[2]]

        return None

    # ---------- internals ----------
    def _enter(self, state: S, text: str):
        self.state = state
        self._enter_time = time.monotonic()
        self.status_text = text

    def _elapsed(self) -> float:
        return time.monotonic() - self._enter_time

    def _reset_search(self):
        self._last_obs = None
        self._stable_count = 0
        self.locked_target = None

    def _is_stable(self, obs: TargetObservation) -> bool:
        if not obs.workspace_ready or obs.world_x is None or obs.world_y is None:
            self._reset_search()
            return False
        if self._last_obs is None:
            self._last_obs = obs
            self._stable_count = 1
            return False
        same_color = obs.color_name == self._last_obs.color_name
        dist = math.hypot(obs.world_x - self._last_obs.world_x,
                          obs.world_y - self._last_obs.world_y)
        self._stable_count = self._stable_count + 1 if (same_color and dist <= self.cfg.stable_distance_mm) else 1
        self._last_obs = obs
        return self._stable_count >= self.cfg.stable_frames_required

    def _build_cycle(self, obs: TargetObservation, kin: DemoKinematics):
        self.locked_target = obs
        wx, wy = obs.world_x, obs.world_y
        pick_down = kin.pick_pose(wx, wy, "down")
        place_down = kin.place_pose(obs.color_name, "down")
        self.poses = {
            "pick_above":  kin.pick_pose(wx, wy, "above"),
            "pick_down":   pick_down,
            "grab":        kin.with_claw(pick_down, kin.cfg.claw_closed, "grab_close"),
            "lift":        kin.pick_pose(wx, wy, "lift"),
            "place_above": kin.place_pose(obs.color_name, "above"),
            "place_down":  place_down,
            "release":     kin.with_claw(place_down, kin.cfg.claw_open, "release_open"),
            "return_home": kin.home(),
        }
