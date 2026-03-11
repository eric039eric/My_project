# planner_commented.py
# 這個檔案負責流程控制，也就是狀態機：什麼時候找目標、什麼時候下降、什麼時候夾、什麼時候放。

from __future__ import annotations  # 允許先寫型別名稱再定義。

from dataclasses import dataclass  # 用 dataclass 定義設定資料結構。
from enum import Enum, auto  # Enum 用來建立狀態機狀態名稱。
from typing import Optional  # Optional 表示某些值可以是 None。
import math  # 用來算距離，判斷目標是否穩定。
import time  # 用來控制每個狀態停留多久。

from vision_commented import TargetObservation  # 從 vision 模組匯入觀測結果型別。
from kinematics_commented import DemoKinematics, ArmPose  # 匯入運動學與姿態型別。


class PlannerState(Enum):  # 定義整個流程中可能出現的狀態。
    IDLE = auto()  # 待命狀態，什麼都不做。
    SEARCH = auto()  # 搜尋目標。
    PICK_ABOVE = auto()  # 移到目標上方。
    PICK_DOWN = auto()  # 向下接近目標。
    GRAB = auto()  # 夾爪閉合抓取。
    LIFT = auto()  # 抬起目標。
    PLACE_ABOVE = auto()  # 移到放置點上方。
    PLACE_DOWN = auto()  # 下降到放置位置。
    RELEASE = auto()  # 放開夾爪。
    RETURN_HOME = auto()  # 回到 Home 位置。


@dataclass  # 用來集中管理狀態機參數。
class PlannerConfig:
    stable_frames_required: int = 8  # 同一目標要穩定出現幾幀才算鎖定。
    stable_distance_mm: float = 12.0  # 連續兩次觀測的允許位置差，超過代表不穩定。
    dwell_pick_above_sec: float = 0.9  # 在目標上方姿態停留時間。
    dwell_pick_down_sec: float = 0.9  # 在下壓姿態停留時間。
    dwell_grab_sec: float = 0.8  # 夾爪閉合後等待時間。
    dwell_lift_sec: float = 0.9  # 抬起後等待時間。
    dwell_place_above_sec: float = 0.9  # 在放置點上方等待時間。
    dwell_place_down_sec: float = 0.9  # 下降到放置點後等待時間。
    dwell_release_sec: float = 0.8  # 放開夾爪後等待時間。
    dwell_return_sec: float = 1.0  # 回 Home 後等待時間。


class SortPlanner:
    # 建構子：初始化所有狀態與中間暫存資料。
    def __init__(self, config: PlannerConfig):
        self.cfg = config  # 存下設定。
        self.state = PlannerState.IDLE  # 一開始先待命。
        self.enabled = False  # 預設自動流程關閉。

        self.last_obs: Optional[TargetObservation] = None  # 存上一筆觀測值，用來判斷穩定度。
        self.stable_count = 0  # 記錄目前連續穩定幀數。
        self.locked_target: Optional[TargetObservation] = None  # 一旦鎖定目標，就把目標資料存下來。

        self.pick_above_pose: Optional[ArmPose] = None  # 目標上方姿態。
        self.pick_down_pose: Optional[ArmPose] = None  # 目標抓取姿態。
        self.grab_pose: Optional[ArmPose] = None  # 夾住時姿態。
        self.lift_pose: Optional[ArmPose] = None  # 抬起姿態。
        self.place_above_pose: Optional[ArmPose] = None  # 放置點上方姿態。
        self.place_down_pose: Optional[ArmPose] = None  # 放置點下降姿態。
        self.release_pose: Optional[ArmPose] = None  # 放開時姿態。
        self.return_pose: Optional[ArmPose] = None  # 回 Home 姿態。

        self.state_enter_time = time.monotonic()  # 記錄進入目前狀態的時間。
        self.status_text = "IDLE"  # 給 HUD 顯示的文字。

    def start(self):  # 啟動自動分揀。
        self.enabled = True  # 打開總開關。
        self.state = PlannerState.SEARCH  # 切到搜尋狀態。
        self.last_obs = None  # 清除上一筆觀測。
        self.stable_count = 0  # 清除穩定計數。
        self.locked_target = None  # 清除鎖定目標。
        self.state_enter_time = time.monotonic()  # 重設狀態進入時間。
        self.status_text = "SEARCH"  # 更新狀態文字。

    def stop(self):  # 停止自動分揀。
        self.enabled = False  # 關閉總開關。
        self.state = PlannerState.IDLE  # 回到待命狀態。
        self.last_obs = None  # 清除上一筆觀測。
        self.stable_count = 0  # 清除穩定計數。
        self.locked_target = None  # 清除鎖定目標。
        self.status_text = "IDLE"  # 更新狀態文字。

    def _enter(self, state: PlannerState, text: str):  # 小工具：切換狀態時統一處理。
        self.state = state  # 更新狀態。
        self.state_enter_time = time.monotonic()  # 記錄新狀態開始時間。
        self.status_text = text  # 設定對應狀態文字。

    def _elapsed(self) -> float:  # 小工具：回傳目前狀態經過多久。
        return time.monotonic() - self.state_enter_time  # 目前時間減去進入時間。

    def _is_stable(self, obs: TargetObservation) -> bool:  # 判斷目前觀測到的目標是否已經穩定。
        if not obs.workspace_ready or obs.world_x is None or obs.world_y is None:  # 如果工作區還沒準備好，或沒有世界座標。
            self.last_obs = None  # 清除上一筆觀測。
            self.stable_count = 0  # 重置穩定次數。
            return False  # 不算穩定。

        if self.last_obs is None:  # 如果這是第一筆觀測。
            self.last_obs = obs  # 先存起來當基準。
            self.stable_count = 1  # 穩定次數從 1 開始。
            return False  # 第一筆還不能算穩定。

        same_color = obs.color_name == self.last_obs.color_name  # 檢查顏色是否和上一筆相同。
        dist = math.hypot(obs.world_x - self.last_obs.world_x, obs.world_y - self.last_obs.world_y)  # 算兩次世界座標距離。

        if same_color and dist <= self.cfg.stable_distance_mm:  # 如果顏色相同且位置變化不大。
            self.stable_count += 1  # 穩定次數加一。
        else:  # 否則代表目標變了或飄太多。
            self.stable_count = 1  # 重新開始計數。

        self.last_obs = obs  # 無論如何都更新上一筆觀測。
        return self.stable_count >= self.cfg.stable_frames_required  # 達到需求幀數才算鎖定成功。

    def _build_cycle(self, obs: TargetObservation, kin: DemoKinematics):  # 在正式進入流程前，把這次抓取/放置所需姿態全部先算好。
        self.locked_target = obs  # 存下鎖定目標。

        self.pick_above_pose = kin.pick_pose(obs.world_x, obs.world_y, stage="above")  # 算目標上方姿態。
        self.pick_down_pose = kin.pick_pose(obs.world_x, obs.world_y, stage="down")  # 算下降抓取姿態。
        self.grab_pose = kin.with_claw(self.pick_down_pose, kin.cfg.claw_closed, name="grab_close")  # 在抓取姿態上把夾爪改成閉合。
        self.lift_pose = kin.pick_pose(obs.world_x, obs.world_y, stage="lift")  # 算抬起姿態。
        self.place_above_pose = kin.place_pose(obs.color_name, stage="above")  # 根據顏色決定放置點上方姿態。
        self.place_down_pose = kin.place_pose(obs.color_name, stage="down")  # 根據顏色決定放置點下降姿態。
        self.release_pose = kin.with_claw(self.place_down_pose, kin.cfg.claw_open, name="release_open")  # 在放置姿態上把夾爪改成張開。
        self.return_pose = kin.home()  # 最後回 Home 的姿態。

    def update(self, obs: Optional[TargetObservation], kin: DemoKinematics) -> Optional[ArmPose]:  # 狀態機主更新函式，每次主迴圈都會呼叫它。
        if not self.enabled:  # 如果系統沒啟動。
            return None  # 就不送任何新姿態。

        if self.state == PlannerState.SEARCH:  # 目前在搜尋狀態。
            self.status_text = f"SEARCH stable={self.stable_count}/{self.cfg.stable_frames_required}"  # 更新 HUD 顯示目前穩定度。

            if obs is None:  # 如果這一幀沒看到目標。
                self.last_obs = None  # 清除上一筆。
                self.stable_count = 0  # 穩定次數歸零。
                return None  # 繼續等待下一幀。

            if not self._is_stable(obs):  # 如果還沒穩定到足夠幀數。
                return None  # 繼續搜尋。

            self._build_cycle(obs, kin)  # 一旦穩定，先把整個抓放流程的姿態全部建立好。
            self._enter(PlannerState.PICK_ABOVE, f"PICK_ABOVE {obs.color_name}")  # 切進第一個動作狀態。
            return self.pick_above_pose  # 回傳第一個要執行的姿態。

        if self.state == PlannerState.PICK_ABOVE and self._elapsed() >= self.cfg.dwell_pick_above_sec:  # 在上方等夠久後。
            self._enter(PlannerState.PICK_DOWN, "PICK_DOWN")  # 切到下降狀態。
            return self.pick_down_pose  # 回傳下降姿態。

        if self.state == PlannerState.PICK_DOWN and self._elapsed() >= self.cfg.dwell_pick_down_sec:  # 下降完成後。
            self._enter(PlannerState.GRAB, "GRAB")  # 切到夾取狀態。
            return self.grab_pose  # 回傳夾取姿態。

        if self.state == PlannerState.GRAB and self._elapsed() >= self.cfg.dwell_grab_sec:  # 夾爪關閉等夠久後。
            self._enter(PlannerState.LIFT, "LIFT")  # 切到抬起狀態。
            return self.lift_pose  # 回傳抬起姿態。

        if self.state == PlannerState.LIFT and self._elapsed() >= self.cfg.dwell_lift_sec:  # 抬起完成後。
            self._enter(PlannerState.PLACE_ABOVE, "PLACE_ABOVE")  # 切到放置點上方。
            return self.place_above_pose  # 回傳放置點上方姿態。

        if self.state == PlannerState.PLACE_ABOVE and self._elapsed() >= self.cfg.dwell_place_above_sec:  # 到放置點上方後。
            self._enter(PlannerState.PLACE_DOWN, "PLACE_DOWN")  # 切到放置下降狀態。
            return self.place_down_pose  # 回傳放置下降姿態。

        if self.state == PlannerState.PLACE_DOWN and self._elapsed() >= self.cfg.dwell_place_down_sec:  # 放置下降後。
            self._enter(PlannerState.RELEASE, "RELEASE")  # 切到放開狀態。
            return self.release_pose  # 回傳放開姿態。

        if self.state == PlannerState.RELEASE and self._elapsed() >= self.cfg.dwell_release_sec:  # 放開完成後。
            self._enter(PlannerState.RETURN_HOME, "RETURN_HOME")  # 切到回 Home 狀態。
            return self.return_pose  # 回傳回 Home 姿態。

        if self.state == PlannerState.RETURN_HOME and self._elapsed() >= self.cfg.dwell_return_sec:  # 回 Home 後等夠久。
            self.locked_target = None  # 清除本次鎖定目標。
            self.last_obs = None  # 清掉上一筆觀測。
            self.stable_count = 0  # 穩定次數歸零。
            self._enter(PlannerState.SEARCH, "SEARCH")  # 回到搜尋狀態，等待下一個物體。
            return None  # 這個時刻不一定要送新姿態。

        return None  # 如果目前狀態還沒到切換時間，就不送新姿態。
