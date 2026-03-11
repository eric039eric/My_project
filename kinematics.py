# kinematics_commented.py
# 這個檔案負責把「座標」翻譯成「四顆伺服馬達角度」。

from __future__ import annotations  # 允許前向參考型別名稱。

from dataclasses import dataclass, field  # dataclass 用於資料結構；field 可設定預設 dict。
from typing import Dict, Tuple  # 型別標註用。
import numpy as np  # 用於插值與範圍限制。


@dataclass  # ArmPose 用來表示一個完整的手臂姿態。
class ArmPose:
    base: int  # 底座角度。
    shoulder: int  # 肩關節角度。
    elbow: int  # 手肘角度。
    claw: int  # 夾爪角度。
    name: str = ""  # 這個姿態的名稱，方便 debug 顯示。

    def as_tuple(self):  # 把姿態轉成 tuple，方便顯示或傳送。
        return self.base, self.shoulder, self.elbow, self.claw  # 回傳四軸角度。


@dataclass  # 這個 dataclass 用來集中管理運動學相關設定。
class KinematicsConfig:
    work_width_mm: float = 300.0  # 工作區寬度。
    work_height_mm: float = 200.0  # 工作區高度。

    base_min: int = 20  # 底座最小安全角度。
    base_max: int = 160  # 底座最大安全角度。

    shoulder_min: int = 45  # 肩關節最小安全角度。
    shoulder_max: int = 135  # 肩關節最大安全角度。

    elbow_min: int = 30  # 手肘最小安全角度。
    elbow_max: int = 150  # 手肘最大安全角度。

    claw_min: int = 20  # 夾爪最小安全角度。
    claw_max: int = 120  # 夾爪最大安全角度。

    claw_open: int = 95  # 夾爪打開角度。
    claw_closed: int = 55  # 夾爪閉合角度。

    home_base: int = 90  # Home 姿態的底座角度。
    home_shoulder: int = 90  # Home 姿態的肩關節角度。
    home_elbow: int = 90  # Home 姿態的手肘角度。
    home_claw: int = 95  # Home 姿態的夾爪角度。

    place_points: Dict[str, Tuple[float, float]] = field(default_factory=lambda: {  # 各顏色的放置點世界座標。
        "RED": (260.0, 160.0),
        "GREEN": (260.0, 100.0),
        "BLUE": (260.0, 40.0),
    })


class DemoKinematics:
    # 建構子：只需要把設定存起來。
    def __init__(self, config: KinematicsConfig):
        self.cfg = config  # 存下設定。

    def clamp_pose(self, pose: ArmPose) -> ArmPose:  # 把姿態各軸限制在安全範圍內。
        return ArmPose(
            base=int(np.clip(pose.base, self.cfg.base_min, self.cfg.base_max)),  # 限制底座。
            shoulder=int(np.clip(pose.shoulder, self.cfg.shoulder_min, self.cfg.shoulder_max)),  # 限制肩關節。
            elbow=int(np.clip(pose.elbow, self.cfg.elbow_min, self.cfg.elbow_max)),  # 限制手肘。
            claw=int(np.clip(pose.claw, self.cfg.claw_min, self.cfg.claw_max)),  # 限制夾爪。
            name=pose.name  # 名稱保留。
        )

    def home(self) -> ArmPose:  # 回傳 Home 姿態。
        return self.clamp_pose(ArmPose(
            self.cfg.home_base,  # 使用設定中的 Home 底座角度。
            self.cfg.home_shoulder,  # 使用設定中的 Home 肩關節角度。
            self.cfg.home_elbow,  # 使用設定中的 Home 手肘角度。
            self.cfg.home_claw,  # 使用設定中的 Home 夾爪角度。
            name="home"  # 這個姿態名稱叫 home。
        ))

    def with_claw(self, pose: ArmPose, claw_value: int, name: str = "") -> ArmPose:  # 在原本姿態基礎上，只改夾爪角度。
        return self.clamp_pose(ArmPose(
            base=pose.base,  # 底座沿用原姿態。
            shoulder=pose.shoulder,  # 肩關節沿用原姿態。
            elbow=pose.elbow,  # 手肘沿用原姿態。
            claw=claw_value,  # 夾爪改成指定值。
            name=name or pose.name  # 如果有傳新名稱就用新名稱，否則沿用原名稱。
        ))

    def _map_world_to_arm(self, world_x: float, world_y: float):  # 核心映射：把工作區座標映射成 base/shoulder/elbow。
        x = float(np.clip(world_x, 0.0, self.cfg.work_width_mm))  # 先把 x 限制在工作區範圍內。
        y = float(np.clip(world_y, 0.0, self.cfg.work_height_mm))  # 先把 y 限制在工作區範圍內。

        base = int(np.interp(x, [0.0, self.cfg.work_width_mm], [155, 25]))  # x 越往右，底座角度越小。
        shoulder = int(np.interp(y, [0.0, self.cfg.work_height_mm], [74, 118]))  # y 越大，肩關節角度越大。
        elbow = int(np.interp(y, [0.0, self.cfg.work_height_mm], [122, 72]))  # y 越大，手肘角度越小。

        return base, shoulder, elbow  # 回傳三軸角度。

    def pick_pose(self, world_x: float, world_y: float, stage: str = "above") -> ArmPose:  # 根據抓取流程不同階段，產生對應姿態。
        base, shoulder, elbow = self._map_world_to_arm(world_x, world_y)  # 先取得基礎映射角度。

        if stage == "above":  # 如果是目標上方姿態。
            shoulder -= 6  # 稍微抬高肩部。
            elbow += 6  # 手肘稍微收回，讓夾爪位於上方。
            claw = self.cfg.claw_open  # 夾爪打開。
        elif stage == "down":  # 如果是向下抓取姿態。
            shoulder += 10  # 肩關節往下壓。
            elbow -= 12  # 手肘往前伸一些。
            claw = self.cfg.claw_open  # 夾爪仍保持打開，準備夾取。
        elif stage == "lift":  # 如果是抬起姿態。
            shoulder -= 4  # 肩部抬高一些。
            elbow += 8  # 手肘收回一些。
            claw = self.cfg.claw_closed  # 已經夾住，所以夾爪閉合。
        else:  # 若 stage 不是上述三種。
            claw = self.cfg.claw_open  # 預設夾爪打開。

        return self.clamp_pose(ArmPose(base, shoulder, elbow, claw, name=f"pick_{stage}"))  # 回傳限制後的姿態。

    def place_pose(self, color_name: str, stage: str = "above") -> ArmPose:  # 根據顏色與流程階段，產生放置姿態。
        color_name = color_name.upper()  # 保險起見轉成大寫，避免 key 不一致。
        if color_name not in self.cfg.place_points:  # 如果設定裡沒有這個顏色。
            raise KeyError(f"place_points 裡沒有顏色 {color_name}")  # 直接報錯。

        world_x, world_y = self.cfg.place_points[color_name]  # 取出這個顏色的放置點座標。
        base, shoulder, elbow = self._map_world_to_arm(world_x, world_y)  # 把放置點座標轉成基礎角度。

        if stage == "above":  # 放置點上方姿態。
            shoulder -= 8  # 肩部抬高，先停在上方。
            elbow += 8  # 手肘收一些。
            claw = self.cfg.claw_closed  # 這時仍然夾住物體。
        elif stage == "down":  # 真正放置的下降姿態。
            shoulder += 8  # 肩部往下。
            elbow -= 8  # 手肘伸出去。
            claw = self.cfg.claw_closed  # 先保持夾住，等 release 才打開。
        else:  # 其他未知 stage。
            claw = self.cfg.claw_closed  # 預設保持夾住。

        return self.clamp_pose(ArmPose(base, shoulder, elbow, claw, name=f"place_{color_name}_{stage}"))  # 回傳限制後的姿態。
