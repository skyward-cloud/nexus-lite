#!/usr/bin/env python3
"""
根据绘制生成 gz sim 地图

打开一个画布，使用可调整大小的画笔绘制直线。
画布与真实地图坐标系、尺寸一致（X 向右，Y 向上，单位米）。
画笔大小表示墙壁厚度；全局统一高度可调整（单位米）。
绘制后可保存、可加载；可根据绘制的墙壁生成 gz sim SDF 地图。

用法:
  python3 simulation/scripts/make_world.py
"""

from __future__ import annotations

import json
import math
import tkinter as tk
from dataclasses import asdict, dataclass
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import List, Optional, Tuple

SCRIPT_DIR = Path(__file__).resolve().parent
WORLDS_DIR = SCRIPT_DIR.parent / "worlds"
TEMPLATE_PATH = WORLDS_DIR / "_world_template.sdf"
DEFAULT_DRAWING_DIR = WORLDS_DIR / "drawings"
DEFAULT_EXPORT_PATH = WORLDS_DIR / "custom.sdf"

MIN_WALL_LENGTH = 0.05

# 与 empty.sdf 一致的碰撞表面参数
COLLISION_SURFACE_XML = """
          <surface>
            <friction>
              <ode/>
            </friction>
            <bounce/>
            <contact/>
          </surface>"""


@dataclass
class WallSegment:
    x1: float
    y1: float
    x2: float
    y2: float
    thickness: float

    def length(self) -> float:
        return math.hypot(self.x2 - self.x1, self.y2 - self.y1)

    def sdf_pose_and_size(self, wall_height: float) -> Optional[Tuple[float, float, float, float, float, float]]:
        length = self.length()
        if length < MIN_WALL_LENGTH:
            return None
        cx = (self.x1 + self.x2) / 2.0
        cy = (self.y1 + self.y2) / 2.0
        cz = wall_height / 2.0
        yaw = math.atan2(self.y2 - self.y1, self.x2 - self.x1)
        return cx, cy, cz, length, self.thickness, wall_height, yaw


@dataclass
class WorldDrawing:
    world_width: float = 20.0
    world_height: float = 20.0
    wall_height: float = 5.0
    brush_thickness: float = 0.2
    walls: List[WallSegment] = None

    def __post_init__(self) -> None:
        if self.walls is None:
            self.walls = []

    def to_dict(self) -> dict:
        return {
            "world_width": self.world_width,
            "world_height": self.world_height,
            "wall_height": self.wall_height,
            "brush_thickness": self.brush_thickness,
            "walls": [asdict(w) for w in self.walls],
        }

    @classmethod
    def from_dict(cls, data: dict) -> "WorldDrawing":
        walls = [WallSegment(**w) for w in data.get("walls", [])]
        return cls(
            world_width=data.get("world_width", 20.0),
            world_height=data.get("world_height", 20.0),
            wall_height=data.get("wall_height", 5.0),
            brush_thickness=data.get("brush_thickness", 0.2),
            walls=walls,
        )


def format_pose(cx: float, cy: float, cz: float, yaw: float) -> str:
    return f"{cx:.4f} {cy:.4f} {cz:.4f} 0 0 {yaw:.6f}"


def wall_sdf_block(index: int, wall: WallSegment, wall_height: float) -> str:
    params = wall.sdf_pose_and_size(wall_height)
    if params is None:
        return ""
    cx, cy, cz, length, thickness, height, yaw = params
    pose = format_pose(cx, cy, cz, yaw)
    size = f"{length:.4f} {thickness:.4f} {height:.4f}"
    name = f"wall_{index:03d}"
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="wall_link">
        <visual name="wall_visual">
          <geometry>
            <box>
              <size>{size}</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.5 0.4 1</ambient>
            <diffuse>0.7 0.6 0.5 1</diffuse>
          </material>
        </visual>
        <collision name="wall_collision">
          <geometry>
            <box>
              <size>{size}</size>
            </box>
          </geometry>{COLLISION_SURFACE_XML}
        </collision>
      </link>
    </model>"""


def ground_sdf_block(world_width: float, world_height: float) -> str:
    """生成与 empty.sdf 一致的 ground_plane（无限平面碰撞 + 可视地面）。"""
    visual_extent = max(world_width, world_height, 20.0) * 2.0
    return f"""
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>1 1</size>
            </plane>
          </geometry>{COLLISION_SURFACE_XML}
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{visual_extent:.4f} {visual_extent:.4f}</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
        <pose>0 0 0 0 -0 0</pose>
        <inertial>
          <pose>0 0 0 0 -0 0</pose>
          <mass>1</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
        <enable_wind>false</enable_wind>
      </link>
      <pose>0 0 0 0 -0 0</pose>
      <self_collide>false</self_collide>
    </model>"""


def generate_sdf(drawing: WorldDrawing, template_path: Path = TEMPLATE_PATH) -> str:
    template = template_path.read_text(encoding="utf-8")
    ground = ground_sdf_block(drawing.world_width, drawing.world_height)
    walls_xml = ""
    for i, wall in enumerate(drawing.walls):
        walls_xml += wall_sdf_block(i, wall, drawing.wall_height)
    if "<!-- INSERT_GROUND -->" not in template or "<!-- INSERT_WALLS -->" not in template:
        raise ValueError(f"模板缺少占位符: {template_path}")
    result = template.replace("<!-- INSERT_GROUND -->", ground)
    result = result.replace("<!-- INSERT_WALLS -->", walls_xml)
    return result


def save_drawing(drawing: WorldDrawing, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(drawing.to_dict(), indent=2), encoding="utf-8")


def load_drawing(path: Path) -> WorldDrawing:
    data = json.loads(path.read_text(encoding="utf-8"))
    return WorldDrawing.from_dict(data)


class WorldEditorApp:
    CANVAS_MARGIN = 40

    def __init__(self) -> None:
        self.drawing = WorldDrawing()
        self.pending_point: Optional[Tuple[float, float]] = None
        self.preview_line_id: Optional[int] = None
        self.scale_px_per_m = 20.0

        self.root = tk.Tk()
        self.root.title("GZ Sim 地图绘制器")
        self.root.minsize(900, 650)

        self._build_ui()
        self._redraw_canvas()

    def _build_ui(self) -> None:
        control = ttk.Frame(self.root, padding=8)
        control.pack(side=tk.TOP, fill=tk.X)

        self._add_spinbox(control, "地图宽 (m)", "world_width", 5, 100, self.drawing.world_width, self._on_world_size_change)
        self._add_spinbox(control, "地图高 (m)", "world_height", 5, 100, self.drawing.world_height, self._on_world_size_change)
        self._add_spinbox(control, "墙高 (m)", "wall_height", 0.5, 20, self.drawing.wall_height, self._on_param_change)
        self._add_spinbox(control, "画笔厚 (m)", "brush_thickness", 0.05, 2.0, self.drawing.brush_thickness, self._on_brush_change, step=0.05)

        ttk.Button(control, text="撤销", command=self._undo_wall).pack(side=tk.LEFT, padx=4)
        ttk.Button(control, text="清空", command=self._clear_walls).pack(side=tk.LEFT, padx=4)
        ttk.Button(control, text="保存", command=self._save_drawing).pack(side=tk.LEFT, padx=4)
        ttk.Button(control, text="加载", command=self._load_drawing).pack(side=tk.LEFT, padx=4)
        ttk.Button(control, text="导出 SDF", command=self._export_sdf).pack(side=tk.LEFT, padx=4)

        hint = ttk.Label(
            self.root,
            text="在画布上点击两点绘制墙壁直线；坐标与仿真世界一致（米）。ESC 取消当前线段。",
            padding=(8, 0),
        )
        hint.pack(side=tk.TOP, anchor=tk.W)

        canvas_frame = ttk.Frame(self.root, padding=8)
        canvas_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(canvas_frame, bg="#e8e8e8", highlightthickness=1, highlightbackground="#999")
        self.canvas.pack(fill=tk.BOTH, expand=True)

        self.canvas.bind("<Button-1>", self._on_canvas_click)
        self.canvas.bind("<Motion>", self._on_canvas_motion)
        self.root.bind("<Escape>", self._cancel_pending)
        self.canvas.bind("<Configure>", lambda _e: self._redraw_canvas())

        self.status = ttk.Label(self.root, text="", padding=(8, 4))
        self.status.pack(side=tk.BOTTOM, anchor=tk.W)
        self._update_status()

    def _add_spinbox(
        self,
        parent: ttk.Frame,
        label: str,
        attr: str,
        min_val: float,
        max_val: float,
        initial: float,
        callback,
        step: float = 0.5,
    ) -> None:
        frame = ttk.Frame(parent)
        frame.pack(side=tk.LEFT, padx=6)
        ttk.Label(frame, text=label).pack(side=tk.TOP)
        spin = ttk.Spinbox(
            frame,
            from_=min_val,
            to=max_val,
            increment=step,
            width=8,
            command=callback,
        )
        spin.set(f"{initial:g}")
        spin.pack(side=tk.TOP)
        spin.bind("<Return>", lambda _e: callback())
        spin.bind("<FocusOut>", lambda _e: callback())
        setattr(self, f"spin_{attr}", spin)

    def _read_spin(self, attr: str, default: float) -> float:
        spin = getattr(self, f"spin_{attr}")
        try:
            return float(spin.get())
        except ValueError:
            return default

    def _sync_drawing_from_spinboxes(self) -> None:
        self.drawing.world_width = self._read_spin("world_width", self.drawing.world_width)
        self.drawing.world_height = self._read_spin("world_height", self.drawing.world_height)
        self.drawing.wall_height = self._read_spin("wall_height", self.drawing.wall_height)
        self.drawing.brush_thickness = self._read_spin("brush_thickness", self.drawing.brush_thickness)

    def _on_world_size_change(self) -> None:
        self._sync_drawing_from_spinboxes()
        self._redraw_canvas()

    def _on_param_change(self) -> None:
        self._sync_drawing_from_spinboxes()
        self._update_status()

    def _on_brush_change(self) -> None:
        self._sync_drawing_from_spinboxes()
        self._update_status()

    def _canvas_size(self) -> Tuple[int, int]:
        w = max(self.canvas.winfo_width(), 400)
        h = max(self.canvas.winfo_height(), 400)
        return w, h

    def _fit_scale(self) -> float:
        cw, ch = self._canvas_size()
        usable_w = cw - 2 * self.CANVAS_MARGIN
        usable_h = ch - 2 * self.CANVAS_MARGIN
        sx = usable_w / self.drawing.world_width
        sy = usable_h / self.drawing.world_height
        return min(sx, sy)

    def world_to_canvas(self, x: float, y: float) -> Tuple[float, float]:
        scale = self._fit_scale()
        cw, ch = self._canvas_size()
        cx = cw / 2.0 + x * scale
        cy = ch / 2.0 - y * scale
        return cx, cy

    def canvas_to_world(self, cx: float, cy: float) -> Tuple[float, float]:
        scale = self._fit_scale()
        cw, ch = self._canvas_size()
        x = (cx - cw / 2.0) / scale
        y = (ch / 2.0 - cy) / scale
        return x, y

    def _clamp_world(self, x: float, y: float) -> Tuple[float, float]:
        half_w = self.drawing.world_width / 2.0
        half_h = self.drawing.world_height / 2.0
        return (
            max(-half_w, min(half_w, x)),
            max(-half_h, min(half_h, y)),
        )

    def _draw_grid(self) -> None:
        scale = self._fit_scale()
        half_w = self.drawing.world_width / 2.0
        half_h = self.drawing.world_height / 2.0

        x0, y_top = self.world_to_canvas(-half_w, half_h)
        x1, y_bottom = self.world_to_canvas(half_w, -half_h)
        self.canvas.create_rectangle(x0, y_top, x1, y_bottom, outline="#666", width=2)

        step = 1.0
        if self.drawing.world_width > 30:
            step = 2.0
        x = -half_w
        while x <= half_w:
            px0, py0 = self.world_to_canvas(x, -half_h)
            px1, py1 = self.world_to_canvas(x, half_h)
            color = "#bbb" if abs(x) > 0.01 else "#888"
            self.canvas.create_line(px0, py0, px1, py1, fill=color, dash=(2, 4))
            if abs(x) > 0.01 or step >= 1:
                self.canvas.create_text(px0 + 4, py1 + 12, text=f"{x:g}", fill="#555", anchor=tk.W)
            x += step

        y = -half_h
        while y <= half_h:
            px0, py0 = self.world_to_canvas(-half_w, y)
            px1, py1 = self.world_to_canvas(half_w, y)
            color = "#bbb" if abs(y) > 0.01 else "#888"
            self.canvas.create_line(px0, py0, px1, py1, fill=color, dash=(2, 4))
            if abs(y) > 0.01 or step >= 1:
                self.canvas.create_text(px0 - 4, py0, text=f"{y:g}", fill="#555", anchor=tk.E)
            y += step

        ox, oy = self.world_to_canvas(0, 0)
        self.canvas.create_line(ox - 8, oy, ox + 8, oy, fill="#c44", width=2)
        self.canvas.create_line(ox, oy - 8, ox, oy + 8, fill="#44c", width=2)
        self.canvas.create_text(ox + 12, oy - 4, text="X", fill="#c44")
        self.canvas.create_text(ox + 4, oy - 12, text="Y", fill="#44c")

        self.canvas.create_text(
            x1 - 4, y_top + 4,
            text=f"1 m = {scale:.1f} px",
            fill="#555",
            anchor=tk.NE,
        )

    def _draw_wall_on_canvas(self, wall: WallSegment, color: str = "#3a5a8a", tag: str = "wall") -> None:
        x1, y1 = self.world_to_canvas(wall.x1, wall.y1)
        x2, y2 = self.world_to_canvas(wall.x2, wall.y2)
        scale = self._fit_scale()
        line_w = max(2, wall.thickness * scale)
        self.canvas.create_line(x1, y1, x2, y2, fill=color, width=line_w, capstyle=tk.ROUND, tags=tag)

    def _redraw_canvas(self) -> None:
        self.canvas.delete("all")
        self._draw_grid()
        for wall in self.drawing.walls:
            self._draw_wall_on_canvas(wall)
        if self.pending_point is not None:
            px, py = self.world_to_canvas(*self.pending_point)
            self.canvas.create_oval(px - 4, py - 4, px + 4, py + 4, fill="#e67e22", outline="#c0392b")
        if self.preview_line_id is not None:
            self.preview_line_id = None

    def _update_status(self) -> None:
        pending = "已选起点，点击终点" if self.pending_point else "点击起点"
        self.status.config(
            text=(
                f"{pending} | 墙数: {len(self.drawing.walls)} | "
                f"地图 {self.drawing.world_width:g}×{self.drawing.world_height:g} m | "
                f"墙高 {self.drawing.wall_height:g} m | 画笔 {self.drawing.brush_thickness:g} m"
            )
        )

    def _on_canvas_click(self, event: tk.Event) -> None:
        self._sync_drawing_from_spinboxes()
        wx, wy = self._clamp_world(*self.canvas_to_world(event.x, event.y))
        if self.pending_point is None:
            self.pending_point = (wx, wy)
            self._redraw_canvas()
        else:
            wall = WallSegment(
                x1=self.pending_point[0],
                y1=self.pending_point[1],
                x2=wx,
                y2=wy,
                thickness=self.drawing.brush_thickness,
            )
            if wall.length() >= MIN_WALL_LENGTH:
                self.drawing.walls.append(wall)
            self.pending_point = None
            self._clear_preview()
            self._redraw_canvas()
        self._update_status()

    def _on_canvas_motion(self, event: tk.Event) -> None:
        if self.pending_point is None:
            return
        self._clear_preview()
        wx, wy = self._clamp_world(*self.canvas_to_world(event.x, event.y))
        preview = WallSegment(
            x1=self.pending_point[0],
            y1=self.pending_point[1],
            x2=wx,
            y2=wy,
            thickness=self.drawing.brush_thickness,
        )
        x1, y1 = self.world_to_canvas(preview.x1, preview.y1)
        x2, y2 = self.world_to_canvas(preview.x2, preview.y2)
        scale = self._fit_scale()
        line_w = max(2, preview.thickness * scale)
        self.preview_line_id = self.canvas.create_line(
            x1, y1, x2, y2,
            fill="#f39c12",
            width=line_w,
            capstyle=tk.ROUND,
            dash=(6, 4),
            tags="preview",
        )

    def _clear_preview(self) -> None:
        self.canvas.delete("preview")
        self.preview_line_id = None

    def _cancel_pending(self, _event=None) -> None:
        self.pending_point = None
        self._clear_preview()
        self._redraw_canvas()
        self._update_status()

    def _undo_wall(self) -> None:
        if self.drawing.walls:
            self.drawing.walls.pop()
            self._redraw_canvas()
            self._update_status()

    def _clear_walls(self) -> None:
        if not self.drawing.walls:
            return
        if messagebox.askyesno("确认", "清空所有墙壁？"):
            self.drawing.walls.clear()
            self.pending_point = None
            self._redraw_canvas()
            self._update_status()

    def _save_drawing(self) -> None:
        self._sync_drawing_from_spinboxes()
        DEFAULT_DRAWING_DIR.mkdir(parents=True, exist_ok=True)
        path = filedialog.asksaveasfilename(
            title="保存绘制",
            initialdir=str(DEFAULT_DRAWING_DIR),
            defaultextension=".json",
            filetypes=[("JSON", "*.json")],
        )
        if not path:
            return
        save_drawing(self.drawing, Path(path))
        messagebox.showinfo("保存成功", f"已保存到 {path}")

    def _load_drawing(self) -> None:
        path = filedialog.askopenfilename(
            title="加载绘制",
            initialdir=str(DEFAULT_DRAWING_DIR),
            filetypes=[("JSON", "*.json")],
        )
        if not path:
            return
        try:
            self.drawing = load_drawing(Path(path))
        except (json.JSONDecodeError, TypeError, KeyError) as exc:
            messagebox.showerror("加载失败", str(exc))
            return
        self.spin_world_width.set(f"{self.drawing.world_width:g}")
        self.spin_world_height.set(f"{self.drawing.world_height:g}")
        self.spin_wall_height.set(f"{self.drawing.wall_height:g}")
        self.spin_brush_thickness.set(f"{self.drawing.brush_thickness:g}")
        self.pending_point = None
        self._redraw_canvas()
        self._update_status()
        messagebox.showinfo("加载成功", f"已从 {path} 加载 {len(self.drawing.walls)} 面墙")

    def _export_sdf(self) -> None:
        self._sync_drawing_from_spinboxes()
        if not self.drawing.walls:
            messagebox.showwarning("导出", "尚未绘制任何墙壁。")
            return
        if not TEMPLATE_PATH.is_file():
            messagebox.showerror("导出失败", f"找不到模板: {TEMPLATE_PATH}")
            return
        path = filedialog.asksaveasfilename(
            title="导出 SDF",
            initialdir=str(WORLDS_DIR),
            initialfile="custom.sdf",
            defaultextension=".sdf",
            filetypes=[("SDF", "*.sdf")],
        )
        if not path:
            return
        try:
            sdf = generate_sdf(self.drawing)
            Path(path).write_text(sdf, encoding="utf-8")
        except OSError as exc:
            messagebox.showerror("导出失败", str(exc))
            return
        messagebox.showinfo("导出成功", f"已生成 {path}\n墙壁数量: {len(self.drawing.walls)}")

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    app = WorldEditorApp()
    app.run()


if __name__ == "__main__":
    main()
