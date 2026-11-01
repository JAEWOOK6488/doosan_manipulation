"""기준/제안 동작 비교와 놓기 호버 클로즈업 MP4를 생성한다."""
from __future__ import annotations

import subprocess
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

import collision_detector
import hover_servo
import hybrid_detector
import servo_controller
from scheduled_servo import _smoothstep
from sim.nominal import Nominal
from sim.runner import run_episode
from sim.trajectory import T_GRASP, T_RELEASE


HERE = Path(__file__).resolve().parent
OUT = HERE / "results" / "videos"
OUT.mkdir(parents=True, exist_ok=True)
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc"
FONT_BOLD = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"


def font(size: int, bold: bool = False):
    return ImageFont.truetype(FONT_BOLD if bold else FONT, size)


def phase(t: float) -> str:
    if t < T_GRASP:
        return "접근"
    if t < T_GRASP + .08:
        return "집기 · 하중 전달"
    if t < T_RELEASE - .20:
        return "운반"
    if t < T_RELEASE:
        return "놓기 접근 · 호버"
    if t < T_RELEASE + .10:
        return "그리퍼 해제 · 호버 유지"
    return "복귀"


def hover_mm(t: float) -> float:
    if t < T_RELEASE - .20:
        return 0.0
    if t < T_RELEASE - .05:
        return 1.25 * _smoothstep((t - (T_RELEASE - .20)) / .15)
    if t < T_RELEASE + .10:
        return 1.25
    if t < T_RELEASE + .25:
        return 1.25 * (1 - _smoothstep((t - (T_RELEASE + .10)) / .15))
    return 0.0


class Scene:
    def __init__(self, log, close=False):
        self.log = log
        self.m = log["plant"].m
        self.d = mujoco.MjData(self.m)
        self.renderer = mujoco.Renderer(self.m, 480, 640)
        self.camera = mujoco.MjvCamera()
        if close:
            self.camera.lookat[:] = [.487, .76, .17]
            self.camera.distance = .60
            self.camera.azimuth = 150
            self.camera.elevation = -15
        else:
            self.camera.lookat[:] = [.25, .10, .30]
            self.camera.distance = 2.8
            self.camera.azimuth = 150
            self.camera.elevation = -20

    def frame(self, k: int) -> Image.Image:
        self.d.qpos[:] = self.log["qpos"][k]
        mujoco.mj_forward(self.m, self.d)
        self.renderer.update_scene(self.d, camera=self.camera)
        return Image.fromarray(self.renderer.render().copy())

    def close(self):
        self.renderer.close()


def encoder(path: Path, size: tuple[int, int], fps=30):
    return subprocess.Popen(
        [
            "ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo",
            "-pix_fmt", "rgb24", "-s", f"{size[0]}x{size[1]}",
            "-r", str(fps), "-i", "-", "-an", "-c:v", "libx264",
            "-preset", "medium", "-crf", "18", "-pix_fmt", "yuv420p",
            "-movflags", "+faststart", str(path),
        ],
        stdin=subprocess.PIPE,
    )


def compose_header(left: Image.Image, right: Image.Image, t: float, subtitle: str):
    canvas = Image.new("RGB", (1280, 560), "#111713")
    canvas.paste(left, (0, 80)); canvas.paste(right, (640, 80))
    draw = ImageDraw.Draw(canvas)
    draw.rectangle((0, 0, 1279, 79), fill="#18231d")
    draw.text((24, 10), "BEFORE · 순간 하중 전환", font=font(26, True), fill="#f0ad96")
    draw.text((664, 10), "AFTER · 연속 하중 + 1.25 mm 호버", font=font(26, True), fill="#8fe0c3")
    draw.text((24, 48), f"t = {t:5.3f} s  ·  {subtitle}", font=font(18), fill="#e9efe9")
    draw.line((640, 0, 640, 560), fill="#d6ded8", width=2)
    return canvas


def make_overview(before, after):
    a, b = Scene(before), Scene(after)
    path = OUT / "before_after_motion.mp4"
    process = encoder(path, (1280, 560))
    try:
        for t in np.arange(1.15, 4.35, .020):
            k = min(int(round(t * 1000)), 5999)
            frame = compose_header(a.frame(k), b.frame(k), t, phase(t))
            draw = ImageDraw.Draw(frame)
            draw.text((24, 520), f"작업대 힘 {before['f_table'][k]:5.1f} N", font=font(18), fill="white")
            draw.text((664, 520), f"작업대 힘 {after['f_table'][k]:5.1f} N", font=font(18), fill="white")
            process.stdin.write(np.asarray(frame, dtype=np.uint8).tobytes())
    finally:
        process.stdin.close(); process.wait(); a.close(); b.close()
    if process.returncode:
        raise RuntimeError("ffmpeg overview encoding failed")
    return path


def graph(draw, box, times, values, current, color, label, bounds):
    x0, y0, x1, y1 = box
    lo, hi = bounds
    draw.rectangle(box, outline="#617268", width=1)
    points = []
    for t, value in zip(times, values):
        x = x0 + (t - times[0]) / (times[-1] - times[0]) * (x1 - x0)
        y = y1 - (value - lo) / max(hi - lo, 1e-9) * (y1 - y0)
        points.append((x, y))
    draw.line(points, fill=color, width=3)
    cursor = x0 + (current - times[0]) / (times[-1] - times[0]) * (x1 - x0)
    draw.line((cursor, y0, cursor, y1), fill="white", width=2)
    draw.text((x0 + 6, y0 + 4), label, font=font(15), fill=color)


def make_closeup(before, after):
    a, b = Scene(before, close=True), Scene(after, close=True)
    path = OUT / "hover_closeup.mp4"
    process = encoder(path, (1280, 700))
    timeline = np.linspace(3.70, 4.15, 181)
    h = np.array([hover_mm(t) for t in timeline])
    table_before = np.interp(timeline, before["t"], before["f_table"])
    table_after = np.interp(timeline, after["t"], after["f_table"])
    try:
        for t in np.arange(3.70, 4.151, .003):
            k = min(int(round(t * 1000)), 5999)
            base = compose_header(a.frame(k), b.frame(k), t, phase(t))
            canvas = Image.new("RGB", (1280, 700), "#111713")
            canvas.paste(base, (0, 0))
            draw = ImageDraw.Draw(canvas)
            draw.rectangle((0, 560, 1279, 699), fill="#111713")
            graph(draw, (45, 575, 615, 680), timeline, table_before, t,
                  "#f0ad96", f"BEFORE 작업대 힘 · 현재 {before['f_table'][k]:.1f} N", (0, 110))
            graph(draw, (665, 575, 1235, 630), timeline, h, t,
                  "#8fe0c3", f"AFTER 추가 목표 높이 · 현재 {hover_mm(t):.2f} mm", (0, 1.4))
            graph(draw, (665, 635, 1235, 690), timeline, table_after, t,
                  "#e5bb79", f"AFTER 작업대 힘 · 현재 {after['f_table'][k]:.1f} N", (0, 110))
            if T_RELEASE - .05 <= t <= T_RELEASE + .10:
                draw.text((900, 535), "호버 1.25 mm 유지", font=font(19, True), fill="#8fe0c3")
            process.stdin.write(np.asarray(canvas, dtype=np.uint8).tobytes())
    finally:
        process.stdin.close(); process.wait(); a.close(); b.close()
    if process.returncode:
        raise RuntimeError("ffmpeg close-up encoding failed")
    return path


def make_impact_comparison(before, after):
    """접촉 순간의 힘 피크 감소를 같은 축 위에서 직접 보여준다."""
    a, b = Scene(before, close=True), Scene(after, close=True)
    path = OUT / "impact_force_reduction.mp4"
    process = encoder(path, (1280, 760))
    timeline = np.linspace(3.80, 4.02, 221)
    table_before = np.interp(timeline, before["t"], before["f_table"])
    table_after = np.interp(timeline, after["t"], after["f_table"])
    weight = 8.0 * 9.81
    peak_before = float(table_before.max())
    peak_after = float(table_after.max())
    impulse_before = float(np.trapezoid(np.maximum(table_before - weight, 0), timeline))
    impulse_after = float(np.trapezoid(np.maximum(table_after - weight, 0), timeline))

    def xy(t, force):
        return (
            70 + (t - timeline[0]) / (timeline[-1] - timeline[0]) * 1140,
            730 - force / 110.0 * 135,
        )

    try:
        # 0.22 s의 접촉 과도를 약 5.5초 슬로모션으로 재생한다.
        for t in np.arange(3.80, 4.021, .00133):
            k = min(int(round(t * 1000)), 5999)
            base = compose_header(a.frame(k), b.frame(k), t, "접촉 순간 슬로모션")
            canvas = Image.new("RGB", (1280, 760), "#111713")
            canvas.paste(base, (0, 0))
            draw = ImageDraw.Draw(canvas)
            draw.rectangle((0, 560, 1279, 759), fill="#111713")
            draw.text((70, 568),
                      f"접촉 피크  {peak_before:.1f} → {peak_after:.1f} N  (−{peak_before-peak_after:.1f} N)",
                      font=font(21, True), fill="#e9efe9")
            draw.text((700, 568),
                      f"무게 초과 충격량  {impulse_before:.3f} → {impulse_after:.3f} N·s",
                      font=font(19, True), fill="#e9efe9")
            draw.rectangle((70, 595, 1210, 730), outline="#617268", width=1)

            # 8 kg 정적 하중 기준선과 현재 시점 커서.
            y_weight = xy(timeline[0], weight)[1]
            draw.line((70, y_weight, 1210, y_weight), fill="#aab7af", width=2)
            draw.text((78, y_weight - 23), "8 kg 정적 하중 78.5 N", font=font(15), fill="#cbd5ce")
            cursor_x = xy(t, 0)[0]
            draw.line((cursor_x, 595, cursor_x, 730), fill="white", width=2)

            pts_before = [xy(x, y) for x, y in zip(timeline, table_before)]
            pts_after = [xy(x, y) for x, y in zip(timeline, table_after)]
            draw.line(pts_before, fill="#f0ad96", width=4)
            draw.line(pts_after, fill="#8fe0c3", width=4)

            ib, ia = int(np.argmax(table_before)), int(np.argmax(table_after))
            for idx, value, color, label, dy in (
                (ib, peak_before, "#f0ad96", "BEFORE", -29),
                (ia, peak_after, "#8fe0c3", "AFTER", 8),
            ):
                px, py = xy(timeline[idx], value)
                draw.ellipse((px - 6, py - 6, px + 6, py + 6), fill=color)
                draw.text((px + 10, py + dy), f"{label} {value:.1f} N", font=font(16, True), fill=color)

            draw.text((930, 612),
                      f"현재  BEFORE {before['f_table'][k]:5.1f} N   AFTER {after['f_table'][k]:5.1f} N",
                      font=font(16), fill="white")
            process.stdin.write(np.asarray(canvas, dtype=np.uint8).tobytes())
    finally:
        process.stdin.close(); process.wait(); a.close(); b.close()
    if process.returncode:
        raise RuntimeError("ffmpeg impact comparison encoding failed")
    return path


def main():
    _, before = run_episode(
        servo_controller.Controller(), box_mass=8,
        detector=collision_detector.Detector(),
    )
    _, after = run_episode(
        hover_servo.Controller(), box_mass=8,
        detector=hybrid_detector.Detector(),
    )
    for path in (
        make_overview(before, after),
        make_closeup(before, after),
        make_impact_comparison(before, after),
    ):
        print(path)


if __name__ == "__main__":
    main()
