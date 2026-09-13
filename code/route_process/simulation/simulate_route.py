"""Generate a visual closed-loop route-following simulation and MP4 video."""

import argparse
import csv
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from skimage.color import rgb2hsv
from skimage.measure import label
from skimage.morphology import (
    binary_closing,
    binary_opening,
    disk,
    remove_small_holes,
    remove_small_objects,
)
from skimage.transform import ProjectiveTransform, warp

from route_algorithm import (
    IMAGE_H,
    IMAGE_W,
    NORMAL_SPEED,
    RAMP_SPEED,
    RouteController,
    RouteImageAlgorithm,
)


MAP_W = 1200
MAP_H = 900
CANVAS_W = 1280
CANVAS_H = 720
MAP_PANEL_X = 20
MAP_PANEL_Y = 58
MAP_PANEL_W = 800
MAP_PANEL_H = 600
CAM_PANEL_X = 850
CAM_PANEL_Y = 52
CAM_SCALE = 2
FPS_DEFAULT = 20
FRAMES_DEFAULT = 546

# The photographed course leaves/re-enters the image at its borders and has an
# ambiguous intersection near the center. Treat the visible sections as
# separate closed-loop trials instead of drawing fake connections through the
# blue floor. Every section still uses RouteImageAlgorithm and RouteController.
TOUR_SEGMENTS = (
    {
        "name": "Lower loop + right S-bends",
        "x": 55.0,
        "y": 455.0,
        "heading": 0.0,
        "reference_frames": 284,
    },
    {
        "name": "Top-right hairpin entry",
        "x": 1000.0,
        "y": 40.0,
        "heading": 0.2,
        "reference_frames": 15,
    },
    {
        "name": "Top-right hairpin exit",
        "x": 1100.0,
        "y": 75.0,
        "heading": 2.4,
        "reference_frames": 16,
    },
    {
        "name": "Center ramp + U-turn",
        "x": 680.0,
        "y": 30.0,
        "heading": 1.5,
        "reference_frames": 124,
    },
    {
        "name": "Center diagonal",
        "x": 235.0,
        "y": 285.0,
        "heading": -1.3,
        "reference_frames": 56,
    },
    {
        "name": "Upper-left lane",
        "x": 190.0,
        "y": 30.0,
        "heading": 2.15,
        "reference_frames": 51,
    },
)


def load_font(size, bold=False):
    names = [
        r"C:\Windows\Fonts\consolab.ttf" if bold
        else r"C:\Windows\Fonts\consola.ttf",
        r"C:\Windows\Fonts\arialbd.ttf" if bold
        else r"C:\Windows\Fonts\arial.ttf",
    ]
    for name in names:
        if Path(name).exists():
            return ImageFont.truetype(name, size=size)
    return ImageFont.load_default()


FONT_14 = load_font(14)
FONT_16 = load_font(16)
FONT_18 = load_font(18, bold=True)
FONT_24 = load_font(24, bold=True)


def build_topdown(source_path):
    image = Image.open(source_path).convert("RGB")
    rgb = np.asarray(image, dtype=np.float32) / 255.0
    scale_x = image.width / 1706.0
    scale_y = image.height / 1279.0
    source_points = np.array(
        [[180, 520], [1450, 555], [1705, 1278], [0, 1278]],
        dtype=np.float64,
    )
    source_points[:, 0] *= scale_x
    source_points[:, 1] *= scale_y
    destination_points = np.array(
        [[0, 0], [MAP_W - 1, 0], [MAP_W - 1, MAP_H - 1],
         [0, MAP_H - 1]],
        dtype=np.float64,
    )
    transform = ProjectiveTransform()
    if not transform.estimate(source_points, destination_points):
        raise RuntimeError("could not estimate floor perspective transform")
    topdown = warp(
        rgb,
        inverse_map=transform.inverse,
        output_shape=(MAP_H, MAP_W),
        preserve_range=True,
    )
    return np.clip(topdown * 255.0, 0, 255).astype(np.uint8)


def extract_track_mask(topdown):
    hsv = rgb2hsv(topdown.astype(np.float32) / 255.0)
    mask = (hsv[:, :, 2] > 0.48) & (hsv[:, :, 1] < 0.42)

    mask = binary_closing(mask, disk(7))
    mask = binary_opening(mask, disk(2))
    mask = remove_small_objects(mask, min_size=1200)
    components = label(mask)
    areas = np.bincount(components.ravel())
    areas[0] = 0

    # Keep the three major connected regions that make up the photographed
    # course. Smaller white regions are the rolled mat, signs and floor marks.
    track_labels = np.flatnonzero(areas >= 10000)
    if not len(track_labels):
        raise RuntimeError("could not extract the white course from the image")
    mask = np.isin(components, track_labels)
    mask = remove_small_holes(mask, area_threshold=3500)
    return mask


def allocate_tour_segments(frame_count):
    if frame_count < len(TOUR_SEGMENTS):
        raise ValueError(
            f"whole-course simulation needs at least {len(TOUR_SEGMENTS)} frames"
        )

    reference = np.array(
        [segment["reference_frames"] for segment in TOUR_SEGMENTS],
        dtype=np.float64,
    )
    exact = reference * frame_count / float(np.sum(reference))
    counts = np.floor(exact).astype(np.int32)
    counts[counts < 1] = 1
    remaining = frame_count - int(np.sum(counts))
    fractions = exact - np.floor(exact)

    if remaining > 0:
        for index in np.argsort(-fractions)[:remaining]:
            counts[index] += 1
    elif remaining < 0:
        for index in np.argsort(fractions):
            if remaining == 0:
                break
            if counts[index] > 1:
                counts[index] -= 1
                remaining += 1

    result = []
    for segment, count in zip(TOUR_SEGMENTS, counts):
        item = dict(segment)
        item["frames"] = int(count)
        result.append(item)
    return result


def save_map_artifacts(topdown, track_mask, output_dir):
    Image.fromarray(topdown).save(output_dir / "topdown_track.jpg", quality=94)
    Image.fromarray((track_mask * 255).astype(np.uint8)).save(
        output_dir / "track_mask.png"
    )
    overlay = topdown.copy()
    overlay[track_mask] = (
        0.48 * overlay[track_mask]
        + 0.52 * np.array([70, 220, 100], dtype=np.float32)
    ).astype(np.uint8)
    Image.fromarray(overlay).save(
        output_dir / "track_mask_overlay.jpg", quality=94
    )


def ramp_projection_phase(x, y):
    if not (350 <= y <= 590):
        return "flat", 25.0, 0.65
    if 155 <= x < 195:
        return "ramp-entry", 70.0, 0.35
    if 195 <= x < 220:
        return "ramp-climb", 10.0, 0.60
    if 220 <= x < 245:
        return "ramp-crest", 70.0, 0.35
    if 245 <= x < 290:
        return "ramp-exit", 5.0, 0.50
    return "flat", 25.0, 0.65


def render_camera(track_mask, topdown_gray, x, y, heading, frame_index):
    phase, fov_base, fov_growth = ramp_projection_phase(x, y)
    rows = np.arange(IMAGE_H, dtype=np.float32)[:, None]
    cols = np.arange(IMAGE_W, dtype=np.float32)[None, :]
    vertical = (IMAGE_H - 1.0 - rows) / (IMAGE_H - 1.0)
    forward = 8.0 + np.power(vertical, 1.35) * 260.0
    half_width = fov_base + fov_growth * forward
    lateral = ((cols - IMAGE_W / 2.0) / (IMAGE_W / 2.0)) * half_width

    cos_h = math.cos(heading)
    sin_h = math.sin(heading)
    world_x = x + forward * cos_h - lateral * sin_h
    world_y = y + forward * sin_h + lateral * cos_h
    sample_x = np.rint(world_x).astype(np.int32)
    sample_y = np.rint(world_y).astype(np.int32)
    valid = (
        (sample_x >= 0) & (sample_x < MAP_W)
        & (sample_y >= 0) & (sample_y < MAP_H)
    )
    safe_x = np.clip(sample_x, 0, MAP_W - 1)
    safe_y = np.clip(sample_y, 0, MAP_H - 1)
    road = valid & track_mask[safe_y, safe_x]
    texture = topdown_gray[safe_y, safe_x].astype(np.float32)

    camera = np.where(road, 190.0 + texture * 0.22,
                      16.0 + texture * 0.15)
    deterministic_noise = (
        (rows.astype(np.int32) * 3 + cols.astype(np.int32) * 5
         + frame_index * 7) % 7
    ) - 3
    camera += deterministic_noise
    return np.clip(camera, 0, 255).astype(np.uint8), phase


def camera_frustum(x, y, heading, phase):
    projection = {
        "flat": (25.0, 0.65),
        "ramp-entry": (70.0, 0.35),
        "ramp-climb": (10.0, 0.60),
        "ramp-crest": (70.0, 0.35),
        "ramp-exit": (5.0, 0.50),
    }
    base, growth = projection[phase]
    near = 8.0
    far = 268.0
    near_half = base + growth * near
    far_half = base + growth * far
    cos_h = math.cos(heading)
    sin_h = math.sin(heading)
    right = np.array([-sin_h, cos_h])
    forward = np.array([cos_h, sin_h])
    position = np.array([x, y])
    return [
        position + forward * near - right * near_half,
        position + forward * far - right * far_half,
        position + forward * far + right * far_half,
        position + forward * near + right * near_half,
    ]


def map_to_panel(point):
    return (
        MAP_PANEL_X + float(point[0]) * MAP_PANEL_W / MAP_W,
        MAP_PANEL_Y + float(point[1]) * MAP_PANEL_H / MAP_H,
    )


def make_base_map(topdown, track_mask):
    visual = Image.fromarray(topdown).convert("RGB")
    overlay = Image.new("RGBA", visual.size, (0, 0, 0, 0))
    rgba = np.zeros((MAP_H, MAP_W, 4), dtype=np.uint8)
    slope_zone = np.zeros_like(track_mask)
    slope_zone[350:590, 155:400] = track_mask[350:590, 155:400]
    rgba[slope_zone] = np.array([255, 165, 40, 92], dtype=np.uint8)
    overlay = Image.fromarray(rgba, mode="RGBA")
    visual = Image.alpha_composite(visual.convert("RGBA"), overlay).convert("RGB")
    return visual.resize((MAP_PANEL_W, MAP_PANEL_H), Image.Resampling.LANCZOS)


def draw_camera_panel(canvas, camera, result):
    camera_rgb = np.repeat(camera[:, :, None], 3, axis=2)
    camera_image = Image.fromarray(camera_rgb).resize(
        (IMAGE_W * CAM_SCALE, IMAGE_H * CAM_SCALE),
        Image.Resampling.NEAREST,
    )
    draw = ImageDraw.Draw(camera_image)
    for row in range(result.reference_row, IMAGE_H, 2):
        if result.edge_flags[row] & 0x01:
            x = int(result.left_edge[row]) * CAM_SCALE
            y = row * CAM_SCALE
            draw.ellipse((x - 2, y - 2, x + 2, y + 2), fill=(255, 55, 55))
        if result.edge_flags[row] & 0x02:
            x = int(result.right_edge[row]) * CAM_SCALE
            y = row * CAM_SCALE
            draw.ellipse((x - 2, y - 2, x + 2, y + 2), fill=(255, 55, 55))
        if result.edge_flags[row]:
            x = int(result.center_line[row]) * CAM_SCALE
            y = row * CAM_SCALE
            draw.ellipse((x - 2, y - 2, x + 2, y + 2), fill=(40, 255, 100))
    canvas.paste(camera_image, (CAM_PANEL_X, CAM_PANEL_Y))
    return camera_image.size


def draw_speed_graph(draw, history, x, y, width, height):
    draw.rectangle((x, y, x + width, y + height), fill=(21, 28, 38),
                   outline=(82, 94, 110), width=1)
    draw.text((x + 8, y + 6), "Speed loop: target / actual (ticks per 100 ms)",
              font=FONT_14, fill=(225, 231, 239))
    plot_left = x + 10
    plot_top = y + 30
    plot_right = x + width - 10
    plot_bottom = y + height - 12
    max_value = 110.0
    for value, color in ((100, (70, 80, 94)), (50, (55, 65, 78))):
        py = plot_bottom - value / max_value * (plot_bottom - plot_top)
        draw.line((plot_left, py, plot_right, py), fill=color, width=1)
    recent = history[-80:]
    if len(recent) < 2:
        return
    target_points = []
    actual_points = []
    for index, item in enumerate(recent):
        px = plot_left + index * (plot_right - plot_left) / max(len(recent) - 1, 1)
        target = (item["left_target"] + item["right_target"]) / 2.0
        actual = (item["left_speed"] + item["right_speed"]) / 2.0
        target_points.append((px, plot_bottom - target / max_value
                              * (plot_bottom - plot_top)))
        actual_points.append((px, plot_bottom - actual / max_value
                              * (plot_bottom - plot_top)))
    draw.line(target_points, fill=(255, 205, 55), width=2)
    draw.line(actual_points, fill=(65, 235, 130), width=2)


def draw_frame(base_map, camera, result, state, trajectories, history,
               frame_index, frame_count, fps):
    canvas = Image.new("RGB", (CANVAS_W, CANVAS_H), (13, 18, 25))
    canvas.paste(base_map, (MAP_PANEL_X, MAP_PANEL_Y))
    draw = ImageDraw.Draw(canvas)
    draw.text((20, 17), "TC264 whole-course route-following", font=FONT_24,
              fill=(240, 245, 250))
    draw.text((520, 25),
              f"{state['segment_count']} visible sections / same embedded algorithm",
              font=FONT_14, fill=(165, 179, 197))
    draw.rectangle(
        (MAP_PANEL_X, MAP_PANEL_Y,
         MAP_PANEL_X + MAP_PANEL_W, MAP_PANEL_Y + MAP_PANEL_H),
        outline=(82, 94, 110), width=1,
    )

    for index, trajectory in enumerate(trajectories):
        if len(trajectory) <= 1:
            continue
        color = ((65, 225, 245) if index == state["segment_index"]
                 else (60, 150, 175))
        draw.line([map_to_panel(point) for point in trajectory],
                  fill=color, width=3)

    frustum = [map_to_panel(point) for point in camera_frustum(
        state["x"], state["y"], state["heading"], state["phase"]
    )]
    draw.line(frustum + [frustum[0]], fill=(255, 205, 55), width=1)

    x = state["x"]
    y = state["y"]
    heading = state["heading"]
    forward = np.array([math.cos(heading), math.sin(heading)])
    right = np.array([-math.sin(heading), math.cos(heading)])
    car = np.array([x, y])
    car_points = [
        car + forward * 25,
        car - forward * 18 + right * 14,
        car - forward * 18 - right * 14,
    ]
    draw.polygon([map_to_panel(point) for point in car_points],
                 fill=(30, 225, 245), outline=(230, 255, 255))
    nose = map_to_panel(car + forward * 42)
    center = map_to_panel(car)
    draw.line((center[0], center[1], nose[0], nose[1]),
              fill=(255, 255, 255), width=2)
    zone_label = map_to_panel((175, 365))
    draw.text(zone_label, "SIMULATED UPHILL LOAD", font=FONT_14,
              fill=(255, 222, 135))

    camera_size = draw_camera_panel(canvas, camera, result)
    draw = ImageDraw.Draw(canvas)
    draw.text((CAM_PANEL_X, 23), "Virtual MT9V034 188x120",
              font=FONT_18, fill=(240, 245, 250))
    draw.rectangle(
        (CAM_PANEL_X, CAM_PANEL_Y,
         CAM_PANEL_X + camera_size[0], CAM_PANEL_Y + camera_size[1]),
        outline=(82, 94, 110), width=1,
    )

    status_y = 308
    status = [
        f"frame       {frame_index + 1:03d}/{frame_count:03d}   "
        f"time {frame_index / fps:5.2f} s",
        f"section     {state['segment_index'] + 1}/{state['segment_count']} "
        f"{state['segment_name']}",
        f"track       {'VALID' if result.track_valid else 'LOST ':5s}   "
        f"threshold {result.white_min:3d}",
        f"line error  {result.steering_error:+4d} px   "
        f"steering {state['steering']:+.3f}",
        f"servo angle {state['servo_angle']:5.1f} deg   "
        f"camera {state['phase']}",
        f"ramp state  {result.ramp_state:d}   width delta "
        f"{result.ramp_width_delta:+3d}",
        f"speed L/R   {state['left_speed']:5.1f} / "
        f"{state['right_speed']:5.1f}",
        f"target L/R  {state['left_target']:3d} / "
        f"{state['right_target']:3d}",
        f"duty L/R    {state['left_duty']:3d}% / "
        f"{state['right_duty']:3d}%   load x{state['load']:.2f}",
    ]
    for index, line in enumerate(status):
        color = (255, 205, 55) if (
            "ramp state" in line and result.ramp_active
        ) else (219, 227, 237)
        draw.text((CAM_PANEL_X, status_y + index * 23), line,
                  font=FONT_16, fill=color)

    draw_speed_graph(draw, history, CAM_PANEL_X, 505, 400, 175)
    draw.text((20, 675),
              "red=edge  green=center  cyan=vehicle path  yellow=camera FOV",
              font=FONT_14, fill=(165, 179, 197))
    return canvas


def save_trajectory_image(base_map, trajectories, segments, output_path):
    image = base_map.copy()
    draw = ImageDraw.Draw(image)
    colors = ((10, 225, 245), (255, 205, 55),
              (255, 105, 170), (115, 240, 120),
              (185, 125, 255), (255, 135, 70))
    for index, trajectory in enumerate(trajectories):
        points = [(
            float(point[0]) * MAP_PANEL_W / MAP_W,
            float(point[1]) * MAP_PANEL_H / MAP_H,
        ) for point in trajectory]
        if len(points) <= 1:
            continue
        color = colors[index % len(colors)]
        draw.line(points, fill=color, width=5)
        start = points[0]
        draw.ellipse((start[0] - 5, start[1] - 5,
                      start[0] + 5, start[1] + 5), fill=color)
        draw.text((start[0] + 7, start[1] - 8), str(index + 1),
                  font=FONT_14, fill=color)
    image.save(output_path, quality=94)


def find_ffmpeg(script_dir):
    bundled_candidates = [
        script_dir / "tools" / "ffmpeg.exe",
        script_dir.parent / "tools" / "ffmpeg.exe",
    ]
    for candidate in bundled_candidates:
        if candidate.exists():
            return candidate
    vendor = script_dir / "vendor"
    if vendor.exists():
        sys.path.insert(0, str(vendor))
    try:
        import imageio_ffmpeg
        return Path(imageio_ffmpeg.get_ffmpeg_exe())
    except (ImportError, RuntimeError):
        pass
    system_ffmpeg = shutil.which("ffmpeg")
    if system_ffmpeg:
        return Path(system_ffmpeg)
    raise RuntimeError("FFmpeg was not found in tools, imageio-ffmpeg, or PATH")


def encode_video(script_dir, frames_dir, output_path, fps):
    ffmpeg = find_ffmpeg(script_dir)
    command = [
        str(ffmpeg), "-y", "-loglevel", "error",
        "-framerate", str(fps), "-start_number", "0",
        "-i", str(frames_dir / "frame_%04d.jpg"),
        "-c:v", "libx264", "-preset", "medium", "-crf", "18",
        "-pix_fmt", "yuv420p", "-movflags", "+faststart",
        str(output_path),
    ]
    subprocess.run(command, check=True)
    return ffmpeg


def simulate(source_path, output_dir, frame_count, fps, make_video=True):
    output_dir.mkdir(parents=True, exist_ok=True)
    frames_dir = output_dir / "frames"
    snapshots_dir = output_dir / "snapshots"
    frames_dir.mkdir(exist_ok=True)
    snapshots_dir.mkdir(exist_ok=True)
    for stale_frame in frames_dir.glob("frame_*.jpg"):
        stale_frame.unlink()
    for stale_snapshot in snapshots_dir.glob("snapshot_*.jpg"):
        stale_snapshot.unlink()

    topdown = build_topdown(source_path)
    track_mask = extract_track_mask(topdown)
    save_map_artifacts(topdown, track_mask, output_dir)
    topdown_gray = np.asarray(
        Image.fromarray(topdown).convert("L"), dtype=np.uint8
    )
    base_map = make_base_map(topdown, track_mask)

    segments = allocate_tour_segments(frame_count)
    segment_index = -1
    segment_start_frame = 0
    route_algorithm = None
    controller = None
    x = y = heading = steering = 0.0
    left_speed = right_speed = 0.0
    left_target = right_target = NORMAL_SPEED
    left_duty = right_duty = 0
    trajectories = [[] for _ in segments]
    history = []
    metrics = []
    section_starts = []
    cursor = 0
    for segment in segments:
        section_starts.append(cursor)
        cursor += segment["frames"]
    snapshot_frames = {0, frame_count - 1}
    for start, segment in zip(section_starts, segments):
        snapshot_frames.add(start)
        snapshot_frames.add(start + segment["frames"] // 2)
        snapshot_frames.add(start + segment["frames"] - 1)

    for frame_index in range(frame_count):
        if (segment_index + 1 < len(segments)
                and frame_index == section_starts[segment_index + 1]):
            segment_index += 1
            segment = segments[segment_index]
            segment_start_frame = frame_index
            route_algorithm = RouteImageAlgorithm()
            controller = RouteController()
            x = segment["x"]
            y = segment["y"]
            heading = segment["heading"]
            steering = 0.0
            left_speed = 0.0
            right_speed = 0.0
            left_target = NORMAL_SPEED
            right_target = NORMAL_SPEED
            left_duty = 0
            right_duty = 0
            history = []

        segment = segments[segment_index]
        segment_frame = frame_index - segment_start_frame
        camera, phase = render_camera(
            track_mask, topdown_gray, x, y, heading, frame_index
        )
        result = route_algorithm.process(camera)
        steering = controller.update_steering(result)

        if frame_index % max(fps // 10, 1) == 0:
            left_target, right_target, left_duty, right_duty = (
                controller.update_speed(
                    result, int(left_speed + 0.5), int(right_speed + 0.5)
                )
            )

        on_slope = 155 <= x < 400 and 350 <= y <= 590
        load = 0.56 if on_slope else 1.0
        speed_response = 1.0 - math.exp(-(1.0 / fps) / 0.28)
        desired_left_speed = left_duty * 3.05 * load
        desired_right_speed = right_duty * 3.05 * load
        left_speed += (desired_left_speed - left_speed) * speed_response
        right_speed += (desired_right_speed - right_speed) * speed_response

        mean_ticks_100ms = (left_speed + right_speed) * 0.5
        distance = mean_ticks_100ms * (10.0 / fps) * 0.18
        steering_angle = math.radians(20.0 * steering)
        heading += distance / 58.0 * math.tan(steering_angle)
        x += distance * math.cos(heading)
        y += distance * math.sin(heading)

        trajectories[segment_index].append((x, y))
        state = {
            "x": x,
            "y": y,
            "heading": heading,
            "phase": phase,
            "steering": steering,
            "servo_angle": 80.0 - 20.0 * steering,
            "left_speed": left_speed,
            "right_speed": right_speed,
            "left_target": left_target,
            "right_target": right_target,
            "left_duty": left_duty,
            "right_duty": right_duty,
            "load": load,
            "segment_index": segment_index,
            "segment_count": len(segments),
            "segment_name": segment["name"],
        }
        history.append(state.copy())
        metrics.append({
            "frame": frame_index,
            "section": segment_index + 1,
            "section_name": segment["name"],
            "section_frame": segment_frame,
            "time_s": frame_index / fps,
            "x": x,
            "y": y,
            "heading_deg": math.degrees(heading),
            "track_valid": result.track_valid,
            "line_error_px": result.steering_error,
            "steering": steering,
            "servo_angle_deg": state["servo_angle"],
            "ramp_state": result.ramp_state,
            "ramp_width_delta": result.ramp_width_delta,
            "left_target": left_target,
            "right_target": right_target,
            "left_speed": left_speed,
            "right_speed": right_speed,
            "left_duty_percent": left_duty,
            "right_duty_percent": right_duty,
            "load_factor": load,
            "camera_phase": phase,
        })

        frame_image = draw_frame(
            base_map, camera, result, state, trajectories, history,
            frame_index, frame_count, fps
        )
        frame_path = frames_dir / f"frame_{frame_index:04d}.jpg"
        frame_image.save(frame_path, quality=91, subsampling=0)
        if frame_index in snapshot_frames:
            frame_image.save(
                snapshots_dir / f"snapshot_{frame_index:04d}.jpg",
                quality=94,
                subsampling=0,
            )

    with (output_dir / "metrics.csv").open("w", newline="",
                                                   encoding="utf-8-sig") as file:
        writer = csv.DictWriter(file, fieldnames=metrics[0].keys())
        writer.writeheader()
        writer.writerows(metrics)

    save_trajectory_image(base_map, trajectories, segments,
                          output_dir / "trajectory.jpg")
    video_path = output_dir / "route_tracking_simulation.mp4"
    ffmpeg = None
    if make_video:
        ffmpeg = encode_video(Path(__file__).resolve().parent,
                              frames_dir, video_path, fps)

    summary = {
        "source": str(source_path),
        "frames": frame_count,
        "fps": fps,
        "duration_s": frame_count / fps,
        "whole_course": True,
        "sections": [
            {
                "name": segment["name"],
                "frames": segment["frames"],
                "valid_frames": int(sum(
                    item["track_valid"] for item in metrics
                    if item["section"] == index + 1
                )),
            }
            for index, segment in enumerate(segments)
        ],
        "valid_frames": int(sum(item["track_valid"] for item in metrics)),
        "ramp_frames": int(sum(1 <= item["ramp_state"] <= 4
                               for item in metrics)),
        "maximum_abs_line_error_px": int(max(
            abs(item["line_error_px"]) for item in metrics
        )),
        "final_position": [round(x, 2), round(y, 2)],
        "ffmpeg": str(ffmpeg) if ffmpeg else None,
        "video": str(video_path) if make_video else None,
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return summary


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=FRAMES_DEFAULT)
    parser.add_argument("--fps", type=int, default=FPS_DEFAULT)
    parser.add_argument("--no-video", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    summary = simulate(
        args.source.resolve(), args.output.resolve(), args.frames, args.fps,
        make_video=not args.no_video,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
