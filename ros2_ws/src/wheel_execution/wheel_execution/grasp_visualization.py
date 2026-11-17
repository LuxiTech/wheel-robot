"""Render recorded CAD contact targets on their exact inference image."""

import base64
import html
import json

import cv2
import numpy as np


def render_grasp_svg(rgb_bytes, detail):
    """Writer-thread rendering; timestamps stay in their original clock domains."""
    rgb = np.frombuffer(rgb_bytes, dtype=np.uint8).reshape(detail["image_shape"])
    height, width = rgb.shape[:2]
    ok, encoded = cv2.imencode(".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    if not ok:
        raise RuntimeError("Grasp evidence image encoding failed")
    camera_from_origin = np.linalg.inv(np.asarray(detail["origin_camera"], dtype=float))
    intrinsic = np.asarray(detail["intrinsics"], dtype=float)
    target = detail["target"]
    marks = []
    projections = {}
    for stage in ("grasp", "pregrasp"):
        for side, matrix in target[stage].items():
            point = camera_from_origin @ np.asarray(matrix, dtype=float)[:, 3]
            key = f"{side}_{stage}"
            if point[2] <= 0:
                projections[key] = dict(visible=False, reason="behind_camera")
                continue
            pixel = intrinsic @ point[:3]
            x, y = pixel[:2] / pixel[2]
            visible = bool(0 <= x < width and 0 <= y < height)
            projections[key] = dict(pixel=[float(x), float(y)], visible=visible)
            if not visible:
                continue
            color = "#00ff88" if side == "left" else "#ff9933"
            fill = color if stage == "grasp" else "none"
            marks.append(
                f'<circle cx="{x:.2f}" cy="{y:.2f}" r="6" fill="{fill}" '
                f'stroke="{color}" stroke-width="2"/>'
                f'<text x="{x + 9:.2f}" y="{y:.2f}" fill="{color}" '
                f'font-size="14" stroke="black" stroke-width="0.3">{key}</text>'
            )
    detail["projections"] = projections
    caption = html.escape(
        json.dumps(
            dict(
                pair_id=target["pair_id"],
                source_stamp=detail["source_metadata"].get("stamp"),
                generated_wall_time_ns=detail["generated_wall_time_ns"],
            ),
            ensure_ascii=False,
        )
    )
    payload = base64.b64encode(encoded).decode("ascii")
    return (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height + 85}">'
        f'<rect width="100%" height="100%" fill="#151515"/>'
        f'<image width="{width}" height="{height}" href="data:image/jpeg;base64,{payload}"/>'
        + "".join(marks)
        + f'<text x="8" y="{height + 20}" fill="white" font-size="11">{caption}</text>'
        + f'<text x="8" y="{height + 42}" fill="white" font-size="12">'
        "Filled: grasp; hollow: pregrasp. CAD contact midpoint, not wrist or measured tip.</text>"
        + f'<text x="8" y="{height + 64}" fill="white" font-size="12">'
        "Source timestamps retain original clock domain; see matching JSON.</text></svg>"
    ).encode("utf-8")
