"""Fixed-view wall-frame projection used for calibration tests."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True, slots=True)
class FixedView:
    position_m: tuple[float, float, float]
    yaw_deg: float = 0.0
    pitch_deg: float = 0.0
    horizontal_fov_deg: float = 50.0


def project_point(
    point_m: tuple[float, float, float],
    view: FixedView,
    viewport: tuple[int, int],
) -> tuple[float, float] | None:
    """Project a wall-frame point to pixels, or return None when behind the viewer."""
    width, height = viewport
    if width <= 0 or height <= 0:
        raise ValueError("viewport dimensions must be positive")
    if not 1.0 < view.horizontal_fov_deg < 179.0:
        raise ValueError("horizontal_fov_deg must be between 1 and 179")

    yaw = math.radians(view.yaw_deg)
    pitch = math.radians(view.pitch_deg)
    sin_yaw, cos_yaw = math.sin(yaw), math.cos(yaw)
    sin_pitch, cos_pitch = math.sin(pitch), math.cos(pitch)

    right = (cos_yaw, -sin_yaw, 0.0)
    forward = (sin_yaw * cos_pitch, cos_yaw * cos_pitch, sin_pitch)
    up = (-sin_yaw * sin_pitch, -cos_yaw * sin_pitch, cos_pitch)
    delta = tuple(point_m[i] - view.position_m[i] for i in range(3))

    camera_x = sum(delta[i] * right[i] for i in range(3))
    camera_y = sum(delta[i] * up[i] for i in range(3))
    camera_z = sum(delta[i] * forward[i] for i in range(3))
    if camera_z <= 1e-6:
        return None

    tan_half_h = math.tan(math.radians(view.horizontal_fov_deg) / 2.0)
    tan_half_v = tan_half_h * height / width
    ndc_x = camera_x / (camera_z * tan_half_h)
    ndc_y = camera_y / (camera_z * tan_half_v)
    return ((ndc_x + 1.0) * width / 2.0, (1.0 - ndc_y) * height / 2.0)
