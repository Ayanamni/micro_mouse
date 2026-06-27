"""Track centerline representation and OpenGL polyline rendering."""

from typing import Tuple

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.optimize import minimize_scalar


class TrackCenterline:
    """
    Spline-based track centerline with arc-length parameterization.
    Also holds precomputed polyline data for efficient rendering.
    """

    def __init__(self, filepath: str, render_sample_spacing: float = 0.005,
                 close_loop: bool = True):
        """
        Args:
            filepath: Path to a Robotrace *_points.txt file.
            render_sample_spacing: Arc-length spacing for render polylines (m).
            close_loop: Append first points to the end for smooth spline closure.
        """
        raw = np.loadtxt(filepath)
        if close_loop:
            n_pad = min(5, len(raw))
            raw = np.vstack([raw, raw[:n_pad]])
        self._waypoints = raw
        self._n_points = len(raw)
        self._build_spline()

        # Precompute render polylines (center + left/right edges)
        self._build_render_lines(render_sample_spacing)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def waypoints(self) -> np.ndarray:
        return self._waypoints

    @property
    def total_length(self) -> float:
        return self._s_total

    @property
    def render_center(self) -> np.ndarray:
        """(M, 3) array of center line points for rendering."""
        return self._render_center

    @property
    def render_left(self) -> np.ndarray:
        """(M, 3) array of left edge points for rendering."""
        return self._render_left

    @property
    def render_right(self) -> np.ndarray:
        """(M, 3) array of right edge points for rendering."""
        return self._render_right

    def set_track_width(self, width: float):
        """Update edge lines to new track width."""
        self._track_width = width
        self._build_render_lines(self._render_spacing)

    def project(self, pos: np.ndarray) -> Tuple[float, float, float]:
        """
        Returns (arc_length, lateral_error, heading).
        lateral_error > 0 = left of track.
        """
        # Find closest waypoint as initial guess
        dx = self._waypoints[:, 0] - pos[0]
        dy = self._waypoints[:, 1] - pos[1]
        dists = dx * dx + dy * dy
        idx = int(np.argmin(dists))
        s0 = float(self._s_array[idx])

        # Narrow search window around initial guess to avoid local minima
        # on closed-loop tracks. ±0.5 m is safe for waypoint spacing ~1cm.
        window = 0.5
        lo = max(0.0, s0 - window)
        hi = min(self._s_total, s0 + window)
        result = minimize_scalar(
            lambda s: self._squared_distance(s, pos),
            bounds=(lo, hi),
            method="bounded",
            options={"xatol": 1e-8, "maxiter": 100},
        )
        s_closest = result.x

        xy = self._s_to_xy(s_closest)
        tangent = self._s_to_tangent(s_closest)
        heading = np.arctan2(tangent[1], tangent[0])
        delta = pos - xy
        lateral = -delta[0] * tangent[1] + delta[1] * tangent[0]
        return s_closest, lateral, heading

    def curvature_at(self, s: float) -> float:
        s = np.clip(s, 1e-6, self._s_total - 1e-6)
        ds = 1e-4
        t1 = self._s_to_tangent(s - ds)
        t2 = self._s_to_tangent(s + ds)
        theta1 = np.arctan2(t1[1], t1[0])
        theta2 = np.arctan2(t2[1], t2[0])
        dtheta = theta2 - theta1
        dtheta = np.arctan2(np.sin(dtheta), np.cos(dtheta))
        return dtheta / (2 * ds)

    def heading_at(self, s: float) -> float:
        t = self._s_to_tangent(s)
        return np.arctan2(t[1], t[0])

    def point_at(self, s: float) -> np.ndarray:
        return self._s_to_xy(s)

    def tangent_at(self, s: float) -> np.ndarray:
        return self._s_to_tangent(s)

    # ------------------------------------------------------------------
    def _build_spline(self) -> None:
        diffs = np.diff(self._waypoints, axis=0)
        seg_lengths = np.sqrt((diffs ** 2).sum(axis=1))
        self._s_array = np.zeros(self._n_points)
        self._s_array[1:] = np.cumsum(seg_lengths)
        self._s_total = float(self._s_array[-1])
        self._spline_x = CubicSpline(self._s_array, self._waypoints[:, 0])
        self._spline_y = CubicSpline(self._s_array, self._waypoints[:, 1])

    def _s_to_xy(self, s: float) -> np.ndarray:
        s = np.clip(s, 0.0, self._s_total)
        return np.array([float(self._spline_x(s)), float(self._spline_y(s))])

    def _s_to_tangent(self, s: float) -> np.ndarray:
        s = np.clip(s, 0.0, self._s_total)
        dx = float(self._spline_x.derivative()(s))
        dy = float(self._spline_y.derivative()(s))
        norm = np.sqrt(dx * dx + dy * dy)
        if norm < 1e-12:
            return np.array([1.0, 0.0])
        return np.array([dx / norm, dy / norm])

    def _squared_distance(self, s: float, pos: np.ndarray) -> float:
        xy = self._s_to_xy(s)
        return (xy[0] - pos[0]) ** 2 + (xy[1] - pos[1]) ** 2

    def _build_render_lines(self, spacing: float) -> None:
        """Precompute polyline points for center, left edge, right edge."""
        self._render_spacing = spacing
        self._track_width = getattr(self, '_track_width', 0.180)  # keep existing or default
        half_w = self._track_width / 2.0
        z = 0.0005  # barely above floor

        n = max(2, int(self._s_total / spacing))
        s_vals = np.linspace(0, self._s_total, n)
        pts = np.zeros((n, 2))

        for i, s in enumerate(s_vals):
            pts[i] = self._s_to_xy(s)

        # Center line
        self._render_center = np.column_stack([pts, np.full(n, z)])

        # Edge lines: compute normals at each point
        left_pts = np.zeros((n, 2))
        right_pts = np.zeros((n, 2))
        for i, s in enumerate(s_vals):
            t = self._s_to_tangent(s)
            n_vec = np.array([-t[1], t[0]])  # leftward normal
            left_pts[i] = pts[i] + n_vec * half_w
            right_pts[i] = pts[i] - n_vec * half_w

        self._render_left = np.column_stack([left_pts, np.full(n, z)])
        self._render_right = np.column_stack([right_pts, np.full(n, z)])


def generate_track_lines_xml(
    centerline: TrackCenterline,
    track_width: float = 0.180,
    line_width: float = 0.004,
    segment_length: float = 0.1,
) -> str:
    """
    Generate sparse capsule geoms for the center line only.
    Every ~25 cm — capsules are purely visual (contype=0, conaffinity=0).
    The line sensor reads from the spline directly, not from these geoms.
    Edge lines are omitted.
    """
    total = centerline.total_length
    if total < segment_length:
        return ""

    n_segments = max(2, int(total / segment_length))
    s_samples = np.linspace(0, total, n_segments + 1)
    radius = line_width / 2.0
    z = 0.0005

    lines = []

    for i in range(n_segments):
        s_mid = (s_samples[i] + s_samples[i + 1]) / 2.0
        seg_len = s_samples[i + 1] - s_samples[i]
        c = centerline.point_at(s_mid)
        tangent = centerline.tangent_at(s_mid)
        half_l = seg_len / 2.0 + radius * 0.3

        lines.append(
            f'    <geom name="cl{i}" type="capsule"'
            f' pos="{c[0]:.6f} {c[1]:.6f} {z:.6f}"'
            f' size="{radius:.6f} {half_l:.6f}"'
            f' zaxis="{tangent[0]:.6f} {tangent[1]:.6f} 0"'
            f' rgba="1 1 1 0.9" contype="0" conaffinity="0"/>'
        )

    return "\n".join(lines)


def inject_track_lines(base_xml: str, lines_xml: str) -> str:
    """Inject (empty) track content — all rendering is in user_scn."""
    placeholder = "<!-- WALLS -->"
    if placeholder in base_xml:
        return base_xml.replace(placeholder, lines_xml)
    return base_xml
