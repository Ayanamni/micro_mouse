"""MJCF model assembly and track data loading."""

from pathlib import Path
from typing import Optional

from .track import TrackCenterline, generate_track_lines_xml, inject_track_lines


def load_track(filepath: str) -> TrackCenterline:
    """Load a track centerline from a Robotrace points file."""
    return TrackCenterline(filepath)


def build_model_xml(
    base_xml_path: str,
    track: Optional[TrackCenterline] = None,
    track_width: float = 0.180,
    line_width: float = 0.003,
) -> str:
    """
    Build complete MJCF XML string.

    Injects flat line markings (white center + red edges) instead of 3D walls.
    """
    base = Path(base_xml_path).read_text(encoding="utf-8")

    if track is not None:
        lines_xml = generate_track_lines_xml(
            track,
            track_width=track_width,
            line_width=line_width,
        )
        base = inject_track_lines(base, lines_xml)
    else:
        base = base.replace("<!-- WALLS -->", "")

    return base
