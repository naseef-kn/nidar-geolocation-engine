"""Survivor detections in image pixel coordinates.

PIXEL COORDINATE CONVENTION (fixed for this project):
    Origin (0, 0) is the TOP-LEFT corner of the image.
    x increases to the RIGHT.
    y increases DOWNWARD.
    Valid x range: 0 .. frame_width - 1
    Valid y range: 0 .. frame_height - 1

This matches OpenCV / PIL image indexing. Do not change it without
also changing every downstream geolocation calculation.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class PixelDetection:
    """One survivor detection reported by the scout drone's camera.

    Attributes:
        x: Horizontal pixel position, 0 at the left edge.
        y: Vertical pixel position, 0 at the top edge.
        confidence: Detector confidence, 0.0 (none) to 1.0 (certain).
        frame_width: Width of the source image in pixels.
        frame_height: Height of the source image in pixels.
    """

    x: int
    y: int
    confidence: float
    frame_width: int
    frame_height: int

    def __post_init__(self) -> None:
        if self.frame_width <= 0:
            raise ValueError(
                f"frame_width must be positive, got {self.frame_width}"
            )
        if self.frame_height <= 0:
            raise ValueError(
                f"frame_height must be positive, got {self.frame_height}"
            )
        if not 0 <= self.x <= self.frame_width - 1:
            raise ValueError(
                f"x must be within 0..{self.frame_width - 1}, got {self.x}"
            )
        if not 0 <= self.y <= self.frame_height - 1:
            raise ValueError(
                f"y must be within 0..{self.frame_height - 1}, got {self.y}"
            )
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError(
                f"confidence must be within 0.0..1.0, got {self.confidence}"
            )