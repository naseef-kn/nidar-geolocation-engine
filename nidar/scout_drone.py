"""Scout drone simulation — detects survivors in flooded areas."""

from dataclasses import dataclass

from nidar.detection import PixelDetection


@dataclass
class ScoutDrone:
    """Simulated scout drone that detects survivors in an image.

    Attributes:
        frame_width: Width of the camera image in pixels.
        frame_height: Height of the camera image in pixels.
    """

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

    def detect_survivors(self) -> list[PixelDetection]:
        """Simulate detecting survivors in the current frame.

        Returns:
            A list of PixelDetection objects, one per detected survivor.
            In simulation, always returns exactly one detection at the centre.
        """
        # SIMULATION: hard-code one survivor at the centre of the frame
        centre_x = self.frame_width // 2
        centre_y = self.frame_height // 2
        confidence = 0.95

        detection = PixelDetection(
            x=centre_x,
            y=centre_y,
            confidence=confidence,
            frame_width=self.frame_width,
            frame_height=self.frame_height,
        )

        return [detection]