"""Animated timeline playback for dive visualization."""

import numpy as np
import pyvista as pv
from scene import DiveScene


class DiveTimeline:
    """Animated playback of dive path with optional camera following."""

    def __init__(self, scene: DiveScene, data: dict, speed: float = 1.0, follow: bool = False):
        """Initialize timeline animator.

        Args:
            scene: DiveScene instance (must be built)
            data: Parsed navigation data
            speed: Playback speed multiplier (1.0 = realtime)
            follow: Enable camera follow mode
        """
        self.scene = scene
        self.data = data
        self.speed = speed
        self.follow = follow

        self.plotter = scene.get_plotter()
        self.current_index = 0
        self.marker_actor = None

    def animate(self) -> None:
        """Start animated playback."""
        marker_sphere = pv.Sphere(radius=self.scene.tube_radius * 0.8)
        self.marker_actor = self.plotter.add_mesh(marker_sphere, color='red', opacity=0.8)

        def update_marker(**kwargs) -> None:
            """Update marker position and camera."""
            if self.current_index >= len(self.scene.viz_positions):
                return

            pos = self.scene.viz_positions[self.current_index]

            marker_mesh = pv.Sphere(radius=self.scene.tube_radius * 0.8, center=pos)
            self.plotter.remove_actor(self.marker_actor)
            self.marker_actor = self.plotter.add_mesh(marker_mesh, color='red', opacity=0.8)

            if self.follow and self.current_index > 0:
                self._update_camera(pos)

            samples_per_frame = max(1, int(self.speed))
            self.current_index += samples_per_frame

        self.plotter.add_timer_event(max_steps=len(self.scene.viz_positions), duration=50, callback=update_marker)
        self.plotter.show()

    def _update_camera(self, current_pos: np.ndarray) -> None:
        """Update camera to follow the current position.

        Args:
            current_pos: Current position in viz coordinates
        """
        if self.current_index < 1:
            return

        prev_pos = self.scene.viz_positions[self.current_index - 1]
        direction = current_pos - prev_pos
        direction_norm = np.linalg.norm(direction)

        if direction_norm < 1e-6:
            return

        direction = direction / direction_norm

        camera_offset = -direction * self.scene.tube_radius * 10
        camera_offset[1] += self.scene.tube_radius * 5

        camera_pos = current_pos + camera_offset
        focal_point = current_pos + direction * self.scene.tube_radius * 5

        self.plotter.camera_position = [
            camera_pos,
            focal_point,
            [0, 1, 0],
        ]
