"""Export utilities for screenshots, videos, and mesh files."""

import numpy as np
import pyvista as pv
from tqdm import tqdm
from scene import DiveScene


class DiveExporter:
    """Export dive visualizations to various formats."""

    def __init__(self, scene: DiveScene, data: dict):
        """Initialize exporter.

        Args:
            scene: Built DiveScene instance
            data: Parsed navigation data
        """
        self.scene = scene
        self.data = data
        self.plotter = scene.get_plotter()

    def screenshot(self, path: str) -> None:
        """Save a screenshot of the current view.

        Args:
            path: Output file path (PNG/JPEG based on extension)
        """
        self.plotter.screenshot(path)
        print(f"Screenshot saved to {path}")

    def gif(self, path: str, n_frames: int = 120, orbit: bool = True) -> None:
        """Export an animated GIF.

        Args:
            path: Output GIF file path
            n_frames: Number of frames
            orbit: If True, orbital rotation; if False, follow-cam path
        """
        import imageio

        self.plotter.open_gif(path)

        if orbit:
            self._render_orbit(n_frames)
        else:
            self._render_path_follow(n_frames)

        self.plotter.close()
        print(f"GIF saved to {path}")

    def video(self, path: str, n_frames: int = 300, fps: int = 30) -> None:
        """Export an MP4 video.

        Args:
            path: Output MP4 file path
            n_frames: Number of frames
            fps: Frames per second
        """
        import imageio

        frames = []

        print("Rendering frames...")
        for i in tqdm(range(n_frames)):
            angle = 360.0 * i / n_frames
            self.plotter.camera.azimuth = angle
            self.plotter.render()

            frame = self.plotter.screenshot(return_img=True)
            frames.append(frame)

        print(f"Writing video to {path}...")
        imageio.mimsave(path, frames, fps=fps, codec='libx264')
        print(f"Video saved to {path}")

    def export_mesh(self, path: str) -> None:
        """Export the dive path mesh to file.

        Args:
            path: Output file path (PLY/STL/OBJ based on extension)
        """
        spline_path = self.scene._create_spline_path()
        spline_polyline = pv.Spline(spline_path, n_points=len(spline_path))
        tube = spline_polyline.tube(radius=self.scene.tube_radius, n_sides=12)

        tube.save(path)
        print(f"Mesh exported to {path}")

    def _render_orbit(self, n_frames: int) -> None:
        """Render orbital rotation frames."""
        for i in tqdm(range(n_frames), desc="Rendering orbit"):
            angle = 360.0 * i / n_frames
            self.plotter.camera.azimuth = angle
            self.plotter.write_frame()

    def _render_path_follow(self, n_frames: int) -> None:
        """Render path follow frames."""
        positions = self.scene.viz_positions
        step = max(1, len(positions) // n_frames)

        for i in tqdm(range(0, len(positions), step), desc="Rendering path"):
            if i >= len(positions):
                break

            pos = positions[i]

            if i < len(positions) - 1:
                direction = positions[i + 1] - pos
            else:
                direction = pos - positions[i - 1]

            direction_norm = np.linalg.norm(direction)
            if direction_norm > 1e-6:
                direction = direction / direction_norm

                camera_offset = -direction * self.scene.tube_radius * 10
                camera_offset[1] += self.scene.tube_radius * 5

                camera_pos = pos + camera_offset
                focal_point = pos + direction * self.scene.tube_radius * 5

                self.plotter.camera_position = [
                    camera_pos,
                    focal_point,
                    [0, 1, 0],
                ]

            self.plotter.write_frame()
