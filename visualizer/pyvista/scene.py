"""PyVista scene construction for dive visualization."""

import numpy as np
import pyvista as pv
from scipy.interpolate import CubicSpline
from flag_decoder import flag_severity


class DiveScene:
    """3D visualization scene for cave dive navigation data."""

    def __init__(self, data: dict, tube_radius: float = 0.15, downsample: int = 10):
        """Initialize dive scene.

        Args:
            data: Parsed navigation data from csv_parser
            tube_radius: Radius of the path tube in meters
            downsample: Downsample factor for spline interpolation
        """
        self.data = data
        self.tube_radius = tube_radius
        self.downsample = downsample

        self.viz_positions = self._transform_coordinates(data['positions'])
        self.plotter = None

    def _transform_coordinates(self, positions: np.ndarray) -> np.ndarray:
        """Transform firmware coordinates to PyVista Y-up visualization coordinates.

        Firmware: px (forward), py (lateral), pz (depth, negative down)
        PyVista:  x (forward), y (height, up), z (lateral)

        Args:
            positions: (N, 3) array in firmware coords [px, py, pz]

        Returns:
            (N, 3) array in viz coords [x, y, z]
        """
        viz = np.zeros_like(positions)
        viz[:, 0] = positions[:, 0]   # x = px (forward)
        viz[:, 1] = -positions[:, 2]  # y = -pz (invert depth to height)
        viz[:, 2] = positions[:, 1]   # z = py (lateral)
        return viz

    def _compute_forward_vector(self, quat: np.ndarray) -> np.ndarray:
        """Compute forward vector from quaternion (first column of rotation matrix).

        Args:
            quat: Quaternion [w, x, y, z]

        Returns:
            Forward vector [x, y, z] in firmware coords
        """
        w, x, y, z = quat
        forward = np.array([
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y + w * z),
            2.0 * (x * z - w * y),
        ])
        return forward

    def _create_spline_path(self) -> np.ndarray:
        """Create smooth spline path from downsampled positions.

        Returns:
            (M, 3) array of interpolated points
        """
        indices = np.arange(0, len(self.viz_positions), self.downsample)
        if indices[-1] != len(self.viz_positions) - 1:
            indices = np.append(indices, len(self.viz_positions) - 1)

        key_points = self.viz_positions[indices]

        if len(key_points) < 4:
            return key_points

        t = np.arange(len(key_points))
        cs = CubicSpline(t, key_points, bc_type='natural')

        t_fine = np.linspace(0, len(key_points) - 1, len(key_points) * 10)
        return cs(t_fine)

    def build(
        self,
        color_mode: str = 'depth',
        show_arrows: bool = False,
        arrow_interval: int = 100,
        events: list | None = None,
    ) -> None:
        """Build the 3D scene with path, arrows, and annotations.

        Args:
            color_mode: 'depth', 'speed', or 'flags'
            show_arrows: Whether to show orientation arrows
            arrow_interval: Spacing between arrows in samples
            events: Optional list of event dicts from parse_events_csv
        """
        self.plotter = pv.Plotter()
        self.plotter.set_background('#0a0e14')
        self.plotter.add_text('Camazotz Dive Visualizer', position='upper_left', color='white', font_size=12)

        spline_path = self._create_spline_path()
        spline_polyline = pv.Spline(spline_path, n_points=len(spline_path))

        scalars = self._compute_scalars(color_mode, spline_path)
        cmap, clim = self._get_colormap(color_mode, scalars)

        # Assign scalars to polyline before tubing so they propagate to tube vertices
        spline_polyline['scalars'] = scalars

        tube = spline_polyline.tube(radius=self.tube_radius, n_sides=12)

        self.plotter.add_mesh(
            tube,
            scalars='scalars',
            cmap=cmap,
            clim=clim,
            show_scalar_bar=True,
            scalar_bar_args={'title': self._get_scalar_label(color_mode), 'color': 'white'},
        )

        if show_arrows:
            self._add_orientation_arrows(arrow_interval)

        self._add_flag_overlays()

        if events:
            self._add_event_markers(events)

    def _compute_scalars(self, color_mode: str, spline_path: np.ndarray) -> np.ndarray:
        """Compute scalar values for coloring based on mode."""
        if color_mode == 'depth':
            return spline_path[:, 1]
        elif color_mode == 'speed':
            distances = np.sqrt(np.sum(np.diff(spline_path, axis=0)**2, axis=1))
            speeds = np.concatenate([[0], distances])
            return speeds
        elif color_mode == 'flags':
            flag_scalars = np.zeros(len(spline_path))
            for i in range(len(self.data['flags'])):
                severity = flag_severity(self.data['flags'][i])
                if severity == 'ok':
                    value = 0.0
                elif severity == 'warning':
                    value = 1.0
                else:
                    value = 2.0

                idx = min(i * 10 // self.downsample, len(flag_scalars) - 1)
                flag_scalars[idx] = max(flag_scalars[idx], value)
            return flag_scalars
        else:
            return spline_path[:, 1]

    def _get_colormap(self, color_mode: str, scalars: np.ndarray) -> tuple:
        """Get colormap and limits for the given color mode."""
        if color_mode == 'depth':
            return 'cool', (np.min(scalars), np.max(scalars))
        elif color_mode == 'speed':
            return 'RdYlGn_r', (0, np.max(scalars))
        elif color_mode == 'flags':
            return ['#00ff00', '#ffd700', '#ff0000'], (0, 2)
        else:
            return 'viridis', None

    def _get_scalar_label(self, color_mode: str) -> str:
        """Get label for scalar bar."""
        if color_mode == 'depth':
            return 'Depth (m)'
        elif color_mode == 'speed':
            return 'Speed (m/s)'
        elif color_mode == 'flags':
            return 'Quality'
        else:
            return 'Value'

    def _add_orientation_arrows(self, interval: int) -> None:
        """Add orientation arrows along the path."""
        for i in range(0, len(self.data['quaternions']), interval):
            pos = self.viz_positions[i]
            quat = self.data['quaternions'][i]

            forward_fw = self._compute_forward_vector(quat)
            forward_viz = self._transform_coordinates(forward_fw.reshape(1, 3))[0]
            forward_viz = forward_viz / (np.linalg.norm(forward_viz) + 1e-8)

            arrow_length = self.tube_radius * 3

            arrow = pv.Arrow(
                start=pos,
                direction=forward_viz,
                scale=arrow_length,
                tip_length=0.25,
                tip_radius=0.1,
                shaft_radius=0.05,
            )
            self.plotter.add_mesh(arrow, color='yellow', opacity=0.6)

    def _add_flag_overlays(self) -> None:
        """Add translucent overlays for warning/critical flag segments."""
        warning_segments = []
        critical_segments = []

        current_warning = []
        current_critical = []

        for i, flag in enumerate(self.data['flags']):
            severity = flag_severity(flag)
            pos = self.viz_positions[i]

            if severity == 'warning':
                current_warning.append(pos)
                if current_critical:
                    if len(current_critical) > 1:
                        critical_segments.append(np.array(current_critical))
                    current_critical = []
            elif severity == 'critical':
                current_critical.append(pos)
                if current_warning:
                    if len(current_warning) > 1:
                        warning_segments.append(np.array(current_warning))
                    current_warning = []
            else:
                if current_warning:
                    if len(current_warning) > 1:
                        warning_segments.append(np.array(current_warning))
                    current_warning = []
                if current_critical:
                    if len(current_critical) > 1:
                        critical_segments.append(np.array(current_critical))
                    current_critical = []

        if len(current_warning) > 1:
            warning_segments.append(np.array(current_warning))
        if len(current_critical) > 1:
            critical_segments.append(np.array(current_critical))

        for segment in warning_segments:
            if len(segment) > 1:
                spline = pv.Spline(segment, n_points=len(segment) * 2)
                tube = spline.tube(radius=self.tube_radius * 1.2, n_sides=8)
                self.plotter.add_mesh(tube, color='#ffd700', opacity=0.3)

        for segment in critical_segments:
            if len(segment) > 1:
                spline = pv.Spline(segment, n_points=len(segment) * 2)
                tube = spline.tube(radius=self.tube_radius * 1.2, n_sides=8)
                self.plotter.add_mesh(tube, color='#ff0000', opacity=0.3)

    def _add_event_markers(self, events: list) -> None:
        """Add labeled point markers for events."""
        for event in events:
            seq = event['seq']
            if seq >= len(self.viz_positions):
                continue

            pos = self.viz_positions[seq]
            tag = event['tag']

            sphere = pv.Sphere(radius=self.tube_radius * 0.5, center=pos)
            self.plotter.add_mesh(sphere, color='cyan', opacity=0.8)
            self.plotter.add_point_labels(
                [pos],
                [tag],
                font_size=10,
                text_color='white',
                point_color='cyan',
                point_size=5,
            )

    def add_cross_section(self, index: int) -> None:
        """Add a cross-sectional plane at the given path index.

        Args:
            index: Sample index along the path
        """
        if self.plotter is None:
            raise RuntimeError("Must call build() before add_cross_section()")

        if index < 0 or index >= len(self.viz_positions):
            raise ValueError(f"Index {index} out of range [0, {len(self.viz_positions)})")

        pos = self.viz_positions[index]

        if index < len(self.viz_positions) - 1:
            tangent = self.viz_positions[index + 1] - pos
        else:
            tangent = pos - self.viz_positions[index - 1]

        tangent = tangent / (np.linalg.norm(tangent) + 1e-8)

        disc = pv.Disc(center=pos, normal=tangent, inner=0, outer=self.tube_radius * 5, c_res=32)
        self.plotter.add_mesh(disc, color='white', opacity=0.3)

    def show(self) -> None:
        """Display the interactive plotter window."""
        if self.plotter is None:
            raise RuntimeError("Must call build() before show()")

        self.plotter.show()

    def get_plotter(self) -> pv.Plotter:
        """Get the PyVista plotter instance."""
        if self.plotter is None:
            raise RuntimeError("Must call build() before get_plotter()")

        return self.plotter
