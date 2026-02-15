#!/usr/bin/env python3
"""
Synthetic test data generator for cave dive visualization.

Generates CSV files matching the firmware's SDIO logger format with
realistic dive scenarios including sensor degradation.
"""

import argparse
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.interpolate import CubicSpline
from pathlib import Path
from typing import Tuple, List
from dataclasses import dataclass


# NAV_FLAG constants from types.h
ENCODER_ESTIMATED = 0x01
IMU_ESTIMATED = 0x02
DEPTH_VIRTUAL = 0x04
DEPTH_UNVERIFIED = 0x08
NAV_CRITICAL = 0x10
ENCODER_LOST = 0x20
IMU_LOST = 0x40
SENSOR_CONFLICT = 0x80

# Firmware constants
TICK_RATE_HZ = 100
TICK_PERIOD_MS = 10
ENCODER_WHEEL_RADIUS_M = 0.025


@dataclass
class NavigationRow:
    """Single navigation data row."""
    timestamp_ms: int
    seq: int
    angular_delta: float
    qw: float
    qx: float
    qy: float
    qz: float
    px: float
    py: float
    pz: float
    delta_dist: float
    flags: int


@dataclass
class EventRow:
    """Single event log row."""
    timestamp_ms: int
    seq: int
    tag: str
    flags: int


def euler_to_quat(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Convert euler angles to quaternion (w, x, y, z)."""
    rot = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=False)
    q = rot.as_quat()  # Returns [x, y, z, w]
    return q[3], q[0], q[1], q[2]  # Return as (w, x, y, z)


def quat_to_forward_vector(qw: float, qx: float, qy: float, qz: float) -> np.ndarray:
    """
    Extract forward vector from quaternion.
    Forward = column 0 of rotation matrix = [1-2(yy+zz), 2(xy+wz), 2(xz-wy)]
    """
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    wz = qw * qz
    wy = qw * qy

    return np.array([
        1.0 - 2.0 * (yy + zz),
        2.0 * (xy + wz),
        2.0 * (xz - wy)
    ])


def compute_orientation_from_tangent(tangent: np.ndarray) -> Tuple[float, float, float, float]:
    """
    Compute quaternion from path tangent vector.

    Args:
        tangent: [dx, dy, dz] normalized tangent vector

    Returns:
        (qw, qx, qy, qz) quaternion
    """
    # Normalize tangent
    tangent = tangent / np.linalg.norm(tangent)

    # Compute euler angles
    heading = np.arctan2(tangent[1], tangent[0])  # yaw
    pitch = np.arctan2(-tangent[2], np.sqrt(tangent[0]**2 + tangent[1]**2))
    roll = 0.0

    return euler_to_quat(roll, pitch, heading)


def write_nav_csv(rows: List[NavigationRow], filepath: Path):
    """Write navigation CSV with firmware format."""
    with open(filepath, 'w') as f:
        # Header
        f.write('timestamp_ms,seq,angular_delta,qw,qx,qy,qz,px,py,pz,delta_dist,flags\n')

        # Data rows
        for row in rows:
            f.write(f'{row.timestamp_ms},{row.seq},{row.angular_delta:.6f},'
                   f'{row.qw:.6f},{row.qx:.6f},{row.qy:.6f},{row.qz:.6f},'
                   f'{row.px:.4f},{row.py:.4f},{row.pz:.4f},'
                   f'{row.delta_dist:.6f},0x{row.flags:02X}\n')


def write_events_csv(events: List[EventRow], filepath: Path):
    """Write events CSV with firmware format."""
    with open(filepath, 'w') as f:
        # Header
        f.write('timestamp_ms,seq,tag,flags\n')

        # Data rows
        for event in events:
            f.write(f'{event.timestamp_ms},{event.seq},{event.tag},0x{event.flags:02X}\n')


def generate_straight_dive(output_dir: Path, rng: np.random.Generator):
    """
    Generate Scenario 1: Simple straight dive.

    60 seconds, 50m horizontal, gentle descent to -15m, clean flags.
    """
    duration_s = 60
    num_samples = duration_s * TICK_RATE_HZ

    # Target velocity: 50m / 60s = 0.833 m/s
    target_velocity = 0.833
    delta_dist = target_velocity / TICK_RATE_HZ

    # Generate path
    nav_rows = []
    events = []

    # Initial state
    position = np.array([0.0, 0.0, 0.0])

    # Constant heading (forward along X), gentle downward pitch
    # Descend 15m over 60s = -0.25 m/s vertical
    heading = 0.0
    pitch = -np.arctan2(0.25, target_velocity)  # Small downward angle
    roll = 0.0

    qw, qx, qy, qz = euler_to_quat(roll, pitch, heading)

    # Start event
    events.append(EventRow(
        timestamp_ms=0,
        seq=0,
        tag='START',
        flags=0x00
    ))

    for i in range(num_samples):
        timestamp_ms = i * TICK_PERIOD_MS

        # Compute forward vector and integrate position
        forward = quat_to_forward_vector(qw, qx, qy, qz)
        position = position + forward * delta_dist

        # Add tiny noise for realism
        noisy_position = position + rng.normal(0, 0.001, 3)

        angular_delta = delta_dist / ENCODER_WHEEL_RADIUS_M

        nav_rows.append(NavigationRow(
            timestamp_ms=timestamp_ms,
            seq=i,
            angular_delta=angular_delta,
            qw=qw, qx=qx, qy=qy, qz=qz,
            px=noisy_position[0],
            py=noisy_position[1],
            pz=noisy_position[2],
            delta_dist=delta_dist,
            flags=0x00
        ))

    # End event
    events.append(EventRow(
        timestamp_ms=(num_samples - 1) * TICK_PERIOD_MS,
        seq=num_samples - 1,
        tag='END',
        flags=0x00
    ))

    # Write files
    nav_path = output_dir / 'straight_dive_nav.csv'
    events_path = output_dir / 'straight_dive_events.csv'
    write_nav_csv(nav_rows, nav_path)
    write_events_csv(events, events_path)

    # Print summary
    final_pos = nav_rows[-1]
    distance = np.sqrt(final_pos.px**2 + final_pos.py**2 + final_pos.pz**2)
    print(f'\nStraight Dive Summary:')
    print(f'  Rows: {len(nav_rows)}')
    print(f'  Duration: {duration_s}s')
    print(f'  Distance: {distance:.2f}m')
    print(f'  Final position: ({final_pos.px:.2f}, {final_pos.py:.2f}, {final_pos.pz:.2f})')
    print(f'  Depth range: 0.00m to {abs(final_pos.pz):.2f}m')
    print(f'  Files: {nav_path.name}, {events_path.name}')


def generate_complex_cave(output_dir: Path, rng: np.random.Generator):
    """
    Generate Scenario 2: Complex cave system with sensor degradation.

    200 seconds, splined path through waypoints, multiple degraded sections.
    """
    duration_s = 200
    num_samples = duration_s * TICK_RATE_HZ

    # Define waypoints (time, x, y, z)
    # Cave system: entry, left chamber, restriction, main gallery descent, exit
    waypoints = np.array([
        [0,    0,    0,    0],     # Entry
        [30,   25,   0,   -5],     # Initial descent
        [50,   35,  -10,  -7],     # Left turn into side chamber
        [80,   40,  -15,  -10],    # Side chamber
        [100,  50,  -15,  -12],    # Restriction start
        [120,  60,  -12,  -15],    # Restriction end
        [140,  75,  -8,   -25],    # Main gallery descent
        [170,  90,  -5,   -30],    # Deepest point
        [200,  110,  0,   -10],    # Exit ascent
    ])

    # Create spline for smooth path
    t_waypoints = waypoints[:, 0]
    x_spline = CubicSpline(t_waypoints, waypoints[:, 1])
    y_spline = CubicSpline(t_waypoints, waypoints[:, 2])
    z_spline = CubicSpline(t_waypoints, waypoints[:, 3])

    # Time array
    t_samples = np.arange(num_samples) / TICK_RATE_HZ

    # Evaluate spline positions
    x_path = x_spline(t_samples)
    y_path = y_spline(t_samples)
    z_path = z_spline(t_samples)

    # Compute tangents (derivatives)
    dx = x_spline(t_samples, 1)
    dy = y_spline(t_samples, 1)
    dz = z_spline(t_samples, 1)

    # Compute inter-point distances along the spline
    # In firmware, position = accumulated sum of forward_vector * delta_dist.
    # The spline positions represent that accumulated result.
    positions = np.column_stack([x_path, y_path, z_path])
    deltas = np.diff(positions, axis=0)
    distances = np.linalg.norm(deltas, axis=1)
    distances = np.insert(distances, 0, 0.0)  # First tick has zero displacement

    # Generate navigation rows
    nav_rows = []
    events = []

    # Degraded sections (time ranges in seconds)
    degraded_sections = [
        (40, 50, SENSOR_CONFLICT | ENCODER_ESTIMATED, 'ENCODER_SLIP'),
        (80, 90, IMU_ESTIMATED, 'IMU_STALE'),
        (120, 130, NAV_CRITICAL | DEPTH_VIRTUAL, 'DEPTH_VIRTUAL'),
        (160, 170, ENCODER_LOST, 'ENCODER_LOST'),
    ]

    # Start event
    events.append(EventRow(
        timestamp_ms=0,
        seq=0,
        tag='START',
        flags=0x00
    ))

    # Track last degradation state for event generation
    last_flags = 0x00

    for i in range(num_samples):
        timestamp_ms = i * TICK_PERIOD_MS
        t = i / TICK_RATE_HZ

        # Compute orientation from tangent
        tangent = np.array([dx[i], dy[i], dz[i]])
        qw, qx, qy, qz = compute_orientation_from_tangent(tangent)

        # Use spline position directly + small noise for realism
        position = positions[i]
        noisy_position = position + rng.normal(0, 0.001, 3)

        delta_dist = float(distances[i])
        angular_delta = float(delta_dist / ENCODER_WHEEL_RADIUS_M)

        # Determine flags based on degraded sections
        flags = 0x00
        for start_t, end_t, section_flags, tag_prefix in degraded_sections:
            if start_t <= t < end_t:
                flags |= section_flags

        # Generate events on flag transitions
        if flags != last_flags:
            if flags != 0x00:
                # Entering degraded state
                for start_t, end_t, section_flags, tag_prefix in degraded_sections:
                    if start_t <= t < end_t and (section_flags & flags):
                        events.append(EventRow(
                            timestamp_ms=timestamp_ms,
                            seq=i,
                            tag=f'{tag_prefix}_START',
                            flags=flags
                        ))
            else:
                # Exiting degraded state
                for start_t, end_t, section_flags, tag_prefix in degraded_sections:
                    if abs(t - end_t) < 0.01:  # Within one tick of end
                        events.append(EventRow(
                            timestamp_ms=timestamp_ms,
                            seq=i,
                            tag=f'{tag_prefix}_END',
                            flags=flags
                        ))
            last_flags = flags

        nav_rows.append(NavigationRow(
            timestamp_ms=timestamp_ms,
            seq=i,
            angular_delta=angular_delta,
            qw=float(qw), qx=float(qx), qy=float(qy), qz=float(qz),
            px=float(noisy_position[0]),
            py=float(noisy_position[1]),
            pz=float(noisy_position[2]),
            delta_dist=float(delta_dist),
            flags=flags
        ))

    # End event
    events.append(EventRow(
        timestamp_ms=(num_samples - 1) * TICK_PERIOD_MS,
        seq=num_samples - 1,
        tag='END',
        flags=0x00
    ))

    # Write files
    nav_path = output_dir / 'complex_cave_nav.csv'
    events_path = output_dir / 'complex_cave_events.csv'
    write_nav_csv(nav_rows, nav_path)
    write_events_csv(events, events_path)

    # Print summary
    final_pos = nav_rows[-1]
    total_distance = sum(row.delta_dist for row in nav_rows)
    min_depth = min(row.pz for row in nav_rows)
    max_depth = max(row.pz for row in nav_rows)

    print(f'\nComplex Cave Summary:')
    print(f'  Rows: {len(nav_rows)}')
    print(f'  Events: {len(events)}')
    print(f'  Duration: {duration_s}s')
    print(f'  Total distance: {total_distance:.2f}m')
    print(f'  Final position: ({final_pos.px:.2f}, {final_pos.py:.2f}, {final_pos.pz:.2f})')
    print(f'  Depth range: {max_depth:.2f}m to {abs(min_depth):.2f}m')
    print(f'  Files: {nav_path.name}, {events_path.name}')

    # Print degradation summary
    print(f'\n  Degraded sections:')
    for start_t, end_t, section_flags, tag_prefix in degraded_sections:
        print(f'    {tag_prefix}: {start_t}-{end_t}s (flags=0x{section_flags:02X})')


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Generate synthetic cave dive test data'
    )
    parser.add_argument(
        '--seed',
        type=int,
        default=42,
        help='Random seed for reproducibility (default: 42)'
    )
    parser.add_argument(
        '--output-dir',
        type=Path,
        default=Path(__file__).parent,
        help='Output directory for CSV files (default: same as script)'
    )

    args = parser.parse_args()

    # Create output directory if needed
    args.output_dir.mkdir(parents=True, exist_ok=True)

    # Initialize RNG
    rng = np.random.default_rng(args.seed)

    print(f'Generating synthetic test data (seed={args.seed})')
    print(f'Output directory: {args.output_dir}')

    # Generate scenarios
    generate_straight_dive(args.output_dir, rng)
    generate_complex_cave(args.output_dir, rng)

    print('\nGeneration complete.')


if __name__ == '__main__':
    main()
