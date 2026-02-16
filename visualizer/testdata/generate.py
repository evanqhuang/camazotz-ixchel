#!/usr/bin/env python3
"""
Synthetic test data generator for cave dive visualization.

Generates CSV files matching the firmware's SDIO logger format with
a realistic 2-hour cave dive including sensor degradation.
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


def compute_orientation_from_tangent(tangent: np.ndarray) -> Tuple[float, float, float, float]:
    """
    Compute quaternion from path tangent vector.

    Args:
        tangent: [dx, dy, dz] normalized tangent vector

    Returns:
        (qw, qx, qy, qz) quaternion
    """
    # Normalize tangent
    norm = np.linalg.norm(tangent)
    if norm > 1e-9:
        tangent = tangent / norm
    else:
        # Zero tangent, default to identity orientation
        return (1.0, 0.0, 0.0, 0.0)

    # Compute euler angles
    heading = np.arctan2(tangent[1], tangent[0])  # yaw
    pitch = np.arctan2(-tangent[2], np.sqrt(tangent[0]**2 + tangent[1]**2))
    roll = 0.0

    return euler_to_quat(roll, pitch, heading)


def write_events_csv(events: List[EventRow], filepath: Path):
    """Write events CSV with firmware format."""
    with open(filepath, 'w') as f:
        # Header
        f.write('timestamp_ms,seq,tag,flags\n')

        # Data rows
        for event in events:
            f.write(f'{event.timestamp_ms},{event.seq},{event.tag},0x{event.flags:02X}\n')


def generate_full_cave_dive(output_dir: Path, rng: np.random.Generator):
    """
    Generate a realistic 2-hour Ponderosa-style Yucatan cenote cave dive.

    This is a one-way exploration/mapping dive with NO return path.
    The diver enters, maps the cave system, and the data records the full survey.
    """
    duration_s = 7200  # 2 hours
    num_samples = duration_s * TICK_RATE_HZ

    # Stick map with gradual bends at T-junctions (~45-50 deg turns) (time_s, x, y, z)
    waypoints = np.array([
        # Entry & Descent heading ENE (0-900s)
        (0,      0,    0,     0),       # Surface
        (60,     5,    1,    -3),       # Descent
        (200,    15,   3,   -10),       # Cave mouth
        (500,    38,   8,   -12),       # ENE passage
        (900,    60,  15,   -13),       # Approaching T1

        # T1: bend left — ENE to NNE (900-2400s)
        (1100,   68,  22,   -14),      # Passage curving left
        (1300,   74,  32,   -13),      # Through the bend
        (1600,   80,  52,   -15),      # Now heading NNE
        (1900,   84,  72,   -14),      # NNE passage
        (2200,   87,  92,   -16),      # Continuing NNE
        (2400,   88, 108,   -15),      # Approaching T2

        # T2: bend right — NNE to ENE (2400-4200s)
        (2600,   92, 118,   -16),      # Passage curving right
        (2800,  100, 126,   -14),      # Through the bend
        (3100,  118, 135,   -17),      # Now heading ENE
        (3400,  138, 142,   -18),      # ENE passage
        (3700,  158, 148,   -16),      # Continuing ENE
        (4000,  175, 153,   -19),      # Approaching T3
        (4200,  188, 156,   -17),      # Near T3

        # T3: bend left — ENE to NNE (4200-7200s)
        (4400,  196, 162,   -18),      # Passage curving left
        (4600,  202, 172,   -16),      # Through the bend
        (4900,  206, 190,   -19),      # Now heading NNE
        (5200,  210, 210,   -20),      # Deepest section
        (5500,  213, 228,   -18),      # NNE passage
        (5800,  216, 245,   -17),      # Continuing NNE
        (6200,  218, 268,   -19),      # Deep passage
        (6600,  220, 288,   -18),      # Continuing
        (7000,  222, 308,   -17),      # Near end
        (7200,  223, 318,   -18),      # End of survey
    ])

    # Create cubic splines for smooth path
    t_waypoints = waypoints[:, 0]
    x_spline = CubicSpline(t_waypoints, waypoints[:, 1])
    y_spline = CubicSpline(t_waypoints, waypoints[:, 2])
    z_spline = CubicSpline(t_waypoints, waypoints[:, 3])

    # Vectorized evaluation at all sample times
    t_samples = np.arange(num_samples) / TICK_RATE_HZ

    x_path = x_spline(t_samples)
    y_path = y_spline(t_samples)
    z_path = z_spline(t_samples)

    # Compute tangents (derivatives)
    dx = x_spline(t_samples, 1)
    dy = y_spline(t_samples, 1)
    dz = z_spline(t_samples, 1)

    # Compute positions and distances
    positions = np.column_stack([x_path, y_path, z_path])
    deltas = np.diff(positions, axis=0)
    distances = np.linalg.norm(deltas, axis=1)
    distances = np.insert(distances, 0, 0.0)  # First tick has zero displacement

    # Add small noise to positions
    noisy_positions = positions + rng.normal(0, 0.001, positions.shape)

    # Compute angular deltas
    angular_deltas = distances / ENCODER_WHEEL_RADIUS_M

    # Compute quaternions from tangents (vectorized)
    # We'll compute this in a loop since quaternion conversion is not easily vectorized
    quaternions = np.zeros((num_samples, 4))  # (qw, qx, qy, qz)
    for i in range(num_samples):
        tangent = np.array([dx[i], dy[i], dz[i]])
        quaternions[i] = compute_orientation_from_tangent(tangent)

    # Initialize flags array
    flags = np.zeros(num_samples, dtype=np.uint8)

    # Degraded sections spread across the full 7200s one-way path
    degraded_sections = [
        (400,   430,  SENSOR_CONFLICT | ENCODER_ESTIMATED, 'ENCODER_SLIP_ENTRY'),
        (900,   930,  IMU_ESTIMATED, 'IMU_STALE_SOUTH_PASSAGE'),
        (1600,  1640, DEPTH_UNVERIFIED, 'DEPTH_UNVERIFIED_GALLERY'),
        (2100,  2200, NAV_CRITICAL | ENCODER_ESTIMATED, 'RESTRICTION_NORTH'),
        (3200,  3260, IMU_LOST, 'IMU_LOST_JUNCTION'),
        (4200,  4280, DEPTH_VIRTUAL | NAV_CRITICAL, 'DEPTH_VIRTUAL_DEEP'),
        (5000,  5100, NAV_CRITICAL | ENCODER_ESTIMATED, 'RESTRICTION_PENETRATION'),
        (5800,  5830, SENSOR_CONFLICT, 'SENSOR_CONFLICT_SE'),
        (6500,  6560, ENCODER_LOST, 'ENCODER_LOST_DEEP'),
        (7000,  7050, IMU_ESTIMATED | DEPTH_UNVERIFIED, 'MULTI_DEGRADE_END'),
    ]

    # Apply degraded flags
    events = []
    events.append(EventRow(
        timestamp_ms=0,
        seq=0,
        tag='START',
        flags=0x00
    ))

    for start_t, end_t, section_flags, tag in degraded_sections:
        start_idx = int(start_t * TICK_RATE_HZ)
        end_idx = int(end_t * TICK_RATE_HZ)
        flags[start_idx:end_idx] |= section_flags

        # Add start event
        events.append(EventRow(
            timestamp_ms=start_idx * TICK_PERIOD_MS,
            seq=start_idx,
            tag=f'{tag}_START',
            flags=section_flags
        ))

        # Add end event
        if end_idx < num_samples:
            events.append(EventRow(
                timestamp_ms=end_idx * TICK_PERIOD_MS,
                seq=end_idx,
                tag=f'{tag}_END',
                flags=0x00
            ))

    # Add end event
    events.append(EventRow(
        timestamp_ms=(num_samples - 1) * TICK_PERIOD_MS,
        seq=num_samples - 1,
        tag='END',
        flags=0x00
    ))

    # Sort events by timestamp
    events.sort(key=lambda e: e.timestamp_ms)

    # Write navigation CSV using vectorized approach
    nav_path = output_dir / 'cave_dive_nav.nav'
    with open(nav_path, 'w') as f:
        # Header
        f.write('timestamp_ms,seq,angular_delta,qw,qx,qy,qz,px,py,pz,delta_dist,flags\n')

        # Build format strings for efficient writing
        timestamps_ms = np.arange(num_samples) * TICK_PERIOD_MS
        seqs = np.arange(num_samples)

        # Write data rows
        for i in range(num_samples):
            f.write(f'{timestamps_ms[i]},{seqs[i]},{angular_deltas[i]:.6f},'
                   f'{quaternions[i,0]:.6f},{quaternions[i,1]:.6f},'
                   f'{quaternions[i,2]:.6f},{quaternions[i,3]:.6f},'
                   f'{noisy_positions[i,0]:.4f},{noisy_positions[i,1]:.4f},'
                   f'{noisy_positions[i,2]:.4f},{distances[i]:.6f},0x{flags[i]:02X}\n')

    # Write events CSV
    events_path = output_dir / 'cave_dive_events.event'
    write_events_csv(events, events_path)

    # Print summary
    total_distance = np.sum(distances)
    min_depth = np.min(noisy_positions[:, 2])  # Most negative (deepest)
    max_depth = np.max(noisy_positions[:, 2])  # Least negative (shallowest)

    print(f'\nPonderosa-Style Cave Survey Summary:')
    print(f'  Rows: {num_samples}')
    print(f'  Events: {len(events)}')
    print(f'  Duration: {duration_s}s ({duration_s/60:.1f} min)')
    print(f'  Total distance: {total_distance:.2f}m')
    print(f'  Final position: ({noisy_positions[-1,0]:.2f}, {noisy_positions[-1,1]:.2f}, {noisy_positions[-1,2]:.2f})')
    print(f'  Depth range: {max_depth:.2f}m (shallowest) to {min_depth:.2f}m (deepest)')
    print(f'  Files: {nav_path.name}, {events_path.name}')

    # Print degradation summary
    print(f'\n  Degraded sections ({len(degraded_sections)} total):')
    for start_t, end_t, section_flags, tag in degraded_sections:
        duration = end_t - start_t
        print(f'    {tag}: {start_t}-{end_t}s ({duration}s, flags=0x{section_flags:02X})')


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Generate synthetic 2-hour cave dive test data'
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

    print(f'Generating synthetic 2-hour cave dive (seed={args.seed})')
    print(f'Output directory: {args.output_dir}')

    # Generate cave dive
    generate_full_cave_dive(args.output_dir, rng)

    print('\nGeneration complete.')


if __name__ == '__main__':
    main()
