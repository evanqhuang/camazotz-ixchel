"""Dive statistics computation and formatting."""

import numpy as np
from flag_decoder import flag_severity


def compute_stats(data: dict) -> dict:
    """Compute dive statistics from parsed navigation data.

    Args:
        data: Dict from parse_nav_csv()

    Returns:
        Dict of statistics
    """
    positions = data['positions']
    timestamps = data['timestamps']
    flags = data['flags']
    delta_distances = data['delta_distances']

    duration_ms = timestamps[-1] - timestamps[0]
    duration_s = duration_ms / 1000.0

    total_distance = np.sum(delta_distances)

    depths = -positions[:, 2]
    max_depth = np.max(depths)

    avg_speed = total_distance / duration_s if duration_s > 0 else 0.0

    flagged_count = np.sum(flags != 0)
    flagged_percent = 100.0 * flagged_count / data['count']

    warning_count = sum(1 for f in flags if flag_severity(f) == 'warning')
    critical_count = sum(1 for f in flags if flag_severity(f) == 'critical')

    bbox_min = np.min(positions, axis=0)
    bbox_max = np.max(positions, axis=0)
    bbox_size = bbox_max - bbox_min

    return {
        'duration_s': duration_s,
        'duration_ms': duration_ms,
        'total_distance': total_distance,
        'max_depth': max_depth,
        'avg_speed': avg_speed,
        'sample_count': data['count'],
        'flagged_count': flagged_count,
        'flagged_percent': flagged_percent,
        'warning_count': warning_count,
        'critical_count': critical_count,
        'bbox_min': bbox_min,
        'bbox_max': bbox_max,
        'bbox_size': bbox_size,
    }


def print_stats(stats: dict) -> None:
    """Pretty-print dive statistics to console."""
    print("\n=== Dive Statistics ===")
    print(f"Duration:        {stats['duration_s']:>10.2f} s")
    print(f"Total Distance:  {stats['total_distance']:>10.2f} m")
    print(f"Max Depth:       {stats['max_depth']:>10.2f} m")
    print(f"Avg Speed:       {stats['avg_speed']:>10.4f} m/s")
    print(f"Sample Count:    {stats['sample_count']:>10d}")
    print(f"\n=== Data Quality ===")
    print(f"Flagged Samples: {stats['flagged_count']:>10d} ({stats['flagged_percent']:.1f}%)")
    print(f"  Warnings:      {stats['warning_count']:>10d}")
    print(f"  Critical:      {stats['critical_count']:>10d}")
    print(f"\n=== Bounding Box ===")
    print(f"Min:             ({stats['bbox_min'][0]:>7.2f}, {stats['bbox_min'][1]:>7.2f}, {stats['bbox_min'][2]:>7.2f})")
    print(f"Max:             ({stats['bbox_max'][0]:>7.2f}, {stats['bbox_max'][1]:>7.2f}, {stats['bbox_max'][2]:>7.2f})")
    print(f"Size:            ({stats['bbox_size'][0]:>7.2f}, {stats['bbox_size'][1]:>7.2f}, {stats['bbox_size'][2]:>7.2f})")
    print()
