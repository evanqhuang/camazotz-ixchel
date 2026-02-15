"""CSV parsing for navigation and event log files."""

import numpy as np


def parse_nav_csv(filepath: str) -> dict:
    """Parse navigation CSV file into numpy arrays.

    Args:
        filepath: Path to navigation CSV file

    Returns:
        Dict with numpy arrays:
            - timestamps: uint64 array of timestamps in ms
            - sequences: uint32 array of sequence numbers
            - angular_deltas: float32 array of angular deltas
            - quaternions: float32 array of shape (N, 4) [w,x,y,z]
            - positions: float64 array of shape (N, 3) [x,y,z] in firmware coords
            - delta_distances: float32 array of delta distances
            - flags: uint8 array of flag bitfields
            - count: int, number of data points
    """
    data_lines = []

    with open(filepath, 'r') as f:
        header = f.readline().strip()
        if not header.startswith('timestamp_ms'):
            raise ValueError(f"Invalid CSV header: {header}")

        for line in f:
            line = line.strip()
            if not line:
                continue

            parts = line.split(',')
            if len(parts) != 12:
                continue

            data_lines.append(parts)

    count = len(data_lines)
    if count == 0:
        raise ValueError("No data found in CSV file")

    timestamps = np.zeros(count, dtype=np.uint64)
    sequences = np.zeros(count, dtype=np.uint32)
    angular_deltas = np.zeros(count, dtype=np.float32)
    quaternions = np.zeros((count, 4), dtype=np.float32)
    positions = np.zeros((count, 3), dtype=np.float64)
    delta_distances = np.zeros(count, dtype=np.float32)
    flags = np.zeros(count, dtype=np.uint8)

    for i, parts in enumerate(data_lines):
        timestamps[i] = int(parts[0])
        sequences[i] = int(parts[1])
        angular_deltas[i] = float(parts[2])
        quaternions[i, 0] = float(parts[3])  # w
        quaternions[i, 1] = float(parts[4])  # x
        quaternions[i, 2] = float(parts[5])  # y
        quaternions[i, 3] = float(parts[6])  # z
        positions[i, 0] = float(parts[7])    # px
        positions[i, 1] = float(parts[8])    # py
        positions[i, 2] = float(parts[9])    # pz
        delta_distances[i] = float(parts[10])
        flags[i] = int(parts[11], 16)  # Parse hex flag

    return {
        'timestamps': timestamps,
        'sequences': sequences,
        'angular_deltas': angular_deltas,
        'quaternions': quaternions,
        'positions': positions,
        'delta_distances': delta_distances,
        'flags': flags,
        'count': count,
    }


def parse_events_csv(filepath: str) -> list[dict]:
    """Parse events CSV file.

    Args:
        filepath: Path to events CSV file

    Returns:
        List of event dicts with keys: timestamp_ms, seq, tag, flags
    """
    events = []

    with open(filepath, 'r') as f:
        header = f.readline().strip()
        if not header.startswith('timestamp_ms'):
            raise ValueError(f"Invalid events CSV header: {header}")

        for line in f:
            line = line.strip()
            if not line:
                continue

            parts = line.split(',')
            if len(parts) < 3:
                continue

            event = {
                'timestamp_ms': int(parts[0]),
                'seq': int(parts[1]),
                'tag': parts[2],
                'flags': int(parts[3], 16) if len(parts) > 3 else 0,
            }
            events.append(event)

    return events
