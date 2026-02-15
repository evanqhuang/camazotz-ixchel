#!/usr/bin/env python3
"""PyVista-based 3D cave dive visualization CLI."""

import argparse
import sys
from pathlib import Path

from csv_parser import parse_nav_csv, parse_events_csv
from dive_stats import compute_stats, print_stats
from scene import DiveScene
from timeline import DiveTimeline
from exporter import DiveExporter


def main():
    """CLI entry point."""
    parser = argparse.ArgumentParser(
        description='Camazotz 3D Cave Dive Visualizer',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument(
        'csv_file',
        type=str,
        help='Path to navigation CSV file',
    )

    parser.add_argument(
        '--events',
        type=str,
        help='Path to events CSV file',
    )

    parser.add_argument(
        '--color',
        type=str,
        choices=['depth', 'speed', 'flags'],
        default='depth',
        help='Color mode (default: depth)',
    )

    parser.add_argument(
        '--tube-radius',
        type=float,
        default=0.15,
        help='Tube radius in meters (default: 0.15)',
    )

    parser.add_argument(
        '--downsample',
        type=int,
        default=10,
        help='Downsample factor for spline (default: 10)',
    )

    parser.add_argument(
        '--arrows',
        action='store_true',
        help='Show orientation arrows',
    )

    parser.add_argument(
        '--arrow-interval',
        type=int,
        default=100,
        help='Arrow spacing in samples (default: 100)',
    )

    parser.add_argument(
        '--animate',
        action='store_true',
        help='Enable animated playback',
    )

    parser.add_argument(
        '--speed',
        type=float,
        default=1.0,
        help='Playback speed multiplier (default: 1.0)',
    )

    parser.add_argument(
        '--follow',
        action='store_true',
        help='Camera follow mode during animation',
    )

    parser.add_argument(
        '--screenshot',
        type=str,
        help='Save screenshot to path',
    )

    parser.add_argument(
        '--gif',
        type=str,
        help='Export orbital GIF to path',
    )

    parser.add_argument(
        '--video',
        type=str,
        help='Export MP4 video to path',
    )

    parser.add_argument(
        '--export-mesh',
        type=str,
        help='Export mesh to path (PLY/STL/OBJ)',
    )

    parser.add_argument(
        '--no-display',
        action='store_true',
        help="Don't show interactive window",
    )

    parser.add_argument(
        '--stats',
        action='store_true',
        help='Print statistics to terminal',
    )

    args = parser.parse_args()

    csv_path = Path(args.csv_file)
    if not csv_path.exists():
        print(f"Error: CSV file not found: {csv_path}", file=sys.stderr)
        sys.exit(1)

    print(f"Loading navigation data from {csv_path}...")
    try:
        data = parse_nav_csv(str(csv_path))
    except Exception as e:
        print(f"Error parsing CSV: {e}", file=sys.stderr)
        sys.exit(1)

    print(f"Loaded {data['count']} navigation samples")

    events = None
    if args.events:
        events_path = Path(args.events)
        if not events_path.exists():
            print(f"Warning: Events file not found: {events_path}", file=sys.stderr)
        else:
            print(f"Loading events from {events_path}...")
            try:
                events = parse_events_csv(str(events_path))
                print(f"Loaded {len(events)} events")
            except Exception as e:
                print(f"Warning: Error parsing events CSV: {e}", file=sys.stderr)

    if args.stats:
        stats = compute_stats(data)
        print_stats(stats)

    print("Building 3D scene...")
    scene = DiveScene(
        data,
        tube_radius=args.tube_radius,
        downsample=args.downsample,
    )

    scene.build(
        color_mode=args.color,
        show_arrows=args.arrows,
        arrow_interval=args.arrow_interval,
        events=events,
    )

    print("Scene ready")

    exporter = DiveExporter(scene, data)

    if args.screenshot:
        print(f"Saving screenshot to {args.screenshot}...")
        exporter.screenshot(args.screenshot)

    if args.gif:
        print(f"Exporting GIF to {args.gif}...")
        exporter.gif(args.gif, n_frames=120, orbit=True)

    if args.video:
        print(f"Exporting video to {args.video}...")
        exporter.video(args.video, n_frames=300, fps=30)

    if args.export_mesh:
        print(f"Exporting mesh to {args.export_mesh}...")
        exporter.export_mesh(args.export_mesh)

    if args.animate:
        print("Starting animated playback...")
        timeline = DiveTimeline(scene, data, speed=args.speed, follow=args.follow)
        timeline.animate()
    elif not args.no_display:
        print("Showing interactive window...")
        scene.show()

    print("Done")


if __name__ == '__main__':
    main()
