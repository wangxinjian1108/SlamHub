#!/usr/bin/env python3
"""
Launch FAST-LIO2 on a rosbag file and collect outputs.

Designed to run inside the FAST_LIO Docker container where ROS Noetic
and FAST-LIO2 are already built in ~/catkin_ws.

Usage:
    python 02_run_slam.py input.bag --config config/fastlio_at128p.yaml --output-dir output/slam/
"""

import argparse
import shutil
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).parent.parent))

# FAST-LIO2 default output location
FASTLIO_PCD_DIR = Path.home() / "catkin_ws" / "src" / "FAST_LIO" / "PCD"


def check_ros_available():
    """Verify ROS is importable and roscore can be reached."""
    try:
        import rospy  # noqa: F401
        import roslaunch  # noqa: F401
    except ImportError:
        print("Error: ROS Python packages not found.")
        print("This script must run inside the FAST_LIO Docker container")
        print("with ROS Noetic sourced (source /opt/ros/noetic/setup.bash).")
        sys.exit(1)


def is_roscore_running() -> bool:
    """Check if roscore is already running."""
    try:
        result = subprocess.run(
            ["rostopic", "list"],
            capture_output=True, timeout=5,
        )
        return result.returncode == 0
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False


def start_roscore() -> subprocess.Popen:
    """Start roscore as a background process."""
    print("Starting roscore...")
    proc = subprocess.Popen(
        ["roscore"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    # Wait for roscore to be ready
    for _ in range(30):
        time.sleep(0.5)
        if is_roscore_running():
            print("roscore is running.")
            return proc
    print("Warning: roscore may not have started properly.")
    return proc


def launch_fastlio(config_path: Path) -> subprocess.Popen:
    """Launch FAST-LIO2 via roslaunch with the given config."""
    import roslaunch

    # Clear previous PCD outputs
    if FASTLIO_PCD_DIR.exists():
        shutil.rmtree(FASTLIO_PCD_DIR)
    FASTLIO_PCD_DIR.mkdir(parents=True, exist_ok=True)

    print(f"Launching FAST-LIO2 with config: {config_path}")
    proc = subprocess.Popen(
        [
            "roslaunch", "fast_lio", "mapping.launch",
            f"config_file:={config_path.resolve()}",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    # Give FAST-LIO2 time to initialize
    time.sleep(5)
    return proc


def play_bag(bag_path: Path, wait: bool = True) -> subprocess.Popen:
    """Play a rosbag file."""
    print(f"Playing bag: {bag_path}")
    proc = subprocess.Popen(
        ["rosbag", "play", str(bag_path), "--clock"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    if wait:
        proc.wait()
        print("Bag playback finished.")
    return proc


def collect_outputs(output_dir: Path) -> None:
    """Copy FAST-LIO2 outputs to the specified output directory."""
    output_dir.mkdir(parents=True, exist_ok=True)

    if not FASTLIO_PCD_DIR.exists():
        print(f"Warning: FAST-LIO2 output directory not found: {FASTLIO_PCD_DIR}")
        return

    # Copy all files from FAST-LIO2 PCD output
    files_copied = 0
    for src_file in FASTLIO_PCD_DIR.iterdir():
        dst_file = output_dir / src_file.name
        if src_file.is_file():
            shutil.copy2(src_file, dst_file)
            files_copied += 1
        elif src_file.is_dir():
            if dst_file.exists():
                shutil.rmtree(dst_file)
            shutil.copytree(src_file, dst_file)
            files_copied += 1

    print(f"Copied {files_copied} items from {FASTLIO_PCD_DIR} to {output_dir}")


def main():
    parser = argparse.ArgumentParser(
        description="Launch FAST-LIO2 on a rosbag and collect outputs"
    )
    parser.add_argument("bag", type=Path, help="Input rosbag file")
    parser.add_argument(
        "--config", type=Path, required=True,
        help="FAST-LIO2 config YAML file"
    )
    parser.add_argument(
        "--output-dir", type=Path, default=Path("output/slam"),
        help="Directory to copy SLAM outputs to (default: output/slam/)"
    )
    parser.add_argument(
        "--wait-after", type=float, default=5.0,
        help="Seconds to wait after bag finishes for SLAM to complete (default: 5)"
    )

    args = parser.parse_args()

    # Validate inputs
    if not args.bag.exists():
        print(f"Error: Bag file not found: {args.bag}")
        sys.exit(1)
    if not args.config.exists():
        print(f"Error: Config file not found: {args.config}")
        sys.exit(1)

    # Check ROS availability
    check_ros_available()

    # Start roscore if needed
    roscore_proc = None
    if not is_roscore_running():
        roscore_proc = start_roscore()

    try:
        # Launch FAST-LIO2
        fastlio_proc = launch_fastlio(args.config)

        # Play the bag
        play_bag(args.bag, wait=True)

        # Wait for SLAM to finish processing
        print(f"Waiting {args.wait_after}s for SLAM processing to complete...")
        time.sleep(args.wait_after)

        # Terminate FAST-LIO2
        print("Stopping FAST-LIO2...")
        fastlio_proc.terminate()
        fastlio_proc.wait(timeout=10)

    except KeyboardInterrupt:
        print("\nInterrupted. Cleaning up...")
        fastlio_proc.terminate()
    finally:
        # Stop roscore if we started it
        if roscore_proc is not None:
            roscore_proc.terminate()
            roscore_proc.wait(timeout=5)

    # Collect outputs
    collect_outputs(args.output_dir)
    print("Done.")


if __name__ == "__main__":
    main()
