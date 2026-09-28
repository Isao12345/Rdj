"""CLI: python3 -m robomaster_round2.Slam.main

คง run_real/run_simulation ไว้ให้โค้ดที่ import จาก main เดิมใช้งานได้
"""

import argparse

from .runtime import run_real
from .simulation import run_simulation
from .setting import maze_setting as settings


def main(argv=None):
    parser = argparse.ArgumentParser(description="Explore and record a grid maze")
    parser.add_argument("--mode", choices=("sim", "real"), default="sim")
    parser.add_argument("--output", default=None)
    parser.add_argument("--scan-only", action="store_true",
                        help="Real hardware: scan four directions without moving chassis")
    parser.add_argument("--alignment", action="store_true",
                        help="Real hardware: scan once and run alignment without exploring")
    parser.add_argument("--sharp-check", action="store_true",
                        help="Real hardware: log Sharp readings without moving until Ctrl+C")
    parser.add_argument("--max-steps", type=int,
                        help="Stop after this many cell moves (real mode)")
    args = parser.parse_args(argv)
    if args.max_steps is not None and args.max_steps < 1:
        parser.error("--max-steps must be at least 1")
    if args.mode == "sim" and (args.scan_only or args.alignment or args.sharp_check or args.max_steps is not None):
        parser.error("hardware test flags require --mode real")
    if sum((args.scan_only, args.alignment, args.sharp_check,
            args.max_steps is not None)) > 1:
        parser.error("choose only one hardware test flag")
    if args.mode == "real":
        if args.sharp_check:
            run_real(args.output or settings.MAP_OUTPUT_PATH, sharp_check=True)
        elif args.alignment:
            run_real(args.output or settings.MAP_OUTPUT_PATH, alignment_only=True)
        else:
            run_real(args.output or settings.MAP_OUTPUT_PATH, args.scan_only, args.max_steps)
    else:
        run_simulation(args.output or "maps/slam_demo.json")


if __name__ == "__main__":
    main()
