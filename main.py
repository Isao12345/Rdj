"""
==============================================================================
Root Entry Point: main.py
Description: Starts the RoboMaster EP Round 2 autonomous navigation system.
Usage:
    python main.py
    python main.py --mode sim --step-delay 0.05
    python main.py --mode real
==============================================================================
"""

import sys
import os

# Add project root directory to sys.path to enable robomaster_round2 package imports
root_dir = os.path.dirname(os.path.abspath(__file__))
if root_dir not in sys.path:
    sys.path.insert(0, root_dir)

from robomaster_round2.main import run

if __name__ == "__main__":
    run()
