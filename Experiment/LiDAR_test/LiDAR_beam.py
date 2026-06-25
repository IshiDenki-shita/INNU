#!/usr/bin/env python3
"""Measures sensor scanning speed"""

from rplidar import RPLidar

PORT_NAME = "/dev/ttyUSB0"

lidar = RPLidar(PORT_NAME)

try:
    for point in lidar.iter_scans():
        print(point)
except KeyboardInterrupt:
    lidar.stop()
    lidar.disconnect()
