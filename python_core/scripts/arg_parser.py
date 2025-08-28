import argparse


def generate_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--verbose', '-v', help='verbose output', action="store_true")

    # Robot
    parser.add_argument('--field', help='preview robot position on the field', action="store_true")

    # Camera
    parser.add_argument('--camera', help='preview camera object detection', action="store_true")
    parser.add_argument('--ball', help='preview ball detection parameters', action="store_true")
    parser.add_argument('--blue', help='preview blue goal detection parameters', action="store_true")
    parser.add_argument('--yellow', help='preview yellow goal detection parameters', action="store_true")

    # Lidar
    parser.add_argument('--lidar', help='preview lidar point cloud', action="store_true")

    args = parser.parse_args()

    return args