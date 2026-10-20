import math

from models import Pose

# Rear reference (rear-axle midpoint), facing North: the margin-inflated
# footprint spans x 9.5-30.5 cm, y 9-35 cm, inside the 40 x 40 cm start zone.
DEFAULT_START_POSE = Pose(20, 10, math.pi / 2)


def main() -> None:
    try:
        from simulator import MDPSimulator
    except ModuleNotFoundError as error:
        if error.name != "_tkinter":
            raise
        raise SystemExit(
            "this Python installation doesnt include tkinter. on macOS, run "
            "'/usr/bin/python3 main.py', or install the tkinter package matching "
            "your Python version."
        ) from None

    obstacles = []
    MDPSimulator(DEFAULT_START_POSE, obstacles).run()


if __name__ == "__main__":
    main()
