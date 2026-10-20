# Task 2 algorithm

The Task 2 planner lives in [`../rpi/`](../rpi/) so that the Pi program runs from a single folder:

| File | Role |
|---|---|
| `final_run.py` | Staged run and route planning (`plan_to`): the C-curve plan round O1/O2 and back to the carpark |
| `geometry.py` | Robot footprint, swept-path collision checks |
| `course.py` | Course model: free space, O2 bounds |
| `sim.py` | Simulator used by the tests |

See [`../rpi/README.md`](../rpi/README.md) for how to run it.
