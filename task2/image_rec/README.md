# Task 2 arrow recognition

A YOLO26n model that detects the left/right arrow on each Task 2 obstacle. It runs on the Pi with NCNN.

| Folder | Contents |
|---|---|
| `training/` | v1 scripts (`scripts/`) and v2 scripts (`v2/scripts/`): dataset building and augmentation, training, evaluation, model comparison, packaging |
| `deploy/` | The packaged NCNN model's inference script, tests, operating point and metadata |

The model weights (`model.ncnn.bin`, `.param`, `.pt`) and the dataset are not in this repo.

On the robot, arrow voting and ranging are in [`../rpi/perception.py`](../rpi/perception.py), and the laptop picture page is [`../rpi/arrow_server.py`](../rpi/arrow_server.py).
