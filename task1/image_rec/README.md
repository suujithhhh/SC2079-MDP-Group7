# Task 1 image recognition

A YOLO26s-seg model that reads the symbol on each obstacle (image IDs 11-40 plus the bullseye marker). It is trained here and runs on the Pi with NCNN.

| Folder | Contents |
|---|---|
| `training/` | Dataset building and synthesis (`circle_data.py`, `gen_*.py`, `expand_circle_photos.py`), training and fine-tuning (`train_*.py`, `finetune_*.py`, `model.ipynb`), evaluation (`measure_*`, `report_*`, `retest_*`), and NCNN export (`export_*.py`) |
| `deploy/` | The packaged NCNN model's inference script and metadata (`inference.py`, `model_ncnn.py`, `manifest.json`) |

The model weights (`model.ncnn.bin`, `.param`, `.pt`) and the dataset are not in this repo.

On the robot, the camera and inference code that runs on `SNAP` is [`../rpi/vision.py`](../rpi/vision.py).
