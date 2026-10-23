Task 2 arrow V2: YOLO26n detection, fixed 640x640 NCNN

Three local classes: 0 bullseye -> marker, 1 left -> 39, 2 right -> 38.
Legacy IDs are strings. Do not send local class indices as robot image IDs.
Use this detection runner; a former segmentation decoder is not interchangeable.
The V2 runner retains all target detections and prioritizes arrows over bullseyes
in mixed scenes. A bullseye must not replace a visible arrow's requested turn.

From this package folder, in the existing Pi Python environment:
  python inference.py IMAGE.jpg --model . --evidence-dir evidence
  python inference.py IMAGE.jpg --model . --observation-conf 0.35 --evidence-dir evidence
  python -m unittest test_inference.py

The operating_point.json default confirmation threshold is 0.7.
Early observation confidence may expose weaker candidates; it never changes the
confirmation threshold. Candidate counts are not accepted-ID accuracy rates.

Construct Detector once per serial worker; predict accepts original upright BGR
camera frames. For each selected goal obstacle, pass its current image-pixel ROI
and distinct consecutive live frame IDs to:
  ArrowConfirmation(conf=detector.conf,
                    min_arrow_input_side=detector.min_arrow_input_side)
Reset on a new goal. Confirmation needs three associated observations and at
least 12 pixels across the smaller arrow dimension in BOTH the supplied raw
frame and network input. Use original camera frames at least 640 pixels wide;
external enlargement of low-resolution frames cannot restore missing detail.
Bullseye never produces a turn. A single-image result never confirms a turn.

This package observes images only. It does not drive, navigate, park, or stop the
robot. Save one representative frame for each recognized goal obstacle. Evidence
contains an unmodified raw input, a separate annotated image, JSON, and a tiled
contact sheet. The team still handles the competition evidence submission.

The supplied evaluation report is copied unchanged to validation.json.
Model metrics, even perfect synthetic results, do not establish a physical
detection range or live driving reliability. Camera distance, angle, exposure,
motion blur, Pi latency, and temporal confirmation require a separate live test.
Host timing is not Pi timing. No evaluation, tuning, or robot action runs during
packaging. This remains a candidate for live Pi validation.

requirements.txt pins the model runtime used during development. Use the Pi's
platform-compatible torch/torchvision and OpenCV installation. versions.json
describes the packaging host; it is not an instruction to install its CUDA build
on the Pi. manifest.json contains SHA256 checksums for every packaged payload.
