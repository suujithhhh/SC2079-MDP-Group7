Black-circle v3 NCNN candidate

Circle image ID40 (class29); six image ID16 (class5). All new circles are black.
Use network imgsz=640, batch1, FP32, confidence0.35. Keep the complete folder.
Run using the existing Pi Python environment:
  python inference.py sample_circle_160.jpg --conf 0.35
  python inference.py sample_six_160.jpg --conf 0.35
Expected IDs: 40 and 16 respectively.

This candidate adds black-circle distance and background variation using
scene-separated uploaded screenshots and existing training sources. Includes
matching-background sixes, E symbols, other classes and blank cards.
Existing lowres160 and circle_v2 packages are preserved.

Validation figures and limitations are in validation.json. Generated variants
are not independent captures. Screenshots include annotation pixels, so test
live Pi camera frames before deciding to replace the deployed model.
The packaged Python runner uses the same confidence/dedup/largest-box behavior
as v2. Optional C/L/R filtering was not evaluated here.

Verified results (NCNN, confidence0.35, v2 -> v3):
  Reserved screenshots: 1/5 -> 5/5 correct ID40.
  Their reduced160 versions: 1/5 -> 5/5 correct ID40.
  Original distant screenshot: no detection -> ID40, confidence0.948.
  Original closer annotated screenshot: ID24 -> ID40, confidence0.764.
  New small/dense-pattern circle stress: 58/120 -> 116/120.
  New six controls: 72/80 -> 80/80; E controls: 29/40 -> 40/40.
  New blank cards: 80/80 -> 80/80 correct no-detection.
  Existing original160 non-circle IDs: 739/741 -> 739/741.
  Original335-image test correct returns: 329/335 -> 329/335.
No per-class returned-ID regression on original validation or test.
Four synthetic circle cases remain below confidence0.35. This is not a claim
of perfect circle detection on every background/distance.

Extra diagnostic: python inference.py sample_far_circle_160.png --conf 0.35
Expected ID40. This is a reduced copy of the supplied distant screenshot,
reserved from new training. The five screenshot sources are from two capture
families and were used for checkpoint selection, not an independent test set.

Selected epoch2. Three complete epochs were saved; optimization was stopped
in epoch4 after a suitable checkpoint passed the deployment-head gates.
The default PyTorch prediction head differs from the NCNN export. To compare
best.pt with NCNN in Ultralytics, request end2end=False, rect=False and imgsz640.
