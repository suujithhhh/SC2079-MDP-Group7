"""Decision tests import no inference framework; evidence test needs OpenCV/Pillow."""
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from inference import (ArrowConfirmation, select_arrow, save_evidence, operating_confidence,
                       operating_min_arrow_side, Detector)


def target(class_id=1, bbox=(20, 20, 60, 60), confidence=0.9):
    return {"class_id": class_id, "name": {0: "bullseye", 1: "left", 2: "right"}[class_id],
            "confidence": confidence, "bbox": list(bbox), "input_min_side": 40.0}


class DecisionsTest(unittest.TestCase):
    roi = (0, 0, 200, 200)

    def test_three_associated_frames_required(self):
        confirmation = ArrowConfirmation()
        for frame in range(3):
            result = confirmation.update([target(bbox=(20 + frame, 20, 60 + frame, 60))],
                                         self.roi, frame_id=frame)
            self.assertEqual(result["confirmed"], frame == 2)
            self.assertEqual(result["direction"], "left" if frame == 2 else None)

    def test_missing_conflicting_and_bullseye_reset_streak(self):
        for interrupt in ([], [target(2)], [target(0)], [target(), target(2)]):
            confirmation = ArrowConfirmation()
            confirmation.update([target()], self.roi)
            confirmation.update([target()], self.roi)
            self.assertFalse(confirmation.update(interrupt, self.roi)["confirmed"])
            self.assertEqual(confirmation.count, 0)
            self.assertFalse(confirmation.update([target()], self.roi)["confirmed"])

    def test_no_largest_box_or_highest_confidence_guess(self):
        rows = [target(bbox=(0, 0, 100, 100)), target(2, (110, 20, 150, 60), 0.5)]
        self.assertIsNone(select_arrow(rows, self.roi)["direction"])
        self.assertEqual(select_arrow(rows, (105, 0, 160, 80))["direction"], "right")
        self.assertIsNone(select_arrow([target(0)], self.roi)["direction"])

    def test_unassociated_target_restarts(self):
        confirmation = ArrowConfirmation()
        confirmation.update([target()], self.roi)
        confirmation.update([target()], self.roi)
        result = confirmation.update([target(bbox=(140, 140, 180, 180))], self.roi)
        self.assertFalse(result["confirmed"])
        self.assertEqual(result["streak"], 1)

    def test_duplicate_frame_does_not_confirm(self):
        confirmation = ArrowConfirmation()
        confirmation.update([target()], self.roi, frame_id=7)
        for _ in range(4):
            result = confirmation.update([target()], self.roi, frame_id=7)
        self.assertEqual(result["reason"], "duplicate_frame")
        self.assertFalse(result["confirmed"])
        self.assertEqual(confirmation.count, 1)

    def test_threshold_and_roi_validation(self):
        self.assertEqual(select_arrow([target(confidence=0.2)], self.roi)["reason"], "missing")
        with self.assertRaises(ValueError):
            select_arrow([], (0, 0, 0, 10))

    def test_tiny_arrow_remains_observed_but_cannot_confirm(self):
        confirmation = ArrowConfirmation()
        tiny = target()
        tiny["input_min_side"] = 8.15
        for _ in range(3):
            result = confirmation.update([tiny], self.roi)
            self.assertEqual(result["reason"], "arrow_too_small")
            self.assertIsNone(result["direction"])
            self.assertFalse(result["confirmed"])
            self.assertEqual(result["detection"], tiny)
        self.assertEqual(confirmation.update([target()], self.roi)["streak"], 1)
        bullseye = target(0)
        bullseye["input_min_side"] = 3
        self.assertEqual(select_arrow([bullseye], self.roi)["reason"], "bullseye")

    def test_unknown_arrow_size_abstains_unless_explicit_optout(self):
        unknown = target()
        del unknown["input_min_side"]
        self.assertEqual(select_arrow([unknown], self.roi)["reason"], "target_size_unknown")
        self.assertEqual(select_arrow([unknown], self.roi, min_arrow_input_side=0)["direction"], "left")
        confirmation = ArrowConfirmation(min_arrow_input_side=0)
        for _ in range(3):
            result = confirmation.update([unknown], self.roi)
        self.assertTrue(result["confirmed"])

    def test_upscaled_low_resolution_arrow_cannot_pass(self):
        arrow = target()
        arrow.update(input_min_side=40.0, source_min_side=10.0)
        result = select_arrow([arrow], self.roi)
        self.assertEqual(result["reason"], "arrow_too_small")
        self.assertIsNone(result["direction"])
        for value in (0, -1, float("nan"), float("inf"), True):
            arrow["source_min_side"] = value
            self.assertEqual(select_arrow([arrow], self.roi)["reason"], "target_size_unknown")
        arrow["source_min_side"] = 12.0
        self.assertEqual(select_arrow([arrow], self.roi)["direction"], "left")

    def test_early_observation_does_not_weaken_confirmation(self):
        import numpy as np
        detector = Detector.__new__(Detector)  # no model/framework import needed
        detector.conf, detector.imgsz = 0.85, 640
        recorded_thresholds = []

        def predict(frame, **kwargs):
            recorded_thresholds.append(kwargs["conf"])
            rows = [[20, 20, 60, 60, 0.48, 1]] if kwargs["conf"] <= 0.48 else []
            data = SimpleNamespace(cpu=lambda: SimpleNamespace(tolist=lambda: rows))
            return [SimpleNamespace(boxes=SimpleNamespace(data=data))]

        detector.model = SimpleNamespace(predict=predict)
        frame = np.zeros((640, 640, 3), dtype=np.uint8)
        self.assertEqual(detector.predict(frame), [])
        candidates = detector.predict(frame, observation_conf=0.35)
        self.assertEqual(len(candidates), 1)
        self.assertFalse(candidates[0]["meets_confirmation_confidence"])
        self.assertEqual(recorded_thresholds, [0.85, 0.35])
        self.assertEqual(detector.conf, 0.85)
        confirmation = ArrowConfirmation(conf=detector.conf)
        for _ in range(3):
            decision = confirmation.update(candidates, self.roi)
        self.assertFalse(decision["confirmed"])
        self.assertIsNone(decision["direction"])
        for invalid in (0, float("nan"), float("inf"), True):
            with self.assertRaises(ValueError):
                detector.predict(frame, observation_conf=invalid)

    def test_packaged_arrow_size_gate(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parents[1]) as temp:
            folder = Path(temp)
            self.assertEqual(operating_min_arrow_side(folder), 12.0)
            for value in (16, 0):
                (folder / "operating_point.json").write_text(json.dumps({"min_arrow_input_side": value}))
                self.assertEqual(operating_min_arrow_side(folder), value)
            for value in (-1, float("nan"), float("inf"), True, "12"):
                (folder / "operating_point.json").write_text(json.dumps({"min_arrow_input_side": value}))
                with self.assertRaises(ValueError):
                    operating_min_arrow_side(folder)

    def test_packaged_confidence_and_explicit_override(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parents[1]) as temp:
            folder = Path(temp)
            self.assertEqual(operating_confidence(folder), 0.45)
            (folder / "operating_point.json").write_text('{"confidence": 0.85}')
            self.assertEqual(operating_confidence(folder), 0.85)
            self.assertEqual(operating_confidence(folder, 0.65), 0.65)
            (folder / "operating_point.json").write_text('{"confidence": 0}')
            with self.assertRaises(ValueError):
                operating_confidence(folder)
            for invalid in (float("nan"), float("inf"), True, "0.85"):
                with self.assertRaises(ValueError):
                    operating_confidence(folder, invalid)

    def test_evidence_preserves_raw_and_writes_separate_overlay(self):
        import cv2
        import numpy as np
        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parents[1]) as temp:
            folder = Path(temp)
            source = folder / "camera.jpg"
            cv2.imwrite(str(source), np.full((80, 80, 3), 255, np.uint8))
            original = source.read_bytes()
            result = save_evidence(source, [target()], {"reason": "pending_confirmation"},
                                   folder / "evidence")
            event = Path(result)
            metadata = json.loads((event / "metadata.json").read_text())
            self.assertEqual((event / metadata["raw_file"]).read_bytes(), original)
            self.assertNotEqual((event / metadata["annotated_file"]).read_bytes(), original)
            self.assertTrue((folder / "evidence/contact_sheet.jpg").is_file())


if __name__ == "__main__":
    unittest.main()
