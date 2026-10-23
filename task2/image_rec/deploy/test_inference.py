"""Decision tests import no inference framework; evidence test needs OpenCV/Pillow."""
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from inference import (ArrowConfirmation, select_arrow, save_evidence, operating_confidence,
                       operating_min_arrow_side, Detector, order_detections)


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

    def test_single_arrow_with_higher_confidence_bullseyes(self):
        bullseyes = [target(0, (90, 20, 130, 60), .99),
                    target(0, (140, 20, 180, 60), .98)]
        for cls, count in ((1, 1), (1, 2), (2, 2)):
            with self.subTest(class_id=cls, bullseyes=count):
                arrow = target(cls, confidence=.9)
                rows = [*bullseyes[:count], arrow]
                observation = select_arrow(rows, self.roi, conf=.85)
                self.assertEqual(observation["direction"], "left" if cls == 1 else "right")
                self.assertIs(observation["detection"], arrow)
                confirmation = ArrowConfirmation(conf=.85)
                for frame in range(3):
                    decision = confirmation.update(rows, self.roi, frame_id=frame)
                    self.assertEqual(decision["confirmed"], frame == 2)
                    self.assertEqual(decision["direction"], observation["direction"] if frame == 2 else None)

    def test_bullseyes_alone_never_turn(self):
        rows = [target(0, confidence=.99), target(0, (90, 20, 130, 60), .98)]
        confirmation = ArrowConfirmation()
        for frame in range(4):
            decision = confirmation.update(rows, self.roi, frame_id=frame)
            self.assertFalse(decision["confirmed"])
            self.assertIsNone(decision["direction"])
            self.assertEqual(decision["reason"], "bullseye")
            self.assertEqual(decision["streak"], 0)

    def test_multiple_arrows_with_bullseyes_still_abstain(self):
        for second_class in (1, 2):
            with self.subTest(second_arrow=second_class):
                rows = [target(), target(second_class, (90, 20, 130, 60), .88),
                        target(0, (140, 20, 180, 60), .99)]
                result = select_arrow(rows, self.roi, conf=.85)
                self.assertIsNone(result["direction"])
                self.assertEqual(result["reason"], "ambiguous_targets")
                confirmation = ArrowConfirmation(conf=.85)
                confirmation.update([target()], self.roi)
                confirmation.update([target()], self.roi)
                self.assertFalse(confirmation.update(rows, self.roi)["confirmed"])
                self.assertEqual(confirmation.count, 0)

    def test_bullseyes_do_not_bypass_arrow_confidence_or_size_gates(self):
        bullseye = target(0, (100, 20, 140, 60), .99)
        for arrow, reason in ((target(confidence=.5), "bullseye"),
                              ({**target(), "input_min_side": 8.}, "arrow_too_small"),
                              ({**target(), "source_min_side": 8.}, "arrow_too_small")):
            with self.subTest(reason=reason, arrow=arrow):
                confirmation = ArrowConfirmation(conf=.85)
                for frame in range(3):
                    result = confirmation.update([bullseye, arrow], self.roi, frame_id=frame)
                    self.assertFalse(result["confirmed"])
                    self.assertIsNone(result["direction"])
                    self.assertEqual(result["reason"], reason)

    def test_arrow_must_be_inside_selected_roi_even_with_bullseye_inside(self):
        rows = [target(bbox=(250, 20, 290, 60)), target(0, confidence=.99)]
        observation = select_arrow(rows, self.roi)
        self.assertIsNone(observation["direction"])
        self.assertEqual(observation["reason"], "bullseye")

    def test_all_detections_retained_with_deterministic_arrow_first_order(self):
        import numpy as np
        detector = Detector.__new__(Detector)
        detector.conf, detector.imgsz = .85, 640
        rows = [[140, 20, 180, 60, .99, 0], [90, 20, 130, 60, .97, 0],
                [20, 20, 60, 60, .90, 1]]
        data = SimpleNamespace(cpu=lambda: SimpleNamespace(tolist=lambda: rows))
        detector.model = SimpleNamespace(predict=lambda *a, **kw: [SimpleNamespace(boxes=SimpleNamespace(data=data))])
        detections = detector.predict(np.zeros((640, 640, 3), dtype=np.uint8))
        self.assertEqual([d["class_id"] for d in detections], [1, 0, 0])
        self.assertEqual([d["confidence"] for d in detections], [.90, .99, .97])
        self.assertEqual(len(detections), len(rows))
        ties = [target(2, (90, 20, 130, 60), .9), target(1, confidence=.9),
                target(0, (140, 20, 180, 60), .99), target(2, (60, 90, 100, 130), .95)]
        ordered = order_detections(ties)
        self.assertEqual([d["confidence"] for d in ordered], [.95, .9, .9, .99])
        self.assertEqual(ordered[1]["bbox"], [20, 20, 60, 60])
        self.assertEqual(ordered, order_detections(list(reversed(ties))))
        self.assertEqual(len(ties), len(ordered))

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
