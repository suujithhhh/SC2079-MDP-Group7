"""The mirror fallback of the vision worker (perception.unmirror): detections made on a mirrored frame mapped back.
Run from the task2 folder: python3 -m unittest discover -s tests"""

from __future__ import annotations

import unittest

from perception import Detection, decide_arrow, has_arrow, unmirror


class UnmirrorTest(unittest.TestCase):
    def test_a_left_arrow_on_the_mirrored_frame_is_a_right_arrow(self):
        seen = [Detection("left", 0.93, 100.0, 50.0, 300.0, 250.0)]
        back = unmirror(seen, 640)
        self.assertEqual(back, [Detection("right", 0.93, 340.0, 50.0, 540.0, 250.0)])
        self.assertEqual(decide_arrow([back])[0], "RIGHT")

    def test_other_labels_keep_their_name(self):
        self.assertEqual(unmirror([Detection("bullseye", 0.8, 0.0, 0.0, 40.0, 40.0)], 640)[0].label, "bullseye")

    def test_has_arrow_needs_the_confidence(self):
        self.assertFalse(has_arrow([Detection("right", 0.3, 0, 0, 1, 1)]))
        self.assertTrue(has_arrow([Detection("right", 0.6, 0, 0, 1, 1)]))


if __name__ == "__main__":
    unittest.main()
