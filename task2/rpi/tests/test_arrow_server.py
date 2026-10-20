"""arrow_server.py (the PC page for the two Task 2 arrow pictures) and final_run's ArrowUploader. Local HTTP only."""

from __future__ import annotations

import json
import tempfile
import threading
import unittest
import urllib.error
import urllib.request
from pathlib import Path

import arrow_server
import sim
from final_run import ArrowUploader, FinalRun

JPEG = b"\xff\xd8\xff\xe0" + b"picture" + b"\xff\xd9"


class ServerTest(unittest.TestCase):
    def setUp(self):
        self.server = arrow_server.create_server("127.0.0.1", 0, verbose=False)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.base = f"http://127.0.0.1:{self.server.server_address[1]}"

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()

    def post(self, query: str, body: bytes = JPEG, content_type: str = "image/jpeg"):
        request = urllib.request.Request(f"{self.base}/images?{query}", data=body, method="POST",
                                         headers={"Content-Type": content_type})
        try:
            with urllib.request.urlopen(request, timeout=5) as reply:
                return reply.status, json.loads(reply.read())
        except urllib.error.HTTPError as exc:
            return exc.code, json.loads(exc.read())

    def get(self, path: str):
        with urllib.request.urlopen(f"{self.base}{path}", timeout=5) as reply:
            return reply.status, reply.headers.get_content_type(), reply.read()

    def test_two_pictures_listed_and_served(self):
        self.assertEqual(self.post("run_id=r1&obstacle_id=1&arrow=LEFT")[0], 200)
        self.assertEqual(self.post("run_id=r1&obstacle_id=2&arrow=RIGHT")[0], 200)
        _, kind, body = self.get("/images")
        listing = json.loads(body)
        self.assertEqual(kind, "application/json")
        self.assertEqual(listing["run_id"], "r1")
        self.assertEqual([(i["obstacle_id"], i["arrow"]) for i in listing["images"]], [(1, "LEFT"), (2, "RIGHT")])
        status, kind, body = self.get(listing["images"][1]["url"])
        self.assertEqual((status, kind, body), (200, "image/jpeg", JPEG))

    def test_a_new_run_clears_the_old_pair(self):
        self.post("run_id=r1&obstacle_id=1&arrow=LEFT")
        self.post("run_id=r1&obstacle_id=2&arrow=LEFT")
        self.post("run_id=r2&obstacle_id=1&arrow=RIGHT")
        listing = json.loads(self.get("/images")[2])
        self.assertEqual((listing["run_id"], [i["obstacle_id"] for i in listing["images"]]), ("r2", [1]))

    def test_bad_uploads_are_refused(self):
        for query, body, content_type, status in (
                ("run_id=r&obstacle_id=3&arrow=LEFT", JPEG, "image/jpeg", 400),
                ("run_id=r&obstacle_id=1&arrow=UP", JPEG, "image/jpeg", 400),
                ("obstacle_id=1&arrow=LEFT", JPEG, "image/jpeg", 400),
                ("run_id=r&obstacle_id=1&arrow=LEFT", b"not a jpeg", "image/jpeg", 400),
                ("run_id=r&obstacle_id=1&arrow=LEFT", JPEG, "text/plain", 415)):
            with self.subTest(query=query, content_type=content_type):
                code, reply = self.post(query, body, content_type)
                self.assertEqual((code, reply["type"]), (status, "IMAGE_ERROR"))
        self.assertEqual(json.loads(self.get("/images")[2])["images"], [])

    def test_the_page(self):
        status, kind, body = self.get("/")
        self.assertEqual((status, kind), (200, "text/html"))
        self.assertIn(b"Obstacle 1", body)
        self.assertIn(b"Obstacle 2", body)


class UploaderTest(unittest.TestCase):
    def test_sends_the_picture_to_the_page(self):
        server = arrow_server.create_server("127.0.0.1", 0, verbose=False)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        try:
            with tempfile.TemporaryDirectory() as folder:
                path = Path(folder) / "o1_arrow0_boxes.jpg"
                path.write_bytes(JPEG)
                lines: list[str] = []
                uploader = ArrowUploader(f"127.0.0.1:{server.server_address[1]}", "run-a", log=lines.append)
                uploader.send(1, "RIGHT", path)
                uploader.close()
            self.assertEqual(uploader.sent, [1], lines)
            self.assertEqual(server.images.get("1"), JPEG)
            self.assertEqual(server.images.listing()["run_id"], "run-a")
        finally:
            server.shutdown()
            server.server_close()

    def test_an_absent_pc_is_logged_never_raised(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "o2.jpg"
            path.write_bytes(JPEG)
            lines: list[str] = []
            uploader = ArrowUploader("127.0.0.1:9", "run-b", log=lines.append, timeout_s=1.0)
            uploader.send(2, "LEFT", path)
            uploader.close()
        self.assertEqual(uploader.sent, [])
        self.assertTrue(any("NOT sent" in line for line in lines), lines)

    def test_default_port(self):
        self.assertEqual(ArrowUploader("192.168.1.5", "r").url, "http://192.168.1.5:5001/images")
        self.assertEqual(ArrowUploader("http://laptop:6000/", "r").url, "http://laptop:6000/images")


class RunKeepsArrowLooksTest(unittest.TestCase):
    def test_both_arrow_looks_are_kept_for_the_display(self):
        w = sim.World("LR", 120.0, 60.0, 120.0, sim.FIXTURE_FREE, walls=False)
        stm = sim.FakeStm(w, sim.fixture_params(120.0), front_us=sim.SimFrontUs())
        run = FinalRun(stm, sim.FakeCamera(stm), log=lambda _: None, wall_hug=False)
        self.assertTrue(run.run())
        self.assertEqual(sorted(run.arrow_looks), [1, 2])


if __name__ == "__main__":
    unittest.main()
