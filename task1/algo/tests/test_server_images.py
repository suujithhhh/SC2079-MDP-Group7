"""RAW camera images, RPi to PC (`PROTOCOL.md` section 7.1): `image_store.py`
and the `POST /images`, `GET /images`, `GET /images/<id>` endpoints.

A real `PlanningHTTPServer` runs on an ephemeral port for each HTTP test. The
photos are display-only: these tests also check that an upload never touches
the planner state.
"""

import http.client
import json
import socket
import subprocess
import sys
import threading
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import image_store as store_module  # noqa: E402
import rpi_image_test_client as pi_client  # noqa: E402
import server as server_module  # noqa: E402

HOST = "127.0.0.1"


def jpeg(size: int = 300, fill: bytes = b"\x11") -> bytes:
    """A body the PC accepts as a complete JPEG: FF D8 FF ... FF D9."""
    assert size >= 8
    return b"\xff\xd8\xff\xe0" + fill * (size - 6) + b"\xff\xd9"


class ValidationRules(unittest.TestCase):
    def test_obstacle_id_is_decimal_digits_from_1_to_8(self):
        for good in range(1, 9):
            self.assertEqual(store_module.parse_obstacle_id(str(good)), good)
        for bad in ("0", "9", "08", "-1", "+3", " 3", "3 ", "1.5", "a", "", "１", "99", "١"):
            with self.subTest(bad=bad), self.assertRaises(store_module.ImageRejected) as caught:
                store_module.parse_obstacle_id(bad)
            self.assertEqual(caught.exception.status, 400)
        for not_text in (None, 3, 3.0, True, ["3"]):
            with self.subTest(not_text=not_text), self.assertRaises(store_module.ImageRejected):
                store_module.parse_obstacle_id(not_text)

    def test_target_id_is_two_digits_from_11_to_40(self):
        for good in range(11, 41):
            self.assertEqual(store_module.parse_target_id(str(good)), good)
        for bad in ("0", "1", "10", "41", "99", "100", "011", "-22", "+22", " 22", "22 ", "22.0", "2e1", "a", "",
                    "２２", "٢٢", "0x16"):
            with self.subTest(bad=bad), self.assertRaises(store_module.ImageRejected) as caught:
                store_module.parse_target_id(bad)
            self.assertEqual(caught.exception.status, 400)
        for not_text in (None, 22, 22.0, True, ["22"]):
            with self.subTest(not_text=not_text), self.assertRaises(store_module.ImageRejected):
                store_module.parse_target_id(not_text)

    def test_mission_id_rules(self):
        self.assertEqual(store_module.validate_mission_id("web-auto-1"), "web-auto-1")
        self.assertEqual(store_module.validate_mission_id("x" * 128), "x" * 128)
        self.assertEqual(store_module.validate_mission_id("mission 7 é"), "mission 7 é")
        for bad in ("", "x" * 129, "a\nb", "a\x00b", "a\x7fb", "\t", None, 5, ["m"]):
            with self.subTest(bad=bad), self.assertRaises(store_module.ImageRejected) as caught:
                store_module.validate_mission_id(bad)
            self.assertEqual(caught.exception.status, 400)

    def test_only_a_complete_jpeg_within_the_size_limit_passes(self):
        store_module.validate_jpeg(jpeg(8))
        store_module.validate_jpeg(jpeg(store_module.MAX_IMAGE_BYTES))
        cases = {
            "empty": (b"", 400),
            "png": (b"\x89PNG\r\n\x1a\n" + b"\x00" * 50 + b"\xff\xd9", 400),
            "truncated (no end marker)": (b"\xff\xd8\xff\xe0" + b"\x11" * 100, 400),
            "no start marker": (b"\x00\xd8\xff\xe0" + b"\x11" * 100 + b"\xff\xd9", 400),
            "text": (b"hello world, not a picture", 400),
            "just the two markers": (b"\xff\xd8\xff\xd9", 400),
            "too large": (jpeg(store_module.MAX_IMAGE_BYTES + 1), 413),
        }
        for name, (data, status) in cases.items():
            with self.subTest(name), self.assertRaises(store_module.ImageRejected) as caught:
                store_module.validate_jpeg(data)
            self.assertEqual(caught.exception.status, status)


class ImageStoreRules(unittest.TestCase):
    def make(self):
        ticks = iter(range(1000, 2000))
        return store_module.ImageStore(clock=lambda: float(next(ticks)))

    def test_empty_store(self):
        listing = self.make().list_images()
        self.assertEqual(listing, {"type": "IMAGE_LIST", "mission_id": None, "revision": 0, "images": []})

    def test_a_stored_photo_is_listed_and_readable(self):
        store = self.make()
        data = jpeg(500)
        reply = store.store("m1", 3, data)
        self.assertEqual(reply, {"image_id": "3-1", "mission_id": "m1", "obstacle_id": 3, "target_id": None,
                                 "bytes": 500, "replaced": False})
        listing = store.list_images()
        self.assertEqual(listing["mission_id"], "m1")
        self.assertEqual(listing["revision"], 1)
        self.assertEqual([i["image_id"] for i in listing["images"]], ["3-1"])
        self.assertEqual(listing["images"][0]["bytes"], 500)
        self.assertIsNone(listing["images"][0]["target_id"])
        self.assertEqual(store.get("3-1"), data)

    def test_a_target_id_is_stored_and_listed_and_never_changes_the_picture(self):
        store = self.make()
        data = jpeg(200)
        reply = store.store("m1", 3, data, 22)
        self.assertEqual(reply["target_id"], 22)
        self.assertEqual([(i["obstacle_id"], i["target_id"]) for i in store.list_images()["images"]], [(3, 22)])
        self.assertEqual(store.get(reply["image_id"]), data)

    def test_a_newer_photo_replaces_the_target_id_too(self):
        store = self.make()
        store.store("m1", 3, jpeg(100))
        self.assertTrue(store.store("m1", 3, jpeg(100), 22)["replaced"])
        self.assertEqual(store.list_images()["images"][0]["target_id"], 22)
        store.store("m1", 3, jpeg(100), 31)
        self.assertEqual(store.list_images()["images"][0]["target_id"], 31)
        store.store("m1", 3, jpeg(100))  # re-uploaded with no ID: the old ID is not kept
        self.assertIsNone(store.list_images()["images"][0]["target_id"])

    def test_the_same_target_id_may_appear_on_two_obstacles(self):
        store = self.make()
        store.store("m1", 1, jpeg(64), 22)
        store.store("m1", 2, jpeg(64), 22)
        self.assertEqual([i["target_id"] for i in store.list_images()["images"]], [22, 22])

    def test_one_photo_per_obstacle_and_the_newer_replaces_the_older(self):
        store = self.make()
        store.store("m1", 2, jpeg(100, b"\x01"))
        reply = store.store("m1", 2, jpeg(120, b"\x02"))
        self.assertTrue(reply["replaced"])
        self.assertEqual([i["obstacle_id"] for i in store.list_images()["images"]], [2])
        self.assertIsNone(store.get("2-1"), "the replaced photo is gone")
        self.assertEqual(store.get(reply["image_id"]), jpeg(120, b"\x02"))

    def test_a_different_mission_starts_a_new_set(self):
        store = self.make()
        store.store("m1", 1, jpeg(100))
        store.store("m1", 2, jpeg(100))
        reply = store.store("m2", 5, jpeg(100))
        self.assertFalse(reply["replaced"])
        listing = store.list_images()
        self.assertEqual(listing["mission_id"], "m2")
        self.assertEqual([i["obstacle_id"] for i in listing["images"]], [5])
        self.assertIsNone(store.get("1-1"))

    def test_never_more_than_eight_photos(self):
        store = self.make()
        for _ in range(3):
            for obstacle in range(1, 9):
                store.store("m1", obstacle, jpeg(64))
        self.assertEqual(len(store.list_images()["images"]), 8)

    def test_photos_are_listed_in_the_order_they_were_taken(self):
        store = self.make()
        for obstacle in (5, 2, 7):
            store.store("m1", obstacle, jpeg(64))
        self.assertEqual([i["obstacle_id"] for i in store.list_images()["images"]], [5, 2, 7])
        store.store("m1", 5, jpeg(64))  # retaken: now the newest
        self.assertEqual([i["obstacle_id"] for i in store.list_images()["images"]], [2, 7, 5])

    def test_image_ids_never_repeat_and_unknown_or_malformed_ids_read_nothing(self):
        store = self.make()
        ids = [store.store("m1", 1, jpeg(64))["image_id"] for _ in range(4)]
        self.assertEqual(len(set(ids)), 4)
        for bad in ("", "1", "1-", "-1", "../1-1", "1-1/", "1-1\n", "a-1", "1-1x", "100-1", None, 5):
            with self.subTest(bad=bad):
                self.assertIsNone(store.get(bad))

    def test_a_rejected_photo_changes_nothing(self):
        store = self.make()
        store.store("m1", 1, jpeg(64))
        before = store.list_images()
        for args in (("m2", 1, b"not a jpeg"), ("", 1, jpeg(64)), ("m1", 9, jpeg(64)), ("m1", True, jpeg(64)),
                     ("m2", 1, jpeg(64), 10), ("m2", 1, jpeg(64), 41), ("m2", 1, jpeg(64), True),
                     ("m2", 1, jpeg(64), "22"), ("m2", 1, jpeg(64), 22.0)):
            with self.subTest(args=args[:2]), self.assertRaises(store_module.ImageRejected):
                store.store(*args)
        self.assertEqual(store.list_images(), before)


class _ServerTestCase(unittest.TestCase):
    """A real PlanningHTTPServer on an ephemeral port for each test."""

    def setUp(self):
        self.server = server_module.create_server(host=HOST, port=0, verbose=False)
        self.port = self.server.server_address[1]
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=5)

    def request(self, method, path, body=None, headers=None):
        conn = http.client.HTTPConnection(HOST, self.port, timeout=10)
        try:
            conn.request(method, path, body=body, headers=headers or {})
            response = conn.getresponse()
            raw = response.read()
            return response.status, dict(response.getheaders()), raw
        finally:
            conn.close()

    def upload(self, mission_id="m1", obstacle_id=3, data=None, content_type="image/jpeg", query=None):
        data = jpeg() if data is None else data
        if query is None:
            query = f"mission_id={mission_id}&obstacle_id={obstacle_id}"
        status, headers, raw = self.request(
            "POST", f"/images?{query}", body=data,
            headers={"Content-Type": content_type, "Content-Length": str(len(data))},
        )
        return status, json.loads(raw)

    def listing(self):
        status, _, raw = self.request("GET", "/images")
        self.assertEqual(status, 200)
        return json.loads(raw)


class UploadAndRead(_ServerTestCase):
    def test_a_valid_upload_is_stored_listed_and_served_back_byte_for_byte(self):
        data = jpeg(1500, b"\x42")
        status, body = self.upload("run-7", 3, data)
        self.assertEqual(status, 200)
        self.assertEqual(body["type"], "IMAGE_STORED")
        self.assertEqual({k: body[k] for k in ("mission_id", "obstacle_id", "bytes", "replaced")},
                         {"mission_id": "run-7", "obstacle_id": 3, "bytes": 1500, "replaced": False})
        listing = self.listing()
        self.assertEqual(listing["type"], "IMAGE_LIST")
        self.assertEqual(listing["mission_id"], "run-7")
        self.assertEqual([i["image_id"] for i in listing["images"]], [body["image_id"]])
        status, headers, raw = self.request("GET", f"/images/{body['image_id']}")
        self.assertEqual(status, 200)
        self.assertEqual(raw, data)
        self.assertEqual(headers["Content-Type"], "image/jpeg")
        self.assertEqual(headers["X-Content-Type-Options"], "nosniff")
        self.assertEqual(headers["Cache-Control"], "no-store")
        self.assertEqual(headers["Content-Length"], "1500")

    def test_a_target_id_is_stored_listed_and_echoed(self):
        status, body = self.upload(query="mission_id=m1&obstacle_id=3&target_id=22")
        self.assertEqual((status, body["type"], body["target_id"]), (200, "IMAGE_STORED", 22))
        self.assertEqual([(i["obstacle_id"], i["target_id"]) for i in self.listing()["images"]], [(3, 22)])

    def test_without_a_target_id_it_is_null(self):
        status, body = self.upload("m1", 3)
        self.assertEqual((status, body["target_id"]), (200, None))
        self.assertEqual([i["target_id"] for i in self.listing()["images"]], [None])

    def test_a_second_upload_adds_the_target_id_and_one_without_clears_it(self):
        self.upload("m1", 3)
        status, body = self.upload(query="mission_id=m1&obstacle_id=3&target_id=17")
        self.assertEqual((status, body["replaced"]), (200, True))
        images = self.listing()["images"]
        self.assertEqual([(i["obstacle_id"], i["target_id"], i["image_id"]) for i in images],
                         [(3, 17, body["image_id"])])
        self.upload("m1", 3)
        self.assertEqual([i["target_id"] for i in self.listing()["images"]], [None])

    def test_json_replies_are_not_cached(self):
        _, headers, _ = self.request("GET", "/images")
        self.assertEqual(headers["Cache-Control"], "no-store")
        self.assertTrue(headers["Content-Type"].startswith("application/json"))

    def test_a_percent_encoded_mission_id_round_trips(self):
        status, body = self.upload(query="mission_id=web%2Dauto%20%C3%A9&obstacle_id=1")
        self.assertEqual(status, 200)
        self.assertEqual(body["mission_id"], "web-auto é")
        self.assertEqual(self.listing()["mission_id"], "web-auto é")

    def test_retaking_a_photo_replaces_it_and_a_new_mission_clears_the_old_ones(self):
        _, first = self.upload("m1", 2, jpeg(200, b"\x01"))
        _, second = self.upload("m1", 2, jpeg(220, b"\x02"))
        self.assertTrue(second["replaced"])
        status, _, _ = self.request("GET", f"/images/{first['image_id']}")
        self.assertEqual(status, 404)
        self.upload("m1", 4)
        self.assertEqual(sorted(i["obstacle_id"] for i in self.listing()["images"]), [2, 4])
        self.upload("m2", 6)
        listing = self.listing()
        self.assertEqual((listing["mission_id"], [i["obstacle_id"] for i in listing["images"]]), ("m2", [6]))

    def test_empty_listing_before_any_upload(self):
        listing = self.listing()
        self.assertEqual((listing["mission_id"], listing["images"], listing["revision"]), (None, [], 0))

    def test_unknown_and_malformed_image_ids_are_404_with_a_json_error(self):
        self.upload("m1", 1)
        for path in ("/images/9-9", "/images/abc", "/images/", "/images/../server.py", "/images/1-1/extra",
                     "/images/%2e%2e%2fserver.py", "/images/1-1%00"):
            with self.subTest(path=path):
                status, headers, raw = self.request("GET", path)
                self.assertEqual(status, 404)
                self.assertEqual(json.loads(raw)["type"], "IMAGE_ERROR")

    def test_an_upload_never_touches_the_planner_state(self):
        before = self.server.get_v2_state()
        self.upload("m1", 1)
        self.upload("m1", 2)
        self.assertEqual(self.server.get_v2_state(), before)
        status, _, raw = self.request("GET", "/state/v2")
        self.assertEqual(json.loads(raw)["status"], "idle")


class UploadRejections(_ServerTestCase):
    def assertRejected(self, status, expected_status, error_part=None):
        code, body = status
        self.assertEqual(code, expected_status, body)
        self.assertEqual(body["type"], "IMAGE_ERROR")
        if error_part:
            self.assertIn(error_part, body["error"])
        self.assertEqual(self.listing()["images"], [], "nothing was stored")

    def test_the_content_type_must_be_exactly_image_jpeg(self):
        for content_type in ("image/png", "application/json", "text/plain", "application/octet-stream", "image/jpg"):
            with self.subTest(content_type=content_type):
                self.assertRejected(self.upload(content_type=content_type), 415, "image/jpeg")
        status, _, raw = self.request("POST", "/images?mission_id=m1&obstacle_id=1", body=jpeg(),
                                      headers={"Content-Length": str(len(jpeg()))})
        self.assertEqual(status, 415)

    def test_bad_or_missing_parameters_are_400(self):
        queries = [
            "", "mission_id=m1", "obstacle_id=3", "mission_id=&obstacle_id=3",
            "mission_id=m1&obstacle_id=0", "mission_id=m1&obstacle_id=9", "mission_id=m1&obstacle_id=x",
            "mission_id=m1&obstacle_id=1.5", "mission_id=m1&obstacle_id=", "mission_id=m1&obstacle_id=%2B3",
            "mission_id=m1&mission_id=m2&obstacle_id=3", "mission_id=m1&obstacle_id=3&obstacle_id=4",
            f"mission_id={'x' * 129}&obstacle_id=3", "mission_id=a%0Ab&obstacle_id=3", "mission_id=a%00b&obstacle_id=3",
        ]
        for query in queries:
            with self.subTest(query=query[:40]):
                self.assertRejected(self.upload(query=query), 400)

    def test_a_bad_target_id_is_400_and_stores_nothing(self):
        for value in ("0", "10", "41", "99", "011", "-22", "%2B22", "22.0", "x", "", "%2022", "22%20", "1e1"):
            with self.subTest(target_id=value):
                self.assertRejected(self.upload(query=f"mission_id=m1&obstacle_id=3&target_id={value}"), 400,
                                    "target_id")
        self.assertRejected(self.upload(query="mission_id=m1&obstacle_id=3&target_id=22&target_id=23"), 400,
                            "target_id")

    def test_a_bad_target_id_does_not_replace_the_earlier_photo(self):
        self.upload(query="mission_id=m1&obstacle_id=3&target_id=22")
        before = self.listing()
        self.upload(query="mission_id=m1&obstacle_id=3&target_id=99")
        self.assertEqual(self.listing(), before)

    def test_a_body_that_is_not_a_complete_jpeg_is_400(self):
        for name, data in {
            "png": b"\x89PNG\r\n\x1a\n" + b"\x00" * 50,
            "truncated jpeg": b"\xff\xd8\xff\xe0" + b"\x11" * 200,
            "text": b"not a picture at all",
            "start only": b"\xff\xd8\xff",
        }.items():
            with self.subTest(name):
                self.assertRejected(self.upload(data=data), 400, "JPEG")

    def test_a_missing_or_zero_content_length_is_400(self):
        for headers in ({"Content-Type": "image/jpeg", "Content-Length": "0"},
                        {"Content-Type": "image/jpeg", "Content-Length": "abc"}):
            with self.subTest(headers=headers):
                conn = http.client.HTTPConnection(HOST, self.port, timeout=10)
                try:
                    conn.putrequest("POST", "/images?mission_id=m1&obstacle_id=1")
                    for key, value in headers.items():
                        conn.putheader(key, value)
                    conn.endheaders()
                    response = conn.getresponse()
                    body = json.loads(response.read())
                finally:
                    conn.close()
                self.assertEqual((response.status, body["type"]), (400, "IMAGE_ERROR"))

    def test_an_oversize_body_is_413_from_the_header_alone(self):
        limit = store_module.MAX_IMAGE_BYTES
        request = (
            f"POST /images?mission_id=m1&obstacle_id=1 HTTP/1.1\r\nHost: {HOST}\r\n"
            f"Content-Type: image/jpeg\r\nContent-Length: {limit + 1}\r\n\r\n"
        ).encode("ascii")
        with socket.create_connection((HOST, self.port), timeout=10) as sock:
            sock.sendall(request)  # no body is ever sent: the server must not wait for it
            chunks = []
            while True:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                chunks.append(chunk)
        reply = b"".join(chunks).decode("utf-8")
        self.assertTrue(reply.startswith("HTTP/1.0 413") or reply.startswith("HTTP/1.1 413"), reply[:60])
        self.assertIn("Connection: close", reply)
        self.assertIn("IMAGE_ERROR", reply)
        self.assertEqual(self.listing()["images"], [])

    def test_exactly_the_limit_is_accepted(self):
        status, body = self.upload("m1", 1, jpeg(store_module.MAX_IMAGE_BYTES))
        self.assertEqual((status, body["bytes"]), (200, store_module.MAX_IMAGE_BYTES))

    def test_a_rejected_upload_leaves_earlier_photos_alone(self):
        self.upload("m1", 1)
        before = self.listing()
        self.upload("m2", 2, data=b"not a jpeg")
        self.upload("m2", 2, content_type="text/plain")
        self.upload("m2", 99)
        self.assertEqual(self.listing(), before)


class OtherEndpointsStillWork(_ServerTestCase):
    def test_health_state_v2_and_the_pages_are_unaffected(self):
        status, _, raw = self.request("GET", "/health")
        self.assertEqual((status, json.loads(raw)["status"]), (200, "ok"))
        status, _, raw = self.request("GET", "/state/v2")
        self.assertEqual(status, 200)
        for path, marker in (("/", b"auto.js"), ("/gallery.js", b"selectGalleryImages")):
            status, headers, body = self.request("GET", path)
            self.assertEqual(status, 200, path)
            self.assertIn(marker, body, path)

    def test_post_to_other_paths_is_still_404(self):
        status, _, raw = self.request("POST", "/images/extra", body=b"x", headers={"Content-Type": "image/jpeg"})
        self.assertEqual(status, 404)
        status, _, raw = self.request("POST", "/nope", body=b"x", headers={"Content-Type": "image/jpeg"})
        self.assertEqual(status, 404)


class FakePiClient(_ServerTestCase):
    """`rpi_image_test_client.py`, the stand-in for the RPi's camera."""

    def run_client(self, *extra):
        script = Path(__file__).resolve().parents[1] / "rpi_image_test_client.py"
        return subprocess.run(
            [sys.executable, str(script), "--host", HOST, "--port", str(self.port), *extra],
            capture_output=True, text=True, timeout=60,
        )

    def test_the_built_in_demo_jpeg_is_a_real_picture_the_pc_accepts(self):
        data = pi_client.demo_jpeg()
        store_module.validate_jpeg(data)
        self.assertLess(len(data), 5000)
        status, reply = pi_client.send_image(f"http://{HOST}:{self.port}/images", "m1", 4, data)
        self.assertEqual((status, reply["type"], reply["obstacle_id"]), (200, "IMAGE_STORED", 4))

    def test_the_command_line_sends_a_photo_and_exits_zero(self):
        done = self.run_client("--mission-id", "run-1", "--obstacle-id", "5", "--demo")
        self.assertEqual(done.returncode, 0, done.stderr)
        self.assertIn("IMAGE_STORED", done.stdout)
        self.assertEqual([i["obstacle_id"] for i in self.listing()["images"]], [5])

    def test_the_command_line_can_send_a_target_id(self):
        done = self.run_client("--mission-id", "run-1", "--obstacle-id", "5", "--target-id", "22", "--demo")
        self.assertEqual(done.returncode, 0, done.stderr)
        self.assertIn("target ID 22", done.stdout)
        self.assertEqual([(i["obstacle_id"], i["target_id"]) for i in self.listing()["images"]], [(5, 22)])

    def test_a_bad_target_id_exits_one(self):
        done = self.run_client("--mission-id", "run-1", "--obstacle-id", "5", "--target-id", "50", "--demo")
        self.assertEqual(done.returncode, 1)
        self.assertIn("IMAGE_ERROR", done.stdout)
        self.assertEqual(self.listing()["images"], [])

    def test_a_file_is_sent_byte_for_byte(self):
        import tempfile
        data = jpeg(2000, b"7")
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "photo.jpg"
            path.write_bytes(data)
            done = self.run_client("--mission-id", "run-1", "--obstacle-id", "2", "--image", str(path))
        self.assertEqual(done.returncode, 0, done.stderr)
        image_id = self.listing()["images"][0]["image_id"]
        status, _, raw = self.request("GET", f"/images/{image_id}")
        self.assertEqual((status, raw), (200, data))

    def test_a_rejected_photo_exits_one(self):
        done = self.run_client("--mission-id", "run-1", "--obstacle-id", "9", "--demo")
        self.assertEqual(done.returncode, 1)
        self.assertIn("IMAGE_ERROR", done.stdout)
        self.assertEqual(self.listing()["images"], [])

    def test_an_unreachable_pc_exits_nonzero_with_a_message(self):
        script = Path(__file__).resolve().parents[1] / "rpi_image_test_client.py"
        done = subprocess.run(
            [sys.executable, str(script), "--host", HOST, "--port", "1", "--mission-id", "m", "--obstacle-id", "1",
             "--demo", "--timeout", "3"],
            capture_output=True, text=True, timeout=60,
        )
        self.assertNotEqual(done.returncode, 0)
        self.assertIn("Request failed", done.stderr)


if __name__ == "__main__":
    unittest.main()
