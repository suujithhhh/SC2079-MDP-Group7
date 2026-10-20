"""Manual smoke client for the RAW camera image upload (`POST /images`,
`PROTOCOL.md` section 7.1): a stand-in for the Raspberry Pi's camera.

After each `SNAP` the real RPi will send the photo it took with exactly this
request. This script sends one photo so the web page's Camera gallery can be
checked without the robot. It is not RPi code and does no recognition.

    python rpi_image_test_client.py --host 127.0.0.1 --mission-id <id> --obstacle-id 3 --image photo.jpg
    python rpi_image_test_client.py --host 127.0.0.1 --mission-id <id> --obstacle-id 3 --demo
    python rpi_image_test_client.py --host 127.0.0.1 --mission-id <id> --obstacle-id 3 --target-id 22 --demo

`--target-id` (11-40) is optional: the image ID the RPi recognised. Leave it
out to send a photo with no ID; sending the same obstacle again replaces it.

`--mission-id` must equal the `mission_id` of the plan the RPi asked for (the
page shows only photos of the mission on screen). Exit code 0 only when the PC
replies `IMAGE_STORED`; any other reply or a connection failure exits 1.
"""

import argparse
import base64
import json
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

# A small real JPEG (64 x 36) so `--demo` needs no file. Real photos should be
# 1280 x 720, quality 85, aim below 1 MB (hard limit 2 MB).
_DEMO_JPEG_BASE64 = (
    "/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAAUDBAQEAwUEBAQFBQUGBwwIBwcHBw8LCwkMEQ8SEhEP"
    "ERETFhwXExQaFRERGCEYGh0dHx8fExciJCIeJBweHx7/2wBDAQUFBQcGBw4ICA4eFBEUHh4eHh4e"
    "Hh4eHh4eHh4eHh4eHh4eHh4eHh4eHh4eHh4eHh4eHh4eHh4eHh4eHh4eHh7/wAARCAAkAEADASIA"
    "AhEBAxEB/8QAHwAAAQUBAQEBAQEAAAAAAAAAAAECAwQFBgcICQoL/8QAtRAAAgEDAwIEAwUFBAQA"
    "AAF9AQIDAAQRBRIhMUEGE1FhByJxFDKBkaEII0KxwRVS0fAkM2JyggkKFhcYGRolJicoKSo0NTY3"
    "ODk6Q0RFRkdISUpTVFVWV1hZWmNkZWZnaGlqc3R1dnd4eXqDhIWGh4iJipKTlJWWl5iZmqKjpKWm"
    "p6ipqrKztLW2t7i5usLDxMXGx8jJytLT1NXW19jZ2uHi4+Tl5ufo6erx8vP09fb3+Pn6/8QAHwEA"
    "AwEBAQEBAQEBAQAAAAAAAAECAwQFBgcICQoL/8QAtREAAgECBAQDBAcFBAQAAQJ3AAECAxEEBSEx"
    "BhJBUQdhcRMiMoEIFEKRobHBCSMzUvAVYnLRChYkNOEl8RcYGRomJygpKjU2Nzg5OkNERUZHSElK"
    "U1RVVldYWVpjZGVmZ2hpanN0dXZ3eHl6goOEhYaHiImKkpOUlZaXmJmaoqOkpaanqKmqsrO0tba3"
    "uLm6wsPExcbHyMnK0tPU1dbX2Nna4uPk5ebn6Onq8vP09fb3+Pn6/9oADAMBAAIRAxEAPwDxzQNI"
    "v9d1RNN02OJ7h45JP3s6QoqRo0jszyEKoCKxJJHStn/hBdb/AOf7wv8A+FPp3/x+j4Y/8jJd/wDY"
    "D1f/ANN1zXL17et7I83Sx1H/AAgut/8AP94X/wDCn07/AOP0f8ILrf8Az/eF/wDwp9O/+P1y9a1x"
    "4d1aC2W4litwhiilIF3EWRJApRnUNlAQy8sB94etZVK0KTSnNK+19L/iUouWyNL/AIQXW/8An+8L"
    "/wDhT6d/8fo/4QXW/wDn+8L/APhT6d/8frl6K1tLuTodR/wgut/8/wB4X/8ACn07/wCP1ja/pF/o"
    "WqPpupRxJcJHHJ+6nSZGSRFkRleMlWBRlIIJ61QrqPid/wAjJaf9gPSP/TdbUa3sw0sHwx/5GS7/"
    "AOwHq/8A6brmuXro/hzd2Fn4mZtSvorC3n03ULX7RKjskbzWc0SFhGrNje65wpqX/hGNE/6KL4X/"
    "APAfUf8A5EpXtIdro5eu6svFVjbeKtGlhjtobRF0tL68VJTIyQLAXUgkjCvF/CoJ2Dr3zf8AhGNE"
    "/wCii+F//AfUf/kSj/hGNE/6KL4X/wDAfUf/AJErjxmCo4vSpfZr77flZW6eRpTqSp/CcvRXUf8A"
    "CMaJ/wBFF8L/APgPqP8A8iUf8Ixon/RRfC//AID6j/8AIldvMjKxy9dR8Tv+RktP+wHpH/putqP+"
    "EY0T/oovhf8A8B9R/wDkSoviNd2F54mVtNvor+3g03T7X7REjqkjw2cMTlRIqtjejYyopXvIdrI5"
    "yiiirJCiiigAooooAKKKKAP/2Q=="
)


def demo_jpeg() -> bytes:
    return base64.b64decode(_DEMO_JPEG_BASE64)


def send_image(url: str, mission_id: str, obstacle_id: int, data: bytes, timeout: float = 30.0,
               target_id=None):
    """POST one JPEG; returns `(status, parsed_json_reply)`. `target_id` is optional."""
    params = {"mission_id": mission_id, "obstacle_id": obstacle_id}
    if target_id is not None:
        params["target_id"] = target_id
    query = urllib.parse.urlencode(params)
    request = urllib.request.Request(
        f"{url}?{query}", data=data, method="POST",
        headers={"Content-Type": "image/jpeg", "Content-Length": str(len(data))},
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout) as reply:
            return reply.status, json.load(reply)
    except urllib.error.HTTPError as error:
        try:
            return error.code, json.load(error)
        finally:
            error.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Send one RAW camera image to the PC (POST /images).")
    parser.add_argument("--host", required=True, help="PC IP or hostname")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--mission-id", required=True, help="the mission_id of the plan on screen")
    parser.add_argument("--obstacle-id", type=int, required=True, help="the obstacle just snapped, 1-8")
    parser.add_argument("--target-id", type=int, default=None, help="optional recognised image ID, 11-40")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--image", type=Path, help="path to a JPEG file")
    source.add_argument("--demo", action="store_true", help="send a small built-in demo JPEG")
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()

    data = demo_jpeg() if args.demo else args.image.read_bytes()
    url = f"http://{args.host}:{args.port}/images"
    try:
        status, reply = send_image(url, args.mission_id, args.obstacle_id, data, args.timeout, args.target_id)
    except (OSError, json.JSONDecodeError, urllib.error.URLError) as error:
        raise SystemExit(f"Request failed: {error}") from None

    print(json.dumps(reply, indent=2))
    if status != 200 or reply.get("type") != "IMAGE_STORED":
        raise SystemExit(1)
    verb = "replaced the earlier photo of" if reply.get("replaced") else "stored a photo of"
    print(f"{len(data)} bytes sent; the PC {verb} obstacle {reply['obstacle_id']} "
          f"(mission {reply['mission_id']!r}, image {reply['image_id']}, target ID {reply['target_id']}).")


if __name__ == "__main__":
    main()
