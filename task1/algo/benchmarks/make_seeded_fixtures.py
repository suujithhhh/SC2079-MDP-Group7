"""Regenerate (or verify) the deterministic D_seeded benchmark fixtures.

    python benchmarks/make_seeded_fixtures.py --write   # (re)write D_seeded_*.json
    python benchmarks/make_seeded_fixtures.py --check   # exit 1 if a committed file differs

Run from `algo/`. The layouts are a pure function of `(n_obstacles, seed)`
(see `benchmarks.fixtures.generate_seeded`); the A/B/C fixtures are
hand-authored and are not touched by this script.
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from benchmarks import fixtures as bf  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--write", action="store_true")
    mode.add_argument("--check", action="store_true")
    args = parser.parse_args()

    stale = []
    for n in bf.SEEDED_SIZES:
        for variant in bf.SEEDED_VARIANTS:
            data = bf.seeded_fixture_data(n, variant)
            text = bf.dump_fixture(data)
            path = bf.FIXTURE_DIR / f"{data['fixture_id']}.json"
            if args.write:
                path.write_text(text, encoding="utf-8", newline="")
                print(f"wrote {path.name}")
            elif not path.is_file() or path.read_text(encoding="utf-8").replace("\r\n", "\n") != text:
                stale.append(path.name)
    if args.check:
        if stale:
            print("stale or missing: " + ", ".join(stale), file=sys.stderr)
            return 1
        print("all seeded fixtures match the generator")
    return 0


if __name__ == "__main__":
    sys.exit(main())
