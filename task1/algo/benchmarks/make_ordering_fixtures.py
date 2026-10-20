"""Regenerate (or verify) the deterministic E_ordering candidate fixtures.

    python benchmarks/make_ordering_fixtures.py --write   # (re)write E_ordering_04_c*.json
    python benchmarks/make_ordering_fixtures.py --check   # exit 1 if a committed file differs

Run from `algo/`. The layouts are a pure function of the candidate number (see
`benchmarks.fixtures.generate_ordering_candidate`); the candidate list and every rule are
fixed before any planner run, and `benchmarks/ordering_discovery.py` judges the candidates.
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
    for candidate in bf.ORDERING_CANDIDATES:
        data = bf.ordering_fixture_data(candidate)
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
        print("all ordering fixtures match the generator")
    return 0


if __name__ == "__main__":
    sys.exit(main())
