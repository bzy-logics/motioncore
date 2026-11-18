#!/usr/bin/env python3
"""Measure inter-axis timing skew of a coordinated move from a logic-analyser
capture of the STEP lines.

In a MoveJ every axis follows the same normalised progress s(t) from 0 to 1,
so axis i takes its k-th step when s(t) = (k - 0.5) / N_i (N_i = its total
step count). The axis with the most steps gives the finest estimate of s(t);
for every other axis the expected time of each step is read off that
reference and compared with the measured edge. The median difference is the
channel's timing offset (skew) relative to the reference; the spread is its
jitter. No plan or start time is needed, only one MoveJ captured from rest
to rest, every axis moving in one direction.

Input: CSV with a time column (seconds) and one column per digital channel
(0/1), as exported by sigrok-cli (`-O csv`) or Saleae Logic ("Time [s]").
Rows may be every sample or only transitions. Lines starting with ';' are
ignored.

    python step_skew.py capture.csv --step D0,D2,D4,D6,D8,D10
    python step_skew.py --self-test

Exit status 1 when a limit is exceeded.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import math
import statistics
import sys
import tempfile
from pathlib import Path


def read_edges(path: Path, columns: list[str]) -> dict[str, list[float]]:
    """Rising-edge times (s) per selected column."""
    with path.open(newline="") as f:
        rows = [r for r in csv.reader(f) if r and not r[0].lstrip().startswith(";")]
    header = [h.strip() for h in rows[0]]
    idx = {}
    for c in columns:
        if c in header:
            idx[c] = header.index(c)
        elif c.isdigit() and int(c) < len(header):
            idx[c] = int(c)
        else:
            raise SystemExit(f"column {c!r} not in header {header}")
    edges: dict[str, list[float]] = {c: [] for c in columns}
    prev: dict[str, int | None] = {c: None for c in columns}
    for r in rows[1:]:
        t = float(r[0])
        for c, i in idx.items():
            v = 1 if r[i].strip() in ("1", "1.0", "high", "HIGH") else 0
            if prev[c] == 0 and v == 1:
                edges[c].append(t)
            prev[c] = v
    return edges


def analyse(edges: dict[str, list[float]], trim: float = 0.05) -> dict:
    """Per-channel offset (s) and jitter relative to the busiest channel."""
    counts = {c: len(e) for c, e in edges.items()}
    moving = [c for c in edges if counts[c] >= 2]
    if not moving:
        raise SystemExit("no channel has steps")
    ref = max(moving, key=lambda c: counts[c])
    rt = edges[ref]
    n_ref = counts[ref]
    rs = [(k + 0.5) / n_ref for k in range(n_ref)]  # s at each reference edge

    def t_of(s: float) -> float:
        # Piecewise-linear inverse of s(t) through the reference edges.
        j = bisect.bisect_left(rs, s)
        if j <= 0:
            return rt[0]
        if j >= n_ref:
            return rt[-1]
        f = (s - rs[j - 1]) / (rs[j] - rs[j - 1])
        return rt[j - 1] + f * (rt[j] - rt[j - 1])

    out = {"reference": ref, "channels": {}}
    for c in edges:
        n = counts[c]
        if n < 2:
            out["channels"][c] = {"steps": n, "offset_us": None, "jitter_us": None}
            continue
        lo, hi = int(n * trim), max(int(n * (1 - trim)), int(n * trim) + 1)
        diffs = [edges[c][k] - t_of((k + 0.5) / n) for k in range(lo, min(hi, n))]
        off = statistics.median(diffs)
        spread = max(abs(d - off) for d in diffs)
        out["channels"][c] = {
            "steps": n,
            "first_s": edges[c][0],
            "last_s": edges[c][-1],
            "offset_us": off * 1e6,
            "jitter_us": spread * 1e6,
        }
    return out


def report(res: dict, max_skew_us: float, max_jitter_us: float) -> bool:
    ok = True
    print(f"reference channel: {res['reference']} (most steps)")
    print(f"{'channel':>10} {'steps':>8} {'offset us':>10} {'jitter us':>10}")
    offsets = []
    for c, r in res["channels"].items():
        if r["offset_us"] is None:
            print(f"{c:>10} {r['steps']:>8} {'-':>10} {'-':>10}   (not moving)")
            continue
        offsets.append(r["offset_us"])
        flag = ""
        if r["jitter_us"] > max_jitter_us:
            flag, ok = "  JITTER", False
        print(f"{c:>10} {r['steps']:>8} {r['offset_us']:>10.2f} {r['jitter_us']:>10.2f}{flag}")
    skew = max(offsets) - min(offsets) if offsets else 0.0
    verdict = "ok" if skew <= max_skew_us else "TOO LARGE"
    if skew > max_skew_us:
        ok = False
    print(f"inter-axis skew (max - min offset): {skew:.2f} us  [limit {max_skew_us:g} us: {verdict}]")
    return ok


# ---------------------------------------------------------------- self-test


def _synthetic(path: Path, delays_us: list[float], steps: list[int], jitter_us: float = 0.0) -> None:
    """Six STEP channels of a synchronised trapezoid with known delays."""
    ta, tc = 0.25, 1.0  # shared phase times (s)
    total = 2 * ta + tc
    v = 1.0 / (ta + tc)  # peak ds/dt
    a = v / ta

    def s_of(t: float) -> float:
        if t <= ta:
            return 0.5 * a * t * t
        if t <= ta + tc:
            return 0.5 * a * ta * ta + v * (t - ta)
        d = t - ta - tc
        return 0.5 * a * ta * ta + v * tc + v * d - 0.5 * a * d * d

    def t_of(s: float) -> float:
        lo, hi = 0.0, total
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            if s_of(mid) < s:
                lo = mid
            else:
                hi = mid
        return 0.5 * (lo + hi)

    events = []
    rnd = 12345
    for ch, (n, d) in enumerate(zip(steps, delays_us)):
        for k in range(n):
            rnd = (rnd * 1103515245 + 12345) % 2**31
            j = (rnd / 2**31 - 0.5) * 2 * jitter_us
            t = 0.010 + t_of((k + 0.5) / n) + (d + j) * 1e-6
            events.append((t, ch, 1))
            events.append((t + 2e-6, ch, 0))  # 2 us pulse
    events.sort()
    state = [0] * len(steps)
    with path.open("w", newline="") as f:
        f.write("; synthetic capture (sigrok-like)\n")
        w = csv.writer(f)
        w.writerow(["Time [s]"] + [f"D{2 * i}" for i in range(len(steps))])
        w.writerow([0.0] + state)
        for t, ch, v in events:
            state[ch] = v
            w.writerow([f"{t:.9f}"] + state)


def self_test() -> int:
    delays = [0.0, 2.5, 5.0, 7.5, 10.0, 12.5]
    steps = [8000, 1200, 3500, 640, 5000, 2600]
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "cap.csv"
        _synthetic(p, delays, steps, jitter_us=0.2)
        cols = [f"D{2 * i}" for i in range(6)]
        res = analyse(read_edges(p, cols))
    assert res["reference"] == "D0", res["reference"]
    for i, c in enumerate(cols):
        got = res["channels"][c]["offset_us"]
        want = delays[i] - delays[0]
        assert math.isclose(got, want, abs_tol=0.5), (c, got, want)
        assert res["channels"][c]["steps"] == steps[i]
    print("self-test ok: recovered per-channel offsets", [round(res["channels"][c]["offset_us"], 2) for c in cols])
    return 0


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture", nargs="?", type=Path, help="CSV export of the capture")
    ap.add_argument("--step", help="comma-separated STEP column names (or indices), e.g. D0,D2,D4,D6,D8,D10")
    ap.add_argument("--max-skew-us", type=float, default=50.0, help="limit on max - min channel offset (default 50)")
    ap.add_argument("--max-jitter-us", type=float, default=250.0, help="limit on per-channel deviation (default 250: one 200 us window + margin)")
    ap.add_argument("--self-test", action="store_true", help="check the analysis on a synthetic capture")
    a = ap.parse_args(argv)
    if a.self_test:
        return self_test()
    if not a.capture or not a.step:
        ap.error("capture and --step are required")
    res = analyse(read_edges(a.capture, [c.strip() for c in a.step.split(",")]))
    return 0 if report(res, a.max_skew_us, a.max_jitter_us) else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
