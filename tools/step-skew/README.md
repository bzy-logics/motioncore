# step-skew

Measures the inter-axis timing skew of a coordinated joint move (MoveJ) from
a logic-analyser capture of the STEP lines. Python 3, standard library only.

In a MoveJ every axis follows the same normalised progress s(t), so axis *i*
takes its *k*-th step when s(t) = (k − ½) / Nᵢ. The channel with the most
steps gives the finest estimate of s(t); for every other channel the tool
compares each measured edge with the time the reference says it was due. The
median difference is that channel's **offset** (its skew relative to the
reference) and the largest deviation from it is its **jitter**. No plan file
or trigger alignment is needed.

```bash
python step_skew.py --self-test                       # check the analysis on synthetic data
python step_skew.py capture.csv --step D0,D2,D4,D6,D8,D10
python step_skew.py capture.csv --step D0,D2,D4,D6,D8,D10 --max-skew-us 20
```

Input is a CSV export: a time column in seconds, then one column per digital
channel (0/1). sigrok-cli (`-O csv`) and Saleae Logic ("Time [s]") exports
work; rows may be every sample or only transitions. Pass the STEP columns by
name or index. Exit status is 1 when the skew (largest minus smallest
offset) exceeds `--max-skew-us` (default 50 µs), or when an edge deviates
from its channel's offset by more than `--max-jitter-us` **plus**
`--max-jitter-steps` steps at that edge's local step rate (defaults: 250 µs,
i.e. one 200 µs PIO window plus margin on the RP2040, plus 1 step).

Why the step allowance: correct firmware puts every edge within one window of
its ideal time plus up to about one step (step quantisation and per-segment
step rounding). Where a channel steps slowly, in the ramps or on a slow axis,
one step is long: at 2 800 steps/s it is 355 µs. A pure µs limit would fail
healthy hardware there (#110). Jitter is reported both in µs and in steps
(deviation × local rate). The simulator timing tests
(`cargo test -p motioncore-simulator --test timing`) check the tool against
simulated ground truth with these defaults.

Capture one MoveJ from rest to rest in which every axis moves in a single
direction, for example on the six-axis RP2040 bench profile after homing:

```bash
motioncore movej j1=0 j2=0 j3=-35 j4=60 j5=-10 j6=90
```

See [RP2040 hardware validation, test 15](../../docs/rp2040.md#91-coordinated-move-skew-test-15) for the full procedure.
