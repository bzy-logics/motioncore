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
offset) or a channel's jitter exceeds its limit (defaults: 50 µs skew,
250 µs jitter, i.e. one 200 µs PIO window plus margin on the RP2040).

Capture one MoveJ from rest to rest in which every axis moves in a single
direction, for example on the six-axis RP2040 bench profile after homing:

```bash
motioncore movej j1=0 j2=0 j3=-35 j4=60 j5=-10 j6=90
```

See `docs/rp2040.md` §9 (test 15) for the full procedure.
