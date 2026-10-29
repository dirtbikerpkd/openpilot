# RDX drive-log analysis

Pulls drive logs off the comma four, extracts the steering and radar signals, lines them up
against an optional OBD-II log, and writes a report on how much the EPS is actually doing.

It exists to answer questions like: how much torque does the RDX 3G EPS deliver for what we
command, how late does it respond, does assist fade out at highway speed, and does that line
up with anything the engine or car was doing at the time.

Everything here is **read-only**. It never sends anything to the car, and the only thing it
does to the device is copy log files off it.

> Status: the synthetic test passes (see [Testing](#testing)) and `extract` has run on one real set of
> RDX 3G drives (2026-10-28/29). Treat the fade thresholds as starting points and expect to tune them.

## Requirements

- [`uv`](https://docs.astral.sh/uv/) (installs the Python dependencies on first run)
- Python 3.12 (`--python 3.12`; `pycapnp` doesn't ship wheels for the newest Pythons)
- `rsync` and SSH access to the device (`ssh comma@<device-ip>` must work without a prompt)

No compiled openpilot build is needed. The script loads `cereal/log.capnp` straight from this
repo to decode the logs.

## Quick start

```sh
RUN="uv run --python 3.12 tools/rdx_analysis/rdx_analysis.py"

# 1. copy the 5 newest segment dirs from the device (rlog = full rate, qlog = decimated fallback)
$RUN pull --host 192.168.1.116 --dest ~/rdx_logs --last 5

# 2. decode them into one resampled CSV (50 Hz by default)
$RUN extract ~/rdx_logs -o drive.csv

# 3. report, optionally lined up against an OBD log
$RUN report drive.csv --obd obd.csv -o report.md --merged-csv merged.csv
```

`report` prints markdown and can also save it (`-o`) plus the merged drive+OBD table
(`--merged-csv`) for your own plotting.

### `pull`

| flag | meaning |
|---|---|
| `--host` | device IP or hostname (required) |
| `--dest` | where to put the segments (default `~/rdx_logs`) |
| `--last N` | only the N most recently modified segment dirs |
| `--since DATE` | only dirs modified after this date/time, e.g. `2026-11-02` |

Only `rlog*` and `qlog*` are copied, not the camera video. The `boot` log dir is skipped. The
segment currently being recorded may be incomplete, so skip it if the car is still on.

### `extract`

Decodes every segment under the given paths (a segment dir prefers `rlog` over `qlog`), in
order, and resamples to a fixed rate with each signal held at its last value. The drive CSV
columns are described in [SIGNALS.md](SIGNALS.md#comma-signals-used).

The `wall` column is Unix time, derived from the log's `clocks`/`initData` wall-clock stamps.
That is what the OBD log is aligned against.

### `report`

| flag | default | meaning |
|---|---|---|
| `--obd FILE` | none | OBD CSV to line up (format below) |
| `--obd-tz` | local | timezone for OBD timestamps that have no zone |
| `--obd-offset` | 0 | seconds added to OBD times, to correct clock drift |
| `--obd-start` | none | ISO time, for relative OBD timestamps |
| `--obd-tol` | 3 | max age in seconds of an OBD sample used for a row |
| `--steer-max` | 4095 | CAN torque at actuator 1.0 (the RDX 3G's `torqueBP[-1]`) |
| `--min-speed` | 20 | mph above which lateral-active time is analyzed |
| `--min-cmd` | 600 | ignore commands smaller than this (CAN units) for ratios and fades |
| `--fade-window` | 1.0 | seconds of rolling median for the efficiency measure |
| `--fade-thresh` | 0.3 | a fade is efficiency below this fraction of baseline |
| `--fade-min-s` | 2.0 | minimum fade length in seconds |

What the report contains, and what each number means, is in
[SIGNALS.md](SIGNALS.md#what-the-report-computes).

## OBD log format

Any CSV works if it has one time column and one numeric column per signal. The time column is
the first column or one named `timestamp`, `time`, `datetime`, `date`, `ts`, `epoch`,
`seconds` or `secs`. Accepted time forms:

- ISO or common date strings (`2026-11-02T14:00:03`, `2026-11-02 09:00:03`). If there is no
  zone, `--obd-tz` applies (default: this machine's current zone).
- Epoch seconds or milliseconds.
- Relative seconds from the start of the log. This needs `--obd-start`.

Column names are kept as-is and prefixed `obd_` in the merged table. Using the OBDb signal IDs
(`RDX_GENERATOR`, `RDX_AAT_FIRSTGEN`, ...) makes the report easy to read; see
[SIGNALS.md](SIGNALS.md#obd-signals-for-the-rdx-3g).

Clock accuracy matters. Phone and dongle clocks drift by seconds, which blurs correlation with
events a couple of seconds long. Check one obvious event in both logs (a hard brake, engine
start) and use `--obd-offset` if they disagree. OBD signals are polled at about 1 Hz, so each
OBD value is held for up to `--obd-tol` seconds.

Keep OBD polling light (a few requests per second at most). It shares the car's diagnostic
bus.

## Limits worth knowing

- **EPS torque source.** Honda's carState never fills `steeringTorqueEps` (it is always 0), so
  `extract` decodes `STEER_MOTOR_TORQUE` (0x1AB) and `STEER_STATUS` (0x18F) from the rlog `can`
  events. Use rlogs, not qlogs. If no EPS torque is found the report says so and skips the
  authority, lag and fade sections instead of printing zeros.
- **Torque units.** `carOutput` torque is a normalized command that the script scales by
  `--steer-max`. `STEER_MOTOR_TORQUE` is a different scale (on the RDX 3G the EPS/command ratio
  is about 0.02), so ratios are compared against the drive's own baseline rather than judged
  absolutely.
- **Sign.** `STEER_MOTOR_TORQUE` is an unsigned magnitude, so lag is estimated on magnitudes.
- **Checksums.** CAN checksums are not verified. A corrupt frame would show as a one-sample spike.
- **Percentages.** They use real driving time only; gaps between drives are excluded.
- **Fade detection is a heuristic.** Hard driver input, tight curves, or a rough road can look
  like low efficiency. Fades that coincide with `steerFaultTemporary` are the strongest
  candidates.
- **Lag resolution** is 1/rate (20 ms at 50 Hz).
- **OBD correlations are not causes.** They only rank which signals are worth a closer look.
- **Hardware.** This analyzes logs. It does not measure the EPS on a bench, and it does not
  produce a safe limit by itself. Any value that ends up in the control code needs your
  review and a careful on-road check.

## Testing

```sh
uv run --python 3.12 tools/rdx_analysis/test_rdx_analysis.py
```

Builds a synthetic 90 s rlog with a known 0.15 s EPS lag (EPS torque carried as raw CAN frames,
like a real Honda log), one assist fade with `low_speed_lockout` and a fault, and a lead vehicle,
plus a matching OBD CSV. The test checks that the analysis recovers the lag, the fade, the fault,
the status table, the radar share, and the OBD alignment and ranking, and that CAN loopback
frames are ignored.

OBD signal descriptions are based on the community-maintained
[OBDb/Acura-RDX](https://github.com/OBDb/Acura-RDX) signal set (CC-BY-SA-4.0).
