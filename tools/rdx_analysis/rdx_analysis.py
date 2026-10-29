#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10,<3.14"
# dependencies = ["pycapnp", "zstandard", "numpy", "pandas"]
# ///
"""RDX drive-log analysis: pull comma segments, extract steering/radar signals, line them up
against an OBD log and report on EPS assist.

  uv run --python 3.12 tools/rdx_analysis/rdx_analysis.py pull --host 192.168.1.116 --dest ~/rdx_logs
  uv run --python 3.12 tools/rdx_analysis/rdx_analysis.py extract ~/rdx_logs -o drive.csv
  uv run --python 3.12 tools/rdx_analysis/rdx_analysis.py report drive.csv --obd obd.csv

See tools/rdx_analysis/README.md. Read-only: nothing here talks to the car or changes the device.
"""
import argparse
import bz2
import datetime as dt
import io
import subprocess
import sys
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[2]
REMOTE_REALDATA = "/data/media/0/realdata"
MPH = 0.44704

# Raw CAN frames the RDX 3G EPS broadcasts. carState does not expose motor torque for Honda, so
# these are decoded straight from the rlog `can` events (layouts from opendbc _honda_common.dbc /
# _bosch_2018.dbc: STEER_MOTOR_TORQUE 0x1AB, STEER_STATUS 0x18F). Checksums are not verified.
ADDR_STEER_MOTOR_TORQUE = 427
ADDR_STEER_STATUS = 399
STEER_STATUS_NAMES = {0: "normal", 1: "driver_steering", 2: "no_torque_alert_1", 3: "low_speed_lockout",
                      4: "no_torque_alert_2", 5: "fault_1", 6: "tmp_fault", 7: "permanent_fault"}

_log = None


def decode_motor_torque(dat):
  """STEER_MOTOR_TORQUE.MOTOR_TORQUE: 10-bit unsigned, big-endian, start bit 1. A magnitude, no sign."""
  return ((dat[0] & 0x03) << 8) | dat[1] if len(dat) >= 2 else None


def decode_steer_status(dat):
  """STEER_STATUS.STEER_STATUS (4 bits, start bit 39) and STEER_CONTROL_ACTIVE (bit 35), both in byte 4."""
  return (dat[4] >> 4, (dat[4] >> 3) & 1) if len(dat) >= 5 else None


def load_schema():
  """Load cereal's log.capnp straight from the repo, so no compiled openpilot build is needed."""
  global _log
  if _log is None:
    import capnp
    capnp.remove_import_hook()
    cereal = ROOT / "openpilot" / "cereal"
    _log = capnp.load(str(cereal / "log.capnp"),
                      imports=[str(cereal), str(cereal / "include"), str(ROOT / "opendbc_repo" / "opendbc" / "car"),
                               str(ROOT / "openpilot"), str(ROOT)])
  return _log


# ---------------------------------------------------------------------------------------------
# pull
# ---------------------------------------------------------------------------------------------
def cmd_pull(args):
  dest = Path(args.dest).expanduser()
  dest.mkdir(parents=True, exist_ok=True)
  ssh = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8"]
  find = f"find {REMOTE_REALDATA} -mindepth 1 -maxdepth 1 -type d ! -name boot"
  if args.since:
    find += f" -newermt '{args.since}'"
  listing = subprocess.run([*ssh, f"comma@{args.host}", f"{find} -printf '%T@ %f\\n' | sort -rn"],
                           capture_output=True, text=True, check=True).stdout.split("\n")
  segs = [ln.split(" ", 1)[1] for ln in listing if ln.strip()]
  if args.last:
    segs = segs[:args.last]
  if not segs:
    print("no segments found on the device")
    return 1
  print(f"pulling {len(segs)} segment dir(s) to {dest}")
  # rlog is the full-rate log; qlog is the decimated fallback. Skip videos so this stays small.
  rsync = ["rsync", "-a", "--partial", "-e", " ".join(ssh), "--include=*/", "--include=rlog*", "--include=qlog*", "--exclude=*"]
  for s in segs:
    subprocess.run([*rsync, f"comma@{args.host}:{REMOTE_REALDATA}/{s}/", str(dest / s) + "/"], check=True)
  print("done")
  return 0


# ---------------------------------------------------------------------------------------------
# extract
# ---------------------------------------------------------------------------------------------
def find_logs(paths):
  """Per segment dir, prefer rlog over qlog. Sorted by name so segments stay in time order."""
  files = []
  for p in map(Path, paths):
    p = p.expanduser()
    if p.is_file():
      files.append(p)
      continue
    by_dir = {}
    for f in sorted(p.rglob("*log*")):
      if f.is_file() and f.name.split(".")[0] in ("rlog", "qlog"):
        by_dir.setdefault(f.parent, {})[f.name.split(".")[0]] = f
    for d in sorted(by_dir):
      files.append(by_dir[d].get("rlog") or by_dir[d]["qlog"])
  return files


def read_events(path):
  raw = Path(path).read_bytes()
  if path.suffix == ".zst":
    import zstandard
    raw = zstandard.ZstdDecompressor().stream_reader(io.BytesIO(raw), read_across_frames=True).read()
  elif path.suffix == ".bz2":
    raw = bz2.decompress(raw)
  return load_schema().Event.read_multiple_bytes(raw)


def _lead(ld):
  return ld.present, ld.radar, ld.dRel, ld.vLead, ld.vRel


def extract(paths):
  """Returns (DataFrame of per-service samples keyed on monotonic seconds, wall-clock offset or None)."""
  rows = {k: [] for k in ("carState", "carOutput", "carControl", "radarState", "selfdriveState", "epsMotor", "steerStatus", "controlsState")}
  offsets = []
  for f in find_logs(paths):
    for ev in read_events(f):
      w = ev.which()
      t = ev.logMonoTime * 1e-9
      if w == "carState":
        c = ev.carState
        rows[w].append((t, c.vEgo, c.steeringAngleDeg, c.steeringTorque, c.steeringTorqueEps, c.steeringPressed,
                        c.steerFaultTemporary, c.steerFaultPermanent, c.gasPressed, c.brakePressed))
      elif w == "carOutput":
        rows[w].append((t, ev.carOutput.actuatorsOutput.torque))
      elif w == "carControl":
        rows[w].append((t, ev.carControl.latActive))
      elif w == "radarState":
        rows[w].append((t, *_lead(ev.radarState.leadOne)))
      elif w == "selfdriveState":
        rows[w].append((t, ev.selfdriveState.enabled, ev.selfdriveState.active))
      elif w == "controlsState":
        c = ev.controlsState
        ls = c.lateralControlState
        kind = ls.which()
        st = getattr(ls, kind)
        pid = kind == "pidState"
        rows[w].append((t, c.curvature, c.desiredCurvature,
                        st.steeringAngleDesiredDeg if pid else np.nan, st.angleError if pid else np.nan,
                        getattr(st, "output", np.nan), getattr(st, "saturated", False)))
      elif w == "can":
        for c in ev.can:
          if c.src >= 128:  # loopback of our own sendcan
            continue
          if c.address == ADDR_STEER_MOTOR_TORQUE:
            m = decode_motor_torque(c.dat)
            if m is not None:
              rows["epsMotor"].append((t, m))
          elif c.address == ADDR_STEER_STATUS:
            st = decode_steer_status(c.dat)
            if st is not None:
              rows["steerStatus"].append((t, *st))
      elif w == "clocks" and ev.clocks.wallTimeNanos:
        offsets.append(ev.clocks.wallTimeNanos * 1e-9 - t)
      elif w == "initData" and ev.initData.wallTimeNanos:
        offsets.append(ev.initData.wallTimeNanos * 1e-9 - t)
  cols = {
    "carState": ["t", "vEgo", "steeringAngleDeg", "steeringTorque", "steeringTorqueEps", "steeringPressed",
                 "steerFaultTemporary", "steerFaultPermanent", "gasPressed", "brakePressed"],
    "carOutput": ["t", "cmd_torque"],
    "carControl": ["t", "latActive"],
    "radarState": ["t", "lead_present", "lead_radar", "lead_dRel", "lead_vLead", "lead_vRel"],
    "selfdriveState": ["t", "sd_enabled", "sd_active"],
    "controlsState": ["t", "curvature", "desiredCurvature", "angle_des_deg", "angle_err_deg", "lat_output", "lat_saturated"],
    "epsMotor": ["t", "eps_motor"],
    "steerStatus": ["t", "steer_status", "steer_ctrl_active"],
  }
  frames = {k: pd.DataFrame(v, columns=cols[k]).sort_values("t") for k, v in rows.items()}
  return frames, (float(np.median(offsets)) if offsets else None)


def build_grid(frames, offset, hz):
  base = frames["carState"]
  if base.empty:
    raise SystemExit("no carState messages found in these logs")
  grid = pd.DataFrame({"t": np.arange(base.t.iloc[0], base.t.iloc[-1], 1.0 / hz)})
  tol = max(0.5, 2.0 / hz)
  for name, df in frames.items():
    if not df.empty:
      grid = pd.merge_asof(grid, df, on="t", direction="backward", tolerance=tol)
  for c in grid.columns:
    if grid[c].dtype == object:
      grid[c] = grid[c].astype(float)
  grid["wall"] = grid.t + offset if offset is not None else np.nan
  return grid


def cmd_extract(args):
  frames, offset = extract(args.paths)
  if offset is None:
    print("warning: no wall-clock reference in these logs (no clocks/initData); OBD alignment will need --obd-start", file=sys.stderr)
  grid = build_grid(frames, offset, args.hz)
  grid.to_csv(args.out, index=False)
  dur = grid.t.iloc[-1] - grid.t.iloc[0]
  print(f"wrote {args.out}: {len(grid)} rows at {args.hz} Hz, {dur / 60:.1f} min")
  return 0


# ---------------------------------------------------------------------------------------------
# OBD log
# ---------------------------------------------------------------------------------------------
TIME_COLS = ("timestamp", "time", "datetime", "date", "ts", "epoch", "seconds", "secs")


def load_obd(path, tz, offset_s, start_wall):
  df = pd.read_csv(path)
  df.columns = [c.strip() for c in df.columns]
  tcol = next((c for c in df.columns if c.lower() in TIME_COLS), df.columns[0])
  tv = df[tcol]
  if pd.api.types.is_numeric_dtype(tv):
    v = tv.astype(float)
    if v.max() > 1e11:
      wall = v / 1e3  # epoch milliseconds
    elif v.max() > 1e8:
      wall = v  # epoch seconds
    else:
      if start_wall is None:
        raise SystemExit("OBD timestamps are relative; pass --obd-start (ISO time the OBD log began)")
      wall = start_wall + (v - v.iloc[0])  # relative seconds
  else:
    ts = pd.to_datetime(tv)
    if ts.dt.tz is None:
      zone = dt.datetime.now().astimezone().tzinfo if tz == "local" else tz
      ts = ts.dt.tz_localize(zone, ambiguous="NaT", nonexistent="NaT")
    wall = ts.dt.tz_convert("UTC").map(lambda x: x.timestamp() if pd.notna(x) else np.nan)
  out = df.drop(columns=[tcol]).apply(pd.to_numeric, errors="coerce")
  out.insert(0, "wall", wall.astype(float) + offset_s)
  return out.dropna(subset=["wall"]).sort_values("wall").dropna(axis=1, how="all")


def attach_obd(grid, obd, tol_s):
  obd = obd.rename(columns={c: f"obd_{c}" for c in obd.columns if c != "wall"})
  return pd.merge_asof(grid.sort_values("wall"), obd, on="wall", direction="backward", tolerance=tol_s)


# ---------------------------------------------------------------------------------------------
# analysis
# ---------------------------------------------------------------------------------------------
def runs(mask, min_len):
  """(start, end) index pairs of True runs at least min_len long."""
  m = np.asarray(mask, dtype=bool)
  edges = np.diff(np.r_[0, m.view(np.int8), 0])
  starts, ends = np.where(edges == 1)[0], np.where(edges == -1)[0]
  return [(s, e) for s, e in zip(starts, ends) if e - s >= min_len]


def estimate_lag(cmd, eps, hz, max_lag_s=0.6):
  """Lag (s) and correlation of the strongest cmd -> eps match. Pass abs() of both for an unsigned eps."""
  best = (0.0, 0.0)
  for k in range(0, int(max_lag_s * hz) + 1):
    a, b = (cmd[:len(cmd) - k], eps[k:]) if k else (cmd, eps)
    if len(a) < 50 or a.std() == 0 or b.std() == 0:
      continue
    r = np.corrcoef(a, b)[0, 1]
    if abs(r) > abs(best[1]):
      best = (k / hz, r)
  return best


def fmt_short(frac, turning):
  """Mean under-steer share over the turning samples of a window, as text."""
  m = turning & np.isfinite(frac)
  return f"{np.mean(frac[m]) * 100:.0f}%" if m.sum() >= 10 else "-"


def fmt_time(wall):
  if wall is None or not np.isfinite(wall):
    return "n/a"
  return dt.datetime.fromtimestamp(wall, dt.timezone.utc).astimezone().strftime("%Y-%m-%d %H:%M:%S")


def analyze(df, args, hz):
  L = []
  w = L.append
  df = df.reset_index(drop=True)
  v = df.vEgo.fillna(0).to_numpy()
  lat = df.get("latActive", pd.Series(0, index=df.index)).fillna(0).to_numpy() > 0
  cmd_can = (df.cmd_torque.fillna(0) * args.steer_max).to_numpy()
  # Honda carState never fills steeringTorqueEps, so prefer the motor torque decoded from raw CAN.
  eps_col = "eps_motor" if "eps_motor" in df and df.eps_motor.fillna(0).abs().max() > 0 else "steeringTorqueEps"
  eps = df[eps_col].fillna(0).to_numpy()
  unsigned = eps_col == "eps_motor"  # STEER_MOTOR_TORQUE is a 10-bit magnitude
  active = lat & (v > args.min_speed * MPH)
  have_eps = bool(np.abs(eps[active]).max() > 0) if active.any() else False

  # Lateral tracking: positive shortfall = the car is turning less than the (lag-adjusted) desired curvature.
  have_track = "desiredCurvature" in df and df.desiredCurvature.notna().any()
  if have_track:
    des = df.desiredCurvature.fillna(0).to_numpy()
    act_c = df.curvature.fillna(0).to_numpy()
    short = np.sign(des) * (des - act_c)
    short_frac = short / np.maximum(np.abs(des), 1e-6)
    turning = active & (np.abs(des) > args.min_curv)
  else:
    short_frac = np.full(len(df), np.nan)
    turning = np.zeros(len(df), dtype=bool)
  # The grid spans boot gaps between drives (carState rows are NaN there); only count real driving time.
  valid = df.vEgo.notna().to_numpy()
  nvalid = max(int(valid.sum()), 1)
  dur = nvalid / hz

  w("# RDX drive report\n")
  w(f"- span: {fmt_time(df.wall.iloc[0] if 'wall' in df else None)} -> {fmt_time(df.wall.iloc[-1] if 'wall' in df else None)}"
    f" ({dur / 60:.1f} min, {hz:g} Hz)")
  w(f"- lateral active: {active.sum() / nvalid * 100:.1f}% of the time ({active.sum() / hz / 60:.1f} min above {args.min_speed:g} mph)")
  if "sd_active" in df:
    w(f"- openpilot engaged (selfdriveState.active): {df.sd_active.fillna(0).sum() / nvalid * 100:.1f}%")
  if active.sum() < 50:
    w("\nNot enough lateral-active time to analyze steering. (Drive engaged, then re-extract.)")
    return "\n".join(L), df, None

  fault = df.steerFaultTemporary.fillna(0).to_numpy() > 0
  edges = np.where(np.diff(np.r_[0, fault.astype(np.int8)]) == 1)[0]
  eps_fade = []
  eff_rel = np.full(len(df), np.nan)
  if not have_eps:
    w("\n## EPS authority\n")
    w("**No EPS motor torque in these logs** (neither `STEER_MOTOR_TORQUE` on CAN nor `carState.steeringTorqueEps` is "
      "non-zero while lateral-active), so the authority, lag and assist-fade analysis is skipped. Honda's carState does not "
      "fill `steeringTorqueEps`; `extract` decodes 0x1AB from the rlog `can` events, so re-run `extract` on rlogs "
      "(qlog is decimated and may not carry it).")
  else:
    # --- authority per speed bin --------------------------------------------------------------
    w("\n## EPS authority by speed\n")
    w("Commanded torque is carOutput.actuatorsOutput.torque * steer_max (CAN units). EPS torque is "
      + ("STEER_MOTOR_TORQUE decoded from CAN (10-bit magnitude, not the same units as the command, so read the ratio against "
         "this drive's own baseline)" if unsigned else "carState.steeringTorqueEps")
      + f". Only samples where |cmd| > {args.min_cmd:g} count for the ratio.\n")
    w("| mph | active s | cmd p50 | cmd p95 | cmd max | eps p50 | eps p95 | eps max | eps/cmd (median) |")
    w("|---|---|---|---|---|---|---|---|---|")
    for lo, hi in ((0, 25), (25, 45), (45, 60), (60, 80), (80, 200)):
      m = active & (v >= lo * MPH) & (v < hi * MPH)
      if m.sum() < 20:
        continue
      c, e = np.abs(cmd_can[m]), np.abs(eps[m])
      big = c > args.min_cmd
      ratio = f"{np.median(e[big] / c[big]):.2f}" if big.sum() >= 20 else "-"
      w(f"| {lo}-{hi} | {m.sum() / hz:.0f} | {np.median(c):.0f} | {np.percentile(c, 95):.0f} | {c.max():.0f} "
        f"| {np.median(e):.0f} | {np.percentile(e, 95):.0f} | {e.max():.0f} | {ratio} |")
    sat = (np.abs(cmd_can[active]) >= 0.98 * args.steer_max).mean() * 100
    w(f"\n- command at >=98% of steer_max ({args.steer_max:g}): {sat:.1f}% of active samples")

    # --- actuator lag -------------------------------------------------------------------------
    lag, r = estimate_lag(np.abs(cmd_can[active]) if unsigned else cmd_can[active], eps[active], hz)
    w("\n## Actuator lag\n")
    if r:
      w(f"- best cross-correlation of cmd vs EPS torque: lag **{lag:.2f} s** (r = {r:+.2f}); current steerActuatorDelay is 0.15 s")
      w(f"- resolution is 1/{hz:g} s; " + ("both signals are compared as magnitudes (the EPS torque has no sign)" if unsigned else
        "a negative r just means the EPS reports torque with the opposite sign convention"))
    else:
      w("- not enough steering variation to estimate lag")

    # --- assist fade --------------------------------------------------------------------------
    win = max(3, int(args.fade_window * hz))
    c_med = pd.Series(np.abs(cmd_can)).rolling(win, center=True, min_periods=1).median().to_numpy()
    e_med = pd.Series(np.abs(eps)).rolling(win, center=True, min_periods=1).median().to_numpy()
    usable = active & (c_med > args.min_cmd)
    eff = np.full(len(df), np.nan)
    eff[usable] = e_med[usable] / c_med[usable]
    base = np.nanmedian(eff) if np.isfinite(eff).any() else np.nan
    eff_rel = eff / base if base else eff
    fade_mask = np.nan_to_num(eff_rel, nan=9) < args.fade_thresh
    eps_fade = runs(fade_mask & usable, int(args.fade_min_s * hz))
    df["eff_rel"] = eff_rel


    w("\n## Assist fade\n")
    w(f"Efficiency = rolling median |EPS torque| / |command| over {args.fade_window:g} s, relative to this drive's median "
      f"({base:.2f}). A fade is efficiency below {args.fade_thresh:g}x baseline for >= {args.fade_min_s:g} s while the command "
      f"stays above {args.min_cmd:g}.\n")
    if eps_fade:
      w("| start | length s | mph | mean cmd | mean eps | steerFaultTemporary during |")
      w("|---|---|---|---|---|---|")
      for s, e in eps_fade:
        when = fmt_time(df.wall.iloc[s]) if "wall" in df else f"t={df.t.iloc[s]:.1f}"
        w(f"| {when} | {(e - s) / hz:.1f} | {v[s:e].mean() / MPH:.0f} "
          f"| {np.abs(cmd_can[s:e]).mean():.0f} | {np.abs(eps[s:e]).mean():.0f} | {'yes' if fault[s:e].any() else 'no'} |")
    else:
      w("No fade episodes detected.")

    if eps_fade and {"steeringTorque", "steeringPressed", "steeringAngleDeg"} <= set(df.columns):
      # driver vs motor: is someone holding the wheel while the command is high and the motor is low?
      tq = df.steeringTorque.fillna(0).to_numpy()
      pr = df.steeringPressed.fillna(0).to_numpy() > 0
      ang = pd.Series(df.steeringAngleDeg.fillna(0).to_numpy()).rolling(5, center=True, min_periods=1).mean().to_numpy()
      rate = np.gradient(ang) * hz
      infade = np.zeros(len(df), dtype=bool)
      for s_, e_ in eps_fade:
        infade[s_:e_] = True
      w("\n### Driver vs motor during fades\n")
      w("`steeringTorque` is the torsion-bar sensor: it rises when the EPS fails to follow a hard request, whoever is holding the "
        "wheel, so a high value is not proof the driver is fighting. Low `steeringPressed` with a high command and a "
        "near-zero motor torque is the stronger sign of the motor not following. `corr` is the correlation of command "
        "and driver torque (negative = opposed). `under-steer` is how much less than the desired curvature the car achieved "
      "(see Lateral tracking), `-` if the fade is not in a turn.\n")
      w("| start | length s | mph | mean cmd | mean eps | mean driver tq | pressed | mean wheel rate deg/s | corr | under-steer |")
      w("|---|---|---|---|---|---|---|---|---|---|")
      for s_, e_ in eps_fade:
        c_, t_ = cmd_can[s_:e_], tq[s_:e_]
        cc = np.corrcoef(c_, t_)[0, 1] if c_.std() > 0 and t_.std() > 0 else float("nan")
        when = fmt_time(df.wall.iloc[s_]) if "wall" in df else f"t={df.t.iloc[s_]:.1f}"
        w(f"| {when} | {(e_ - s_) / hz:.1f} | {v[s_:e_].mean() / MPH:.0f} | {np.abs(c_).mean():.0f} | {np.abs(eps[s_:e_]).mean():.1f} "
          f"| {np.abs(t_).mean():.0f} | {pr[s_:e_].mean() * 100:.0f}% | {np.abs(rate[s_:e_]).mean():.0f} | {cc:+.2f} "
          f"| {fmt_short(short_frac[s_:e_], turning[s_:e_])} |")
      hi = active & (c_med > 2500) & ~infade
      w("\n| samples | seconds | mean driver tq | pressed | mean eps |")
      w("|---|---|---|---|---|")
      for name, m in (("all lateral-active", active), ("high command (>2500), not in a fade", hi), ("in a fade", infade)):
        if m.sum():
          w(f"| {name} | {m.sum() / hz:.0f} | {np.abs(tq[m]).mean():.0f} | {pr[m].mean() * 100:.0f}% | {np.abs(eps[m]).mean():.1f} |")
  w(f"\n- steerFaultTemporary rising edges: {len(edges)} (the LOW_SPEED_LOCKOUT check is one cause among several)")
  for i in edges[:10]:
    w(f"  - {fmt_time(df.wall.iloc[i]) if 'wall' in df else f't={df.t.iloc[i]:.1f}'} at {v[i] / MPH:.0f} mph")

  # --- lateral tracking ---------------------------------------------------------------------
  if have_track:
    w("\n## Lateral tracking (did the car turn as much as openpilot wanted?)\n")
    w(f"`controlsState.desiredCurvature` (lag-adjusted) vs `curvature` (achieved). Shortfall = how much less the car turned, as a share of "
      f"the desired curvature, only where |desired| > {args.min_curv:g} 1/m (radius under {1 / args.min_curv:.0f} m) and lateral-active above "
      f"{args.min_speed:g} mph. Under-steer = shortfall above {args.under_frac * 100:.0f}%.\n")
    w("| mph | turning s | under-steer share | median shortfall | p95 shortfall | command at ceiling during under-steer |")
    w("|---|---|---|---|---|---|")
    under = turning & (short_frac > args.under_frac)
    cmd_sat = np.abs(cmd_can) >= 0.98 * args.steer_max
    for lo, hi in ((20, 45), (45, 60), (60, 80), (80, 200)):
      m = turning & (v >= lo * MPH) & (v < hi * MPH)
      if m.sum() < 20:
        continue
      u = m & under
      sat_u = f"{cmd_sat[u].mean() * 100:.0f}%" if u.sum() else "-"
      w(f"| {lo}-{hi} | {m.sum() / hz:.0f} | {u.sum() / m.sum() * 100:.1f}% | {np.median(short_frac[m]) * 100:.0f}% "
        f"| {np.percentile(short_frac[m], 95) * 100:.0f}% | {sat_u} |")
    eps_ep = runs(under, int(args.under_min_s * hz))
    pressed_all = df.steeringPressed.fillna(0).to_numpy() > 0 if "steeringPressed" in df else np.zeros(len(df), dtype=bool)
    w(f"\nUnder-steer episodes of at least {args.under_min_s:g} s: **{len(eps_ep)}**. Label is a heuristic: `assist-limited` = driver "
      "pressed under 25% of the time (the car fell short without you pushing), `driver-involved` = pressed 50% or more "
      "(you were likely correcting), `mixed` otherwise.\n")
    if eps_ep:
      w("| start | length s | mph | radius m | mean shortfall | mean cmd | at ceiling | mean eps | pressed | angle err deg | label |")
      w("|---|---|---|---|---|---|---|---|---|---|---|")
      counts = {"assist-limited": 0, "mixed": 0, "driver-involved": 0}
      for s_, e_ in eps_ep:
        pc = pressed_all[s_:e_].mean()
        label = "assist-limited" if pc < 0.25 else "driver-involved" if pc >= 0.5 else "mixed"
        counts[label] += 1
        when = fmt_time(df.wall.iloc[s_]) if "wall" in df else f"t={df.t.iloc[s_]:.1f}"
        ae = df.angle_err_deg.iloc[s_:e_].abs().mean() if "angle_err_deg" in df else float("nan")
        w(f"| {when} | {(e_ - s_) / hz:.1f} | {v[s_:e_].mean() / MPH:.0f} | {1 / np.abs(des[s_:e_]).mean():.0f} "
          f"| {np.mean(short_frac[s_:e_]) * 100:.0f}% | {np.abs(cmd_can[s_:e_]).mean():.0f} | {cmd_sat[s_:e_].mean() * 100:.0f}% "
          f"| {np.abs(eps[s_:e_]).mean():.1f} | {pc * 100:.0f}% | {ae:.1f} | {label} |")
      w(f"\n- by label: {counts['assist-limited']} assist-limited, {counts['mixed']} mixed, {counts['driver-involved']} driver-involved")
      fade_ids = np.zeros(len(df), dtype=bool)
      for s_, e_ in eps_fade:
        fade_ids[s_:e_] = True
      both = sum(fade_ids[s_:e_].any() for s_, e_ in eps_ep)
      w(f"- {both} of these episodes overlap an assist-fade episode (command high, motor torque low)")

  # --- STEER_STATUS -------------------------------------------------------------------------
  if "steer_status" in df and df.steer_status.notna().any():
    st = df.steer_status.fillna(-1).to_numpy().astype(int)
    w("\n## STEER_STATUS while lateral-active\n")
    w(f"Raw EPS status broadcast (0x18F), lateral-active above {args.min_speed:g} mph. `eps/cmd` is the median EPS-torque over "
      "command ratio (cmd above the minimum) in that state, to show whether assist really drops in a given status.\n")
    w("| status | seconds | share | eps/cmd (median) |")
    w("|---|---|---|---|")
    for code in sorted(set(st[active])):
      m = active & (st == code)
      big = m & (np.abs(cmd_can) > args.min_cmd)
      ratio = f"{np.median(np.abs(eps[big]) / np.abs(cmd_can[big])):.2f}" if have_eps and big.sum() >= 20 else "-"
      w(f"| {STEER_STATUS_NAMES.get(code, 'missing' if code < 0 else f'unknown_{code}')} | {m.sum() / hz:.0f} "
        f"| {m.sum() / max(active.sum(), 1) * 100:.1f}% | {ratio} |")
    lock = active & (st == 3)
    if lock.any():
      starts = np.where(np.diff(np.r_[0, lock.astype(np.int8)]) == 1)[0]
      w(f"\n- low_speed_lockout episodes while lateral-active: {len(starts)}; "
        f"{(fault[lock]).mean() * 100:.0f}% of that time steerFaultTemporary was also set")

  # --- radar --------------------------------------------------------------------------------
  if "lead_present" in df:
    w("\n## Radar / lead\n")
    st = df.lead_present.fillna(0).to_numpy() > 0
    w(f"- lead present: {st.sum() / nvalid * 100:.1f}% of the drive")
    if st.any():
      w(f"- of that, radar-matched (not vision-only): {(df.lead_radar.fillna(0).to_numpy()[st] > 0).mean() * 100:.0f}%")
      w(f"- lead distance: median {df.lead_dRel[st].median():.0f} m, min {df.lead_dRel[st].min():.0f} m; "
        f"lead speed: median {df.lead_vLead[st].median() / MPH:.0f} mph")
  else:
    w("\n## Radar / lead\n\nNo radarState in these logs.")

  # --- OBD covariates -----------------------------------------------------------------------
  obd_cols = [c for c in df.columns if c.startswith("obd_")]
  if obd_cols:
    w("\n## OBD covariates\n")
    w("Mean of each OBD signal over lateral-active time vs during fade episodes, and Pearson r with the efficiency measure. "
      "Correlation, not cause: use this to pick what to look at.\n")
    fm = np.zeros(len(df), dtype=bool)
    for s, e in eps_fade:
      fm[s:e] = True
    w("| signal | valid | active mean | fade mean | r vs efficiency |")
    w("|---|---|---|---|---|")
    table = []
    for c in obd_cols:
      x = df[c].to_numpy(dtype=float)
      ok = np.isfinite(x) & active
      if ok.sum() < 10 or np.nanstd(x[ok]) == 0:
        continue
      okr = ok & np.isfinite(eff_rel)
      rr = np.corrcoef(x[okr], eff_rel[okr])[0, 1] if okr.sum() > 10 and np.nanstd(x[okr]) > 0 else np.nan
      fmean = f"{np.nanmean(x[fm & ok]):.2f}" if (fm & ok).any() else "-"
      table.append((abs(rr) if np.isfinite(rr) else -1, f"| {c[4:]} | {ok.sum()} | {np.nanmean(x[ok]):.2f} | {fmean} | {rr:+.2f} |"))
    for _, row in sorted(table, reverse=True):
      w(row)
    if not table:
      w("| (no OBD signal had enough valid, varying samples) | | | | |")
  return "\n".join(L), df, eps_fade


def cmd_report(args):
  df = pd.read_csv(args.drive)
  hz = 1.0 / float(np.median(np.diff(df.t.to_numpy())))
  if args.obd:
    if "wall" not in df or df.wall.isna().all():
      if not args.obd_start:
        raise SystemExit("drive CSV has no wall clock; pass --obd-start (ISO time of the first drive row)")
      df["wall"] = pd.Timestamp(args.obd_start).timestamp() + (df.t - df.t.iloc[0])
    obd = load_obd(args.obd, args.obd_tz, args.obd_offset, pd.Timestamp(args.obd_start).timestamp() if args.obd_start else None)
    overlap = max(0.0, min(df.wall.max(), obd.wall.max()) - max(df.wall.min(), obd.wall.min()))
    print(f"OBD overlap with drive: {overlap / 60:.1f} min ({len(obd.columns) - 1} signals)", file=sys.stderr)
    if overlap <= 0:
      print("warning: OBD and drive do not overlap in time; check --obd-tz / --obd-offset", file=sys.stderr)
    df = attach_obd(df, obd, args.obd_tol)
  text, merged, _ = analyze(df, args, hz)
  print(text)
  if args.out:
    Path(args.out).write_text(text + "\n")
  if args.merged_csv:
    merged.to_csv(args.merged_csv, index=False)
  return 0


def main(argv=None):
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  sub = p.add_subparsers(dest="cmd", required=True)

  a = sub.add_parser("pull", help="rsync rlog/qlog segments from the comma device")
  a.add_argument("--host", required=True, help="device IP or hostname")
  a.add_argument("--dest", default="~/rdx_logs")
  a.add_argument("--last", type=int, help="only the N most recent segment dirs")
  a.add_argument("--since", help="only segment dirs modified after this date/time, e.g. 2026-10-28")
  a.set_defaults(fn=cmd_pull)

  a = sub.add_parser("extract", help="decode rlogs into one resampled CSV")
  a.add_argument("paths", nargs="+")
  a.add_argument("-o", "--out", default="drive.csv")
  a.add_argument("--hz", type=float, default=50.0)
  a.set_defaults(fn=cmd_extract)

  a = sub.add_parser("report", help="analyze a drive CSV, optionally lined up with an OBD log")
  a.add_argument("drive")
  a.add_argument("--obd", help="OBD CSV: one time column + one column per signal")
  a.add_argument("--obd-tz", default="local", help="timezone for naive OBD timestamps (default: this machine's)")
  a.add_argument("--obd-offset", type=float, default=0.0, help="seconds added to OBD times (clock drift correction)")
  a.add_argument("--obd-start", help="ISO time, for relative OBD timestamps or logs without a wall clock")
  a.add_argument("--obd-tol", type=float, default=3.0, help="max age in seconds of an OBD sample used for a row")
  a.add_argument("--steer-max", type=float, default=4095.0, help="CAN torque at actuator 1.0 (RDX 3G: 4095)")
  a.add_argument("--min-speed", type=float, default=20.0, help="mph above which to analyze (EPS min is ~3 mph)")
  a.add_argument("--min-cmd", type=float, default=600.0, help="ignore |cmd| below this (CAN units) for ratios and fades")
  a.add_argument("--fade-window", type=float, default=1.0)
  a.add_argument("--fade-thresh", type=float, default=0.3, help="fade = efficiency below this fraction of baseline")
  a.add_argument("--fade-min-s", type=float, default=2.0)
  a.add_argument("--min-curv", type=float, default=0.002, help="desired |curvature| (1/m) above which a sample counts as a turn")
  a.add_argument("--under-frac", type=float, default=0.3, help="under-steer = achieved curvature short of desired by this fraction")
  a.add_argument("--under-min-s", type=float, default=1.0, help="minimum under-steer episode length in seconds")
  a.add_argument("-o", "--out", help="also write the report here")
  a.add_argument("--merged-csv", help="write the drive+OBD merged table here")
  a.set_defaults(fn=cmd_report)

  args = p.parse_args(argv)
  return args.fn(args)


if __name__ == "__main__":
  sys.exit(main())
