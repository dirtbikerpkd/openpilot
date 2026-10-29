#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10,<3.14"
# dependencies = ["pycapnp", "zstandard", "numpy", "pandas"]
# ///
"""Synthetic end-to-end check for rdx_analysis.py (no real drive logs needed).

  uv run --python 3.12 tools/rdx_analysis/test_rdx_analysis.py

Builds a 90 s rlog with a known 0.15 s EPS lag, one 8 s assist fade (EPS torque decays to 0 with
steerFaultTemporary raised), a lead vehicle for the first 30 s, and a matching OBD CSV, then checks
the analysis recovers them.
"""
import datetime as dt
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
import pandas as pd
import zstandard

sys.path.insert(0, str(Path(__file__).parent))
import rdx_analysis as ra  # noqa: E402

HZ = 100
LAG_S = 0.15
STEER_MAX = 4095
WALL0 = dt.datetime(2026, 11, 2, 14, 0, 0, tzinfo=dt.timezone.utc).timestamp()
MONO0 = 5000.0
FADE = (50.0, 58.0)


def build_rlog(path):
  log = ra.load_schema()
  rng = np.random.default_rng(1)
  out = bytearray()

  def add(t, which, fill):
    ev = log.Event.new_message(logMonoTime=int((MONO0 + t) * 1e9))
    fill(ev.init(which))
    out.extend(ev.to_bytes())

  add(0.0, "clocks", lambda m: setattr(m, "wallTimeNanos", int((WALL0 + 0.0) * 1e9)))

  def cmd_at(t):
    return 2200 * np.sin(2 * np.pi * t / 7.0) + 900 * np.sin(2 * np.pi * t / 2.3)

  for i in range(int(90 * HZ)):
    t = i / HZ
    cmd = cmd_at(t)
    in_fade = FADE[0] <= t < FADE[1]
    gain = 0.9 * (max(0.0, 1.0 - (t - FADE[0]) / 3.0) if in_fade else 1.0)
    eps = gain * cmd_at(t - LAG_S) + rng.normal(0, 10)
    add(t, "carOutput", lambda m, c=cmd: setattr(m.actuatorsOutput, "torque", float(c / STEER_MAX)))
    add(t, "carControl", lambda m: setattr(m, "latActive", True))

    def cs(m, in_fade=in_fade):
      # like the real car: carState.steeringTorqueEps stays 0; the EPS torque is only on CAN
      m.vEgo = 28.0
      m.steerFaultTemporary = bool(in_fade and t > FADE[0] + 3.0)
    add(t, "carState", cs)
    if i % 2 == 0:  # 50 Hz, like the real STEER_MOTOR_TORQUE
      mt = int(min(1023, abs(eps) / 4))  # 10-bit magnitude, different scale from the command
      status = 3 if (in_fade and t > FADE[0] + 3.0) else 0
      def frames(m, mt=mt, status=status):
        m[0].address, m[0].src = ra.ADDR_STEER_MOTOR_TORQUE, 1
        m[0].dat = bytes([(mt >> 8) & 3, mt & 0xFF, 0])
        m[1].address, m[1].src = ra.ADDR_STEER_STATUS, 1
        m[1].dat = bytes([0, 0, 0, 0, status << 4, 0, 0])
        m[2].address, m[2].src = ra.ADDR_STEER_MOTOR_TORQUE, 129  # sendcan loopback, must be ignored
        m[2].dat = bytes([3, 255, 0])
      ev = log.Event.new_message(logMonoTime=int((MONO0 + t) * 1e9))
      frames(ev.init("can", 3))
      out.extend(ev.to_bytes())
    if i % 5 == 0 and t < 30:
      def lead(m):
        m.leadOne.present = True
        m.leadOne.radar = True
        m.leadOne.dRel = 40.0
        m.leadOne.vLead = 25.0
      add(t, "radarState", lead)
  raw = bytes(out)
  Path(path).write_bytes(zstandard.ZstdCompressor().compress(raw))


def build_obd(path):
  # 1 Hz poll; the "outside air temp" drops during the fade so it should show up as a correlated covariate.
  rows = []
  for s in range(90):
    t = dt.datetime.fromtimestamp(WALL0 + s, dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%S+00:00")
    rows.append((t, 12.0 - (4.0 if FADE[0] <= s < FADE[1] else 0.0), 20.0 + (s % 3)))
  pd.DataFrame(rows, columns=["timestamp", "RDX_AAT_FIRSTGEN", "RDX_GENERATOR"]).to_csv(path, index=False)


class TestRdxAnalysis(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.tmp = tempfile.TemporaryDirectory()
    d = Path(cls.tmp.name)
    seg = d / "route--0"
    seg.mkdir()
    build_rlog(seg / "rlog.zst")
    build_obd(d / "obd.csv")
    cls.drive = d / "drive.csv"
    cls.obd = d / "obd.csv"
    cls.report_path = d / "report.md"
    assert ra.main(["extract", str(d), "-o", str(cls.drive), "--hz", "50"]) == 0
    assert ra.main(["report", str(cls.drive), "--obd", str(cls.obd), "--obd-tz", "UTC", "-o", str(cls.report_path),
                    "--merged-csv", str(d / "merged.csv")]) == 0
    cls.text = cls.report_path.read_text()
    cls.merged = pd.read_csv(d / "merged.csv")

  @classmethod
  def tearDownClass(cls):
    cls.tmp.cleanup()

  def test_wall_clock_alignment(self):
    self.assertAlmostEqual(pd.read_csv(self.drive).wall.iloc[0], WALL0, delta=0.1)

  def test_lag_recovered(self):
    line = next(ln for ln in self.text.splitlines() if "best cross-correlation" in ln)
    lag = float(line.split("lag **")[1].split(" s")[0])
    self.assertAlmostEqual(lag, LAG_S, delta=0.05)

  def test_single_fade_found_with_fault(self):
    fade_rows = [ln for ln in self.text.split("## Assist fade")[1].split("### Driver vs motor")[0].splitlines()
                 if ln.startswith("| 20")]
    self.assertEqual(len(fade_rows), 1, self.text)
    self.assertTrue(fade_rows[0].rstrip("| ").endswith("yes"))
    length = float(fade_rows[0].split("|")[2])
    self.assertGreater(length, 3.0)

  def test_fault_edge_counted(self):
    self.assertIn("rising edges: 1", self.text)

  def test_radar_lead_fraction(self):
    line = next(ln for ln in self.text.splitlines() if "lead present" in ln)
    pct = float(line.split(": ")[1].split("%")[0])
    self.assertAlmostEqual(pct, 100 * 30 / 90, delta=3.0)
    self.assertIn("radar-matched (not vision-only): 100%", self.text)

  def test_obd_aligned_and_ranked(self):
    self.assertIn("obd_RDX_AAT_FIRSTGEN", self.merged.columns)
    section = self.text.split("## OBD covariates")[1]
    first = next(ln for ln in section.splitlines() if ln.startswith("| RDX_"))
    self.assertTrue(first.startswith("| RDX_AAT_FIRSTGEN"), first)

  def test_motor_torque_decoded_from_can(self):
    d = pd.read_csv(self.drive)
    self.assertGreater(d.eps_motor.max(), 100)
    self.assertLess(d.eps_motor.max(), 1024)  # the 1023 loopback frame was ignored
    self.assertTrue((d.steeringTorqueEps.fillna(0) == 0).all())
    self.assertIn("decoded from CAN", self.text)

  def test_steer_status_table(self):
    section = self.text.split("## STEER_STATUS")[1].split("## Radar")[0]
    self.assertIn("| low_speed_lockout |", section)
    self.assertIn("| normal |", section)
    self.assertIn("low_speed_lockout episodes while lateral-active: 1", section)

  def test_driver_vs_motor_section(self):
    section = self.text.split("### Driver vs motor during fades")[1].split("## STEER_STATUS")[0]
    self.assertEqual(len([ln for ln in section.splitlines() if ln.startswith("| 20")]), 1, section)
    self.assertIn("| in a fade |", section)
    self.assertIn("| all lateral-active |", section)

  def test_decoders(self):
    self.assertEqual(ra.decode_motor_torque(bytes([0x02, 0x34, 0x00])), 0x234)
    self.assertEqual(ra.decode_steer_status(bytes([0, 0, 0, 0, 0x38, 0, 0])), (3, 1))
    self.assertIsNone(ra.decode_motor_torque(b""))

  def test_authority_table_present(self):
    self.assertIn("EPS authority by speed", self.text)
    self.assertIn("| 60-80 |", self.text)  # 28 m/s = 62.6 mph


if __name__ == "__main__":
  unittest.main()
