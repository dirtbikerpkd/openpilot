# Signals and metrics reference

What every column and number in `rdx_analysis.py` means, and which OBD signals are worth
logging on an RDX 3G (2019-21). See [README.md](README.md) for how to run it.

## Comma signals used

These come from the openpilot logs (`rlog`). `extract` writes one row per time step with the
value of each at that moment.

| column | source | unit | what it is | why it matters here |
|---|---|---|---|---|
| `t` | `logMonoTime` | s | boot-relative monotonic time | continuous across segments |
| `wall` | `clocks` / `initData` | s (Unix) | wall-clock time of the row | aligns the OBD log |
| `vEgo` | `carState` | m/s | vehicle speed | EPS assist and the `LOW_SPEED_LOCKOUT` issue are speed dependent |
| `steeringAngleDeg` | `carState` | deg | steering wheel angle | context for tight curves |
| `steeringTorque` | `carState` | CAN units | driver torque on the wheel | high values mean the driver is fighting or helping |
| `steeringTorqueEps` | `carState` | CAN units | **always 0 on Honda**: openpilot's Honda carState never fills it | kept only for non-Honda logs; the report falls back to it when `eps_motor` is absent |
| `eps_motor` | raw `can`, addr 0x1AB (`STEER_MOTOR_TORQUE`) | EPS units, 0-1023 | the EPS motor torque, 10-bit unsigned magnitude (no sign), 50 Hz | **the key number**: what the EPS actually delivers. Not the same scale as the command (RDX 3G: roughly 1/50 of it) |
| `steer_status` | raw `can`, addr 0x18F (`STEER_STATUS`) | code 0-7 | EPS status: 0 normal, 1 driver_steering, 2 no_torque_alert_1, 3 low_speed_lockout, 4 no_torque_alert_2, 5 fault_1, 6 tmp_fault, 7 permanent_fault | shows `LOW_SPEED_LOCKOUT` directly, independent of the carState fault flag |
| `curvature` | `controlsState` | 1/m | curvature the car actually achieved (vehicle model) | what the turn really was |
| `desiredCurvature` | `controlsState` | 1/m | lag-adjusted curvature the lateral controller is trying to hit | what openpilot wanted |
| `angle_des_deg`, `angle_err_deg` | `controlsState.lateralControlState.pidState` | deg | PID desired steering angle and error (NaN for non-PID cars) | size of the miss in degrees |
| `lat_output`, `lat_saturated` | `controlsState.lateralControlState` | normalized, bool | controller output and its saturation flag | the controller asking for everything it can |
| `steer_ctrl_active` | raw `can`, addr 0x18F | bool | EPS reports it is applying openpilot's torque request | context |
| `steeringPressed` | `carState` | bool | driver is overriding | excludes driver-caused apparent fades |
| `steerFaultTemporary` | `carState` | bool | temporary steer fault | on the RDX 3G this also flags `LOW_SPEED_LOCKOUT` above the minimum steer speed (the silent assist-loss fix) |
| `steerFaultPermanent` | `carState` | bool | permanent steer fault | should stay false |
| `gasPressed`, `brakePressed` | `carState` | bool | pedal state | context |
| `cmd_torque` | `carOutput.actuatorsOutput.torque` | normalized | the torque request actually sent to the car after rate limits | multiplied by `--steer-max` to get CAN units |
| `latActive` | `carControl` | bool | openpilot is steering | restricts analysis to lateral-active time |
| `sd_enabled`, `sd_active` | `selfdriveState` | bool | openpilot enabled / actively engaged | context |
| `lead_present` | `radarState.leadOne.present` | bool | a lead vehicle exists | radar/lead availability |
| `lead_radar` | `radarState.leadOne.radar` | bool | the lead is matched to a radar track (not vision-only) | shows the Bosch-A radar decoder is contributing |
| `lead_dRel` | `radarState.leadOne` | m | distance to lead | |
| `lead_vLead` | `radarState.leadOne` | m/s | absolute lead speed | what the on-screen lead speed shows |
| `lead_vRel` | `radarState.leadOne` | m/s | lead speed relative to you | |

## What the report computes

### Span and activity

Duration, the share of time lateral control was active above `--min-speed`, and the share of
time openpilot was engaged. If lateral-active time is too small, the steering sections are
skipped.

### EPS authority by speed

For lateral-active samples, grouped into speed bins (mph):

- **cmd p50 / p95 / max** are the commanded torque in CAN units (`cmd_torque * steer-max`),
  absolute value.
- **eps p50 / p95 / max** are the absolute EPS torque (`eps_motor`, else `steeringTorqueEps`).
- **eps/cmd (median)** is the median of EPS torque over command, only for samples where the
  command exceeds `--min-cmd`. This is roughly how much of the request the EPS delivers.
  A falling ratio at higher speed points to reduced authority.
- **Saturation** is the share of active samples where the command is at least 98% of
  `--steer-max`. High values mean the controller is running out of range and not the EPS.

### Actuator lag

Cross-correlates the command with the EPS torque over lag 0-0.6 s (as magnitudes when the EPS torque comes from `eps_motor`, which has no sign; signed otherwise) and reports
the lag with the strongest correlation, plus the correlation `r`. Compare this with
`steerActuatorDelay` (0.15 s on this branch). `r` of small magnitude means the signals did not
track each other, so do not trust the lag.

### Assist fade

1. Take the rolling median (`--fade-window`, 1 s) of `|EPS torque|` and `|command|`.
2. **Efficiency** is `|EPS| / |command|`, computed only where the command exceeds `--min-cmd`.
3. Divide by the drive's median efficiency to get a relative value (`eff_rel` in the merged
   CSV). 1.0 is typical for this drive.
4. A **fade** is relative efficiency below `--fade-thresh` for at least `--fade-min-s`.

Each fade row shows start time, length, mean speed, mean command and EPS torque, and whether
`steerFaultTemporary` was set at any point in it. A fade with a fault is the pattern recorded
for the `LOW_SPEED_LOCKOUT` problem: the EPS keeps reporting `LOW_SPEED_LOCKOUT` while its motor
torque decays toward zero over 8-15 seconds at 58-80 mph.

### Driver vs motor during fades

For each fade: mean driver torque, the share of the fade with `steeringPressed`, mean wheel rate,
and the correlation between command and driver torque (negative means opposed). A second table
compares all lateral-active samples, high-command samples outside fades, and fade samples.
`steeringTorque` is the torsion-bar sensor, so it rises when the EPS fails to follow a hard
request regardless of who holds the wheel; low `steeringPressed` with a high command and
near-zero motor torque is the stronger sign of the motor not following.

The report also lists every `steerFaultTemporary` rising edge with time and speed.

### Lateral tracking

Compares `desiredCurvature` with `curvature` where lateral-active above `--min-speed` and
`|desired| > --min-curv` (default 0.002 1/m, radius under 500 m). Shortfall is
`sign(desired) * (desired - achieved) / |desired|`: positive means the car turned less than
wanted (under-steer), negative means it turned more. Samples with shortfall above
`--under-frac` (30%) are under-steer; runs of at least `--under-min-s` (1 s) are episodes.
Each episode gets a heuristic label from how much of it `steeringPressed` was set:
`assist-limited` (under 25%: the car fell short without the driver pushing), `driver-involved`
(50% or more), `mixed`. "At ceiling" is the share of the episode with the command at 98% or
more of `--steer-max`. The fade table also gets an `under-steer` column (mean shortfall in the
turning part of the fade, `-` if the fade was not in a turn).

### STEER_STATUS while lateral-active

Seconds and share of lateral-active time in each raw EPS status, with the median EPS/command
ratio in that status, and the number of `low_speed_lockout` episodes. A lockout at speed with a
collapsing ratio is the silent assist loss. If this table is all `normal`, that failure did not
happen in the drive, whatever `steerFaultTemporary` did at low speed.

### Radar / lead

Share of the drive with a lead, how much of that was radar-matched rather than vision-only,
and median/minimum lead distance and median lead speed. A low radar-matched share while a car
is clearly ahead suggests the radar decoder is not feeding the lead.

### OBD covariates

For each OBD column with enough valid, varying samples: its mean over lateral-active time, its
mean during fade episodes, and its Pearson correlation `r` with relative efficiency. Rows are
sorted by `|r|`. This ranks what to inspect. It does not show cause.

## OBD signals for the RDX 3G

Source: the community [OBDb/Acura-RDX](https://github.com/OBDb/Acura-RDX) signal set
(CC-BY-SA-4.0). Its `tests/test_cases` list which requests answered on real cars. For the
2019 and 2020 RDX these ECUs answered: `DA10` and `DA11` (engine/powertrain), `DA1D`
(transmission), `DA26` (tire pressure), `DA60` (gauges), plus standard OBD-II. For 2021 only
standard OBD-II was recorded. Everything below is read-only requests (UDS service `22`).

The signal set has **no EPS, steering, radar, or camera signals**. Steering torque comes from
the comma logs, not from OBD.

IDs are the OBDb signal IDs. Scaling and bit positions are in the OBDb `default.json`; most
temperatures there are raw minus 40 °C.

### Engine / powertrain (`DA10`, `DA11`)

| request | signal | what it is |
|---|---|---|
| `2610` | `RDX_GENERATOR` | alternator load, percent |
| `2610` | `RDX_AC_CLUTCH` | A/C compressor clutch state |
| `2611` | `RDX_FUEL_AIR_RATIO`, `RDX_AIRGAS_RATIO` | commanded fuel/air equivalence ratio |
| `2612` | `RDX_VTEC_PSW`, `RDX_VTEC_SOL`, `RDX_VTEC_VALVE_CTRL` | VTEC pressure switch and solenoid state (2020, `DA11`) |
| `2615` | `RDX_ACTIVE_CYL` | number of active cylinders |
| `2660` | `RDX_ODO` (km), `RDX_RUNTM` (s) | odometer and time since engine start |
| `2661` | `RDX_FUELAIR_CAR` | fuel/air equivalence ratio |
| `2662` | `RDX_KNOCK_CTRL`, `RDX_KNOCK_CONTROL_V2` | knock control correction |
| `2662` | `RDX_VTC_DUTY` | variable valve timing duty, percent |
| `2662` | `RDX_CAT_T` | catalyst temperature (`DA11`) |
| `2663` | `RDX_MISFIRE_1`-`6` | per-cylinder misfire counters |
| `2666` | `RDX_EOT_VAR_1`/`2` | engine oil temperature (`DA11`) |

Useful for steering analysis only as context: alternator load and A/C clutch show electrical
and engine load, which you could compare against assist fades. The 3G's turbo lag logic in
`carcontroller.py` is the other reason to log engine state.

### Transmission (`DA1D`)

| request | signal | what it is |
|---|---|---|
| `2201` | `RDX_ATF_T_NEW` | transmission fluid temperature |
| `2203` | `RDX_TRANS_OIL_P2` | transmission oil pressure sensor 2, kPa |
| `2221` | `RDX_GEAR_1D` | current gear (2020 only; needs flow control) |
| `2660` | `RDX_ODO_V2` | odometer |

### Tires (`DA26`)

| request | signal | what it is |
|---|---|---|
| `6001` | `RDX_TP_FL`, `_FR`, `_RL`, `_RR` | tire pressure, kPa |
| `6001` | `RDX_TT_FL`, `_FR`, `_RL`, `_RR` | tire temperature, °C |

Tire pressure changes steering effort, so a low or uneven tire is a plausible confound for
"weak" steering. Worth logging.

### Gauges (`DA60`)

| request | signal | what it is |
|---|---|---|
| `7028` | `RDX_AAT_FIRSTGEN` | outside air temperature, °C |
| `7029` | `RDX_FLV` | fuel level, liters |

Outside air temperature is a cheap covariate: cold EPS fluid or grease is a candidate cause
for changes in assist between drives.

### Standard OBD-II (functional request, mode `01`)

Seen answering on the 2019-21 cars: engine RPM (`010C`), vehicle speed (`010D`), calculated
load (`0104`), manifold pressure (`010B`), spark advance (`010E`), throttle position (`0111`),
barometric pressure (`0133`), catalyst temperature (`013C`), fuel trim, fuel rail pressure,
warm-ups and distance since codes cleared. Any OBD-II app can log these. Vehicle speed is
handy for sanity-checking time alignment against `vEgo`.

### Not confirmed on a 3G

The OBDb file also defines brake signals (`DA28` brake switch/pressure, `DA2A` brake booster
stroke), more engine variants (`DA0E`), and several maintenance counters. None of these
answered in the 2019-21 support lists, and `DA2A` is not the brake-booster address on this
car (openpilot's fingerprint uses `0x18DA2BF1`). They may belong to other generations, so do
not rely on them without testing.

### Not covered at all

Steering torque or motor current, EPS temperature or voltage, EPS status, radar objects,
camera. For the EPS, the broadcast CAN messages (`STEER_STATUS`, `STEER_MOTOR_TORQUE`) are the
source. `carState` does not carry the motor torque on Honda, so `extract` decodes both frames from
the rlog `can` events (0x1AB and 0x18F; layouts from opendbc `_honda_common.dbc` and
`_bosch_2018.dbc`; checksums are not verified). qlog is decimated and may not carry them.
