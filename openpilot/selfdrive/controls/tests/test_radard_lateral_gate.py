from types import SimpleNamespace

from openpilot.selfdrive.controls.radard import match_vision_to_track, LAT_SANE_FLOOR_M


def camera_lead(x=28.5, y=0.2, v=16.4, y_std=0.3):
  # model lead: x is camera-frame distance (radard subtracts RADAR_TO_CAMERA), y is +right so track.yRel ~= -y
  return SimpleNamespace(x=[x], y=[y], v=[v], xStd=[2.0], yStd=[y_std], vStd=[1.0])


def track(d_rel, y_rel, v_rel):
  return SimpleNamespace(dRel=d_rel, yRel=y_rel, vRel=v_rel)


V_EGO = 16.6


def test_matching_in_lane_track_is_accepted():
  t = track(27.0, -0.2, 0.1)
  assert match_vision_to_track(V_EGO, camera_lead(), {1: t}) is t


def test_oncoming_off_lane_track_with_a_clamped_speed_is_rejected():
  # 2026-09-30 false FCW: oncoming car at yRel +6.3 whose clipped vRel of -13.5 gave v_ego + vRel = 3.1 (> 3),
  # distance within 25% of the camera lead. Only the lateral gate stops it.
  t = track(27.0, 6.3, -13.5)
  assert V_EGO + t.vRel > 3
  assert match_vision_to_track(V_EGO, camera_lead(), {1: t}) is None


def test_off_lane_track_is_rejected_even_when_the_real_lead_track_is_missing():
  assert match_vision_to_track(V_EGO, camera_lead(), {1: track(27.0, -6.5, 0.0)}) is None


def test_moderately_offset_track_inside_the_gate_is_still_accepted():
  # a wide vehicle / curve offset below the floor must not regress to camera-only
  t = track(27.0, -(LAT_SANE_FLOOR_M - 0.5), 0.0)
  assert match_vision_to_track(V_EGO, camera_lead(), {1: t}) is t


def test_gate_widens_with_camera_y_uncertainty():
  t = track(27.0, -(LAT_SANE_FLOOR_M + 0.5), 0.0)
  assert match_vision_to_track(V_EGO, camera_lead(y_std=0.3), {1: t}) is None
  assert match_vision_to_track(V_EGO, camera_lead(y_std=2.0), {1: t}) is t


def test_best_ranked_track_decides_and_off_lane_winner_is_not_replaced_by_a_worse_track():
  # same behaviour as stock radard: the max-probability track is checked and, if insane, there is no match
  near_lane = track(27.0, -0.2, 0.1)
  off_lane_but_closer_in_speed = track(27.0, 6.3, 0.1)
  assert match_vision_to_track(V_EGO, camera_lead(), {1: near_lane, 2: off_lane_but_closer_in_speed}) is near_lane
