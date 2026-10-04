"""Real frame source + real detector plumbed into the mission driver
(flight/deploy/frame_feed.py, real_flight --source / --desk)."""
import ast
import inspect
import json
import os
import sys

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
pytest.importorskip("pupil_apriltags")

from flight.camera import CameraModel  # noqa: E402
from flight.deploy import frame_feed as ff  # noqa: E402
from flight.deploy import real_flight as rf  # noqa: E402
from flight.deploy.seeker_loop import ImageDirSource  # noqa: E402

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_TAG_PNG = os.path.join(_REPO, "models", "apriltag_target", "tag36h11_00000.png")
FX = 539.936


def _write_tag_frames(d, n, w=1280, h=960):
    os.makedirs(d, exist_ok=True)
    tag = cv2.imread(_TAG_PNG, cv2.IMREAD_GRAYSCALE)
    for k in range(n):
        side = 10 * (6 + k // 4)
        u, v = w // 2 + 30 + k, h // 2
        g = np.full((h, w), 128, np.uint8)
        g[v - side // 2:v - side // 2 + side, u - side // 2:u - side // 2 + side] = \
            cv2.resize(tag, (side, side), interpolation=cv2.INTER_NEAREST)
        cv2.imwrite(os.path.join(d, f"f{k:04d}.png"), np.dstack([g] * 3))
    return d


def _intrinsics(path, w=1280, h=960, source=None):
    blob = {"fx": FX, "fy": FX, "cx": w / 2, "cy": h / 2,
            "resolution": {"width": w, "height": h}}
    if source:
        blob["source"] = source
    with open(path, "w") as fh:
        json.dump(blob, fh)
    return str(path)


class _SpyInner:
    def __init__(self, inner):
        self.inner, self.frames = inner, []

    def detect(self, frame, t=None):
        self.frames.append(frame)
        return self.inner.detect(frame, t)


class _SpyRecorder:
    def __init__(self):
        self.offers = []

    def offer(self, frame, t_capture):
        self.offers.append((frame, t_capture))
        return True

    def close(self):
        return {"n_written": len(self.offers), "n_dropped": 0, "out_dir": "spy"}


def _desk_args(tmp_path, src, *extra):
    return ["--desk", "--source", src, "--detector", "tag",
            "--target-span-m", "0.5", "--smoke-acquire-after-s", "0",
            "--mission-max-s", "20", "--log-csv", str(tmp_path / "log" / "d.csv"),
            *extra]


def test_real_frames_reach_detector_and_recorder(tmp_path):
    d = _write_tag_frames(str(tmp_path / "replay"), 12)
    cam = CameraModel(FX, FX, 640.0, 480.0, width=1280, height=960)
    feed = ff.FrameFeed(ImageDirSource(d), threaded=False).start()
    assert ff.preflight(feed, cam) is None
    spy = _SpyInner(ff.TagFrameDetector(cam, 0.5))
    det = ff.FeedDetector(feed, spy)
    rec = _SpyRecorder()
    args = rf.build_arg_parser().parse_args(_desk_args(tmp_path, "dir:" + d))
    cfg = rf.build_config(args)
    gcfg = rf.GuidanceConfig(alt_ref_m=cfg.dash_base_alt_m)
    gcfg.target_span_m = 0.5
    sm = rf.RealFlightSM(cfg, guidance=rf.build_terminal(args, cfg, gcfg, cam))
    _sm, rows = rf.run_desk(cfg, sm, rf.GateReadyTrigger(), det, max_s=20.0,
                            realtime=False, frame_recorder=rec, verbose=False)
    files = sorted(os.listdir(d))
    # every replay frame, in order, reached the detector -- the real pixels,
    # not the driver's zeros placeholder
    assert len(spy.frames) == len(files) == 12
    for f, name in zip(spy.frames, files):
        assert np.array_equal(f, cv2.imread(os.path.join(d, name)))
    # the recorder got exactly the frames the detector decoded, at their
    # capture time on the mission clock
    assert len(rec.offers) == 12
    assert all(a is b for (a, _), b in zip(rec.offers, spy.frames))
    assert all(t is not None and t >= 0.0 for _, t in rec.offers)
    assert det.n_hit == 12
    i_state = rf._CSV_FIELDS.index("state")
    i_src = rf._CSV_FIELDS.index("sp_source")
    assert any(r.split(",")[i_state] == rf.State.ENGAGE
               and r.split(",")[i_src] == "guided" for r in rows)


def test_stale_poll_is_not_a_new_measurement_and_records_nothing():
    class _Src:
        def frames(self):
            yield np.zeros((4, 4, 3), np.uint8), "a"

    det = ff.FeedDetector(ff.FrameFeed(_Src(), threaded=False).start(),
                          _SpyInner(type("I", (), {"detect": lambda s, f, t=None:
                                                   ff._miss()})()))
    first = det.detect(None, 1.0)
    assert first.is_new is True and first.frame is not None
    again = det.detect(None, 1.05)
    assert again.is_new is False and again.frame is None and again.range_m is None


def test_threaded_feed_returns_each_frame_once():
    import threading
    gate = threading.Event()

    class _Src:
        def frames(self):
            yield np.ones((2, 2, 3), np.uint8), "a"
            gate.wait(5)

    feed = ff.FrameFeed(_Src(), threaded=True).start()
    assert feed.first_frame(timeout_s=5) is not None
    assert feed.poll() is not None
    assert feed.poll() is None          # same frame is never returned twice
    gate.set()
    feed.stop()


def test_shape_mismatch_is_refused_at_startup(tmp_path, capsys, monkeypatch):
    d = _write_tag_frames(str(tmp_path / "r800"), 3, h=800)
    called = []
    monkeypatch.setattr(rf, "run_desk", lambda *a, **k: called.append(1))
    rc = rf.main(_desk_args(tmp_path, "dir:" + d))
    assert rc == 1 and not called
    assert "REFUSED" in capsys.readouterr().out


def test_picamera_refuses_unstamped_intrinsics(tmp_path, capsys):
    intr = _intrinsics(tmp_path / "sim.json", 1280, 800)       # no source stamp
    rc = rf.main(_desk_args(tmp_path, "picamera", "--intrinsics", intr))
    assert rc == 1
    assert "no `source` stamp" in capsys.readouterr().out


def test_tag_detector_needs_an_explicit_span(tmp_path, capsys):
    d = _write_tag_frames(str(tmp_path / "r"), 2)
    args = [a for a in _desk_args(tmp_path, "dir:" + d)
            if a not in ("--target-span-m", "0.5")]
    assert rf.main(args) == 1
    assert "needs --target-span-m" in capsys.readouterr().out


# ---- the desk mode cannot arm ----------------------------------------------

_ACTUATION_ATTRS = {"arm", "takeoff", "action", "offboard", "set_velocity_ned",
                    "land", "return_to_launch"}


def _names_in(obj):
    tree = ast.parse(inspect.getsource(obj))
    attrs, imports = set(), set()
    for n in ast.walk(tree):
        if isinstance(n, ast.Attribute):
            attrs.add(n.attr)
        elif isinstance(n, ast.Name):
            attrs.add(n.id)
        elif isinstance(n, (ast.Import, ast.ImportFrom)):
            imports |= {(getattr(n, "module", None) or "")} | {a.name for a in n.names}
    return attrs, imports


@pytest.mark.parametrize("obj", [rf.run_desk, rf.run_desk_mode,
                                 rf.build_source_detector, ff])
def test_desk_path_has_no_vehicle_actuation_in_its_source(obj):
    attrs, imports = _names_in(obj)
    assert not (attrs & _ACTUATION_ATTRS), attrs & _ACTUATION_ATTRS
    assert not any("mavsdk" in m for m in imports)
    assert "run_mavsdk_mission" not in attrs


def test_desk_mode_runs_with_mavsdk_unimportable(tmp_path, monkeypatch):
    d = _write_tag_frames(str(tmp_path / "r"), 8)
    monkeypatch.setitem(sys.modules, "mavsdk", None)    # any import raises

    def _boom(*a, **k):
        raise AssertionError("desk mode reached the MAVSDK driver")

    monkeypatch.setattr(rf, "run_mavsdk_mission", _boom)
    assert rf.main(_desk_args(tmp_path, "dir:" + d)) == 0


def test_desk_refuses_a_vehicle_link(tmp_path):
    d = _write_tag_frames(str(tmp_path / "r"), 1)
    with pytest.raises(SystemExit):
        rf.main(_desk_args(tmp_path, "dir:" + d,
                           "--mavsdk-url", "udpin://0.0.0.0:14540"))


def test_no_source_keeps_the_synthetic_default():
    args = rf.build_arg_parser().parse_args(["--sitl-smoke"])
    assert args.source is None and args.desk is False


def test_pnp_range_matches_detector_pose_on_a_pinhole(tmp_path):
    d = _write_tag_frames(str(tmp_path / "r"), 1)
    frame = cv2.imread(os.path.join(d, "f0000.png"))
    cam = CameraModel(FX, FX, 640.0, 480.0, width=1280, height=960)
    tagdet = ff.TagFrameDetector(cam, 0.5)
    out = tagdet.detect(frame)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    raw = tagdet.det.detect(gray, estimate_tag_pose=True,
                            camera_params=(FX, FX, 640.0, 480.0), tag_size=0.5)[0]
    assert abs(tagdet._pnp_range(raw.corners) - out.tag_range_m) / out.tag_range_m < 0.02
