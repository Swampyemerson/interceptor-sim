#!/usr/bin/env python3
"""The DEPLOYED-WEIGHTS pins: which model flies, and is it the model we recorded?

Two independent defects, both found by the 2026-07-25 audit, both invisible to
every gate that existed:

  (1) INTEGRITY (completeness critic, "off-machine backup"). The flight default
      `scripts/seeker/weights/nn_tier/n-mono.onnx` had NO checksum anywhere in the
      repo, is gitignored, and lives on one disk with the corpus that could
      regenerate it. A model is a measurement instrument -- every detection number
      traces through it -- so a corrupted or swapped file silently invalidates
      everything downstream. `weights/DEPLOYED.sha256` is the record; this file
      re-verifies it whenever the weights are present on the machine.

  (2) DRIFT (reader 3, five surfaces). The hardware default moved from the
      sim-trained `drone_finetuned_quad_v2.onnx` to the real-data `n-mono.onnx`
      (AP50 0.442 vs 0.0003 on source-disjoint real held-out imagery, n=4175), but
      several surfaces still named quad_v2 -- including the bench gate that is
      supposed to certify the deployed loop. quad_v2 has an 88.5% false-fire rate
      on real imagery, so a bench run against it certifies the wrong model. These
      tests pin the basename in the code and sweep the doc surfaces.

WEIGHTS-ABSENT POLICY. The .onnx files are gitignored, so on a clean clone or in
CI they do not exist. The hash check is therefore CONDITIONAL -- but it is NOT a
silent skip: `test_manifest_is_wellformed_and_covers_the_flight_default` and every
basename pin run unconditionally, and the conditional case reports which files it
verified so a zero-verification run is visible rather than absorbed.

Run: `.venv/bin/python -m pytest tests/test_deployed_weights.py -v`
"""
import hashlib
import os
import re

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MANIFEST = os.path.join(REPO_ROOT, "weights", "DEPLOYED.sha256")

FLIGHT_DEFAULT_REL = "scripts/seeker/weights/nn_tier/n-mono.onnx"
HISTORICAL_BAR_REL = "scripts/seeker/weights/drone_finetuned_quad_v2.onnx"
SEEKER_LOOP = os.path.join(REPO_ROOT, "flight", "deploy", "seeker_loop.py")


def parse_manifest():
    """{relative_path: sha256}. Plain `sha256sum` format so the standard tool
    verifies it too (`sha256sum -c weights/DEPLOYED.sha256`)."""
    entries = {}
    with open(MANIFEST) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            digest, _sep, path = line.partition("  ")
            assert re.fullmatch(r"[0-9a-f]{64}", digest), (
                f"not a sha256 digest: {line!r}")
            assert path, f"no path in manifest line: {line!r}"
            entries[path.lstrip("*")] = digest
    return entries


def sha256_of(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


# ============================================================ the manifest
def test_manifest_exists_and_is_tracked():
    assert os.path.isfile(MANIFEST), (
        "weights/DEPLOYED.sha256 is missing -- the deployed model has no "
        "integrity record")


def test_manifest_is_wellformed_and_covers_the_flight_default():
    entries = parse_manifest()
    assert FLIGHT_DEFAULT_REL in entries, (
        f"{FLIGHT_DEFAULT_REL} (the flight default) is absent from the manifest; "
        f"manifest covers {sorted(entries)}")
    assert HISTORICAL_BAR_REL in entries, (
        f"{HISTORICAL_BAR_REL} (the historical bar, still selectable via "
        f"--weights) is absent from the manifest")
    for path in entries:
        assert not os.path.isabs(path), (
            f"manifest path {path!r} is absolute -- it must be repo-relative so "
            f"`sha256sum -c` works from any clone")


def test_manifest_digests_are_distinct():
    """quad_v2 and n-mono are within 5 bytes of the same size (both YOLO11n
    exports), so a copy-paste swap is easy and would be invisible by size alone."""
    entries = parse_manifest()
    assert entries[FLIGHT_DEFAULT_REL] != entries[HISTORICAL_BAR_REL]


# ============================================================ the hash check
def test_the_flight_default_matches_the_manifest_when_present():
    """THE integrity check. Conditional on the gitignored file existing, and it
    SAYS SO when it does not run, so an unverified run is visible."""
    path = os.path.join(REPO_ROOT, FLIGHT_DEFAULT_REL)
    if not os.path.isfile(path):
        pytest.skip(f"{FLIGHT_DEFAULT_REL} absent (gitignored; expected on a "
                    f"clean clone / CI) -- integrity NOT verified on this machine")
    entries = parse_manifest()
    actual = sha256_of(path)
    assert actual == entries[FLIGHT_DEFAULT_REL], (
        f"*** DEPLOYED WEIGHTS CHANGED ***  {FLIGHT_DEFAULT_REL}\n"
        f"  manifest: {entries[FLIGHT_DEFAULT_REL]}\n"
        f"  on disk : {actual}\n"
        f"Either the file is corrupt/swapped, or a model swap landed without "
        f"updating weights/DEPLOYED.sha256 (a swap is an ADR-level decision -- "
        f"see weights/README.md).")


def test_every_manifest_entry_that_exists_matches():
    entries = parse_manifest()
    verified, missing = [], []
    for rel, digest in sorted(entries.items()):
        p = os.path.join(REPO_ROOT, rel)
        if not os.path.isfile(p):
            missing.append(rel)
            continue
        assert sha256_of(p) == digest, f"sha256 mismatch for {rel}"
        verified.append(rel)
    if not verified:
        pytest.skip(f"no manifest file present on this machine "
                    f"({len(missing)} gitignored) -- nothing verified here")


# ============================================================ the drift pins
def _seeker_loop_src():
    with open(SEEKER_LOOP) as f:
        return f.read()


def test_seeker_loop_default_weights_basename_is_pinned():
    """THE pin reader 3 asked for. The argparse default is built with os.path.join
    (`weights`, `nn_tier`, `n-mono.onnx`), so match the assembled tail rather than
    a single literal."""
    src = _seeker_loop_src()
    m = re.search(
        r'add_argument\(\s*"--weights"\s*,\s*\n?\s*default=([^\n]*(?:\n[^\n]*){0,3}?)\)',
        src)
    assert m, "could not locate seeker_loop.py's --weights argparse default"
    default_expr = m.group(1)
    assert "n-mono.onnx" in default_expr, (
        f"flight/deploy/seeker_loop.py's DEFAULT weights are no longer n-mono: "
        f"{default_expr!r}. quad_v2 must never be the hardware default (AP50 "
        f"0.0003 / recall 1.1% / false-fire 88.5% on real held-out imagery).")
    assert "nn_tier" in default_expr, default_expr
    assert "quad_v2" not in default_expr, default_expr


def test_seeker_loop_still_documents_quad_v2_as_never_the_default():
    src = _seeker_loop_src()
    assert "must never" in src and "quad_v2" in src, (
        "the header's 'quad_v2 must never be the default on hardware' warning "
        "was removed -- it is the only in-code record of WHY")


@pytest.mark.parametrize("rel", [
    "scripts/check_deploy_bench.sh",
    "flight/deploy/README.md",
    "scripts/pi_setup/requirements-pi.txt",
])
def test_deployed_weight_surfaces_do_not_still_name_quad_v2_as_the_default(rel):
    """DRIFT SWEEP (reader 3 + completeness critic's fifth surface). Each of these
    described the DEPLOYED configuration as quad_v2 after the default moved to
    n-mono. The bench gate is the worst of them: it is supposed to certify the
    loop that flies. quad_v2 may still be MENTIONED (as the historical bar), but
    not as the thing being run/scored -- so the pin is that n-mono is named."""
    path = os.path.join(REPO_ROOT, rel)
    with open(path) as f:
        text = f.read()
    assert "n-mono" in text, (
        f"{rel} never mentions the deployed n-mono model -- it still describes "
        f"the pre-2026-07-21 quad_v2 default (deployed-weights drift)")


def test_bench_gate_default_weights_are_the_flight_default():
    """The strong form for the bench gate: BENCH_WEIGHTS' default must be the
    same file seeker_loop flies, or the gate certifies a different model."""
    path = os.path.join(REPO_ROOT, "scripts", "check_deploy_bench.sh")
    with open(path) as f:
        text = f.read()
    m = re.search(r'^BENCH_WEIGHTS="\$\{BENCH_WEIGHTS:-([^}]*)\}"', text, re.M)
    assert m, "could not locate BENCH_WEIGHTS' default in check_deploy_bench.sh"
    assert m.group(1).endswith("n-mono.onnx"), (
        f"the deploy bench gate defaults to {m.group(1)!r}, not the flight "
        f"default n-mono.onnx -- it would certify a model that does not fly")
