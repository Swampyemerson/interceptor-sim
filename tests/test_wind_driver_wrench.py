"""The persistent wrench must be CLEARED before every publish, or it accumulates.

WHY THIS FILE EXISTS. `scripts/wind_driver.py` applies wind as a force via
gz-sim's ApplyLinkWrench system, on the `/world/<w>/wrench/persistent` topic.
That system does NOT replace a persistent wrench when a new one arrives for the
same entity -- it APPENDS. Verbatim, from the gz-sim8 source
(`src/systems/apply_link_wrench/ApplyLinkWrench.cc`, installed version 8.14.0):

    void ApplyLinkWrenchPrivate::OnWrenchPersistent(const msgs::EntityWrench &_msg)
    {
      ...
      this->persistentWrenches.push_back(_msg);
    }

and `PreUpdate` applies every entry:

    for (auto msg : this->dataPtr->persistentWrenches) ...

So a driver publishing at 20 Hz without clearing stacks one wrench per tick.
After t seconds the link feels roughly `20*t` times the intended force -- at 30 s,
600 superimposed wrenches.

WHAT MAKES THIS DANGEROUS RATHER THAN OBVIOUS: a force ramping linearly with time
does not look like a bug in the output. It looks like "the wind builds during the
dash". The driver's own CSV would still record the INTENDED per-tick force
(that is what it computes and logs), so the log would look perfect while the
physics diverged. Every wind arm measured that way would be quietly worthless,
and the paired control would not catch it -- both arms share the driver.

This is exactly the instrument-defect class in CLAUDE.md, one layer down: not a
bug in a scorer, but a bug in an ACTUATOR whose log reports intent rather than
effect.

These are STATIC checks on the source. The behavioural proof needs a running
Gazebo (the design's W3 physics probe), which is deliberately a separate,
sim-gated step -- but a static guard that fails the moment someone deletes the
clear is worth having in the offline suite, because the sim probe is expensive
and will not run on every commit.
"""
import ast
import os
import re

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DRIVER = os.path.join(REPO, "scripts", "wind_driver.py")


def _publish_fn_source():
    """The body of the inner `publish()` that the driver hands to its core."""
    src = open(DRIVER).read()
    tree = ast.parse(src)
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "publish":
            return ast.get_source_segment(src, node)
    raise AssertionError(
        "wind_driver.py no longer defines a publish() -- if the wrench path was "
        "restructured, this guard must be rewritten, not deleted.")


def test_publish_clears_before_it_publishes():
    """THE REGRESSION. Without the clear, forces accumulate without bound."""
    body = _publish_fn_source()
    assert "clear_pub.publish" in body, (
        "wind_driver.publish() no longer clears the previous persistent wrench "
        "before publishing a new one. gz-sim's ApplyLinkWrench APPENDS "
        "persistent wrenches (push_back), so without the clear the applied "
        "force grows by one tick's worth every tick -- ~600x by the end of a "
        "30 s flight -- while the driver's CSV still logs the INTENDED force. "
        "Every wind arm measured this way would be worthless.")


def test_the_clear_happens_BEFORE_the_force_is_published():
    """Order matters: clearing after publishing would delete the new wrench."""
    body = _publish_fn_source()
    i_clear = body.index("clear_pub.publish")
    i_pub = body.rindex("pub.publish(msg)")
    assert i_clear < i_pub, (
        "the clear must precede the wrench publish; clearing afterwards would "
        "remove the wrench that was just applied and produce ZERO wind")


def test_a_failed_clear_is_fatal_not_swallowed():
    """Fail closed. A swallowed clear failure silently restores the bug, and the
    run would still produce a plausible-looking CSV."""
    body = _publish_fn_source()
    assert "raise" in body, (
        "a failure to clear must re-raise. Swallowing it reinstates the "
        "accumulation bug for the rest of the flight while the log looks fine.")
    # And it must not be a bare `except: pass` around the clear.
    assert not re.search(r"except[^\n]*:\s*\n\s*pass", body), (
        "the clear's exception handler must not be a silent pass")


def test_the_accumulation_hazard_is_documented_at_the_site():
    """The next person to touch this must not have to rediscover gz-sim's
    semantics from a failed flight campaign."""
    body = _publish_fn_source()
    low = body.lower()
    assert "push_back" in low or "append" in low, (
        "the docstring at the publish site must record WHY the clear is "
        "mandatory (ApplyLinkWrench appends rather than replaces) -- an "
        "undocumented clear looks removable")


def test_the_driver_still_clears_on_shutdown():
    """The shutdown clear is separate from the per-tick clear and must survive:
    a persistent wrench outlives the process that set it."""
    src = open(DRIVER).read()
    # Two distinct clear sites: one inside publish(), one in teardown.
    assert src.count("clear_pub.publish") >= 2, (
        "expected a per-tick clear AND a shutdown clear; a persistent wrench "
        "left behind would keep pushing the vehicle in the NEXT flight")


# --------------------------------------------------------------------------
# THE SECOND ACTUATOR-INPUT DEFECT, found by the Gate-0 probe on 2026-08-29.
#
# The wrench itself was fine. What was wrong was the VEHICLE VELOCITY the force
# is computed FROM. The pose callback used to accept the airframe's LINK entity
# and prefer it over the MODEL. In gz's pose topics a nested link's pose is
# reported RELATIVE TO ITS ENCLOSING MODEL, so `base_link` reads a constant
# (0, 0, 0.24) -- x500_base's declared offset -- for the entire flight.
#
# Measured, not theorised (logs/wind_gate0_20260829T154442Z): the aircraft
# climbed to 5.45 m and all 4789 pose callbacks reported z = 0.2400. The finite
# difference of a constant is zero, so `v_veh_n`/`v_veh_e` were 0.00000 in every
# one of the 599 applied rows.
#
# Relative air velocity is (wind - vehicle). With the vehicle term structurally
# zero, a dash arm computes drag from the wind speed alone and omits the
# airframe's own ~9 m/s. Drag is superlinear in that quantity, so the applied
# force is wrong by a large factor -- and, once again, the CSV looks perfect,
# because it faithfully logs the force that was commanded.
#
# This is ADR-0006's root cause recurring in a new file. That ADR says in as
# many words that a "same number, different assumed parent" mistake can bite
# twice. It did. This guard is why it cannot bite a third time here.
def _on_pose_source():
    """The gz-transport pose CALLBACK -- the one nested inside make_on_pose().

    Selected by its enclosing factory on purpose: `WindDriverCore` also has an
    `on_pose` method (a pure state update taking north/east floats), and an
    ast.walk that grabs the first match by name silently guards the wrong
    function. A guard pointed at the wrong code is worse than no guard.
    """
    src = open(DRIVER).read()
    tree = ast.parse(src)
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "make_on_pose":
            for inner in ast.walk(node):
                if isinstance(inner, ast.FunctionDef) and inner.name == "on_pose":
                    return ast.get_source_segment(src, inner)
    raise AssertionError(
        "wind_driver.py no longer defines make_on_pose()/on_pose() -- if the "
        "pose path was restructured, this guard must be rewritten, not deleted.")


def _on_pose_identifiers():
    """Identifiers the callback actually USES -- from the AST, not the text.

    Checking raw source would match the explanatory comment (which names the
    link precisely because it is explaining why the link is wrong) and the
    guard would fail on its own documentation.
    """
    import textwrap
    node = ast.parse(textwrap.dedent(_on_pose_source()))
    return {n.id for n in ast.walk(node) if isinstance(n, ast.Name)}


def test_pose_matches_the_MODEL_entity_only():
    """THE REGRESSION: matching the link reads a model-relative constant."""
    names = _on_pose_identifiers()
    assert "INTERCEPTOR_MODEL" in names, (
        "wind_driver's pose callback must match the top-level MODEL entity -- "
        "that is the only world-relative pose available on these topics")
    assert not ({"INTERCEPTOR_LINK", "scoped"} & names), (
        "wind_driver's pose callback matched the LINK entity again. A nested "
        "link's pose is relative to its enclosing model, so base_link reads a "
        "CONSTANT (0,0,0.24) however the aircraft flies; the differenced "
        "vehicle velocity is then structurally zero and the drag force omits "
        "the airframe's own airspeed entirely. Measured on 2026-08-29: 4789 "
        "callbacks all reporting z=0.2400 during a flight to 5.45 m.")


def test_the_link_relative_pose_hazard_is_documented_at_the_site():
    """The next person must not have to rediscover gz's pose parenting from a
    failed wind campaign -- the same reason the clear is documented above."""
    low = _on_pose_source().lower()
    assert "relative" in low and "model" in low, (
        "the pose callback must record WHY it matches the model and not the "
        "link (link poses are model-relative), or the narrower match looks "
        "like an arbitrary restriction someone can widen back")


def test_the_wrench_still_targets_the_LINK():
    """The fix must not overshoot. The pose to READ is the model's; the entity
    the force is APPLIED to is still the link -- ApplyLinkWrench takes a link.
    Swapping both would silently stop the force landing at all."""
    src = open(DRIVER).read()
    assert re.search(r"msg\.entity\.name\s*=\s*scoped", src), (
        "the wrench must still be addressed to the scoped LINK name; "
        "ApplyLinkWrench applies to links, not models")
    assert re.search(r"clear_ent\.name\s*=\s*scoped", src), (
        "the clear must address the same scoped LINK as the publish")
