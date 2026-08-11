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
