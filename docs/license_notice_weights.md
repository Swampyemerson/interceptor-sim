# License notice: repo code vs. detector weights

Written 2026-07-10 as part of publish-prep (docs/next.md). Not legal advice — a
factual note on a licensing nuance this repo should carry once public.

## The split

The repo's own code (Python, scripts, docs) is **MIT** (see `LICENSE`,
copyright 2026 Emerson). That covers everything *we wrote*.

It does **not** cleanly cover the fine-tuned neural-net detector weights the
markerless seeker lane can use, e.g. `scripts/seeker/weights/drone_finetuned_v2.onnx`
(a YOLO11n backbone fine-tuned on Gazebo-rendered frames, ADR-0040/0042/0057;
still the deployed `MARKERLESS_NN_WEIGHTS`, ADR-0061). YOLO11n's upstream
architecture and training tooling ship from Ultralytics under **AGPL-3.0**
(`docs/nn_transfer_plan.md` SS4, `scripts/seeker/weights/LICENSES.md`). A model
fine-tuned with AGPL-licensed tooling from an AGPL base is plausibly a
**derivative work** — the *weights file itself* is not something we can
unilaterally relicense MIT just because our training script is ours.

## Current state (checked, not assumed)

As of this writing `scripts/seeker/weights/` is **entirely git-ignored**
(`.gitignore`: `scripts/seeker/weights/`, plus an explicit second ignore line
for `yolo11n.pt`) — only `LICENSES.md` (the provenance table) is tracked.
So `drone_finetuned_v2.onnx` is **not currently committed** to this repo at
all; it's a local, regenerable artifact. The AGPL question below is about
what happens *if/when* that changes (e.g. shipping it as a release asset, or
un-ignoring it for reproducibility), not a problem in the repo today.

`docs/nn_transfer_plan.md` already names the clean long-term fix: for a real
public/deployable artifact, prefer an **Apache-2.0** model family —
**YOLOX** or **NanoDet-Plus** — fine-tuned in-domain instead of the AGPL
Ultralytics line (SS "Recommendation," and the `weights/LICENSES.md` provenance
table). That's a real retrain, not a relabel.

## BUILDER DECISION — pick before going public with any weights file

- **(a) Keep weights out of git (status quo) or, if committed, ship them
  with an explicit AGPL-3.0 notice scoped to that one file** — cheapest,
  honest, matches how the repo already documents provenance
  (`LICENSES.md`). Code stays MIT; the weight artifact is separately and
  correctly labeled.
- **(b) Move weights to a release asset (or git-LFS) with its own license
  file**, keeping the main MIT'd source tree free of any AGPL artifact.
- **(c) Retrain on an Apache-2.0 base (YOLOX/NanoDet-Plus) before going
  public**, so the whole repo — code and weights — is permissively
  licensed with no exception. This is the `nn_transfer_plan.md`
  recommendation for the *real hardware* system already; doing it for the
  sim weights too would make the portfolio repo license-uniform.

**Recommendation:** **(a)** for the current private-repo phase — lowest
effort, already factually correct, nothing to retrain. Re-decide before
flipping the repo public: if the weights file is still git-ignored at that
point, (a) is a non-issue; if it gets committed or attached as a release
asset, add the per-file AGPL notice then, and weigh (c) if a clean
Apache-only portfolio story is judged worth the retrain (real cost — see
`nn_transfer_plan.md`'s YOLOX/NanoDet-Plus discussion).
