"""Development smoke test of the picture-loss accounting (seeds outside the test set)."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "scripts"))
import flight_hardware_rerun_ext as ext  # noqa: E402

if __name__ == "__main__":
    jobs = ext.jobs_for("dev", "demo", [1000, 1001], 45, 10)
    keep = ("seed", "miss", "t_cpa", "first_det", "chase_frames", "chase_out", "chase_gap_s",
            "chase_out_pitch", "arr_frames", "arr_out", "arr_gap_s", "arr_out_pitch", "peak_v", "tilt_first")
    for r in ext.run_jobs_fov(jobs, 2):
        print({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items() if k in keep})
