"""Software baseline only: does not certify physical accuracy."""
import json
import math
import sys
from pathlib import Path
baseline = json.loads((Path(__file__).resolve().parents[1] / "tests/scenarios/stage10-synthetic-v1.json").read_text(encoding="utf-8"))
report = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
assert report["schema"] == "rtsim-qualification-v1"
assert report["status"] == "synthetic_or_unknown_not_physically_qualified", report["status"]
assert math.isclose(report["parameters"]["motor_torque_scale"], baseline["expected_motor_torque_scale"], abs_tol=baseline["parameter_absolute_tolerance"])
held_out = [r for r in report["evaluation"] if r.get("split") == "validation" and r["preset"] == "realistic"]
assert held_out and all(m["passed"] for r in held_out for m in r["metrics"])
assert all(m["metric"]["rms"] <= baseline["holdout_speed_rms_max_m_s"] and m["metric"]["coverage"] >= baseline["required_coverage"] for r in held_out for m in r["metrics"])
assert all(d["kind"] == "synthetic" for d in report["datasets"])
print("Synthetic regression passed; physical qualification remains pending.")
