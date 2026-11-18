"""Probe nearby right-side releases and integer policy slots for both completed guard designs."""
import json
from pathlib import Path
import runpy

HERE = Path(__file__).resolve().parent
RIGHT_ANGLES = (93, 94, 96, 97)
LAUNCH_SLOTS = (15, 16, 18, 19)


def main():
    qualification = runpy.run_path(str(HERE / "qualification.py"))
    head = qualification["read"](HERE / "head-profile.json")
    report = {}
    for label, profile in (("before", HERE.parent / "rounded-profile.json"),
                           ("after", HERE / "guard-profile.json")):
        apply = qualification["config"](qualification["read"](profile), head)
        cases = [f"right{angle}" for angle in RIGHT_ANGLES]
        cases += [f"roll{2 * slot:03d}" for slot in LAUNCH_SLOTS]
        report[label] = {case: qualification["case"](apply, case) for case in cases}
        misses = [case for case, row in report[label].items() if not qualification["passed"](row)]
        print(f"{label}: {len(cases) - len(misses)}/{len(cases)} passed; misses {misses}", flush=True)
    (HERE / "neighbors.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
