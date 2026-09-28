"""Create a concise Markdown acceptance record from one report.json file."""
import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    report = json.loads(args.report.read_text(encoding="utf-8"))
    lines = [
        f"# Engineering results — {report['application']}", "",
        "> Generated from the machine-readable acceptance report. Do not edit numerical results by hand.", "",
        f"- Overall acceptance: **{'PASS' if report['passed'] else 'FAIL'}**",
        f"- Samples: {report['samples']}",
        f"- Solver failures: {len(report.get('failures', []))}", "",
        "| Signal | Metric | Actual | Limit | Result |", "|---|---:|---:|---:|---|",
    ]
    for check in report.get("checks", []):
        lines.append(
            f"| {check['signal']} | {check['metric']} | {check['actual']:.10g} | "
            f"{check['limit']:.10g} | {'PASS' if check['passed'] else 'FAIL'} |"
        )
    provenance = report.get("provenance")
    if provenance:
        lines.extend(("", "## Provenance", "", f"- Package: {provenance['package_version']}",
                      f"- Python: {provenance['python_version']}"))
        for name, digest in provenance.get("sha256", {}).items():
            lines.append(f"- `{name}` SHA-256: `{digest}`")
    args.output.write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
