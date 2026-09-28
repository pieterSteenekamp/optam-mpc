"""Compare two identical-time scenarios using fixed-target IAE and peak error."""
import argparse
import csv
import json
from pathlib import Path


def read_csv(path):
    with Path(path).open(newline="", encoding="utf-8") as stream:
        return [{key: float(value) for key, value in row.items()} for row in csv.DictReader(stream)]


def metrics(rows, signal, target):
    errors = [abs(row[signal]-target) for row in rows]
    iae = sum((a+b)/2*(right["time_minutes"]-left["time_minutes"])
              for a,b,left,right in zip(errors, errors[1:], rows, rows[1:]))
    return dict(iae_unit_minutes=iae, peak_abs_error=max(errors), final_abs_error=errors[-1])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("first", type=Path)
    parser.add_argument("second", type=Path)
    parser.add_argument("--signal", nargs=2, action="append", required=True, metavar=("ID", "TARGET"))
    parser.add_argument("--labels", nargs=2, default=["First", "Second"])
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rows = [read_csv(args.first), read_csv(args.second)]
    if [x["time_minutes"] for x in rows[0]] != [x["time_minutes"] for x in rows[1]]:
        raise ValueError("Runs must have identical recorded times and length.")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(len(args.signal), 1, figsize=(10, 3*len(args.signal)), squeeze=False)
    report = {}
    for axis, (signal, target) in zip(axes[:,0], args.signal):
        target = float(target)
        report[signal] = {}
        for label, run in zip(args.labels, rows):
            axis.plot([r["time_minutes"] for r in run], [r[signal] for r in run], label=label)
            report[signal][label] = metrics(run, signal, target)
        axis.axhline(target, color="black", linestyle="--", label="Target")
        axis.set_title(signal)
        axis.set_xlabel("Time (minutes)")
        axis.set_ylabel("Engineering units")
        axis.legend()
        axis.grid(alpha=.25)
    args.output.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(args.output/"comparison.png", dpi=150)
    plt.close(fig)
    (args.output/"comparison.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
