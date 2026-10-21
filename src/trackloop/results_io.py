"""Small helpers shared by the benchmark scripts: results CSV, git state, package versions."""
import csv
import importlib.metadata as md
import platform
import subprocess

PACKAGES = ("numpy", "scipy", "osqp", "control", "matplotlib")


def git(*args):
    try:
        return subprocess.run(["git", *args], capture_output=True, text=True, check=True).stdout.strip()
    except Exception:
        return "unknown"


def version(package):
    try:
        return md.version(package)
    except md.PackageNotFoundError:
        return "not installed"


def environment():
    """Interpreter and package versions the results were produced with."""
    return {"python": platform.python_version(), **{p: version(p) for p in PACKAGES}}


def write_csv(path, rows):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(dict.fromkeys(k for r in rows for k in r)), restval="")
        w.writeheader()
        w.writerows(rows)
