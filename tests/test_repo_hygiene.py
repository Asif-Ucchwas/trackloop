"""Repository hygiene: nothing machine-specific or unfinished in the text files, licence and metadata present,
no assistant attribution trailers in the commit history."""
import re
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SKIP_DIRS = {".git", ".venv", "__pycache__", ".pytest_cache", "build", "dist"}
SKIP_SUFFIXES = {".png", ".npz", ".npy", ".pyc"}
MARKERS = re.compile(r"/home/|/Users/|TODO|FIXME|XXX|HACK")


def text_files():
    """Tracked files when git is available (so ignored local folders do not count), otherwise everything under the root."""
    if shutil.which("git") and (ROOT / ".git").exists():
        names = subprocess.run(["git", "ls-files", "-z"], cwd=ROOT, capture_output=True, text=True, check=True).stdout.split("\0")
        paths = [ROOT / n for n in names if n]
    else:
        paths = [p for p in ROOT.rglob("*") if not (set(p.relative_to(ROOT).parts) & SKIP_DIRS)]
    for p in paths:
        if p.is_file() and p.suffix not in SKIP_SUFFIXES and p.name != Path(__file__).name:
            yield p


def test_no_local_paths_or_unfinished_markers_in_text_files():
    hits = []
    for p in text_files():
        for n, line in enumerate(p.read_text(errors="ignore").splitlines(), 1):
            if MARKERS.search(line):
                hits.append(f"{p.relative_to(ROOT)}:{n}")
    assert hits == []


def test_licence_and_package_metadata_present():
    assert (ROOT / "LICENSE").read_text().startswith("MIT License")
    meta = (ROOT / "pyproject.toml").read_text()
    for key in ("authors", "license", "readme", "[project.urls]"):
        assert key in meta, key


def test_no_assistant_attribution_trailers_in_history():
    if shutil.which("git") is None or not (ROOT / ".git").exists():
        pytest.skip("no git history")
    log = subprocess.run(["git", "log", "--format=%B"], cwd=ROOT, capture_output=True, text=True, check=True).stdout
    assert not re.search(r"Co-Authored-By|Claude-Session", log)
