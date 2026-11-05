"""The README names files, figures and scripts; every one must exist, so a rename or a deleted figure fails a test."""
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
README = (ROOT / "README.md").read_text()
PATH = re.compile(r"(?<![\w~/.-])((?:scripts|docs|data|src|tests)/[\w./-]*[\w/])")


def mentioned():
    return sorted({m.group(1) for m in PATH.finditer(README) if README[m.end():m.end() + 1] != "*"})


def test_every_path_in_readme_exists():
    missing = [p for p in mentioned() if not (ROOT / p).exists()]
    assert not missing, missing


def test_every_python_command_in_readme_names_an_existing_script():
    scripts = re.findall(r"python (scripts/[\w]+\.py)", README)
    assert len(scripts) >= 15
    assert [s for s in scripts if not (ROOT / s).is_file()] == []


def test_markdown_images_and_links_exist():
    targets = re.findall(r"!?\[[^\]]*\]\(([^)#\s]+)\)", README)
    local = [t for t in targets if not t.startswith(("http://", "https://"))]
    assert len(local) >= 20
    assert [t for t in local if not (ROOT / t).exists()] == []
