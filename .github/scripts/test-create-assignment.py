#!/usr/bin/env python3
"""Regression tests: every bundled sample must copy without stale header names."""
import contextlib
import importlib.util
import io
import json
import shutil
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
APPS = REPO / "apps"
SCRIPT = REPO / ".github" / "scripts" / "create-assignment.py"
spec = importlib.util.spec_from_file_location("assignment_creator", SCRIPT)
creator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(creator)

sample_count = 0
for sample in sorted(APPS.iterdir()):
    if not sample.is_dir() or not (sample / (sample.name + ".c")).is_file():
        continue
    with tempfile.TemporaryDirectory() as temp:
        root = Path(temp)
        shutil.copytree(sample, root / sample.name)
        creator.APPS = root
        with contextlib.redirect_stdout(io.StringIO()):
            creator.create(sample.name, "kadai_test", "")
        output = root / "assignments" / "kadai_test"
        for suffix in (".c", ".cfg", ".h", ".cdl"):
            original = sample / (sample.name + suffix)
            if original.is_file():
                assert (output / ("kadai_test" + suffix)).is_file(), sample.name
                assert not (output / (sample.name + suffix)).exists(), sample.name
        assert json.loads((output / "project.json").read_text(encoding="utf-8"))["id"] == "kadai_test"
        canonical = sample / (sample.name + ".h")
        if canonical.is_file():
            for alias in sample.glob("*.h"):
                if alias == canonical or alias.read_bytes() != canonical.read_bytes():
                    continue
                assert not (output / alias.name).exists(), sample.name
                for filename in output.iterdir():
                    if filename.suffix in {".c", ".cfg", ".h", ".cdl"}:
                        assert alias.name not in filename.read_text(encoding="utf-8"), (
                            sample.name, filename, alias.name
                        )
        sample_count += 1

assert sample_count >= 20, f"Too few bundled samples tested: {sample_count}"
print(f"PASS: {sample_count} bundled apps copied with consistent filenames/includes")
