#!/usr/bin/env python3
"""Create a standalone assignment app by copying a sample app."""
import argparse
import json
import re
import shutil
from pathlib import Path

APPS = Path(__file__).resolve().parents[2] / "apps"
SUFFIXES = (".c", ".cfg", ".h", ".cdl")

def create(sample_id, assignment_id, title):
    valid = re.compile(r"[A-Za-z][A-Za-z0-9_]*\Z")
    if not valid.fullmatch(sample_id) or not valid.fullmatch(assignment_id):
        raise ValueError("ID must start with a letter; only ASCII letters, digits and _ are allowed")
    if sample_id == "assignments":
        raise ValueError("Select an app under apps/, not the assignments group")
    source = APPS / sample_id
    destination = APPS / "assignments" / assignment_id
    if not (source / (sample_id + ".c")).is_file():
        raise ValueError(f"Sample app not found: {sample_id}")
    if destination.exists() or (APPS / assignment_id).exists():
        raise ValueError(f"App ID already exists: {assignment_id}")

    shutil.copytree(source, destination)
    for suffix in SUFFIXES:
        old = destination / (sample_id + suffix)
        if old.is_file():
            old.rename(destination / (assignment_id + suffix))
    # Rewrite only references to the renamed app-specific files.
    # Preserve extra headers such as multi_task1.h that the original .c may include.
    for p in destination.rglob("*"):
        if p.is_file() and p.suffix in SUFFIXES:
            original = p.read_text(encoding="utf-8")
            updated = original
            for suffix in SUFFIXES:
                updated = updated.replace(sample_id + suffix, assignment_id + suffix)
            if updated != original:
                p.write_text(updated, encoding="utf-8")

    meta = destination / "project.json"
    original_meta = json.loads(meta.read_text(encoding="utf-8")) if meta.is_file() else {}
    original_meta.update({
        "id": assignment_id,
        "name": title.strip() or f"課題 {assignment_id}",
        "description": f"{sample_id} を元に作成した課題",
        "origin": f"apps/{sample_id}"
    })
    meta.write_text(json.dumps(original_meta, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"Created apps/assignments/{assignment_id} from apps/{sample_id}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--sample", required=True)
    parser.add_argument("--assignment", required=True)
    parser.add_argument("--title", default="")
    args = parser.parse_args()
    create(args.sample, args.assignment, args.title)
