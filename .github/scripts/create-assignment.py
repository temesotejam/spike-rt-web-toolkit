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

    renames = {
        sample_id + suffix: assignment_id + suffix
        for suffix in SUFFIXES
        if (source / (sample_id + suffix)).is_file()
    }

    # PD samples often have two identical application headers with different
    # names, for example pd_LCD_S2.h and LCD_S2.h. Merge them into assignment.h.
    canonical_header = source / (sample_id + ".h")
    duplicate_headers = []
    if canonical_header.is_file():
        for header in source.glob("*.h"):
            if header.name != canonical_header.name and header.read_bytes() == canonical_header.read_bytes():
                renames[header.name] = assignment_id + ".h"
                duplicate_headers.append(header.name)

    shutil.copytree(source, destination)
    for old_name, new_name in renames.items():
        if old_name not in duplicate_headers:
            (destination / old_name).rename(destination / new_name)
    for old_name in duplicate_headers:
        (destination / old_name).unlink()

    # Rewrite references to renamed files, including #include <LCD_S2.h>.
    # Keep genuinely different auxiliary headers and C identifiers unchanged.
    matcher = re.compile(
        r"(?<![A-Za-z0-9_])(?:"
        + "|".join(re.escape(key) for key in sorted(renames, key=len, reverse=True))
        + r")(?![A-Za-z0-9_])"
    )
    for path in destination.rglob("*"):
        if path.is_file() and path.suffix in SUFFIXES:
            original = path.read_text(encoding="utf-8")
            updated = matcher.sub(lambda match: renames[match.group(0)], original)
            if updated != original:
                path.write_text(updated, encoding="utf-8")

    metadata = destination / "project.json"
    original_meta = json.loads(metadata.read_text(encoding="utf-8")) if metadata.is_file() else {}
    original_meta.update({
        "id": assignment_id,
        "name": title.strip() or f"課題 {assignment_id}",
        "description": f"{sample_id} を元に作成した課題",
        "origin": f"apps/{sample_id}",
    })
    metadata.write_text(
        json.dumps(original_meta, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(
        f"Created apps/assignments/{assignment_id} from apps/{sample_id}; "
        f"merged {len(duplicate_headers)} duplicate header(s)"
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--sample", required=True)
    parser.add_argument("--assignment", required=True)
    parser.add_argument("--title", default="")
    args = parser.parse_args()
    create(args.sample, args.assignment, args.title)
