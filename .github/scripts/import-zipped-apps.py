#!/usr/bin/env python3
"""Import a single SPIKE-RT application from apps/*.zip into the build workspace."""
import json
import re
import shutil
import tempfile
import zipfile
from pathlib import Path

apps = Path("apps")
for archive in sorted(apps.glob("*.zip")):
    app_id = re.sub(r"[^a-zA-Z0-9_]", "_", archive.stem)
    app_id = re.sub(r"_?(master|main)$", "", app_id, flags=re.I)
    if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", app_id):
        raise SystemExit(f"{archive}: invalid derived application ID: {app_id}")
    destination = apps / app_id
    if destination.exists():
        raise SystemExit(f"{archive}: output directory already exists: {destination}")

    with tempfile.TemporaryDirectory() as temp:
        root = Path(temp)
        with zipfile.ZipFile(archive) as z:
            for member in z.infolist():
                path = Path(member.filename)
                if (path.is_absolute() or ".." in path.parts or
                        member.is_dir() and member.file_size or
                        (member.external_attr >> 16) & 0o170000 == 0o120000):
                    raise SystemExit(f"{archive}: unsafe ZIP entry: {member.filename}")
            z.extractall(root)

        sources = [
            p for p in root.rglob("*.c")
            if re.search(r"\bmain_task\s*\(", p.read_text(encoding="utf-8", errors="replace"))
        ]
        if len(sources) != 1:
            names = ", ".join(str(p.relative_to(root)) for p in sources)
            raise SystemExit(
                f"{archive}: expected exactly one C source defining main_task(); "
                f"found {len(sources)}: {names}. Convert this ZIP manually into apps/<app>/."
            )

        source = sources[0]
        # Preserve adjacent source/header/configuration files.
        shutil.copytree(source.parent, destination)
        entry = destination / source.name
        renamed = destination / f"{app_id}.c"
        if entry != renamed:
            if renamed.exists():
                raise SystemExit(f"{archive}: {renamed} would be overwritten")
            entry.rename(renamed)

        # Rename matching SPIKE-RT config files, including internal references.
        for suffix in (".h", ".cfg", ".cdl"):
            previous = destination / f"{source.stem}{suffix}"
            target = destination / f"{app_id}{suffix}"
            if previous.exists() and previous != target:
                if target.exists():
                    raise SystemExit(f"{archive}: {target} would be overwritten")
                if suffix == ".h":
                    shutil.copy2(previous, target)  # retain original include name
                else:
                    previous.rename(target)
                if suffix == ".cfg":
                    contents = target.read_text(encoding="utf-8")
                    contents = contents.replace(source.stem + ".h", app_id + ".h")
                    target.write_text(contents, encoding="utf-8")
        metadata = destination / "project.json"
        if not metadata.exists():
            metadata.write_text(json.dumps({
                "name": archive.stem,
                "description": "Imported from " + archive.name,
                "origin": archive.as_posix()
            }, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(f"Imported {archive} -> {destination} (entry: {source.relative_to(root)})")
