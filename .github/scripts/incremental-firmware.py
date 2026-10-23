#!/usr/bin/env python3
"""Incrementally build SPIKE-RT apps, safely reusing verified firmware."""
import hashlib
import os
import re
import shutil
import sys
from pathlib import Path

APPS = Path("spike-rt/user_apps")
CACHE = Path("firmware-cache")
BUILD = Path("spike-rt/build")
LIST = Path("spike-rt/app-list.txt")
TODO = Path("spike-rt/app-to-build.txt")
FINGERPRINTS = Path("spike-rt/app-fingerprints.tsv")
CACHE_VERSION = b"spike-rt-v0.2.0-app-cache-v1\0"
REQUIRED = ("asp.bin", "asp.bin.sha256", "size.txt", "build.log",
            "fingerprint.txt", "built_commit.txt")


def application_ids():
    apps = [s.strip() for s in LIST.read_text().splitlines() if s.strip()]
    if not apps or len(apps) != len(set(apps)):
        raise RuntimeError("No apps or duplicate application IDs")
    if any(not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", x) for x in apps):
        raise RuntimeError("Invalid application ID")
    return apps


def fingerprints():
    # Workflow/toolchain changes invalidate every app's cached binary.
    # A project.json-only change affects site metadata, not compilation.
    global_digest = hashlib.sha256()
    global_digest.update(CACHE_VERSION)
    global_digest.update(os.environ["SPIKE_RT_REF"].encode())
    global_digest.update(b"\0")
    global_digest.update(os.environ["SPIKE_RT_IMAGE"].encode())
    global_digest.update(b"\0")
    global_digest.update(Path(".github/workflows/build-and-deploy.yml").read_bytes())
    prefix = global_digest.digest()
    result = {}
    for app in application_ids():
        folder = APPS / app
        if not (folder / (app + ".c")).is_file():
            raise RuntimeError("Missing source for " + app)
        h = hashlib.sha256(prefix)
        files = sorted(
            (f for f in folder.rglob("*")
             if f.is_file() and f.name != "project.json"),
            key=lambda x: x.relative_to(folder).as_posix(),
        )
        for f in files:
            name = f.relative_to(folder).as_posix().encode()
            data = f.read_bytes()
            h.update(len(name).to_bytes(4, "big"))
            h.update(name)
            h.update(len(data).to_bytes(8, "big"))
            h.update(data)
        result[app] = h.hexdigest()
    return result


def verified(app, fingerprint):
    folder = CACHE / app
    if any(not (folder / x).is_file() for x in REQUIRED):
        return False
    if (folder / "fingerprint.txt").read_text().strip() != fingerprint:
        return False
    if not re.fullmatch(r"[0-9a-f]{40}", (folder / "built_commit.txt").read_text().strip()):
        return False
    binary = folder / "asp.bin"
    if not (0 < binary.stat().st_size <= 992 * 1024):
        return False
    checksum = (folder / "asp.bin.sha256").read_text().split()
    if len(checksum) != 2 or checksum[1] != "asp.bin":
        return False
    if not re.fullmatch(r"[0-9a-f]{64}", checksum[0]):
        return False
    return hashlib.sha256(binary.read_bytes()).hexdigest() == checksum[0]


def summary(message):
    if os.getenv("GITHUB_STEP_SUMMARY"):
        with open(os.environ["GITHUB_STEP_SUMMARY"], "a") as stream:
            stream.write(message + "\n")


def plan():
    expected = fingerprints()
    CACHE.mkdir(exist_ok=True)
    changed = []
    for app, fingerprint in expected.items():
        if verified(app, fingerprint):
            print("REUSE:", app)
        else:
            changed.append(app)
            print("BUILD:", app)
    TODO.write_text("".join(app + "\n" for app in changed))
    FINGERPRINTS.write_text(
        "".join(app + "\t" + value + "\n" for app, value in expected.items())
    )
    status = f"Reusing {len(expected) - len(changed)}; building {len(changed)} of {len(expected)} apps"
    print(status)
    Path("build.log").write_text("Incremental SPIKE-RT build\n" + status + "\n")
    if os.getenv("GITHUB_OUTPUT"):
        with open(os.environ["GITHUB_OUTPUT"], "a") as output:
            output.write("needs_build=" + ("true" if changed else "false") + "\n")
    summary("### SPIKE-RT incremental build\n\n" + status + "\n")
    if changed:
        summary("Rebuilding: " + ", ".join(changed) + "\n")


def finalize():
    expected = fingerprints()
    saved = dict(line.split("\t", 1) for line in FINGERPRINTS.read_text().splitlines() if line)
    if expected != saved:
        raise RuntimeError("App sources changed during build")
    changed = [x for x in TODO.read_text().splitlines() if x]
    for app in changed:
        source = BUILD / ("obj-primehub_" + app)
        log = BUILD / "app-logs" / (app + ".log")
        output_files = [source / "asp.bin", source / "asp.bin.sha256",
                        source / "size.txt", log]
        if not all(f.is_file() for f in output_files):
            raise RuntimeError("Missing build output: " + app)
        stage = CACHE / (app + ".new")
        if stage.exists():
            shutil.rmtree(stage)
        stage.mkdir(parents=True)
        for original, filename in zip(
            output_files, ("asp.bin", "asp.bin.sha256", "size.txt", "build.log")
        ):
            shutil.copy2(original, stage / filename)
        (stage / "fingerprint.txt").write_text(expected[app] + "\n")
        (stage / "built_commit.txt").write_text(os.environ["GITHUB_SHA"] + "\n")
        dest = CACHE / app
        if dest.exists():
            shutil.rmtree(dest)
        stage.rename(dest)
        if not verified(app, expected[app]):
            raise RuntimeError("Bad newly built firmware: " + app)

    invalid = [app for app, sha in expected.items() if not verified(app, sha)]
    if invalid:
        raise RuntimeError("Missing or unverified binaries: " + ", ".join(invalid))
    print(f"VALIDATED {len(expected)} firmware entries (rebuilt {len(changed)})")
    summary(f"All {len(expected)} applications verified for Pages publication.\n")


if __name__ == "__main__":
    if len(sys.argv) != 2 or sys.argv[1] not in ("plan", "finalize"):
        raise SystemExit("usage: incremental-firmware.py [plan|finalize]")
    if sys.argv[1] == "plan":
        plan()
    else:
        finalize()
