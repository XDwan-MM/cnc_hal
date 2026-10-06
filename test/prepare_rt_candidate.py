#!/usr/bin/env python3
"""Create an isolated cnc_rt candidate with the current HAL, without editing cnc_rt."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess


HAL = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--rt-root", type=Path, default=HAL.parent / "cnc_rt")
parser.add_argument("--output-dir", type=Path, required=True,
                    help="new directory inside this HAL's build directory")
parser.add_argument("--rt-patch", type=Path,
                    help="optional reviewed compatibility patch to apply only to the candidate")
args = parser.parse_args()
rt = args.rt_root.resolve()
destination = args.output_dir.resolve()
if not (rt / "CNC_RT.pro").is_file():
    parser.error("--rt-root must contain CNC_RT.pro")
if not destination.is_relative_to((HAL / "build").resolve()) or destination == HAL / "build":
    parser.error("--output-dir must be a new subdirectory of the HAL build directory")
if destination.exists():
    parser.error("--output-dir already exists; choose a new directory")
library = HAL / "build/all/libcnc_hal.a"
if not library.is_file():
    parser.error("build the current HAL first: ./build_all.sh")
rt_patch = args.rt_patch.resolve() if args.rt_patch else None
if rt_patch and (not rt_patch.is_file() or not shutil.which("patch")):
    parser.error("--rt-patch must exist and the patch utility must be available")


def ignore(directory, names):
    skipped = {name for name in names if name == ".git" or name == "build"
               or "Zone.Identifier" in name}
    if Path(directory).resolve() == rt:
        skipped.update(name for name in names if name in {"bin", "rt.log"})
    return skipped


shutil.copytree(rt, destination, ignore=ignore)
if rt_patch:
    command = ["patch", "--batch", "--forward", "-p1", "-d", str(destination), "-i", str(rt_patch)]
    subprocess.run([*command, "--dry-run"], check=True)
    subprocess.run(command, check=True)
package = destination / "lib/cnc_hal"
# Replace only the copied candidate package; the original RT tree is never written.
for directory in ["include", "internal/Greemaster", "internal/common", "lib"]:
    target = package / directory
    if target.exists():
        shutil.rmtree(target)
    target.mkdir(parents=True)
for source_directory, target_directory in [
    (HAL / "include", package / "include"),
    (HAL / "src/Greemaster", package / "internal/Greemaster"),
    (HAL / "src/common", package / "internal/common"),
]:
    for source in source_directory.glob("*.h"):
        shutil.copy2(source, target_directory / source.name)
shutil.copy2(library, package / "lib/libcnc_hal.a")
shutil.copy2(HAL / "src/Greemaster/devices.json", package / "devices.json")


def git(root, *arguments):
    return subprocess.check_output(["git", "-C", str(root), *arguments], text=True).strip()


def hashes(root, files):
    return {str(file.relative_to(root)): hashlib.sha256(file.read_bytes()).hexdigest()
            for file in sorted(files)}


manifest = {
    "hal_root": str(HAL), "hal_head": git(HAL, "rev-parse", "HEAD"),
    "hal_dirty": bool(git(HAL, "status", "--porcelain", "--untracked-files=all")),
    "hal_source_sha256": hashes(HAL, [
        *HAL.glob("include/*.h"), *HAL.glob("src/**/*.h"),
        *HAL.glob("src/**/*.c"), *HAL.glob("src/**/*.cpp"),
        HAL / "src/Greemaster/devices.json", HAL / "CMakeLists.txt", HAL / "build_all.sh",
    ]),
    "rt_root": str(rt), "rt_head": git(rt, "rev-parse", "HEAD"),
    "rt_tracked_diff_sha256": hashlib.sha256(
        git(rt, "diff", "HEAD", "--binary").encode()).hexdigest(),
    "hal_package_sha256": hashes(package, [
        *package.glob("include/*.h"), *package.glob("internal/**/*.h"),
        package / "lib/libcnc_hal.a", package / "devices.json",
    ]),
    "rt_source_sha256": hashes(destination, [
        *destination.glob("src/**/*.c"), *destination.glob("src/**/*.cpp"),
        *destination.glob("src/**/*.h"), destination / "main.cpp", destination / "CNC_RT.pro",
    ]),
}
if rt_patch:
    manifest["rt_candidate_patch"] = {
        "path": str(rt_patch), "sha256": hashlib.sha256(rt_patch.read_bytes()).hexdigest(),
    }
(package / "CANDIDATE.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
(package / "SYNCED.txt").write_text(
    f"Candidate only; original RT not modified\nHAL source: {HAL}\n"
    f"HAL HEAD: {manifest['hal_head']}\nHAL dirty: {manifest['hal_dirty']}\n"
    "Exact headers/library/dictionary hashes: CANDIDATE.json\n")
print(destination)
print("Candidate manifest:", package / "CANDIDATE.json")
