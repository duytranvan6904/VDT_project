#!/usr/bin/env python3
"""Create a reproducible PX4 patch bundle from a modified PX4 checkout.

The script never edits the PX4 checkout. It requires a clean base commit before
changes are made, then exports the working-tree diff plus exact repository
metadata and a SHA-256 checksum.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path


def run_git(px4_dir: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", "-C", str(px4_dir), *args],
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip()


def require_git_repo(px4_dir: Path) -> None:
    try:
        run_git(px4_dir, "rev-parse", "--show-toplevel")
    except (subprocess.CalledProcessError, FileNotFoundError) as exc:
        raise SystemExit(f"Not a git repository: {px4_dir}") from exc


def changed_files(px4_dir: Path) -> list[str]:
    output = run_git(px4_dir, "diff", "--name-only")
    return [line for line in output.splitlines() if line]


def patch_bytes(px4_dir: Path) -> bytes:
    result = subprocess.run(
        ["git", "-C", str(px4_dir), "diff", "--binary", "--full-index"],
        check=True,
        capture_output=True,
    )
    return result.stdout


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Export a PX4 diff with exact base commit/version metadata."
    )
    parser.add_argument("--px4-dir", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--patch-name", default="custom_px4.patch")
    parser.add_argument(
        "--base-ref",
        help="Expected base commit/tag. Recommended when changes were made from a known PX4 release.",
    )
    parser.add_argument(
        "--allow-untracked",
        action="store_true",
        help="Allow untracked files. They are not included in git diff and must be added separately.",
    )
    args = parser.parse_args()

    px4_dir = args.px4_dir.resolve()
    output_dir = args.output_dir.resolve()
    require_git_repo(px4_dir)

    untracked = run_git(px4_dir, "ls-files", "--others", "--exclude-standard").splitlines()
    if untracked and not args.allow_untracked:
        raise SystemExit(
            "PX4 checkout has untracked files. Add/remove them first, or use --allow-untracked."
        )

    commit = run_git(px4_dir, "rev-parse", "HEAD")
    describe = run_git(px4_dir, "describe", "--tags", "--always", "--dirty")
    branch = run_git(px4_dir, "branch", "--show-current") or "DETACHED"
    status = run_git(px4_dir, "status", "--porcelain")
    if not status:
        raise SystemExit("No PX4 changes found. Modify PX4, then run this script again.")

    if args.base_ref:
        expected = run_git(px4_dir, "rev-parse", args.base_ref)
        if expected != commit:
            raise SystemExit(
                f"Base ref mismatch: HEAD={commit}, expected {args.base_ref}={expected}"
            )

    patch = patch_bytes(px4_dir)
    if not patch.strip():
        raise SystemExit("The working tree has no tracked diff to export.")

    output_dir.mkdir(parents=True, exist_ok=True)
    patch_path = output_dir / args.patch_name
    patch_path.write_bytes(patch)

    metadata = {
        "project": "VDT PX4 custom patch",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "px4_repo": str(px4_dir),
        "px4_commit": commit,
        "px4_describe": describe,
        "px4_branch": branch,
        "base_ref": args.base_ref,
        "changed_files": changed_files(px4_dir),
        "patch_file": patch_path.name,
        "patch_sha256": sha256(patch),
        "untracked_files_not_in_patch": untracked,
    }
    metadata_path = patch_path.with_suffix(".json")
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")

    checksum_path = patch_path.with_suffix(patch_path.suffix + ".sha256")
    checksum_path.write_text(f"{metadata['patch_sha256']}  {patch_path.name}\n", encoding="utf-8")

    print(f"Patch:    {patch_path}")
    print(f"Metadata: {metadata_path}")
    print(f"SHA-256:  {checksum_path}")
    print(f"PX4 base: {describe} ({commit})")
    print(f"Files:    {len(metadata['changed_files'])}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
