#!/usr/bin/env python3
"""Build the Subaru Crosstrek opendbc branch from openpilot's stable pin."""

from __future__ import annotations

import os
from pathlib import Path
import re
import shlex
import subprocess
import sys


TARGET_BRANCH = "subaru-crosstrek"
BASELINE_VERSION = (0, 11, 1)
OPENPILOT_URL = "https://github.com/commaai/openpilot.git"
OPENDBC_URL = "https://github.com/commaai/opendbc.git"
JACOB_URL = "https://github.com/jacobwaller/opendbc.git"
JACOB_BRANCH = "jul-angle-based"
SOURCE_BASE = "045cd8d397f305ed6b150212cb05a76ec095fe35"
SOURCE_TIP = "8acbeccdab45a47aa7af170d2d17b61f0eb5e4f0"
SOURCE_COMMITS = (
  "8b4703bbd678f643659102350d54bc8477c4746f",
  "e49bb48bba83078ece71dcfd470990c4a29eee38",
  "7aedc019d208018b6a23f0c071d54eb2a3c86c9d",
  "9d217de804bf95bd42c838c4e2c57aeb45f31ccc",
  "f3cdd5ff7d1f7415bbc10214f3cd83681bf093ce",
  "42889e319eb73599ee66b350a777aa68130b1bca",
  "0dd4797ad9eeaf1de66bd99b8c458209b6110611",
  "128daadb19b61b3d54e0e212043fbc7a0412ba46",
  "a358966bc6099b9e022e0a6f0e1647928f904107",
  "a116120c3b149f02bccbb87665e35122cc9626c8",
  "3d4400559133d0dbcff6c55abfc37f47ec03a620",
  "db8d3aee3c90013e44aa70f56bb9896d8a42c4cb",
  "19df75992398f6661c7ed18a1914219d42e5883f",
  "8acbeccdab45a47aa7af170d2d17b61f0eb5e4f0",
)


def run(repo: Path | None, *args: str, input_text: str | None = None, echo_output: bool = True) -> str:
  cmd = list(args)
  print("+ " + " ".join(shlex.quote(arg) for arg in cmd), file=sys.stderr)
  result = subprocess.run(cmd, cwd=repo, input=input_text, text=True, capture_output=True)
  if result.stdout and echo_output:
    print(result.stdout, end="", file=sys.stderr)
  if result.stderr:
    print(result.stderr, end="", file=sys.stderr)
  if result.returncode != 0:
    raise subprocess.CalledProcessError(result.returncode, cmd)
  return result.stdout.strip()


def parse_version(tag: str) -> tuple[int, ...] | None:
  match = re.fullmatch(r"v(\d+(?:\.\d+)+)", tag)
  return tuple(map(int, match.group(1).split("."))) if match else None


def latest_stable_tag() -> tuple[str, tuple[int, ...]]:
  output = run(None, "git", "ls-remote", "--tags", "--refs", OPENPILOT_URL, echo_output=False)
  candidates: list[tuple[tuple[int, ...], str]] = []
  for line in output.splitlines():
    tag = line.split("refs/tags/", 1)[-1]
    version = parse_version(tag)
    if version is not None:
      candidates.append((version, tag))
  if not candidates:
    raise RuntimeError("No numeric stable openpilot tags were found")
  version, tag = max(candidates)
  return tag, version


def patch_id(repo: Path, commit: str) -> str:
  shown = subprocess.run(
    ["git", "show", "--pretty=format:", "--binary", commit],
    cwd=repo,
    check=True,
    capture_output=True,
  )
  patched = subprocess.run(
    ["git", "patch-id", "--stable"],
    cwd=repo,
    input=shown.stdout,
    check=True,
    capture_output=True,
  )
  return patched.stdout.decode().split()[0]


def author_metadata(repo: Path, commit: str) -> str:
  return run(repo, "git", "show", "-s", "--format=%an%x00%ae%x00%aI%x00%B", commit)


def write_attribution(repo: Path, tag: str, base_sha: str) -> None:
  lines = [
    "# Subaru Crosstrek attribution",
    "",
    "The Subaru Crosstrek angle-steering work is preserved from",
    "[`jacobwaller/opendbc:jul-angle-based`](https://github.com/jacobwaller/opendbc/tree/jul-angle-based).",
    "The original commits remain unchanged in this branch's ancestry and are replayed individually",
    "with their author identity, author date, message, trailers, and patch ID verified.",
    "",
    f"This build uses the `opendbc_repo` pin from openpilot `{tag}`: `{base_sha}`.",
    "",
    "## Source commits",
    "",
  ]
  lines.extend(f"- [`{sha}`](https://github.com/jacobwaller/opendbc/commit/{sha})" for sha in SOURCE_COMMITS)
  (repo / "ATTRIBUTION.md").write_text("\n".join(lines) + "\n")


def main() -> None:
  if len(sys.argv) != 2:
    raise SystemExit(f"usage: {sys.argv[0]} TARGET_CHECKOUT")
  repo = Path(sys.argv[1]).resolve()
  tag, version = latest_stable_tag()
  marker = repo / ".upstream-openpilot-tag"
  current_version = parse_version(marker.read_text().strip()) if marker.exists() else BASELINE_VERSION
  if current_version is None:
    raise RuntimeError("Invalid .upstream-openpilot-tag")
  if version <= current_version:
    print("changed=false")
    print(f"tag={tag}")
    return

  base_sha = run(
    None,
    "gh", "api", f"repos/commaai/openpilot/git/trees/{tag}",
    "--jq", '.tree[] | select(.path == "opendbc_repo") | .sha',
  )
  if not re.fullmatch(r"[0-9a-f]{40}", base_sha):
    raise RuntimeError(f"Invalid opendbc pin for {tag}: {base_sha!r}")

  old_sha = run(repo, "git", "rev-parse", "HEAD")
  run(repo, "git", "config", "user.name", "github-actions[bot]")
  run(repo, "git", "config", "user.email", "41898282+github-actions[bot]@users.noreply.github.com")
  run(repo, "git", "fetch", "--no-tags", OPENDBC_URL, base_sha)
  run(repo, "git", "fetch", "--no-tags", JACOB_URL, f"{JACOB_BRANCH}:refs/remotes/jacob/{JACOB_BRANCH}")
  if run(repo, "git", "rev-parse", f"refs/remotes/jacob/{JACOB_BRANCH}") != SOURCE_TIP:
    raise RuntimeError("Jacob's source branch tip changed; refusing an unreviewed source update")

  base_tree = run(repo, "git", "rev-parse", f"{base_sha}^{{tree}}")
  merge_message = f"Merge opendbc pin for openpilot {tag}\n\nPreserve Jacob Waller's original commits as the first-parent history.\n"
  merge_sha = run(repo, "git", "commit-tree", base_tree, "-p", old_sha, "-p", base_sha, input_text=merge_message)
  run(repo, "git", "reset", "--hard", merge_sha)

  for source_sha in SOURCE_COMMITS:
    run(repo, "git", "cherry-pick", source_sha)
    replay_sha = run(repo, "git", "rev-parse", "HEAD")
    if patch_id(repo, source_sha) != patch_id(repo, replay_sha):
      raise RuntimeError(f"Patch ID changed while replaying {source_sha}")
    if author_metadata(repo, source_sha) != author_metadata(repo, replay_sha):
      raise RuntimeError(f"Author metadata changed while replaying {source_sha}")

  marker.write_text(tag + "\n")
  write_attribution(repo, tag, base_sha)
  run(repo, "git", "add", ".upstream-openpilot-tag", "ATTRIBUTION.md")
  run(repo, "git", "commit", "-m", f"Record Subaru Crosstrek base for {tag}")
  run(repo, "git", "merge-base", "--is-ancestor", SOURCE_TIP, "HEAD")

  print("changed=true")
  print(f"tag={tag}")
  print(f"old_sha={old_sha}")
  print(f"candidate_sha={run(repo, 'git', 'rev-parse', 'HEAD')}")


if __name__ == "__main__":
  main()
