#!/usr/bin/env python3
"""Verify relative links and #anchors in docs/balance_car/*.md (CI helper)."""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
LINK = re.compile(r"\[[^\]]*\]\(([^)\s]+)\)")


def anchors(path):
    out = set()
    for line in path.read_text(encoding="utf-8").splitlines():
        m = re.match(r"#{1,6}\s+(.*)", line)
        if m:
            t = re.sub(r"[`*_]", "", m.group(1)).strip().lower()
            t = re.sub(r"[^\w\- ]", "", t).replace(" ", "-")
            out.add(t)
    return out


def main():
    bad = 0
    for md in sorted(ROOT.glob("*.md")):
        in_code = False
        for n, line in enumerate(md.read_text(encoding="utf-8").splitlines(), 1):
            if line.startswith("```"):
                in_code = not in_code
            if in_code:
                continue
            for tgt in LINK.findall(line):
                if tgt.startswith(("http://", "https://", "mailto:")):
                    continue
                file_part, _, frag = tgt.partition("#")
                dest = (md.parent / file_part).resolve() if file_part else md
                if not dest.exists():
                    print(f"{md.name}:{n}: missing file {tgt}")
                    bad += 1
                elif frag and dest.suffix == ".md" and frag.lower() not in anchors(dest):
                    print(f"{md.name}:{n}: missing anchor {tgt}")
                    bad += 1
    print("link check:", "FAILED" if bad else "ok")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
