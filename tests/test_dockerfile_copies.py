"""Every file a driver Dockerfile names must actually exist.

These Dockerfiles `COPY` sources by name rather than by directory, so a file
that is renamed, moved or deleted leaves a line pointing at nothing — and the
build fails at that layer, after everything above it has already run.

It has happened: `lifecycle.py` was added to Tianyi's COPY list, then moved to
`common/` (which is copied wholesale) and deleted from the driver directory. The
COPY line kept naming it, and the image stopped building with
`"/lifecycle.py": not found`.

The reverse is checked in test_common_lifecycle.py: a module that *is* imported
has to be in the list. This file checks the other direction.

Run: python3 -m pytest tests/test_dockerfile_copies.py -q
"""
from __future__ import annotations

import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]

DOCKERFILES = sorted(
    p for p in ROOT.glob('*/*/Dockerfile')
    if not any(part.startswith('.') for part in p.parts)
)


def _copied_sources(dockerfile: Path):
    """The source operands of each COPY, minus flags and the destination."""
    out = []
    text = dockerfile.read_text(errors='ignore')
    # Join escaped line continuations so a wrapped COPY is read as one line.
    text = re.sub(r'\\\n\s*', ' ', text)
    for line in text.splitlines():
        line = line.strip()
        if not line.upper().startswith('COPY '):
            continue
        # `--from=<stage>` copies out of another build stage, so its source is a
        # path inside that stage and says nothing about this directory.
        if '--from=' in line:
            continue
        parts = [p for p in line.split()[1:] if not p.startswith('--')]
        if len(parts) < 2:
            continue
        for src in parts[:-1]:          # last operand is the destination
            out.append((line, src))
    return out


@pytest.mark.parametrize('dockerfile', DOCKERFILES, ids=lambda p: str(p.relative_to(ROOT)))
def test_every_copied_path_exists(dockerfile):
    context = dockerfile.parent
    missing = []
    for line, src in _copied_sources(dockerfile):
        # Globs and build-ARG interpolation (`${REV}`) both resolve at build
        # time, so the literal string is not expected to exist on disk.
        if any(ch in src for ch in '*?[') or '${' in src or '$' in src:
            continue
        # A COPY source is relative to the build context. These images are built
        # from the driver directory, with common/ staged in beforehand.
        candidate = context / src
        if candidate.exists() or (ROOT / src).exists():
            continue
        missing.append((src, line[:90]))
    assert not missing, (
        f'{dockerfile.relative_to(ROOT)} copies paths that do not exist: '
        + '; '.join(f'{s!r} (in: {l}…)' for s, l in missing)
    )
