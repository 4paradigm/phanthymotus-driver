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


def _copied_names(dockerfile: Path) -> set:
    """Every bare source name a COPY names, plus the directories it copies whole."""
    names, whole_dirs = set(), set()
    for _line, src in _copied_sources(dockerfile):
        if any(ch in src for ch in '*?[') or '$' in src:
            continue
        if src.endswith('/'):
            whole_dirs.add(src.rstrip('/'))
        else:
            names.add(Path(src).name)
    return names | {d + '/' for d in whole_dirs}


def _sibling_imports(driver_dir: Path) -> dict:
    """`{module: [files that import it]}` for bare-name imports of siblings.

    A driver's entry point runs with its own directory on sys.path, so it
    imports its neighbours by bare name — `from servo import ...`. That reads
    identically to a third-party import, which is what makes the failure mode
    here so quiet.
    """
    modules = {p.stem for p in driver_dir.glob('*.py')}
    pattern = re.compile(r'^\s*(?:from|import)\s+([a-z_][a-z0-9_]*)', re.M)
    found: dict = {}
    for source in driver_dir.glob('*.py'):
        for name in pattern.findall(source.read_text(errors='ignore')):
            if name in modules and name != source.stem:
                found.setdefault(name, []).append(source.name)
    return found


@pytest.mark.parametrize('dockerfile', DOCKERFILES, ids=lambda p: str(p.relative_to(ROOT)))
def test_every_imported_sibling_is_copied(dockerfile):
    """A module the driver imports by bare name has to be in the COPY list.

    The other direction of test_every_copied_path_exists, and the more dangerous
    one: a COPY naming a file that does not exist fails the build loudly, while a
    file that exists but is never copied builds a perfectly good image that dies
    on startup with `ModuleNotFoundError`. That is how `servo.py` shipped — added
    to two drivers and registered in their bundles, absent from both Dockerfiles,
    and every test still passed.
    """
    driver_dir = dockerfile.parent
    copied = _copied_names(dockerfile)
    # A directory copied wholesale covers everything under it.
    if any(name.endswith('/') and (driver_dir / name.rstrip('/')).is_dir()
           for name in copied):
        pass
    missing = []
    for module, importers in sorted(_sibling_imports(driver_dir).items()):
        if f'{module}.py' in copied:
            continue
        if not (driver_dir / f'{module}.py').exists():
            continue
        missing.append(f'{module}.py (imported by {", ".join(sorted(importers))})')
    assert not missing, (
        f'{dockerfile.relative_to(ROOT)} does not copy modules the driver '
        f'imports: ' + '; '.join(missing)
    )
