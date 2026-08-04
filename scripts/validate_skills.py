#!/usr/bin/env python3
"""Validate every skills/<name>/SKILL.md against agent skill-loader constraints.

Run from anywhere:

    python3 scripts/validate_skills.py

Exits non-zero if any hard constraint is violated. Soft budgets (skill length)
are reported as warnings so they never block a content contribution.

Deliberately dependency-free: frontmatter here is a flat mapping of scalars and
folded block scalars, so a small parser beats requiring PyYAML to run the checks.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SKILLS_DIR = REPO / "skills"

# Loader constraints. Skills that violate these fail to load or get truncated.
NAME_MAX = 64
DESCRIPTION_MAX = 1024
NAME_PATTERN = re.compile(r"^[a-z0-9]+(?:-[a-z0-9]+)*$")

# Soft budget: past this, a single SKILL.md costs a lot of context every time it
# triggers. Split the detail into references/ loaded on demand.
LINE_BUDGET = 1000

errors: list[str] = []
warnings: list[str] = []


KEY_LINE = re.compile(r"^([A-Za-z][\w-]*):[ \t]*(.*)$")


def parse_frontmatter(raw: str, rel: str) -> dict[str, str]:
    """Parse a flat YAML mapping of plain scalars and '>'/'|' block scalars."""
    data: dict[str, str] = {}
    lines = raw.split("\n")
    i = 0

    while i < len(lines):
        line = lines[i]
        i += 1
        if not line.strip() or line.lstrip().startswith("#"):
            continue

        match = KEY_LINE.match(line)
        if not match:
            errors.append(f"{rel}: cannot parse frontmatter line: {line!r}")
            continue

        key, value = match.group(1), match.group(2).strip()

        if value in (">", ">-", "|", "|-"):
            folded = value.startswith(">")
            block: list[str] = []
            while i < len(lines) and (not lines[i].strip() or lines[i].startswith((" ", "\t"))):
                block.append(lines[i].strip())
                i += 1
            # Folded scalars join on spaces; literal scalars keep line breaks.
            if folded:
                paragraphs = " ".join(block).split("  ")
                data[key] = " ".join(p for p in paragraphs if p).strip()
            else:
                data[key] = "\n".join(block).strip()
        else:
            data[key] = value.strip("\"'")

    return data


def split_frontmatter(text: str, rel: str) -> tuple[dict[str, str], int]:
    """Return (frontmatter mapping, body line count)."""
    if not text.startswith("---\n"):
        errors.append(f"{rel}: missing YAML frontmatter (file must start with '---')")
        return {}, 0

    end = text.find("\n---\n", 3)
    if end == -1:
        errors.append(f"{rel}: frontmatter is never closed with '---'")
        return {}, 0

    body = text[end + 5 :]
    return parse_frontmatter(text[4:end], rel), len(body.splitlines())


def check_skill(skill_dir: Path) -> None:
    rel = f"skills/{skill_dir.name}/SKILL.md"
    skill_file = skill_dir / "SKILL.md"

    if not skill_file.is_file():
        errors.append(f"skills/{skill_dir.name}/: no SKILL.md")
        return

    data, body_lines = split_frontmatter(skill_file.read_text(encoding="utf-8"), rel)
    if not data:
        return

    name = data.get("name")
    if not name:
        errors.append(f"{rel}: frontmatter has no 'name'")
    elif not isinstance(name, str):
        errors.append(f"{rel}: 'name' must be a string")
    else:
        if name != skill_dir.name:
            errors.append(
                f"{rel}: name '{name}' does not match its directory "
                f"'{skill_dir.name}' -- the skill will not load"
            )
        if len(name) > NAME_MAX:
            errors.append(f"{rel}: name is {len(name)} chars (max {NAME_MAX})")
        if not NAME_PATTERN.match(name):
            errors.append(f"{rel}: name '{name}' must be lowercase-kebab-case")

    description = data.get("description")
    if not description:
        errors.append(f"{rel}: frontmatter has no 'description'")
    elif not isinstance(description, str):
        errors.append(f"{rel}: 'description' must be a string")
    else:
        # A folded scalar keeps a trailing newline; the loader sees the stripped text.
        length = len(description.strip())
        if length > DESCRIPTION_MAX:
            errors.append(
                f"{rel}: description is {length} chars (max {DESCRIPTION_MAX}); "
                f"trim {length - DESCRIPTION_MAX} chars"
            )

    unknown = set(data) - {"name", "description", "license", "allowed-tools", "metadata"}
    if unknown:
        warnings.append(f"{rel}: unrecognized frontmatter keys: {', '.join(sorted(unknown))}")

    if body_lines > LINE_BUDGET:
        warnings.append(
            f"{rel}: {body_lines} lines exceeds the {LINE_BUDGET}-line budget; "
            f"consider moving detail into references/ loaded on demand"
        )


def check_readme(skill_names: list[str]) -> None:
    readme = (REPO / "README.md").read_text(encoding="utf-8")
    for name in skill_names:
        if f"skills/{name}/SKILL.md" not in readme:
            errors.append(f"README.md: no link to skills/{name}/SKILL.md")

    for linked in sorted(set(re.findall(r"skills/([a-z0-9-]+)/SKILL\.md", readme))):
        if linked not in skill_names:
            errors.append(f"README.md: links to skills/{linked}/ which does not exist")


def check_install_defaults(skill_names: list[str]) -> None:
    install = (REPO / "install.sh").read_text(encoding="utf-8")
    match = re.search(r"default_skills=\(([^)]*)\)", install)
    if not match:
        errors.append("install.sh: could not find default_skills array")
        return
    for skill in match.group(1).split():
        if skill not in skill_names:
            errors.append(f"install.sh: default skill '{skill}' does not exist")


def main() -> int:
    skill_dirs = sorted(d for d in SKILLS_DIR.iterdir() if d.is_dir())
    if not skill_dirs:
        return int(bool(sys.stderr.write("no skills found under skills/\n")))

    for skill_dir in skill_dirs:
        check_skill(skill_dir)

    names = [d.name for d in skill_dirs]
    check_readme(names)
    check_install_defaults(names)

    for warning in warnings:
        print(f"warning: {warning}")
    for error in errors:
        print(f"error: {error}", file=sys.stderr)

    print(f"\nChecked {len(skill_dirs)} skills: {len(errors)} errors, {len(warnings)} warnings")
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
