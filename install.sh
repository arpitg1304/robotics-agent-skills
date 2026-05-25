#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Install robotics agent skills into another project.

Usage:
  ./install.sh --target <directory> [--skills <skill> ...]
  ./install.sh --list

Examples:
  ./install.sh --target ../my_robot/.claude/skills --skills ros2 robotics-testing robot-bringup
  ./install.sh --target /mnt/skills/user --skills all

If --skills is omitted, the ROS2 application bundle is installed:
  ros2 robotics-software-principles robotics-testing robot-bringup
USAGE
}

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
skills_dir="${repo_dir}/skills"
target=""
declare -a requested_skills=()
declare -a default_skills=(ros2 robotics-software-principles robotics-testing robot-bringup)

list_skills() {
  find "${skills_dir}" -mindepth 1 -maxdepth 1 -type d -printf '%f\n' | sort
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --target)
      target="${2:-}"
      shift 2
      ;;
    --skills)
      shift
      while [[ $# -gt 0 && "$1" != --* ]]; do
        requested_skills+=("$1")
        shift
      done
      ;;
    --list)
      list_skills
      exit 0
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "${target}" ]]; then
  echo "Missing required --target <directory>." >&2
  usage >&2
  exit 2
fi

if [[ ${#requested_skills[@]} -eq 0 ]]; then
  requested_skills=("${default_skills[@]}")
fi

if [[ ${#requested_skills[@]} -eq 1 && "${requested_skills[0]}" == "all" ]]; then
  mapfile -t requested_skills < <(list_skills)
fi

mkdir -p "${target}"

for skill in "${requested_skills[@]}"; do
  src="${skills_dir}/${skill}"
  if [[ ! -d "${src}" ]]; then
    echo "Unknown skill: ${skill}" >&2
    echo "Available skills:" >&2
    list_skills >&2
    exit 1
  fi

  rm -rf "${target}/${skill}"
  cp -R "${src}" "${target}/${skill}"
  echo "Installed ${skill} -> ${target}/${skill}"
done

echo "Done."
