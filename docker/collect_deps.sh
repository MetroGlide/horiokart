#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
DEPS_DIR="${SCRIPT_DIR}/deps"
EXTRA_FILE="${SCRIPT_DIR}/deps_extra.txt"

EXCLUDE_DIRS=(".git" "docker" "log" "nav2_pkg")

build_prune_args() {
  prune_args=()
  for d in "${EXCLUDE_DIRS[@]}"; do
    prune_args+=(-path "${ROOT_DIR}/${d}" -prune -o)
  done
}

rm -rf "${DEPS_DIR}"
mkdir -p "${DEPS_DIR}"

build_prune_args

while IFS= read -r src; do
  rel="${src#${ROOT_DIR}/}"
  dst="${DEPS_DIR}/${rel}"
  mkdir -p "$(dirname "${dst}")"
  cp "${src}" "${dst}"
done < <(find "${ROOT_DIR}" \
  "${prune_args[@]}" \
  -type f \( -name "package.xml" -o -name "*.rosinstall" -o -name "*.repos" -o -name "requirements.txt" \) \
  -print)

if [[ -f "${EXTRA_FILE}" ]]; then
  while IFS= read -r line || [[ -n "${line}" ]]; do
    [[ "${line}" =~ ^[[:space:]]*# ]] && continue
    [[ -z "${line// }" ]] && continue
    src="${ROOT_DIR}/${line}"
    if [[ -f "${src}" ]]; then
      dst="${DEPS_DIR}/${line}"
      mkdir -p "$(dirname "${dst}")"
      cp "${src}" "${dst}"
    else
      echo "WARNING: deps_extra.txt: not found: ${line}" >&2
    fi
  done < "${EXTRA_FILE}"
fi

echo "Collected deps to ${DEPS_DIR}"
