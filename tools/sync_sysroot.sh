#!/usr/bin/env bash
# Mirror the Jetson's libraries into ./sysroot for cross compiling.
# Re-run whenever packages change on the Jetson.
#
#   ./tools/sync_sysroot.sh

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_FILE="${ROOT_DIR}/tools/deploy.env"
[[ -f "${ENV_FILE}" ]] || { echo "Missing ${ENV_FILE} (see deploy.env.example)" >&2; exit 1; }
# shellcheck disable=SC1090
source "${ENV_FILE}"

SYSROOT="${T1_SYSROOT:-${ROOT_DIR}/sysroot}"
REMOTE="${JETSON_USER}@${JETSON_HOST}"
DIRS=(/usr/include /usr/lib/aarch64-linux-gnu /lib/aarch64-linux-gnu /usr/lib/gcc/aarch64-linux-gnu
      /opt/ros/humble /opt/openrobots)

mkdir -p "${SYSROOT}"
for d in "${DIRS[@]}"; do
  echo "==> ${d}"
  rsync -a --delete --relative "${REMOTE}:${d}/" "${SYSROOT}/"
done

# Absolute symlinks would point into the host filesystem; make them sysroot-relative.
echo "==> Fixing absolute symlinks"
find "${SYSROOT}" -type l -lname '/*' | while read -r link; do
  target="$(readlink "${link}")"
  ln -sfn "$(realpath -m --relative-to="$(dirname "${link}")" "${SYSROOT}${target}")" "${link}"
done

echo "Sysroot ready: ${SYSROOT}"
