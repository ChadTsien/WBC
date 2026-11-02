#!/usr/bin/env bash
# Build for Jetson and copy the install tree over.
#
#   ./tools/deploy.sh                 cross compile on this PC, then rsync install-aarch64/
#   ./tools/deploy.sh --remote-build  rsync the repo to the Jetson and build there (fallback)
#   ./tools/deploy.sh ... --packages-select wbc   extra args go to colcon

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_FILE="${ROOT_DIR}/tools/deploy.env"
[[ -f "${ENV_FILE}" ]] || { echo "Missing ${ENV_FILE} (see deploy.env.example)" >&2; exit 1; }
# shellcheck disable=SC1090
source "${ENV_FILE}"
REMOTE="${JETSON_USER}@${JETSON_HOST}"

MODE="cross"
if [[ "${1:-}" == "--remote-build" ]]; then MODE="remote"; shift; fi

if [[ "${MODE}" == "cross" ]]; then
  cd "${ROOT_DIR}"
  colcon build --build-base build-aarch64 --install-base install-aarch64 --merge-install \
    --cmake-args -DCMAKE_TOOLCHAIN_FILE="${ROOT_DIR}/cmake/toolchain-jetson.cmake" \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF "$@"
  rsync -a --delete "${ROOT_DIR}/install-aarch64/" "${REMOTE}:${JETSON_INSTALL_DIR}/install/"
else
  rsync -a --delete --exclude build --exclude install --exclude log --exclude sysroot \
    --exclude 'build-*' --exclude 'install-*' "${ROOT_DIR}/" "${REMOTE}:${JETSON_INSTALL_DIR}/"
  ssh "${REMOTE}" "cd ${JETSON_INSTALL_DIR} && source scripts/env.sh && \
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF $*"
fi

echo "Deployed to ${REMOTE}:${JETSON_INSTALL_DIR}"
