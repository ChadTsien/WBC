# Single source of truth for pinned versions. Sourced by the other scripts.

T1_UBUNTU_CODENAME="jammy"
T1_ROS_DISTRO="humble"
T1_GCC_MAJOR="11"

T1_PINOCCHIO_VER="2.7.0"
T1_HPP_FCL_VER="2.4.4"
T1_PROXSUITE_VER="0.7.3"
T1_PROXSUITE_APT_VER="0.7.3r1"
T1_MUJOCO_VER="3.9.0"
T1_RERUN_VER="0.35.0"

T1_PREFIX="/opt/openrobots"
T1_MUJOCO_DIR="${HOME}/.mujoco/mujoco-${T1_MUJOCO_VER}"
T1_JETSON_TOOLCHAIN="${T1_JETSON_TOOLCHAIN:-${HOME}/env/jetson-aarch64-2022.081}"
T1_JETSON_TRIPLE="aarch64-buildroot-linux-gnu"

# setup_deps*.sh may be invoked with sudo. Re-bind user-owned paths to the
# account that ran sudo, otherwise MuJoCo / pip --user land in /root.
t1_resolve_dev_user() {
  if [[ "$(id -u)" -eq 0 ]]; then
    if [[ -z "${SUDO_USER:-}" || "${SUDO_USER}" == "root" ]]; then
      echo "ERROR: do not run as root. Use: ./scripts/setup_deps.sh" >&2
      return 1
    fi
    T1_DEV_USER="${SUDO_USER}"
    T1_DEV_HOME="$(getent passwd "${T1_DEV_USER}" | cut -d: -f6)"
    if [[ -z "${T1_DEV_HOME}" || ! -d "${T1_DEV_HOME}" ]]; then
      echo "ERROR: cannot resolve home directory for ${T1_DEV_USER}" >&2
      return 1
    fi
  else
    T1_DEV_USER="$(id -un)"
    T1_DEV_HOME="${HOME}"
  fi
  T1_MUJOCO_DIR="${T1_DEV_HOME}/.mujoco/mujoco-${T1_MUJOCO_VER}"
  if [[ "${T1_JETSON_TOOLCHAIN}" == "${HOME}/env/jetson-aarch64-2022.081" ]]; then
    T1_JETSON_TOOLCHAIN="${T1_DEV_HOME}/env/jetson-aarch64-2022.081"
  fi
}
