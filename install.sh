#!/usr/bin/env bash
# Installs unitree_g1_extension into an existing INCAR venv
# Debian 13 and Ubuntu 22.04 and 24.04. are supported
#
#   bash install.sh
#   bash install.sh --venv ~/incar --yes
#
# What it does (it lists this again and asks before changing anything):
#   1. apt installs git, cmake, a C/C++ compiler, the headers of the venv's
#      Python and the OpenGL libraries MuJoCo loads
#   2. builds the CycloneDDS 0.10 C library into <venv>/cyclonedds. The
#      cyclonedds Python package that unitree_sdk2py pins has no wheel for
#      Python 3.12+ and compiles against it, then loads it from there at runtime
#   3. pip installs unitree_sdk2py (editable, from a checkout in <venv>/src) and
#      the packages LeRobot's G1 code and its MuJoCo simulation import
#   4. builds Pinocchio from source with CasADi support: the PyPI wheels lack
#      pinocchio.casadi, which LeRobot's G1 arm IK needs for the gravity
#      compensation this extension turns on
#   5. pip installs LeRobot and this extension without their dependencies
#   6. checks that INCAR is untouched and that the G1 arm IK can be built
#
# Why without dependencies: LeRobot pins torch<2.12 and the PyPI "draccus",
# while INCAR pins torch 2.13 and ships its own draccus fork under the same
# import name. Resolving LeRobot's dependencies downgrades torch and replaces
# INCAR's draccus, which breaks INCAR. Reinstall lerobot or this extension
# with "pip install --no-deps" later too. "pip check" keeps reporting LeRobot's
# unmet pins afterwards; that is expected.
#
# Rerunning it reuses what is already built.
#
# Options:
#   --venv DIR     the INCAR venv; asked if omitted (default: the active venv, or ~/incar)
#   --jobs N       parallel compile jobs; default from the CPU count and free memory
#   -y, --yes      don't ask

# Everything runs from main(), called on the last line, so a partially
# downloaded script does nothing.
set -euo pipefail

# LeRobot main: UnitreeG1Config has end_effector and sim_publish_images, which
# 0.6.1 on PyPI lacks
LEROBOT_URL=https://github.com/huggingface/lerobot
LEROBOT_COMMIT=dd5313e305ff7aaa5b6c7b4769d1b26650ce250b
UNITREE_SDK_URL=https://github.com/unitreerobotics/unitree_sdk2_python
UNITREE_SDK_COMMIT=814556d15970dd2ecf1c9984e845ca02ab07e206
# Any 0.10.x C library serves the 0.10.2 Python package unitree_sdk2py pins
CYCLONEDDS_URL=https://github.com/eclipse-cyclonedds/cyclonedds
CYCLONEDDS_C_VERSION=0.10.5
CYCLONEDDS_C_COMMIT=2cdd114cbd18340c606573b4cc8dc20cc161ec5a
CYCLONEDDS_PY_VERSION=0.10.2
# LeRobot documents Pinocchio 3.x. Its cmeel-boost 1.89 caps numpy below 2.4.
PINOCCHIO_VERSION=3.9.0
# Pinocchio is compiled against this CasADi, so it must not move afterwards
CASADI_VERSION=3.8.2
PINOCCHIO_BUILD_DEPS=("cmeel[build]" "cmeel-boost~=1.89.0" "cmeel-urdfdom[build]>=4"
  "libcoal[build]>=3.0.2,<4" "coal[build]>=3.0.2,<4" "example-robot-data>=4.4.0" scipy
  "casadi==$CASADI_VERSION")
# opencv-python, not LeRobot's opencv-python-headless: the extension, the
# Unitree SDK and the simulation want it, and both install into cv2/
OPENCV_SPEC="opencv-python>=4.13,<5"
# Imported by LeRobot's G1 code and the lerobot/unitree-g1-mujoco simulation.
# Bounds as in LeRobot's pyproject.toml.
RUNTIME_PACKAGES=("gymnasium>=1.1.1,<2" "termcolor>=2.4,<4" "pyzmq>=26.2.1,<28" "pygame>=2.5.1,<2.7"
  "scipy>=1.14,<2" "mujoco>=3.3" "loguru>=0.7" "casadi==$CASADI_VERSION")
# Must be the same afterwards
INCAR_PACKAGES=(incar incar_draccus incar_networking torch torchvision torchcodec)
INDEX_HOST=packages.incar-robotics.se
REPO_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

log() { printf '\033[1m==> %s\033[0m\n' "$*"; }
warn() { printf '\033[33mWarning: %s\033[0m\n' "$*" >&2; }
die() { printf '\033[31mError: %s\033[0m\n' "$*" >&2; exit 1; }
have_tty() { (exec < /dev/tty) 2>/dev/null; }

# ask_yn QUESTION -> success on yes; yes with --yes
ask_yn() {
  local reply=
  (( ASSUME_YES )) && return 0
  printf '\n%s [y/N] ' "$1" > /dev/tty
  read -r reply < /dev/tty || true
  [[ $reply =~ ^[Yy]([Ee][Ss])?$ ]]
}

# run_logged CMD... -> runs quietly into $LOG, shows the log tail on failure
run_logged() {
  if ! "$@" >> "$LOG" 2>&1; then
    tail -n 30 "$LOG" >&2
    die "'$*' failed, full log: $LOG"
  fi
}

parse_args() {
  VENV= JOBS= ASSUME_YES=0
  while (( $# )); do
    case $1 in
      --venv) VENV=${2:?--venv needs a directory}; shift ;;
      --jobs) JOBS=${2:?--jobs needs a number}; shift ;;
      -y|--yes) ASSUME_YES=1 ;;
      -h|--help) sed -n '2,/^$/{s/^# \{0,1\}//;p}' "$0"; exit 0 ;;
      *) die "unknown option $1 (see --help)" ;;
    esac
    shift
  done
  [[ -z $JOBS || $JOBS =~ ^[1-9][0-9]*$ ]] || die "--jobs needs a positive number"
}

# The INCAR installer makes a Python 3.13 venv on Debian 13 and a Python 3.12
# one on Ubuntu (from the deadsnakes PPA on 22.04)
check_system() {
  [[ -r /etc/os-release ]] || die "cannot detect the OS"
  . /etc/os-release
  case "${ID:-}-${VERSION_ID:-}" in
    debian-13) OS_PYTHON=3.13 ;;
    ubuntu-22.04|ubuntu-24.04) OS_PYTHON=3.12 ;;
    *) die "this installer supports Debian 13 and Ubuntu 22.04 and 24.04, found ${PRETTY_NAME:-unknown}" ;;
  esac
  OS_NAME=$PRETTY_NAME
  (( EUID != 0 )) || die "run as your normal user, the script uses sudo where needed"
  grep -qs '^name = "unitree_g1_extension"' "$REPO_DIR/pyproject.toml" \
    || die "run the install.sh inside the unitree_g1_extension repository"
}

ask_venv() {
  local default=${VIRTUAL_ENV:-$HOME/incar}
  if [[ -z $VENV ]] && have_tty && (( ! ASSUME_YES )); then
    read -rp "$(printf '\nWhere is the INCAR venv? [default: %s]: ' "$default")" VENV < /dev/tty || true
  fi
  VENV=${VENV:-$default}
  VENV=${VENV/#\~/$HOME}
  VENV=$(realpath -m "$VENV")
  [[ -f $VENV/bin/activate && -x $VENV/bin/python ]] || die "$VENV is not a venv (no bin/activate)"
  [[ $VENV != *[[:space:]]* ]] || die "the venv path must not contain spaces (Pinocchio's build arguments are split on them)"
  CDDS_PREFIX=$VENV/cyclonedds
}

activate_venv() {
  # activate is not written for set -u
  set +u
  # shellcheck disable=SC1091
  . "$VENV/bin/activate"
  set -u
  [[ $(python -c 'import os, sys; print(os.path.realpath(sys.prefix))') == "$VENV" ]] \
    || die "sourcing $VENV/bin/activate did not activate it (was the venv moved?)"
  PY=$(python -c 'import sys; print("%d.%d" % sys.version_info[:2])')
  python -c 'import sys; sys.exit(sys.version_info < (3, 12))' || die "LeRobot needs Python 3.12 or newer, $VENV has $PY"
  [[ $PY == "$OS_PYTHON" ]] || warn "the INCAR installer makes a Python $OS_PYTHON venv on $OS_NAME, $VENV has Python $PY; continuing with $PY"
}

check_venv() {
  python -c 'import incar' 2>/dev/null \
    || die "incar does not import in $VENV; install INCAR first (https://$INDEX_HOST/install.sh)"
  python -c 'from draccus import ChoiceRegistryBase' 2>/dev/null || die "INCAR's draccus in $VENV was \
replaced by the PyPI draccus, probably by installing LeRobot with its dependencies. Repair it first:
  source $VENV/bin/activate
  pip uninstall -y draccus && pip install --force-reinstall --no-deps incar_draccus"
  # The venv's pip config uses the INCAR index for every package
  if python -m pip config list 2>/dev/null | grep -q "$INDEX_HOST" \
    && ! grep -qsE "machine[[:space:]]+$INDEX_HOST" "$HOME/.netrc"; then
    die "pip in $VENV uses the INCAR package index, but ~/.netrc has no credentials for it; rerun the INCAR installer"
  fi
  INCAR_VERSIONS=$(package_versions "${INCAR_PACKAGES[@]}")
  NUMPY_VERSION=$(package_versions numpy)
}

# package_versions NAME... -> "NAME==VERSION ..." for the installed ones
package_versions() {
  python -c 'import importlib.metadata as m, sys
def version(n):
  try: return f"{n}=={m.version(n)}
  except m.PackageNotFoundError: return ""
print(" ".join(filter(None, map(version, sys.argv[1:]))))' "$@"
}

default_jobs() {
  # Pinocchio's template-heavy files take up to about 3 GB each to compile
  local cpus by_mem
  cpus=$(nproc)
  by_mem=$(( $(awk '/^MemAvailable:/ {print $2}' /proc/meminfo) / 3145728 ))
  (( by_mem >= 1 )) || by_mem=1
  echo $(( cpus < by_mem ? cpus : by_mem ))
}

plan() {
  JOBS=${JOBS:-$(default_jobs)}
  APT_PACKAGES=(git cmake build-essential "python$PY-dev" libgl1 libegl1)
  APT_MISSING=()
  local p
  for p in "${APT_PACKAGES[@]}"; do
    dpkg-query -W -f='${Status}' "$p" 2>/dev/null | grep -q 'install ok installed' || APT_MISSING+=("$p")
  done
}

cyclonedds_is_current() {
  [[ -f $CDDS_PREFIX/.build-ok && $(cat "$CDDS_PREFIX/.build-ok") == "$CYCLONEDDS_C_VERSION" ]]
}

pinocchio_is_current() {
  python -c 'import sys, casadi, pinocchio, pinocchio.casadi
sys.exit(pinocchio.__version__ != sys.argv[1] or casadi.__version__ != sys.argv[2])' \
    "$PINOCCHIO_VERSION" "$CASADI_VERSION" 2>/dev/null
}

confirm_plan() {
  echo
  echo "The installer will, in $VENV (Python $PY, $OS_NAME):"
  if (( ${#APT_MISSING[@]} )); then
    echo "  - install apt packages (sudo): ${APT_MISSING[*]}"
  fi
  if cyclonedds_is_current; then
    echo "  - reuse the CycloneDDS $CYCLONEDDS_C_VERSION C library in $CDDS_PREFIX"
  else
    echo "  - build the CycloneDDS $CYCLONEDDS_C_VERSION C library into $CDDS_PREFIX"
  fi
  cat <<EOF
  - install cyclonedds $CYCLONEDDS_PY_VERSION (compiled against it), unitree_sdk2py (${UNITREE_SDK_COMMIT:0:7}) and $OPENCV_SPEC
  - install ${RUNTIME_PACKAGES[*]%%[<>=]*}
EOF
  if pinocchio_is_current; then
    echo "  - reuse Pinocchio $PINOCCHIO_VERSION with CasADi support"
  else
    echo "  - build Pinocchio $PINOCCHIO_VERSION with CasADi support, $JOBS parallel jobs (takes a long time)"
    [[ ${NUMPY_VERSION#numpy==} == 2.3.* ]] \
      || echo "    Its cmeel-boost 1.89 needs numpy 2.3, so ${NUMPY_VERSION/==/ } becomes numpy 2.3.x"
  fi
  cat <<EOF
  - install LeRobot (${LEROBOT_COMMIT:0:7}) and unitree_g1_extension from $REPO_DIR,
    both without their dependencies
  Unchanged: $INCAR_VERSIONS
EOF
  (( ASSUME_YES )) && return
  have_tty || die "no terminal to confirm on, rerun with --yes to proceed without asking"
  ask_yn "Continue?" || die "aborted, nothing was changed"
}

install_apt_packages() {
  (( ${#APT_MISSING[@]} )) || return 0
  log "Installing apt packages: ${APT_MISSING[*]}"
  local apt=(sudo env DEBIAN_FRONTEND=noninteractive apt-get -y -q)
  "${apt[@]}" update
  apt-cache policy "python$PY-dev" | grep -q 'Candidate: [0-9]' || die "apt has no python$PY-dev. \
On Ubuntu 22.04 it comes from the deadsnakes PPA, which the INCAR installer adds; rerun the INCAR installer first"
  "${apt[@]}" install --no-install-recommends "${APT_MISSING[@]}"
}

build_cyclonedds() {
  if cyclonedds_is_current; then
    log "Reusing the CycloneDDS $CYCLONEDDS_C_VERSION C library in $CDDS_PREFIX"
    return
  fi
  log "Building the CycloneDDS $CYCLONEDDS_C_VERSION C library into $CDDS_PREFIX"
  local src=$BUILD_DIR/cyclonedds
  run_logged git clone --quiet --depth 1 -b "$CYCLONEDDS_C_VERSION" "$CYCLONEDDS_URL" "$src"
  [[ $(git -C "$src" rev-parse HEAD) == "$CYCLONEDDS_C_COMMIT" ]] \
    || die "CycloneDDS tag $CYCLONEDDS_C_VERSION is not commit $CYCLONEDDS_C_COMMIT"
  rm -rf "$CDDS_PREFIX"
  # Shared memory, TLS and DDS security are not used by the Unitree SDK
  run_logged cmake -S "$src" -B "$src/build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$CDDS_PREFIX" \
    -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DBUILD_DDSPERF=OFF \
    -DENABLE_SHM=OFF -DENABLE_SSL=OFF -DENABLE_SECURITY=OFF
  run_logged cmake --build "$src/build" --target install -j "$JOBS"
  echo "$CYCLONEDDS_C_VERSION" > "$CDDS_PREFIX/.build-ok"
}

install_unitree_sdk() {
  log "Installing cyclonedds $CYCLONEDDS_PY_VERSION and unitree_sdk2py"
  # cyclonedds 0.10.x calls _Py_IsFinalizing(), which Python 3.13 renamed to
  # Py_IsFinalizing(); GCC 14 rejects the undeclared call
  local cflags=${CFLAGS:-}
  python -c 'import sys; sys.exit(sys.version_info < (3, 13))' && cflags+=" -D_Py_IsFinalizing=Py_IsFinalizing"
  # Not from pip's cache: a cached build may point at another CycloneDDS
  run_logged env CYCLONEDDS_HOME="$CDDS_PREFIX" CFLAGS="$cflags" python -m pip install --no-cache-dir --force-reinstall \
    --no-deps "cyclonedds==$CYCLONEDDS_PY_VERSION"
  # Editable, as Unitree documents: its setup.py leaves out utils/lib/crc_*.so,
  # which the SDK loads from its source directory. pip clones it into <venv>/src.
  run_logged env CYCLONEDDS_HOME="$CDDS_PREFIX" python -m pip install "cyclonedds==$CYCLONEDDS_PY_VERSION" \
    "$OPENCV_SPEC" --src "$VENV/src" -e "git+$UNITREE_SDK_URL@$UNITREE_SDK_COMMIT#egg=unitree_sdk2py"
}

install_runtime_packages() {
  log "Installing the packages LeRobot's G1 code and its MuJoCo simulation import"
  run_logged python -m pip install "${RUNTIME_PACKAGES[@]}"
}

build_pinocchio() {
  if pinocchio_is_current; then
    log "Reusing Pinocchio $PINOCCHIO_VERSION with CasADi support"
    return
  fi
  log "Building Pinocchio $PINOCCHIO_VERSION with CasADi support ($JOBS jobs), this takes a long time"
  # Built in the venv rather than pip's isolated build environment, so pin
  # compiles against the libpinocchio built here and not the PyPI wheel
  run_logged python -m pip install "${PINOCCHIO_BUILD_DEPS[@]}"
  local casadi_dir pkg
  casadi_dir=$(python -c 'import casadi, os; print(os.path.join(os.path.dirname(casadi.__file__), "cmake"))')
  # USE_LINK_PATH records casadi's library directory in the RPATH, so
  # pinocchio.casadi finds libcasadi also when casadi is not imported first
  for pkg in libpinocchio pin; do
    run_logged env CMEEL_JOBS="$JOBS" CMEEL_RUN_TESTS=OFF\
      CMEEL_CMAKE_ARGS="-DBUILD_WITH_CASADI_SUPPORT=ON -Dcasadi_DIR=$casadi_dir -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON" \
      python -m pip install --no-cache-dir --no-build-isolation --no-deps --force-reinstall \
      --no-binary "$pkg" "$pkg==$PINOCCHIO_VERSION"
  done
  pinocchio_is_current || die "Pinocchio was built, but pinocchio.casadi does not import, see $LOG"
}

install_lerobot_and_extension() {
  log "Installing LeRobot and unitree_g1_extension without their dependencies"
  run_logged python -m pip install --no-deps --force-reinstall "lerobot @ git+$LEROBOT_URL@$LEROBOT_COMMIT"
  run_logged python -m pip install --no-deps --force-reinstall "$REPO_DIR"
}

verify() {
  log "Checking the installation (downloads the G1 model, lerobot/unitree-g1-mujoco, if needed)"
  # INCAR's draccus prints a line for every config class LeRobot registers
  PYGAME_HIDE_SUPPORT_PROMPT=1 python - "$INCAR_VERSIONS" <<'EOF' 2>&1 | grep -vE '^(registering|hot-reloading) type' \
    || die "the check failed, see above"
import importlib.metadata as m, sys
import numpy as np

changed = [f"{s} -> {m.version(s.split('==')[0])}" for s in sys.argv[1].split()
           if m.version(s.split("==")[0]) != s.split("==")[1]]
assert not changed, f"INCAR packages changed: {changed}"
from draccus import ChoiceRegistryBase  # INCAR's draccus fork is still in place
import incar.extensions.cameras

import casadi, cv2, mujoco, pinocchio, pinocchio.casadi, zmq
import unitree_sdk2py.core.channel
from unitree_sdk2py.utils.crc import CRC
CRC()  # loads the SDK's crc_<arch>.so, which LeRobot's G1 needs on connect
from lerobot.robots.unitree_g1 import UnitreeG1Config
missing = {"is_simulation", "robot_ip", "gravity_compensation", "end_effector", "sim_publish_images"} \
    - set(UnitreeG1Config.__dataclass_fields__)
assert not missing, f"this LeRobot's UnitreeG1Config lacks {missing}"
import unitree_g1_extension.robot_interface.run
from unitree_g1_extension.robot_interface.kinematics import ArmIK
from unitree_g1_extension.robot_interface.robot import _lerobot_urdf

# The extension's arm IK
ik = ArmIK(_lerobot_urdf(), "right")
ik.reset(np.zeros(7))
assert np.all(np.isfinite(ik.step(np.array([0.05, 0, 0, 0, 0, 0]), 0.01)))

# LeRobot's arm IK, built on connect for gravity compensation. solve_tau returns
# zeros on any error; with the elbows bent, gravity gives a torque.
from lerobot.robots.unitree_g1.g1_kinematics import G1_29_ArmIK
q = np.zeros(14)
q[[3, 10]] = 1.0
assert np.abs(G1_29_ArmIK().solve_tau(q)).max() > 1e-3, "LeRobot's G1 arm IK returned no gravity torque"

print(f"OK: Pinocchio {pinocchio.__version__} with CasADi {casadi.__version__}, MuJoCo {mujoco.__version__}, "
      f"OpenCV {cv2.__version__}, numpy {np.__version__}")
EOF

  # LeRobot's unmet pins (torch, draccus, numpy, ...) are expected
  local problems
  problems=$(python -m pip check 2>&1 | grep -vE '^(lerobot |No broken requirements)' || true)
  [[ -z $problems ]] || warn "pip check reports more than LeRobot's pins:
$problems"
}

main() {
  parse_args "$@"
  check_system
  ask_venv
  activate_venv
  check_venv
  plan
  confirm_plan
  if (( ${#APT_MISSING[@]} )); then
    command -v sudo >/dev/null || die "sudo is required to install ${APT_MISSING[*]}"
    sudo true   # ask for the password up front, not halfway through
  fi
  BUILD_DIR=$(mktemp -d)
  LOG=$BUILD_DIR/install.log
  log "Build log: $LOG"
  install_apt_packages
  build_cyclonedds
  install_unitree_sdk
  install_runtime_packages
  build_pinocchio
  install_lerobot_and_extension
  verify
  rm -rf "$BUILD_DIR"
  printf '\n\033[1munitree_g1_extension is installed. Start the G1 interface with:\033[0m\n'
  printf '  source %s/bin/activate\n  python -m unitree_g1_extension.robot_interface.run --robot sim\n' "$VENV"
  printf 'For --no-viewer on a machine without a display, set MUJOCO_GL=egl.\n'
}

main "$@"
