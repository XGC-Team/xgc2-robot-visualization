#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

DOCKER_IMAGE="${DOCKER_IMAGE:-ghcr.io/xgc-team/xgc2-images/xgc2-build-focal-ros-noetic:1.0.0}"
WORK_DIR="${WORK_DIR:-${REPO_ROOT}/.work/docker}"
OUTPUT_DIR="${OUTPUT_DIR:-${REPO_ROOT}/debs}"
INSTALL_CHECK="${INSTALL_CHECK:-true}"
XGC2_DEPENDENCY_SET_DIGEST="${XGC2_DEPENDENCY_SET_DIGEST:-}"
EMPTY_DEPENDENCY_SET_DIGEST="4f53cda18c2baa0c0354bb5f9a3ecbe5ed12ab4d8e11ba873c2f11161202b945"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --image)
      DOCKER_IMAGE="$2"
      shift 2
      ;;
    --work-dir)
      WORK_DIR="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --skip-install-check)
      INSTALL_CHECK=false
      shift
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

mkdir -p "${WORK_DIR}" "${OUTPUT_DIR}"

if [[ -n "${XGC2_DEPENDENCY_SET_DIGEST}" &&
      ! "${XGC2_DEPENDENCY_SET_DIGEST}" =~ ^[0-9a-f]{64}$ ]]; then
  echo "XGC2_DEPENDENCY_SET_DIGEST must be empty or 64 lowercase hex characters" >&2
  exit 1
fi
if [[ -n "${XGC2_APT_OVERLAY_URL:-}" && -z "${XGC2_DEPENDENCY_SET_DIGEST}" ]]; then
  echo "XGC2_APT_OVERLAY_URL requires XGC2_DEPENDENCY_SET_DIGEST" >&2
  exit 1
fi

docker pull "${DOCKER_IMAGE}"
docker run --rm \
  -e XGC2_APT_OVERLAY_URL="${XGC2_APT_OVERLAY_URL:-}" \
  -e XGC2_DEPENDENCY_SET_DIGEST="${XGC2_DEPENDENCY_SET_DIGEST}" \
  -e EMPTY_DEPENDENCY_SET_DIGEST="${EMPTY_DEPENDENCY_SET_DIGEST}" \
  -e DEBIAN_FRONTEND=noninteractive \
  -e INSTALL_CHECK="${INSTALL_CHECK}" \
  -v "${REPO_ROOT}:/workspace/repo:ro" \
  -v "${WORK_DIR}:/workspace/work" \
  -v "${OUTPUT_DIR}:/workspace/out" \
  "${DOCKER_IMAGE}" \
  bash -lc '
    set -euo pipefail

    export DEBIAN_FRONTEND=noninteractive
    rm -rf /workspace/work/src /workspace/work/build /workspace/work/devel /workspace/work/install-root
    mkdir -p /workspace/work/src/xgc2_robot_visualization
    rsync -a --delete /workspace/repo/ /workspace/work/src/xgc2_robot_visualization/

    cd /workspace/work
    source /opt/ros/noetic/setup.bash
    catkin_make -DCATKIN_ENABLE_TESTING=ON
    catkin_make run_tests_xgc2_robot_visualization
    catkin_test_results --verbose
    DESTDIR=/workspace/work/install-root catkin_make install \
      -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic \
      -DCATKIN_ENABLE_TESTING=ON

    /workspace/repo/.xgc2/scripts/package_debs.sh \
      --install-root /workspace/work/install-root \
      --output-dir /workspace/out

    if [[ "${INSTALL_CHECK}" == "true" ]]; then
      apt-get install -y /workspace/out/ros-noetic-xgc2-robot-visualization_*.deb
      /workspace/repo/.xgc2/scripts/check_installed_packages.sh
    fi
  '

echo "Debian package output:"
find "${OUTPUT_DIR}" -maxdepth 1 -type f -name "*.deb" -print | sort
