#!/usr/bin/env bash
set -euo pipefail

INSTALL_ROOT=""
OUTPUT_DIR=""
ROS_DISTRO="${ROS_DISTRO:-noetic}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PACKAGE="ros-${ROS_DISTRO}-xgc2-robot-visualization"
ROS_PACKAGE="xgc2_robot_visualization"

product_version() {
  awk -F': *' '/^version:[[:space:]]*/ {print $2; exit}' "${REPO_ROOT}/.xgc2/product.yml"
}

VERSION="${PACKAGE_VERSION:-$(product_version)}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --install-root)
      INSTALL_ROOT="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

if [[ -z "${INSTALL_ROOT}" || -z "${OUTPUT_DIR}" ]]; then
  echo "--install-root and --output-dir are required" >&2
  exit 1
fi
if [[ -z "${VERSION}" ]]; then
  echo "package version is missing" >&2
  exit 1
fi

ARCH="$(dpkg --print-architecture)"
PREFIX="/opt/ros/${ROS_DISTRO}"
PREFIX_ROOT="${INSTALL_ROOT}${PREFIX}"
PKG_ROOT="$(mktemp -d)"

cleanup() {
  rm -rf "${PKG_ROOT}"
}
trap cleanup EXIT

mkdir -p "${OUTPUT_DIR}" "${PKG_ROOT}/DEBIAN" "${PKG_ROOT}/usr/share/doc/${PACKAGE}"
rm -f "${OUTPUT_DIR}"/*.deb

copy_path() {
  local src="$1"
  if [[ -e "${src}" ]]; then
    mkdir -p "${PKG_ROOT}$(dirname "${src#${INSTALL_ROOT}}")"
    cp -a "${src}" "${PKG_ROOT}${src#${INSTALL_ROOT}}"
  fi
}

copy_path "${PREFIX_ROOT}/share/${ROS_PACKAGE}"
copy_path "${PREFIX_ROOT}/include/${ROS_PACKAGE}"
copy_path "${PREFIX_ROOT}/lib/libfs150_uav_visualizer.so"
copy_path "${PREFIX_ROOT}/lib/libscout_ugv_visualizer.so"

cat > "${PKG_ROOT}/DEBIAN/control" <<EOF
Package: ${PACKAGE}
Version: ${VERSION}
Section: misc
Priority: optional
Architecture: ${ARCH}
Maintainer: XGC2 <apt@example.com>
Depends: ros-${ROS_DISTRO}-geometry-msgs, ros-${ROS_DISTRO}-roscpp, ros-${ROS_DISTRO}-std-msgs, ros-${ROS_DISTRO}-visualization-msgs, ros-${ROS_DISTRO}-xgc2-fs150-description (>= 0.1.0-1), ros-${ROS_DISTRO}-xgc2-scout-description (>= 0.4.10-1)
Description: XGC2 reusable robot RViz marker visualization libraries
EOF

printf '%s package\n' "${PACKAGE}" > "${PKG_ROOT}/usr/share/doc/${PACKAGE}/README"
find "${PKG_ROOT}" -type d -exec chmod 0755 {} +
find "${PKG_ROOT}" -type f -exec chmod 0644 {} +
chmod 0755 "${PKG_ROOT}/DEBIAN"

fakeroot dpkg-deb --build "${PKG_ROOT}" "${OUTPUT_DIR}/${PACKAGE}_${VERSION}_${ARCH}.deb" >/dev/null
find "${OUTPUT_DIR}" -maxdepth 1 -type f -name '*.deb' -print | sort
