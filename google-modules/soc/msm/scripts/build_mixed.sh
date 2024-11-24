#!/bin/bash -e
# SPDX-License-Identifier: GPL-2.0
#
# Generate GKI kernel and device kernel binaries

trap 'echo $0: line $LINENO: command exited with status $? >&2' ERR

source private/google-modules/soc/msm/scripts/script_utils.sh

# Need to be exported for build.sh usage.
export LTO=${LTO:-thin}
export BUILD_CONFIG
export GKI_BUILD_CONFIG
export GKI_PREBUILTS_DIR

BUILD_KERNEL=${BUILD_KERNEL:-1}

function check_dirty_common() {
  local device_name=$(echo ${DEVICE_BAZEL_DIST_TARGET} | cut -d '/' -f6 | cut -d ':' -f1)
  local gki_artifacts_dir="out/${device_name}/dist"

  if [[ -f ${gki_artifacts_dir}/vmlinux ]]; then
    local sha_file=vmlinux
  else
    local sha_file=boot.img
  fi

  local prebuilts_sha=$(strings ${gki_artifacts_dir}/${sha_file} |
                        grep "Linux version 5.15" | sed -n "s/^.*-g\([0-9a-f]\{12\}\)-.*/\1/p")
  local common_sha=$(git -C common/ log -1 --abbrev=12 --pretty="format:%h")
  local modified_kernel_files=$(git -C common/ --no-optional-locks status -uno --porcelain ||
                                git -C common/ diff-index --name-only HEAD)

  if [[ "${prebuilts_sha}" != "${common_sha}" ]] || [[ -n "${modified_kernel_files}" ]]; then
    echo "WARNING: There are changes in common/ which are not in the prebuilts."
    echo "Because you did not specify BUILD_KERNEL=1, $0 defaulted to building"
    echo "with the prebuilts. Please be aware that your changes to common/ will not"
    echo "be present in the final images. If you have made changes to common/"
    echo "that you wish to build, it is recommended you explicitly set BUILD_KERNEL=1."
    echo "Otherwise, the prebuilts will be used."
  fi
}

if [[ -z "${DEVICE_BAZEL_BUILD_PARAMETERS}" ]] || \
   [[ -z "${DEVICE_BAZEL_DIST_TARGET}" ]]; then
  echo "DEVICE_BAZEL_BUILD_PARAMETERS, and DEVICE_BAZEL_DIST_TARGET must all be specified to create a mixed bazel build." >&2
  exit 1
fi

BASE_BAZELRC_FILE="private/google-modules/soc/msm/device.bazelrc"

if ! [[ -f "${BASE_BAZELRC_FILE}" ]]; then
  echo "${BASE_BAZELRC_FILE} must exist to create a mixed bazel build." >&2
  exit 1
fi

USE_GKI=`grep "^build --config=download_gki" $BASE_BAZELRC_FILE || true`
NO_USE_GKI=`grep "^build --config=no_download_gki" $BASE_BAZELRC_FILE || true`
NO_SYMBOL_TRIMMING=`grep "^build --config=disable_symbol_trimming" $BASE_BAZELRC_FILE || true`
GKI_BUILD_ID=`sed -n 's/.*gki_prebuilts=\([0-9]\+\).*/\1/p' $BASE_BAZELRC_FILE`

BAZEL_PARAMETERS="--config=stamp --config=fast "
BAZEL_PARAMETERS+="${DEVICE_BAZEL_BUILD_PARAMETERS} "

if [[ -n "${NO_USE_GKI}" ]]; then
  echo "--config=no_download_gki detected in device.bazelrc; building with core-kernel generated from common/ source tree"
elif [[ -n "${NO_SYMBOL_TRIMMING}" ]]; then
  echo "--config=disable_symbol_trimming detected in device.bazelrc; building with core-kernel generated from common/ source tree and symbol trimming disabled"
elif [[ "${DISABLE_SYMBOL_TRIMMING}" == "1" ]]; then
  echo "DISABLE_SYMBOL_TRIMMING=1 detected on the command line; building with core-kernel generated from common/ source tree and symbol trimming disabled"
  BAZEL_PARAMETERS+="--config=disable_symbol_trimming "
elif [[ "${BUILD_KERNEL}" == 1 ]]; then
  echo "BUILD_KERNEL=1 detected on the command line; building with core-kernel generated from common/ source tree"
  BAZEL_PARAMETERS+="--config=no_download_gki "
elif [[ -n "${GKI_BUILD_ID}" ]] && [[ -n "${USE_GKI}" ]]; then
  echo "Building with GKI prebuilts from ab/$GKI_BUILD_ID - kernel_aarch64"
else
  echo "Please check $BASE_BAZELRC_FILE"
  echo -e "  1) If BUILD_KERNEL=1 or \"build --config=no_download_gki\" ----> core kernel generated from common/ source tree"
  echo -e "  2) If \"build --config=download_gki\"                      ----> core-kernel based on GKI prebuilts"
  echo -e "  3) If DISABLE_SYMBOL_TRIMMING=1 or \"build --config=disable_symbol_trimming\" --> core kernel generated from common/ source tree with symbol trimming disabled"
  exit 1
fi

tools/bazel run ${BAZEL_PARAMETERS} ${DEVICE_BAZEL_DIST_TARGET} "$@"

if [[ "${BUILD_KERNEL}" != 1 ]]; then
  check_dirty_common
fi
