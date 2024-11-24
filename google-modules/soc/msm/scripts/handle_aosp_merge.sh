#!/bin/bash
# SPDX-License-Identifier: GPL-2.0
#
# Update the common kernel git sha, merge new changes from the common kernel
# to the device kernel, and update the GKI kernel prebuilt binaries.

source private/google-modules/soc/msm/scripts/script_utils.sh

# Tracking bug of aosp-merge event
MERGE_BUG=${MERGE_BUG:-"NA"}

# GKI boot image build-id
BOOT_IMAGE_BUILDID=${BOOT_IMAGE_BUILDID:-}

# GKI Kernel Branch (on aosp gerrit)
AOSP_BRANCH=${AOSP_BRANCH:-"android14-5.15"}

# Kernel Manifest
KERNEL_MANIFEST=${KERNEL_MANIFEST:-"android14-msm-pixelwatch-5.15"}

# Whether or not to push commits to gerrit
PUSH=${PUSH:-0}

# The branch to push prebuilts updates to.
PREBUILTS_BRANCH=${PREBUILTS_BRANCH:-"android14-msm-pixelwatch-5.15"}

# Gerrit Topic e.g.:
# WK10_AOSP_MERGE_android14-msm-pixelwatch-5.15_AND_android14-5.15
TOPIC=${TOPIC:-WK`date +%U`_AOSP_MERGE_${KERNEL_MANIFEST}_AND_${AOSP_BRANCH}}

DEVICE_KERNEL_DIR=${DEVICE_KERNEL_DIR:-"private/msm-google"}

GKI_KERNEL_DIR=${GKI_KERNEL_DIR:-"common"}

CUR_DIR=$(pwd)

KERNEL_VERSION=""
KERNEL_PATCHLEVEL=""
KERNEL_SUBLEVEL=""
AOSP_COMMIT_INFO=""

readonly MANIFEST_DIR=".repo/manifests"
readonly DEVICE_BAZELRC_DIR="private/google-modules/soc/msm"
readonly PREBUILTS_DIR="${DEVICE_BAZELRC_DIR}"

#-----------------------------------------------------------------------
# Reset color
Color_Off='\033[0m'       # Text Reset
# Regular Colors
Green='\033[0;32m'        # Green
Yellow='\033[0;33m'       # Yellow

function exit_if_error() {
  if [ $1 -ne 0 ]; then
    echo "ERROR: $2: retval=$1" >&2
    cd ${CUR_DIR}
    exit $1
  fi
}

function usage() {
  cat <<- EOF
    Usage:

    BOOT_IMAGE_BUILDID:     The build ID of the prebuilt boot image to download.
    AOSP_BRANCH:            The GKI branch that was used to generate the boot image.
    KERNEL_MANIFEST:        The kernel manifest branch to use for syncing the kernel repos.
    PUSH:                   Optional: Whether or not to automatically push generated commits
                            to gerrit for review. This is set to 0 by default.
    MERGE_BUG:              Optional: The bug number associated with this merge.

   Sample invocation:

   MERGE_BUG=261218518 BOOT_IMAGE_BUILDID=9611439 \\
   AOSP_BRANCH=android14-5.15-2023-02 \\
   KERNEL_MANIFEST=android14-msm-pixelwatch-5.15 $0

   Note:
     1. You can set PUSH=1 when invoking $0 to automatically push the generated commits
     to gerrit for review.
EOF
}

function sanitize_args() {
  if [[ -z "${BOOT_IMAGE_BUILDID}" ]]; then
    usage
    exit 1
  elif [[ ! -d "${CUR_DIR}/${GKI_KERNEL_DIR}" ]]; then
    echo "Cannot find GKI kernel directory ${GKI_KERNEL_DIR}/" >&2
    exit 1
  elif [[ ! -d "${CUR_DIR}/${DEVICE_KERNEL_DIR}" ]]; then
    echo "Cannot find device kernel directory ${DEVICE_KERNEL_DIR}/" >&2
    exit 1
  fi
}

function sync_kernel_repos() {
  local manifest_proj_url="persistent-https://partner-android.git.corp.google.com/kernel/manifest"

  echo -e "\n${Yellow}Running repo init and syncing ${KERNEL_MANIFEST}${Color_Off}"

  repo init -q -u "${manifest_proj_url}" -b "${KERNEL_MANIFEST}"
  exit_if_error $? "Failed to initialize kernel repo branch ${KERNEL_MANIFEST}"

  repo sync -q -cd -j$(nproc)
  exit_if_error $? "Failed to sync kernel repos"
}

function get_gki_kernel_info() {
  pushd ${CUR_DIR}/${GKI_KERNEL_DIR} > /dev/null

  KERNEL_VERSION=$(grep -r -m 1 "VERSION =" Makefile | sed 's/.*VERSION = //g')
  KERNEL_PATCHLEVEL=$(grep -r -m 1 "PATCHLEVEL =" Makefile | \
                      sed 's/.*PATCHLEVEL = //g')
  KERNEL_SUBLEVEL=$(grep -r -m 1 "SUBLEVEL =" Makefile | \
                    sed 's/.*SUBLEVEL = //g')
  AOSP_COMMIT_INFO=$(git log --pretty=format:'%h %s' -1)

  popd > /dev/null
}

function update_manifest_and_sync_gki_kernel() {
  local rev_current_aosp

  /google/data/ro/projects/android/fetch_artifact --bid ${BOOT_IMAGE_BUILDID} \
              --target kernel_aarch64 "manifest_${BOOT_IMAGE_BUILDID}.xml"
  exit_if_error $? "Unable to download manifest_${BOOT_IMAGE_BUILDID}.xml"

  REV_NEW_AOSP=$(grep -r "=\"kernel/common\""\
                 manifest_${BOOT_IMAGE_BUILDID}.xml | \
                 sed 's/.*revision="//g'| sed 's/".*//g')
  rm manifest_${BOOT_IMAGE_BUILDID}.xml

  rev_current_aosp=$(repo info kernel/${GKI_KERNEL_DIR} | grep "Current revision" | \
                     sed 's/Current revision: //g')

  pushd ${MANIFEST_DIR} > /dev/null

  sed -i "s/${rev_current_aosp}/${REV_NEW_AOSP}/" default.xml

  popd > /dev/null

  echo -e "\n${Yellow}Syncing the GKI kernel to new revision: ${REV_NEW_AOSP}${Color_Off}"
  repo sync -q -cd -j$(nproc)
  exit_if_error $? "Unable to sync ${GKI_KERNEL_DIR}/"
}

function commit_and_push_manifest_update() {
  local merge_msg=$(printf "Update ${AOSP_BRANCH} sync point to (${KERNEL_VERSION}.${KERNEL_PATCHLEVEL}.${KERNEL_SUBLEVEL})\n\nSync SHA:\n${AOSP_COMMIT_INFO}\n\nBug: ${MERGE_BUG}")

  pushd ${MANIFEST_DIR} > /dev/null

  git add default.xml
  git commit --quiet -s -m "${merge_msg}"

  if [[ $PUSH -eq 1 ]]; then
    echo -e "\n${Yellow}Pushing manifest updates...${Color_Off}"
    git push origin HEAD:refs/for/${KERNEL_MANIFEST} -o topic=${TOPIC}
  fi

  popd > /dev/null
}

function update_gki_kernel() {
  get_gki_kernel_info
  echo -e "\n${Green}Current GKI Kernel Info${Color_Off}"
  echo -e "KERNEL_VERSION = [${Green}${KERNEL_VERSION}${Color_Off}]"
  echo -e "KERNEL_PATCHLEVEL = [${Green}${KERNEL_PATCHLEVEL}${Color_Off}]"
  echo -e "KERNEL_SUBLEVEL = [${Green}${KERNEL_SUBLEVEL}${Color_Off}]"
  echo -e "AOSP_COMMIT_INFO = [${Green}${AOSP_COMMIT_INFO}${Color_Off}]\n"

  update_manifest_and_sync_gki_kernel

  get_gki_kernel_info
  echo -e "\n${Green}New GKI Kernel Info:${Color_Off}"
  echo -e "KERNEL_VERSION = [${Green}${KERNEL_VERSION}${Color_Off}]"
  echo -e "KERNEL_PATCHLEVEL = [${Green}${KERNEL_PATCHLEVEL}${Color_Off}]"
  echo -e "KERNEL_SUBLEVEL = [${Green}${KERNEL_SUBLEVEL}${Color_Off}]"
  echo -e "AOSP_COMMIT_INFO = [${Green}${AOSP_COMMIT_INFO}${Color_Off}]"

  commit_and_push_manifest_update
}

function merge_aosp_to_device_kernel() {
  local aosp_remote
  local merge_msg
  local bugid_script_dir

  BRANCH_DEVICE_KERNEL=$(repo info kernel/${DEVICE_KERNEL_DIR} | \
                         grep "Manifest revision" | \
                         sed 's/Manifest revision: //g')

  echo -e "\n${Yellow}Merging ${GKI_KERNEL_DIR} SHA ${REV_NEW_AOSP} to ${DEVICE_KERNEL_DIR} ...${Color_Off}"

  bugid_script_dir=${CUR_DIR}/${SCRIPTS_DIR}

  pushd ${CUR_DIR}/${DEVICE_KERNEL_DIR} > /dev/null

  aosp_remote=$(git remote -v | grep -o -m 1 -e "^aosp.*common")
  if [[ -z "${aosp_remote}" ]]; then
    aosp_remote=$(git -C ${CUR_DIR}/${GKI_KERNEL_DIR} remote -v | grep -o -m 1 -e "^aosp.*common")
    git remote add ${aosp_remote}
  fi

  git fetch --quiet aosp "${AOSP_BRANCH}"
  merge_msg=$(printf "Merge ${AOSP_BRANCH} into ${BRANCH_DEVICE_KERNEL}\n\nMerge SHA:\n${AOSP_COMMIT_INFO}\n\nBug: ${MERGE_BUG}")
  git merge --quiet --signoff --no-ff -X patience ${REV_NEW_AOSP} \
            -m "${merge_msg}"

  if ! git diff --cached --quiet HEAD; then
    git add -A
    git commit -a --no-edit --quiet --no-verify

    echo "${merge_msg}" >> CommitInfo.txt
    python3 ${bugid_script_dir}/BugId.py CommitInfo.txt
    git commit --amend --quiet -s -F CommitInfo.txt
    rm CommitInfo.txt

    if [[ $PUSH -eq 1 ]]; then
      echo -e "\n${Yellow}Pushing updates to ${DEVICE_KERNEL_DIR}${Color_Off}"
      git push partner HEAD:refs/for/${BRANCH_DEVICE_KERNEL} -o topic=${TOPIC}
    fi
  else
    echo -e "\n${Yellow}No updates required to ${DEVICE_KERNEL_DIR}${Color_Off}"
  fi

  popd > /dev/null
}

function update_gki_prebuilts() {
  local current_build_id
  local merge_msg=$(printf "Update the GKI prebuilt build ID to ab/${BOOT_IMAGE_BUILDID}\n\nUse GKI prebuilt binaries from ab/${BOOT_IMAGE_BUILDID}\n\nBug: ${MERGE_BUG}")

  echo -e "\n${Yellow}Updating GKI prebuilts ID to ab/${BOOT_IMAGE_BUILDID} in top-level device.bazelrc file ...${Color_Off}"

  pushd ${CUR_DIR}/${PREBUILTS_DIR} > /dev/null

  current_build_id=$(grep -e "gki_prebuilts=[0-9]*" device.bazelrc | grep -oe "[0-9]*")

  sed -i "s/${current_build_id}/${BOOT_IMAGE_BUILDID}/" device.bazelrc

  git add device.bazelrc
  git commit --quiet -s -m "${merge_msg}"

  if [[ $PUSH -eq 1 ]]; then
    echo -e "\n${Yellow}Pushing updates to GKI prebuilts${Color_Off}"
    git push partner HEAD:refs/for/${PREBUILTS_BRANCH} -o topic=${TOPIC}
  fi

  popd > /dev/null
}

function log_push_disabled_notice() {
  cat <<- EOF
    Commits have been generated for integrating ACK SHA ${REV_NEW_AOSP}
    but have not been pushed to gerrit.

    To ensure that the commits are pushed to gerrit, rerun the script with
    PUSH=1 or push each commit to gerrit manually with the following
    commands:

    # Manifest update
    cd ${MANIFEST_DIR}
    git push origin HEAD:refs/for/${KERNEL_MANIFEST}
    cd ..

    # Device kernel update
    cd ${DEVICE_KERNEL_DIR}
    git push partner HEAD:refs/for/${BRANCH_DEVICE_KERNEL}
    cd ..

    # GKI prebuilts update
    cd ${PREBUILTS_DIR}
    git push partner HEAD:refs/for/${PREBUILTS_BRANCH}
    cd ..
EOF
}

sanitize_args

echo -e "\nBegin.\n"
echo -e "TOPIC = [${Green}${TOPIC}${Color_Off}]"

sync_kernel_repos

update_gki_kernel

merge_aosp_to_device_kernel

update_gki_prebuilts

echo -e "\nDone.\n"

if [[ $PUSH -ne 1 ]]; then
  log_push_disabled_notice
fi
