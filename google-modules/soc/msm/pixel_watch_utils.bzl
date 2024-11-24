# SPDX-License-Identifier: GPL-2.0-or-later

"""Utility file for devices that are based on the sw5100 SoC."""

load(
    "@bazel_skylib//lib:paths.bzl",
    "paths",
)
load(
    "@bazel_skylib//rules:write_file.bzl",
    "write_file",
)
load(
    "//build/bazel_common_rules/dist:dist.bzl",
    "copy_to_dist_dir",
)
load(
    "//build/kernel/kleaf:hermetic_tools.bzl",
    "hermetic_genrule",
    "hermetic_toolchain",
)
load(
    "//build/kernel/kleaf:kernel.bzl",
    "kernel_abi",
    "kernel_build",
    "kernel_build_config",
    "kernel_images",
    "kernel_module_group",
    "kernel_modules_install",
    "kernel_unstripped_modules_archive",
    "merged_kernel_uapi_headers",
)
load(
    "//common:modules.bzl",
    "get_gki_modules_list",
)
load(
    "//private/msm-google:monaco.bzl",
    "monaco_in_tree_modules",
)

KERNEL_DIR = "private/msm-google"

SW5100_ADDITIONAL_IN_TREE_MODULES = [
    # keep sorted
    "drivers/i2c/i2c-dev.ko",
    "drivers/irqchip/msm_show_resume_irq.ko",
    "drivers/watchdog/softdog.ko",
]

# These modules are not required on sw5100 devices and
# are disabled by the arch/arm64/configs/sw5100.fragment.
SW5100_MODULES_EXCLUDE_LIST = [
    # keep sorted
    "drivers/power/supply/qcom/qpnp-smblite-main.ko",
    "drivers/power/supply/qcom/qti-qbg-main.ko",
    "drivers/remoteproc/qcom_rproc_slate.ko",
    "drivers/rpmsg/qcom_glink_slatecom.ko",
    "drivers/soc/qcom/slate_events_bridge.ko",
    "drivers/soc/qcom/slate_events_bridge_rpmsg.ko",
    "drivers/soc/qcom/slate_rsb.ko",
    "drivers/soc/qcom/slatecom_event.ko",
    "drivers/soc/qcom/slatecom_interface.ko",
    "drivers/soc/qcom/slatecom_rpmsg.ko",
    "drivers/soc/qcom/slatecom_spi.ko",
    "drivers/soc/qcom/slatersb_rpmsg.ko",
]

SW5100_COMMON_EXTERNAL_MODULES = [
    "//private/msm-google/drivers/power/supply/qcom:qcom_battery_modules",
    "//private/msm-google-modules/audio:monaco_gki_modules",
    "//private/msm-google-modules/dataipa:monaco_gki_modules",
    "//private/msm-google-modules/datarmnet:monaco_gki_modules",
    "//private/msm-google-modules/datarmnet-ext/wlan:monaco_gki_wlan",
    "//private/msm-google-modules/datarmnet-ext/mem:monaco_gki_rmnet_mem",
    "//private/msm-google-modules/display:monaco_gki_display_drivers",
    "//private/msm-google-modules/graphics:monaco_gki_msm_kgsl",
    "//private/msm-google-modules/mm/hw_fence:monaco_gki_msm_hw_fence",
    "//private/msm-google-modules/mm/msm_ext_display:monaco_gki_msm_ext_display",
    "//private/msm-google-modules/mm/sync_fence:monaco_gki_sync_fence",
    "//private/msm-google-modules/securemsm:monaco_gki_modules",
    "//private/msm-google-modules/video:monaco_gki_video_driver_modules",
    "//private/google-modules/bms",
    "//private/google-modules/bms/misc:bms-misc",
    "//private/google-modules/display/watch/mcu-offload/mcu_display",
    "//private/google-modules/modem/modemsmem",
    "//private/google-modules/nanohub",
    "//private/google-modules/nfc/nxp/snxxx:nfc_modules",
    "//private/google-modules/soc/msm/debug_kinfo",
    "//private/google-modules/soc/msm/drivers/extcon/google:extcon-shim",
    "//private/google-modules/soc/msm/drivers/power/supply/google:google-smblite-hvdcp",
    "//private/google-modules/soc/msm/restart_debug",
    "//private/google-modules/sound/mcu_mic_codec",
]

SW5100_COMMON_EXTERNAL_MODULES_DDK_UAPI_HEADERS = [
    "//private/msm-google-modules/audio:msm_audio_uapi_headers",
    "//private/msm-google-modules/dataipa:msm_dataipa_uapi_headers",
    "//private/msm-google-modules/display:msm_display_uapi_headers",
    "//private/msm-google-modules/securemsm:msm_securemsm_uapi_headers",
    "//private/msm-google-modules/video:msm_video_uapi_headers",
]

# TODO: b/323942195 - [Kleaf] Fix system_dlkm dedup_dlkm_modules feature when GKI modules are excluded
#
# Use all GKI modules for now. Otherwise, the GKI modules that are
# generated in the kernel_build() pass will be placed in the vendor_dlkm.img.
# These modules will not load successfully, since they won't be signed.
#
# This list can be trimmed down when support for filtering out GKI modules
# is implemented properly.
SW5100_COMMON_GKI_MODULES = get_gki_modules_list("arm64")

def _extracted_system_dlkm(ctx):
    hermetic_tools = hermetic_toolchain.get(ctx)

    inputs = []

    system_dlkm_archive = None
    for f in ctx.files.images:
        if f.basename == "system_dlkm_staging_archive.tar.gz":
            inputs.append(f)
            system_dlkm_archive = f
            break

    outs = []
    for m in ctx.attr.gki_modules:
        outs.append(ctx.actions.declare_file(m))
    common_outs = outs[0].dirname

    intermediates_dir = paths.join(
        ctx.bin_dir.path,
        paths.dirname(ctx.build_file_path),
        ctx.attr.name + "_intermediates",
    )

    command = hermetic_tools.setup
    command += """
        # Extract GKI modules
        mkdir -p {intermediates_dir}

        tar xf {system_dlkm_archive} -C {intermediates_dir}
        find {intermediates_dir} -name '*.ko' -exec cp -t {common_outs} {{}} \\+

        # Verify the outputs. We don't care if there are more modules extracted
        # than used. For example, we don't use zram.ko.
        all_modules=({all_modules})
        for m in "${{all_modules[@]}}"; do
            if ! [[ -f "{common_outs}/${{m}}" ]]; then
                echo "${{m}} is missing from $(basename {system_dlkm_archive})" >&2
                exit 1
            fi
        done
    """.format(
        all_modules = " ".join([m.basename for m in outs]),
        common_outs = common_outs,
        system_dlkm_archive = system_dlkm_archive.path,
        intermediates_dir = intermediates_dir,
    )

    ctx.actions.run_shell(
        mnemonic = "ExtractedSystemDlkm",
        inputs = inputs,
        outputs = outs,
        tools = hermetic_tools.deps,
        progress_message = "Extracting GKI modules",
        command = command,
    )

    return [DefaultInfo(files = depset(outs))]

extracted_system_dlkm = rule(
    doc = """Extracts the system_dlkm archive so that they can be copied to the dist_dir""",
    implementation = _extracted_system_dlkm,
    attrs = {
        "images": attr.label(
            doc = "The kernel_images target that contains the system_dlkm archive.",
            allow_files = True,
            mandatory = True,
        ),
        "gki_modules": attr.string_list(
            doc = "A list of GKI modules",
            allow_empty = False,
            mandatory = True,
        ),
    },
    toolchains = [hermetic_toolchain.type],
)

def _merged_external_kernel_modules_uapi_headers(ctx):
    hermetic_tools = hermetic_toolchain.get(ctx)

    # Using "kernel-uapi-headers.tar.gz" to remain compatible
    # with the existing tarball name.
    out_uapi_headers_tarball_name = "kernel-uapi-headers.tar.gz"
    out_file = ctx.actions.declare_file(out_uapi_headers_tarball_name)
    inputs = []
    outputs = [out_file]

    for f in ctx.files.merged_kernel_uapi_headers:
        if f.basename == "kernel-uapi-headers.tar.gz":
            inputs.append(f)

    for f in ctx.files.external_modules_kernel_uapi_headers:
        if f.basename.endswith("uapi-headers.tar.gz"):
            inputs.append(f)

    intermediates_dir = paths.join(
        ctx.bin_dir.path,
        paths.dirname(ctx.build_file_path),
        ctx.attr.name + "_intermediates",
    )

    command = hermetic_tools.setup
    command += """
        # Extract all UAPI headers
        mkdir -p {intermediates_dir}

        all_uapi_headers_archives=({all_uapi_headers_archives})

        # Unpack and repack all archives to combine them
        for archive in "${{all_uapi_headers_archives[@]}}"; do
            tar xf ${{archive}} -C {intermediates_dir}
        done

        tar czf {out_name} -C {intermediates_dir} usr
    """.format(
        intermediates_dir = intermediates_dir,
        all_uapi_headers_archives = " ".join([archive.path for archive in inputs]),
        out_name = out_file.path,
    )

    ctx.actions.run_shell(
        mnemonic = "MergedExternalKernelModulesUAPIHeaders",
        inputs = inputs,
        outputs = outputs,
        tools = hermetic_tools.deps,
        progress_message = "Merging external kernel modules UAPI headers",
        command = command,
    )

    return [DefaultInfo(files = depset(outputs))]

merged_external_kernel_modules_uapi_headers = rule(
    doc = """Merges the UAPI headers from external modules with the UAPI headers from the kernel""",
    implementation = _merged_external_kernel_modules_uapi_headers,
    attrs = {
        "merged_kernel_uapi_headers": attr.label(
            doc = "The UAPI headers from the core-kernel and in-tree UAPI headers merged together.",
            allow_files = True,
            mandatory = True,
        ),
        "external_modules_kernel_uapi_headers": attr.label_list(
            doc = "A list of labels referring to ddk_uapi_headers() targets for external modules.",
            allow_files = True,
            mandatory = True,
        ),
    },
    toolchains = [hermetic_toolchain.type],
)

def _get_sw5100_in_tree_modules():
    sw5100_modules = []

    for mod in monaco_in_tree_modules:
        if mod not in SW5100_MODULES_EXCLUDE_LIST:
            sw5100_modules.append(mod)

    sw5100_modules.extend(SW5100_ADDITIONAL_IN_TREE_MODULES)

    return sw5100_modules

def define_sw5100_pixel_watch(
        name,
        device_build_config,
        device_defconfig_fragment,
        device_external_modules,
        device_gki_modules,
        device_tree_sources,
        device_vendor_kernel_boot_modules,
        device_vendor_kernel_boot_recovery_modules,
        device_vendor_dlkm_blocklist,
        dtb_overlays,
        dtbs,
        dtstree_target):
    """Generates the rules to build kernel artifacts for sw5100 based devices.

    This macro generates the rules to compile the kernel, kernel modules,
    devicetree blob, devicetree overlays, and kernel images (e.g. boot.img,
    dtbo.img, vendor_kernel_boot.img, etc).

    The macro also generates the rules to merge the UAPI headers from the GKI
    kernel under common/, the private/msm-google tree and the kernel modules,
    and outputs them as a compressed tarball.

    Finally, the macro also generates the rules to collect all of the
    unstripped kernel modules for debugging purposes in a compressed
    tarball.

    Args:
        name: The name of the device.

        device_build_config: The device-specific build config file.

        device_defconfig_fragment: A device-specific GKI defconfig fragment file.

        device_external_modules: A list containing the device-specific external
            modules.

        device_gki_modules: A device-specific list of GKI modules that is needed
            for the device. This list is concatenated with the SW5100_COMMON_GKI_MODULES
            list.

        device_tree_sources: A label that refers to all of the sources needed
            to compile the devicetree blob and overlays.

        device_vendor_kernel_boot_modules: A file containing the list of
            device-specific kernel modules that should be loaded during first
            stage init. This file is concatenated with the following file:

            `vendor_kernel_boot_modules.monaco`
            `vendor_kernel_boot_modules.sw5100`

        device_vendor_kernel_boot_recovery_modules: A file containing the list of
            device-specific kernel modules that should be loaded during first
            stage init when booting into recovery mode. This list is concatenated
            with the following file:

            `vendor_kernel_boot_modules.monaco`
            `vendor_kernel_boot_modules.sw5100`
            `device_vendor_kernel_boot_modules`
            `vendor_kernel_boot_recovery_modules.sw5100`

        device_vendor_dlkm_blocklist: A file containing the list of modules
            that should be blocked from being loaded automatically during second
            stage init.

        dtb_overlays: A list of the names of the overlays. The names should
            be relative to the base directory of the devicetree source for the
            device.

        dtbs: A list of the base dtbs for the device. The names should be
            relative to the base directory of the devicetree source for the device.

        dtstree_target: A label referring to the `kernel_dtstree()` target
            for the device-specific devicetree.
    """

    device_name = name
    ext_mods = SW5100_COMMON_EXTERNAL_MODULES + device_external_modules
    gki_mods = SW5100_COMMON_GKI_MODULES + device_gki_modules

    kernel_build_config(
        name = "build.config.sw5100_{}.generated".format(device_name),
        srcs = [
            # do not sort
            "//private/google-modules/soc/msm:build.config.sw5100",
            "//{}:build.config.common".format(KERNEL_DIR),
            "//{}:build.config.aarch64".format(KERNEL_DIR),
            device_build_config,
        ],
    )

    # Create a copy of the monaco_gki.fragment file and find out which
    # defconfig options will be overridden by sw5100.fragment. Then,
    # insert "# nocheck: ..." lines into each of those lines, and
    # use the copy when combining the defconfig fragments together.
    #
    # This achieves the following:
    #
    # 1. The build tools will ignore checking to see if a config
    # is ultimately set as expected. This is desirable since the
    # state of the config matters on the order in which the defconfig
    # fragments are applied.
    #
    # 2. The original monaco_gki.fragment file does not need to be
    # modified.
    hermetic_genrule(
        name = "monaco_gki_fragment_generated",
        srcs = [
            "//private/google-modules/soc/msm:monaco_gki_fragment",
            "//private/google-modules/soc/msm:sw5100_gki_fragment",
        ],
        cmd = """
              cp $(location //private/google-modules/soc/msm:monaco_gki_fragment) $@

              config_set="s/^(CONFIG_\\w*)=.*/\\1/p"
              config_not_set="s/^# (CONFIG_\\w*) is not set$$/\\1/p"
              configs=$$(sed -n -E -e "$${config_set}" -e "$${config_not_set}" $@)

              for config in $${configs}; do
                if grep -q $${config} -- $(location //private/google-modules/soc/msm:sw5100_gki_fragment); then
                  new_config_set="s/(^$${config}=.*)/\\1 # nocheck: Overridden by sw5100.fragment/"
                  new_config_not_set="s/(^# "$${config}" is not set$$)/\\1 # nocheck: Overridden by sw5100.fragment/"
                  sed -E -i -e "$${new_config_set}" -e "$${new_config_not_set}" $@
                fi
              done
        """,
        outs = [
            "monaco_gki.fragment.generated",
        ],
    )

    kernel_build(
        name = "sw5100_{}".format(device_name),
        base_kernel = "//common:kernel_aarch64_download_or_build",
        build_config = ":build.config.sw5100_{}.generated".format(device_name),
        collect_unstripped_modules = True,
        defconfig_fragments = [
            # do not sort
            ":monaco_gki_fragment_generated",
            "//private/google-modules/soc/msm:sw5100_gki_fragment",
            device_defconfig_fragment,
        ],
        dtstree = dtstree_target,
        kconfig_ext = "//private/google-modules/soc/msm:sw5100_kconfig_ext",
        kmi_symbol_list = "//common:android/abi_gki_aarch64_pixel_watch",
        make_goals = [
            "modules",
            "dtbs",
        ],
        module_outs = _get_sw5100_in_tree_modules(),
        outs = dtbs + dtb_overlays,
        srcs = [
            "//{}:kernel_aarch64_sources".format(KERNEL_DIR),
            "//private/msm-google/drivers/power/supply/qcom:qcom_battery_modules.kconfig",
            "//private/msm-google/drivers/power/supply/qcom:qcom_battery_modules.makefile",
            "//private/google-modules/soc/msm:sw5100_sources",
            device_defconfig_fragment,
            device_tree_sources,
            device_vendor_kernel_boot_modules,
            device_vendor_kernel_boot_recovery_modules,
            device_vendor_dlkm_blocklist,
        ],
        strip_modules = True,
    )

    kernel_module_group(
        name = "sw5100_{}_ext_modules".format(device_name),
        srcs = ext_mods,
    )

    kernel_modules_install(
        name = "sw5100_{}_modules_install".format(device_name),
        kernel_build = "//private/google-modules/soc/msm:msm_kernel_build",
        kernel_modules = [":sw5100_{}_ext_modules".format(device_name)],
    )

    kernel_unstripped_modules_archive(
        name = "sw5100_{}_unstripped_modules_archive".format(device_name),
        kernel_build = ":sw5100_{}".format(device_name),
        kernel_modules = [":sw5100_{}_ext_modules".format(device_name)],
    )

    merged_kernel_uapi_headers(
        name = "sw5100_{}_merged_kernel_and_in_tree_uapi_headers".format(device_name),
        kernel_build = ":sw5100_{}".format(device_name),
        kernel_modules = [":sw5100_{}_ext_modules".format(device_name)],
    )

    merged_external_kernel_modules_uapi_headers(
        name = "sw5100_{}_merged_kernel_uapi_headers".format(device_name),
        merged_kernel_uapi_headers = ":sw5100_{}_merged_kernel_and_in_tree_uapi_headers".format(device_name),
        external_modules_kernel_uapi_headers = SW5100_COMMON_EXTERNAL_MODULES_DDK_UAPI_HEADERS,
    )

    hermetic_genrule(
        name = "vendor_kernel_boot_modules.sw5100_{}-cat".format(device_name),
        srcs = [
            "//private/google-modules/soc/msm:vendor_kernel_boot_modules.monaco",
            "//private/google-modules/soc/msm:vendor_kernel_boot_modules.sw5100",
            device_vendor_kernel_boot_modules,
        ],
        cmd = """
            cat $(location //private/google-modules/soc/msm:vendor_kernel_boot_modules.monaco) \\
                $(location //private/google-modules/soc/msm:vendor_kernel_boot_modules.sw5100) \\
                $(location {}) \\
                > $@
        """.format(device_vendor_kernel_boot_modules),
        outs = [
            "vendor_kernel_boot_modules.concat",
        ],
    )

    hermetic_genrule(
        name = "vendor_kernel_boot_charger_modules.sw5100_{}-cat".format(device_name),
        srcs = [
            "//private/google-modules/soc/msm:vendor_kernel_boot_modules.monaco",
            "//private/google-modules/soc/msm:vendor_kernel_boot_modules.sw5100",
            device_vendor_kernel_boot_modules,
            "//private/google-modules/soc/msm:vendor_kernel_boot_charger_modules.sw5100",
        ],
        cmd = """
            cat $(location //private/google-modules/soc/msm:vendor_kernel_boot_modules.monaco) \\
                $(location //private/google-modules/soc/msm:vendor_kernel_boot_modules.sw5100) \\
                $(location {}) \\
                $(location //private/google-modules/soc/msm:vendor_kernel_boot_charger_modules.sw5100) \\
                > $@
        """.format(device_vendor_kernel_boot_modules),
        outs = [
            "vendor_kernel_boot_charger_modules.concat",
        ],
    )

    hermetic_genrule(
        name = "vendor_kernel_boot_recovery_modules.sw5100_{}-cat".format(device_name),
        srcs = [
            "//private/google-modules/soc/msm:vendor_kernel_boot_modules.monaco",
            "//private/google-modules/soc/msm:vendor_kernel_boot_modules.sw5100",
            device_vendor_kernel_boot_modules,
            "//private/google-modules/soc/msm:vendor_kernel_boot_recovery_modules.sw5100",
            device_vendor_kernel_boot_recovery_modules,
        ],
        cmd = """
            cat $(location //private/google-modules/soc/msm:vendor_kernel_boot_modules.monaco) \\
                $(location //private/google-modules/soc/msm:vendor_kernel_boot_modules.sw5100) \\
                $(location {}) \\
                $(location //private/google-modules/soc/msm:vendor_kernel_boot_recovery_modules.sw5100) \\
                $(location {}) \\
                > $@
        """.format(device_vendor_kernel_boot_modules, device_vendor_kernel_boot_recovery_modules),
        outs = [
            "vendor_kernel_boot_recovery_modules.concat",
        ],
    )

    write_file(
        name = "system_dlkm_modules_list.sw5100_{}-cat".format(device_name),
        out = "system_dlkm_modules_list.concat",
        content = gki_mods,
    )

    hermetic_genrule(
        name = "vendor_dlkm_blocklist.sw5100_{}-cat".format(device_name),
        srcs = [
            "//private/google-modules/soc/msm:vendor_dlkm.blocklist.monaco",
            device_vendor_dlkm_blocklist,
        ],
        cmd = """
            cat $(location //private/google-modules/soc/msm:vendor_dlkm.blocklist.monaco) \\
                $(location {}) \\
                > $@
        """.format(device_vendor_dlkm_blocklist),
        outs = [
            "vendor_dlkm_blocklist.concat",
        ],
    )

    kernel_images(
        name = "sw5100_{}_images".format(device_name),
        base_kernel_images = "//common:kernel_aarch64_images_download_or_build",
        build_boot = False,
        build_dtbo = True,
        build_initramfs = True,
        build_system_dlkm = True,
        build_vendor_dlkm = True,
        build_vendor_kernel_boot = True,
        dedup_dlkm_modules = True,
        deps = [
            "//prebuilts/boot-artifacts/selinux:file_contexts",
            "//prebuilts/boot-artifacts/selinux:system_file_contexts",
        ],
        dtbo_srcs = [":sw5100_{}/".format(device_name) + file for file in dtb_overlays],
        kernel_build = ":sw5100_{}".format(device_name),
        kernel_modules_install = ":sw5100_{}_modules_install".format(device_name),
        modules_list = ":vendor_kernel_boot_modules.sw5100_{}-cat".format(device_name),
        modules_charger_list = ":vendor_kernel_boot_charger_modules.sw5100_{}-cat".format(device_name),
        modules_recovery_list = ":vendor_kernel_boot_recovery_modules.sw5100_{}-cat".format(device_name),
        system_dlkm_modules_list = ":system_dlkm_modules_list.sw5100_{}-cat".format(device_name),
        system_dlkm_props = "//private/google-modules/soc/msm:system_dlkm.props.sw5100",
        vendor_dlkm_modules_blocklist = ":vendor_dlkm_blocklist.sw5100_{}-cat".format(device_name),
        vendor_dlkm_modules_list = "//private/google-modules/soc/msm:vendor_dlkm_modules.sw5100",
        vendor_dlkm_props = "//private/google-modules/soc/msm:vendor_dlkm.props.sw5100",
    )

    device_dist_targets = [
        ":sw5100_{}".format(device_name),
        ":sw5100_{}_unstripped_modules_archive".format(device_name),
        ":sw5100_{}_merged_kernel_uapi_headers".format(device_name),
        ":sw5100_{}_modules_install".format(device_name),
        ":sw5100_{}_images".format(device_name),
        "//common:kernel_aarch64_download_or_build",
        "//common:kernel_aarch64_headers_download_or_build",
        "//common:kernel_aarch64_gki_artifacts_download_or_build",
    ]

    if len(gki_mods) > 0:
        extracted_system_dlkm(
            name = "sw5100_{}_extracted_system_dlkm".format(device_name),
            gki_modules = [paths.basename(m) for m in gki_mods],
            images = "//common:kernel_aarch64_images_download_or_build",
        )

        device_dist_targets.append("sw5100_{}_extracted_system_dlkm".format(device_name))

    kernel_abi(
        name = "sw5100_{}_abi".format(device_name),
        kernel_build = ":sw5100_{}".format(device_name),
        kernel_modules = [":sw5100_{}_ext_modules".format(device_name)],
        kmi_symbol_list_add_only = True,
        module_grouping = False,
    )

    copy_to_dist_dir(
        name = "sw5100_{}_dist".format(device_name),
        data = device_dist_targets,
        dist_dir = "out/{}/dist".format(device_name),
        flat = True,
        log = "info",
    )
