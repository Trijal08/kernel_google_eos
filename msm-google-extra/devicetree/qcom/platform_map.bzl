_platform_map = {
    "monaco": {
        "dtb_list": [
            {"name": "monaco.dtb"},
            {"name": "monacop.dtb"},
        ],
        "dtbo_list": [
            {"name": "monaco-idp-v1-overlay.dtbo"},
            {"name": "monaco-idp-v1.1-overlay.dtbo"},
            {"name": "monaco-standalone-idp-v1-overlay.dtbo"},
            {"name": "monaco-idp-v2-overlay.dtbo"},
            {"name": "monaco-standalone-idp-v2-overlay.dtbo"},
            {"name": "monaco-idp-v3-overlay.dtbo"},
            {"name": "monaco-standalone-idp-v3-overlay.dtbo"},
            {"name": "monaco-wdp-v1-overlay.dtbo"},
            {"name": "monaco-wdp-v1.1-overlay.dtbo"},
            {"name": "monaco-standalone-wdp-v1-overlay.dtbo"},
            {"name": "monaco-atp-v1-overlay.dtbo"},
            {"name": "monaco-standalone-atp-v1-overlay.dtbo"},
        ],
    },
}

def _get_dtb_lists(target, dt_overlay_supported):
    if not target in _platform_map:
        fail("{} not in device tree platform map!".format(target))

    ret = {
        "dtb_list": [],
        "dtbo_list": [],
    }

    for dtb_node in [target] + _platform_map[target].get("binary_compatible_with", []):
        ret["dtb_list"].extend(_platform_map[dtb_node].get("dtb_list", []))
        if dt_overlay_supported:
            ret["dtbo_list"].extend(_platform_map[dtb_node].get("dtbo_list", []))
        else:
            # Translate the dtbo list into dtbs we can append to main dtb_list
            for dtb in _platform_map[dtb_node].get("dtb_list", []):
                dtb_base = dtb["name"].replace(".dtb", "")
                for dtbo in _platform_map[dtb_node].get("dtbo_list", []):
                    if not dtbo.get("apq", True) and dtb.get("apq", False):
                        continue

                    dtbo_base = dtbo["name"].replace(".dtbo", "")
                    ret["dtb_list"].append({"name": "{}-{}.dtb".format(dtb_base, dtbo_base)})

    return ret

def get_dtb_list(target, dt_overlay_supported = True):
    return [dtb["name"] for dtb in _get_dtb_lists(target, dt_overlay_supported).get("dtb_list", [])]

def get_dtbo_list(target, dt_overlay_supported = True):
    return [dtb["name"] for dtb in _get_dtb_lists(target, dt_overlay_supported).get("dtbo_list", [])]
