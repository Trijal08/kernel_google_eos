AUDIO_DLKM_ENABLE := false
ifeq ($(TARGET_KERNEL_DLKM_DISABLE), true)
  ifeq ($(TARGET_KERNEL_DLKM_AUDIO_OVERRIDE),true)
    AUDIO_DLKM_ENABLE := true
  endif
else
  AUDIO_DLKM_ENABLE := true
endif

BOARD_OPENSOURCE_DIR ?= vendor/qcom/opensource

ifeq ($(AUDIO_DLKM_ENABLE), true)
  ifeq ($(call is-board-platform-in-list,taro kalama bengal monaco msmnile), true)
    include $(BOARD_OPENSOURCE_DIR)/audio-kernel/audio_kernel_modules.mk
  endif
  ifeq ($(ENABLE_AUDIO_LEGACY_TECHPACK),true)
    include $(BOARD_OPENSOURCE_DIR)/audio-kernel/legacy/audio_kernel_modules.mk
  endif
  BOARD_VENDOR_KERNEL_MODULES += $(AUDIO_KERNEL_MODULES)
endif
