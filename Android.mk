LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

ifeq ($(TARGET_KERNEL_CONFIG_NAME),)
$(error "TARGET_KERNEL_CONFIG_NAME not set")
endif

kernelimage: KERNEL_ROOT := $(LOCAL_PATH)
$(PRODUCT_OUT)/uImage: KERNEL_ROOT := $(LOCAL_PATH)
clean-kernel: KERNEL_ROOT := $(LOCAL_PATH)

.PHONY: FORCE
$(LOCAL_PATH)/arch/arm/boot/uImage: FORCE
	@echo "start build kernel with cfgfile: $(TARGET_KERNEL_CONFIG_NAME)"
	+$(MAKE) -C "$(KERNEL_ROOT)" $(TARGET_KERNEL_CONFIG_NAME) \
		ARCH=arm \
		CROSS_COMPILE=${ANDROID_BUILD_TOP}/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin/arm-eabi-
	+$(MAKE) -C "$(KERNEL_ROOT)" uImage \
		ARCH=arm \
		CROSS_COMPILE=${ANDROID_BUILD_TOP}/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin/arm-eabi-

clean-kernel::
	$(MAKE) -C "$(KERNEL_ROOT)" clean

LOCAL_MODULE := kernelimage
LOCAL_MODULE_STEM := uImage
LOCAL_SRC_FILES := arch/arm/boot/uImage
LOCAL_MODULE_PATH := $(PRODUCT_OUT)
LOCAL_MODULE_CLASS := ETC

include $(BUILD_PREBUILT)
