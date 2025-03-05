#********************************************************************************
#Copyright (c) 2025, STMicroelectronics - All Rights Reserved
#This file is licensed under open source license ST SLA0103
#********************************************************************************
#Guidelines : https://developer.android.com/ndk/guides/android_mk

LOCAL_PATH:= $(call my-dir)

# ** src files **
include $(CLEAR_VARS)
INC_CFLAGS=$(LOCAL_PATH)/main $(LOCAL_PATH)/ioctl $(LOCAL_PATH)/utils $(LOCAL_PATH)/flk_autogain $(LOCAL_PATH)/fft
LOCAL_C_INCLUDES := $(INC_CFLAGS)
LOCAL_SRC_FILES += $(PWD)/$(LOCAL_PATH)/main/vd628x_fft_utils.c
LOCAL_SRC_FILES += $(PWD)/$(LOCAL_PATH)/fft/fft-dit.c
LOCAL_SRC_FILES += $(PWD)/$(LOCAL_PATH)/main/vd628x_platform.c
LOCAL_SRC_FILES += $(PWD)/$(LOCAL_PATH)/main/vd628x_flk_detect.c
LOCAL_SRC_FILES += $(PWD)/$(LOCAL_PATH)/main/vd628x_main.cpp
$(warning Compiling $(LOCAL_SRC_FILES))

# ** device and associated features **
LOCAL_CFLAGS := -Wall -Wextra
#LOCAL_CFLAGS += -DVD6282
LOCAL_CFLAGS += -DVD6283
LOCAL_CFLAGS += -DLOCALLY_MEASURED_SPI_FREQUENCY

# ** debug & traces **
#LOCAL_CFLAGS += -DLOG_FFT
#LOCAL_CFLAGS += -DLOG_SAMPLES

# ** module **
LOCAL_MODULE:= vd628x_flicker
LOCAL_LDLIBS:=-llog
#include $(BUILD_EXECUTABLE)
include $(BUILD_SHARED_LIBRARY)

# ******** test ********
# ** flags **
include $(CLEAR_VARS)
INC_CFLAGS=$(LOCAL_PATH)/main
LOCAL_C_INCLUDES := $(INC_CFLAGS)
LOCAL_SRC_FILES := $(PWD)/$(LOCAL_PATH)/test/test.cpp
$(warning Compiling $(LOCAL_SRC_FILES))

# ** debug & traces **
LOCAL_CPPFLAGS := -Wall -Wextra
LOCAL_LDLIBS:=-llog

# ** module **
LOCAL_MODULE:= vd628x_flicker_detect_testapp
include $(BUILD_EXECUTABLE)
