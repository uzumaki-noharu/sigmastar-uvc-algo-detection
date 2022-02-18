INC  += $(DB_BUILD_TOP)/internal/live555/UsageEnvironment/include
INC  += $(DB_BUILD_TOP)/internal/live555/groupsock/include
INC  += $(DB_BUILD_TOP)/internal/live555/liveMedia/include
INC  += $(DB_BUILD_TOP)/internal/live555/BasicUsageEnvironment/include
INC  += $(DB_BUILD_TOP)/internal/live555/mediaServer/include
INC  += $(DB_BUILD_TOP)/internal/iniparser
INC  += $(DB_BUILD_TOP)/internal/algo/include
INC  += ./internal/cus3a
INC  += $(DB_BUILD_TOP)/internal/Algor
INC  += $(DB_BUILD_TOP)/internal/ldc

ST_DEP := common vpe venc vif uvc uac cus3a iniparser live555 rgn
MODE := $(findstring fastboot, $(BOARD))

MODE:=fastboot
LIBS := -lalgor $(LIBS)
LIBS += -L./internal/ldc
LIBS += -lmi_sensor -lmi_vif -lmi_vpe -lmi_venc -lmi_iqserver -lmi_divp -lmi_ipu -lmi_rgn
LIBS += -lmi_isp -lcus3a -lispalgo
LIBS += -L $(DB_BUILD_TOP)/internal/algo/libs -lsstar_algo_detection
ifeq ($(CHIP), i6e)
LIBS +=-lfbc_decode -L $(DB_BUILD_TOP)/internal/ldc -leptz
endif
ifeq ($(MODE), fastboot)
CODEDEFINE += -DFASTBOOT_MODE=1
else
ST_DEP += common vpe venc vif uvc rgn onvif live555
LIBS += -lmi_rgn -lmi_divp -lmi_vdf -lmi_shadow -lOD_LINUX -lMD_LINUX -lVG_LINUX -lmi_ive -lmi_ao
CODEDEFINE += -DFASTBOOT_MODE=0
endif
LIBS += -L$(DB_BUILD_TOP)/internal/Algor/libs
AUDIO:=enable
ifeq ($(AUDIO), enable)
LIBS += -lmi_ao -lmi_ai
LIBS += -lAEC_LINUX -lAED_LINUX -lAPC_LINUX -lBF_LINUX -lSSL_LINUX -lSRC_LINUX -lg711 -lg726
CODEDEFINE += -DAUDIO_ENABLE
endif

