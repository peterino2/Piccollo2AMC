#
_XDCBUILDCOUNT = 0
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = C:/ti/bios_6_35_04_50/packages;C:/Apps/ti/ccsv7/ccs_base;C:/Users/A00885419/Desktop/Piccollo2AMC/Piccollo2AMC/.config
override XDCROOT = C:/ti/xdctools_3_25_03_72
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = C:/ti/bios_6_35_04_50/packages;C:/Apps/ti/ccsv7/ccs_base;C:/Users/A00885419/Desktop/Piccollo2AMC/Piccollo2AMC/.config;C:/ti/xdctools_3_25_03_72/packages;..
HOSTOS = Windows
endif
