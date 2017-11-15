## THIS IS A GENERATED FILE -- DO NOT EDIT
.configuro: .libraries,28L linker.cmd package/cfg/task_p28L.o28L

linker.cmd: package/cfg/task_p28L.xdl
	$(SED) 's"^\"\(package/cfg/task_p28Lcfg.cmd\)\"$""\"C:/Users/A00885419/Desktop/Piccollo2AMC/Piccollo2AMC/.config/xconfig_task/\1\""' package/cfg/task_p28L.xdl > $@
